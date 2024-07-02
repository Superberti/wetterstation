/*
 * ESP-Datenlogger für Druck, Feuchte und Temperatur
 * CBOR-Datenübertragung an die Wetterstation mit LoRa-Funk
 * Baseboard: Firebeetle 2 (ESP32-C6, https://wiki.dfrobot.com/SKU_DFR1075_FireBeetle_2_Board_ESP32_C6)
 * Angeschlossene Sensorik:
 * SHT40 - Temperatur und Luftfeuchte
 * BMP390 - Luftdruck
 * LoRa SX1262
 */

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <fstream>
#include <algorithm>
#include <sys/unistd.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <math.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include <string.h>
#include "driver/gpio.h"
#include <driver/rtc_io.h>
#include "soc/rtc.h"
#include "soc/rtc_periph.h"
#include "sensors/sht40.h"
#include "sensors/bmp390.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_random.h"
#include <esp_sleep.h>
#include "fb2.h"
#include "sensors/ads1015.h"
#include "esp_check.h"
#include "sensors/ds3231.h"
#include "tools/tools.h"
#include "tools/json.hpp"
#include <memory>
#include "esp_timer.h"
#include "lora/lorastructs.h"
#include "lora/lora.h"

#define FB2_VERSION "1.0.0.0"
#define LORA_ORT ORT_SYMPATEC
using json = nlohmann::json;

/*
 * Pindefinitionen:
 *
 * SHT40, SDA -> 19 (I2C-0) (SHT40 und BMP390)
 * SHT40, SCL -> 20 (I2C-0) (SHT40 und BMP390)
 *
 *
 * BOARD_LED -> 15
 *
 * WAKE-UP-Schalter -> 2 (Pull-Up, Active-Low)
 *
 * Batteriespannung über 2x100 kOhm Spannungsteiler an GPIO2
 */

// SX1262 LoRa-Modul (SPI)
#define LORA_PIN_MISO GPIO_NUM_2
#define LORA_PIN_MOSI GPIO_NUM_17
#define LORA_PIN_CLK GPIO_NUM_3
#define LORA_PIN_CS GPIO_NUM_16
#define LORA_PIN_RESET GPIO_NUM_7
#define LORA_PIN_DIO0 GPIO_NUM_6

// SHT40 und BMP390 am I2C-Bus 0
#define PIN_SDA_BUS0 GPIO_NUM_19
#define PIN_SCL_BUS0 GPIO_NUM_20

// BMP390 am I2C-Bus 1
#define PIN_SDA_BUS1 GPIO_NUM_6
#define PIN_SCL_BUS1 GPIO_NUM_7

// GPIOs
#define BOARD_LED GPIO_NUM_15
// #define WAKEUP_INPUT_PIN GPIO_NUM_2

// Interner ADC (ADC1 Channel1)
#define ADC_V_BATT ADC_CHANNEL_0

// I2C-Speed
#define I2C_FREQ_HZ 400000

#define TAG "FB2"

// Globals
vprintf_like_t orig_log_func = NULL;
static RTC_DATA_ATTR struct timeval sleep_enter_time;
static RTC_DATA_ATTR int SleepCounter, LogCounter;

extern "C"
{
  void app_main()
  {
    app_main_cpp();
  }
}

void app_main_cpp()
{
  // vTaskDelay(pdMS_TO_TICKS(1000));
  logger l;
  l.Run();
}

#define NUM_SENSOR_DATA 10

logger::logger()
{
  mADCHandle = NULL;
  i2c_bus_h_0 = NULL;
  mLoggerResetMode = RESET_MODE_POWERON;
  mStartTime = GetTime_us();
  // Log auf eigene Funktion umbiegen
  Sht = new SHT40;
  Bmp = new BMP390;
  LoRa_PinConfiguration PinConfig = {};
  PinConfig.ChipSelect = LORA_PIN_CS;
  PinConfig.Clock = LORA_PIN_CLK;
  PinConfig.DIO0 = LORA_PIN_DIO0;
  PinConfig.Miso = LORA_PIN_MISO;
  PinConfig.Mosi = LORA_PIN_MOSI;
  PinConfig.Reset = LORA_PIN_RESET;
  PinConfig.SPIChannel = SPI2_HOST;

  rtc_gpio_hold_dis(LORA_PIN_RESET);
  //rtc_gpio_hold_dis(BOARD_LED);

  LoRa = new SX1278_LoRa(PinConfig);
  switch (esp_sleep_get_wakeup_cause())
  {
  case ESP_SLEEP_WAKEUP_TIMER:
  {
    // Modus nicht mehr vorgesehen! (macht jetzt der ULP)
    mLoggerResetMode = RESET_MODE_TIMER;
    ESP_LOGI(TAG, "Wake up from ESP sleep-timer. SleepCounter=%d", SleepCounter);
    break;
  }

  default:
  {
    // Direkt nach dem Einschalten
    mLoggerResetMode = RESET_MODE_POWERON;
    ESP_LOGI(TAG, "Power on reset\n");

    SleepCounter = 0;
    LogCounter = 0;
  }
  }
}

void logger::Run()
{
  esp_err_t ret;

  // Initialisierung GPIO
  ESP_LOGI(TAG, "GPIO init...");
  InitGPIO();

  for (int i = 0; i < 10; i++)
  {
    gpio_set_level(BOARD_LED, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(BOARD_LED, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  //-------------ADC1 Init---------------//
  adc_oneshot_unit_init_cfg_t init_config1;
  init_config1.unit_id = ADC_UNIT_1;
  init_config1.ulp_mode = ADC_ULP_MODE_DISABLE;
  init_config1.clk_src = (adc_oneshot_clk_src_t)ADC_DIGI_CLK_SRC_DEFAULT;
  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &mADCHandle));
  adc_oneshot_chan_cfg_t AdcConfig;
  AdcConfig.atten = ADC_ATTEN_DB_12;
  AdcConfig.bitwidth = ADC_BITWIDTH_DEFAULT;
  ESP_ERROR_CHECK(adc_oneshot_config_channel(mADCHandle, ADC_V_BATT, &AdcConfig));

  gpio_set_level(BOARD_LED, 1);

  // Initialisierung I2C und GPIO
  ESP_LOGI(TAG, "I2C init...");
  ret = InitI2C(I2C_NUM_0, PIN_SDA_BUS0, PIN_SCL_BUS0, &i2c_bus_h_0);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim InitI2C(0): %d.", ret);

  // Init Temperatursensor
  ret = Sht->Init(i2c_bus_h_0, SHT40_ADDR, I2C_FREQ_HZ);
  if (ret != ESP_OK)
  {
    SD.SkipSHT40 = true;
    ESP_LOGE(TAG, "Fehler beim Initialisieren des SHT40: %d. Sensor ausgeschaltet!", ret);
  }

  // Init Drucksensor
  ret = Bmp->Init(i2c_bus_h_0, BMP390_ADDRESS, I2C_FREQ_HZ);
  if (ret != ESP_OK)
  {
    SD.SkipBMP390 = true;
    ESP_LOGE(TAG, "Fehler beim Initialisieren des BMP390: %d. Sensor ausgeschaltet!", ret);
  }

  // Sendefrequenz: 434,54 MHz
  // Preambellänge: 14
  // Bandbreite 500 kHz
  // Achtung: Damit auch der neuere SX1262 verwendet wird, muss man sich über die Sync-Words ein paar mehr
  // Gedanken machen. Der SX1262 hat 2 Bytes Sync, nicht nur 1 Byte. Damit sich trotzdem beide verstehen, muss man
  // folgendes beachten:
  // SX1278: Jedes Nibble unterschiedlich von 1-7, also z.B. 0x37 => 0xYZ (Y!=Z, Z und Y <8 && >0)
  // SX1262: 0Y4Z4, also hier 0x3474
  // Sync-Byte: 0x37
  // Spreading-Factor: 8 = 256 Chips/symbol
  // Coding-Rate: 6 = 4/6 = 1,5-facher FEC-Overhead
  // Tx-Power 2-17
  bool LoraOK = true;
  ret = LoRa->SetupModule(LORA_ADDR_GWHS, 434.54e6, 14, LoRaBase::LoRaBandwidth::LORA_BW_500, 0x37, LoRaBase::SpreadingFactor::SF8, LoRaBase::LoRaCodingRate::LORA_CR_4_6, 15);
  if (ret != ESP_OK)
  {
    LoraOK = false;
    ESP_LOGE(TAG, "Fehler beim Initialisieren vom LoRa-Modul: %d. Mopdul ausgeschaltet!", ret);
  }

  uint8_t LoraBuf[255];
  uint16_t iCBORBuildSize;
  gpio_set_level(BOARD_LED, 1);
  ReadSensorData();
  iCBORBuildSize = 0;
  BuildCBORBuf(LoraBuf, sizeof(LoraBuf), iCBORBuildSize, SD);
  if (iCBORBuildSize > sizeof(LoraBuf))
    ESP_LOGE(TAG, "LoRa Buffer overflow...:%d", iCBORBuildSize);

  // LoRa-Paket senden

  int RetryCounter = 0;
  bool SendOK = false;
  const int MaxRetries = 5;
  int64_t StartTime = GetTime_us();
  while (!SendOK && RetryCounter < MaxRetries)
  {
    gpio_set_level(BOARD_LED, 1);
    int64_t ts = GetTime_us();
    ret = LoRa->SendLoraMsg(CMD_CBORDATA, LoraBuf, iCBORBuildSize, SleepCounter);
    SendOK = ret == ESP_OK;
    int64_t te = GetTime_us();
    gpio_set_level(BOARD_LED, 0);
    ESP_LOGI(TAG, "Zeit fuer LoRa: %.1f ms. Paketgroesse: %u", double(te - ts) / 1000.0, iCBORBuildSize);
    if (!SendOK)
    {
      RetryCounter++;
      ESP_LOGE(TAG, "Fehler beim Senden eines LoRa-Paketes: %d Versuch: %d", ret, RetryCounter);
      ESP_LOGE(TAG, "Resette LORA-Modul...");
      LoRa->Reset();
      vTaskDelay(pdMS_TO_TICKS(1000));
      ret = LoRa->SetupModule(LORA_ADDR_GWHS, 434.54e6, 14, LoRaBase::LoRaBandwidth::LORA_BW_500, 0x37, LoRaBase::SpreadingFactor::SF8, LoRaBase::LoRaCodingRate::LORA_CR_4_6, 15);
      if (ret != ESP_OK)
        ESP_LOGI(TAG, "Fehler beim Initialisieren des LoRa Moduls: %d", ret);
      else
        ESP_LOGI(TAG, "Re-Init LORA erfolgreich");
    }
  }
  if (!SendOK)
  {
    ESP_LOGE(TAG, "Wiederholtes Senden fehlgeschlagen!");
  }

  int64_t EndTime = GetTime_us();
  ESP_LOGI(TAG, "LoRa gesendet nach %.1f ms", (EndTime - StartTime) / 1000.0);

  gpio_set_level(BOARD_LED, 0);

  GoSleep();
}

logger::~logger()
{
  delete Sht;
  delete Bmp;
}

#define SLEEP_TIME 10

void logger::GoSleep()
{
  ESP_LOGI(TAG, "Configuring sleep mode...");

  ESP_LOGI(TAG, "Weckzeit einstellen nach %d s", SLEEP_TIME);
  ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(SLEEP_TIME * 1000000));

  // rtc_gpio_pullup_en(WAKEUP_INPUT_PIN);
  // rtc_gpio_pulldown_dis(WAKEUP_INPUT_PIN);

  // ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(WAKEUP_INPUT_PIN, ESP_EXT1_WAKEUP_ANY_LOW));
  gpio_set_level(BOARD_LED, 0);

  //rtc_gpio_isolate(BOARD_LED);
  rtc_gpio_isolate(LORA_PIN_RESET);

  // Pins auf Input
  gpio_set_direction(LORA_PIN_CS, GPIO_MODE_INPUT);
  gpio_set_direction(LORA_PIN_MISO, GPIO_MODE_INPUT);
  gpio_set_direction(LORA_PIN_MOSI, GPIO_MODE_INPUT);
  gpio_set_direction(LORA_PIN_CLK, GPIO_MODE_INPUT);
  gpio_set_direction(LORA_PIN_DIO0, GPIO_MODE_INPUT);
  gpio_set_direction(PIN_SDA_BUS0, GPIO_MODE_INPUT);
  gpio_set_direction(PIN_SCL_BUS0, GPIO_MODE_INPUT);

  // ADC aus
  if (mADCHandle != NULL)
    adc_oneshot_del_unit(mADCHandle);

  SleepCounter++;
  ESP_LOGI(TAG, "Going to sleep now!");
  esp_deep_sleep_start();
}

uint32_t logger::GetSecondsAfterStart()
{
  int64_t MikroSecondsAfterStart = GetTime_us() - mStartTime;
  return MikroSecondsAfterStart / 1000000;
}

std::string logger::ReadSensorData()
{
  std::string res;
  const int MaxRetries = 5;
  esp_err_t ret = ESP_OK;
  int RetryCounter = 0;
  bool BmpErr = false;
  bool ShtErr = false;

  gpio_set_level(BOARD_LED, 1);

  SD.uptime_s = GetSecondsAfterStart();
  // ESP_LOGI(TAG,"Alive time: %d s",tmp.alive_timer_s );

  ret = Bmp->StartReadTempAndPress();
  if (ret != ESP_OK)
  {
    ESP_LOGE(TAG, "Fehler beim Starten des BMP390: %d", ret);
    SD.BMP390Err++;
    BmpErr = true;
  }

  // SHT40 (wir verwenden die Temperatur vom SHT, die ist genauer!)
  ret = Sht->Read(SD.Temp_deg, SD.Hum_per);
  if (ret != ESP_OK)
  {
    ShtErr = true;
    SD.SHT40Err++;
    ESP_LOGE(TAG, "Fehler beim Lesen der Temperatur/Luftfeuchtigkeit des SHT40: %d, Versuch %d", ret, RetryCounter + 1);
  }
  else
  {
    ESP_LOGI(TAG, "SHT40 Temperatur: %.2f°C Luftfeuchte: %.2f %%", SD.Temp_deg, SD.Hum_per);
  }

  // Vorher noch kurz die Batteriespannung abfragen. Am besten mit allen eingeschalteten Geräten, dann wird die Spannung unter
  // Belastung gemessen.
  SD.VBatt_V = GetVBatt();
  ESP_LOGI(TAG, "VBat: %.2f V", SD.VBatt_V);

  // BMP390
  if (!BmpErr)
  {
    float temp_bmp;
    ret = Bmp->ReadTempAndPressAsync(temp_bmp, SD.Press_mBar);
    SD.Press_mBar = Bmp->SeaLevelForAltitude(602, SD.Press_mBar);
    if (ret != ESP_OK)
    {
      BmpErr = true;
      ESP_LOGE(TAG, "Fehler beim Lesen des BMP390: %d", ret);
    }
    else
    {
      ESP_LOGI(TAG, "Druck: %.2f mbar Temp: %.2f°C", SD.Press_mBar, temp_bmp);
      if (ShtErr) // Falls der SHT nicht läuft...
        SD.Temp_deg = temp_bmp;
    }
  }
  SD.PC++;
  if (res.length() == 0)
    res = "OK";

  gpio_set_level(BOARD_LED, 0);
  return res;
}

// Batteriespannung abfragen. Am besten mit allen eingeschalteten Geräten, dann wird die Spannung unter
// Belastung gemessen.
float logger::GetVBatt()
{
  float VBatt_V;
  double AdcMean = 0;
  int AdcValue = 0;
  // Batteriespannung
  for (int i = 0; i < 64; i++)
  {
    ESP_ERROR_CHECK(adc_oneshot_read(mADCHandle, ADC_V_BATT, &AdcValue));
    AdcMean += AdcValue;
  }
  AdcMean /= 64.0;
  // Referenzspannung 1.1V, Spannungsteiler 100K/100K, Abschwächung 0dB (Faktor 1) = 1.1 V bei Vollausschlag (4095)
  // Evtl. braucht es noch Offset/Gain als Kalibrierung für einen vernünftigen Wert. Alternativ an den ADS1115 anschließen...
  // tmp.VBatt_V = AdcMean / 4095 * 8.8; // -> Das ist die Theorie
  VBatt_V = AdcMean / 4095 * 8.8; // -> ... und das die Praxis
  return VBatt_V;
}

esp_err_t logger::InitGPIO()
{
  // Outputs
  const uint64_t OutputBitMask = (1ULL << BOARD_LED);

  gpio_config_t ConfigOutput = {};
  ConfigOutput.pin_bit_mask = OutputBitMask;
  ConfigOutput.mode = GPIO_MODE_INPUT_OUTPUT; // damit man auch den aktuell eingestellten Wert zurücklesen kann!
  ConfigOutput.pull_up_en = GPIO_PULLUP_DISABLE;
  ConfigOutput.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ConfigOutput.intr_type = GPIO_INTR_DISABLE;
  ESP_RETURN_ON_ERROR(gpio_config(&ConfigOutput), TAG, "Fehler bei Init output GPIO");

  // Inputs
  /*
  const uint64_t InputBitMask = (1ULL << WAKEUP_INPUT_PIN);
  gpio_config_t ConfigInput = {};
  ConfigInput.pin_bit_mask = InputBitMask;
  ConfigInput.mode = GPIO_MODE_INPUT;
  ConfigInput.pull_up_en = GPIO_PULLUP_ENABLE;
  ConfigInput.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ConfigInput.intr_type = GPIO_INTR_DISABLE;
  ESP_RETURN_ON_ERROR(gpio_config(&ConfigInput), TAG, "Fehler bei Init input GPIO");
  */
  return ESP_OK;
}

esp_err_t logger::InitI2C(i2c_port_t aPort, gpio_num_t aSDA_Pin, gpio_num_t aSCL_Pin, i2c_master_bus_handle_t *aBusHandle)
{
  if (aPort > 1)
    return ESP_ERR_INVALID_ARG;

  i2c_master_bus_config_t conf;
  conf.i2c_port = aPort;
  conf.sda_io_num = aSDA_Pin;
  conf.scl_io_num = aSCL_Pin;
  conf.clk_source = I2C_CLK_SRC_DEFAULT;
  conf.glitch_ignore_cnt = 7;
  conf.intr_priority = 0;
  conf.trans_queue_depth = 0;
  conf.flags.enable_internal_pullup = 1;
  return i2c_new_master_bus(&conf, aBusHandle);
}

#define MAX_VPBUFLEN 256
char vprintf_buffer[MAX_VPBUFLEN];

void error(const char *format, ...)
{
  va_list myargs;
  va_start(myargs, format);

  vsnprintf(vprintf_buffer, MAX_VPBUFLEN, format, myargs);
  ESP_LOGE(TAG, "%s", vprintf_buffer);
  va_end(myargs);
  int toggle = 0;
  for (;;)
  {
    toggle = 1 - toggle;
    gpio_set_level(BOARD_LED, toggle);
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

esp_err_t logger::BuildCBORBuf(uint8_t *aBuf, uint16_t aMaxBufSize, uint16_t &aCBORBuildSize, const SensorData &aSD)
{
  json j =
      {
          {POS_TAG, LORA_ORT},
          {DATA_TAG,
           {{PC_TAG, aSD.PC},
            {TEMP_TAG, {{VAL_TAG, aSD.Temp_deg}}},
            {HUM_TAG, {{VAL_TAG, aSD.Hum_per}}},
            {PRESS_TAG, {{VAL_TAG, aSD.Press_mBar}}},
            {VOL_TAG, {{VAL_TAG, aSD.VBatt_V}}}}}};

  // serialize it to CBOR
  std::vector<std::uint8_t> v = json::to_cbor(j);
  aCBORBuildSize = v.size();
  if (v.size() > aMaxBufSize)
  {
    ESP_LOGE(TAG, "CBOR file too big!");
    aCBORBuildSize = 0;
    return ESP_OK;
  }
  else
  {
    // ESP_LOGI(TAG, "CBOR file size: %d!",aCBORBuildSize);
    memcpy(aBuf, v.data(), aCBORBuildSize);
  }

  return ESP_OK;
}
