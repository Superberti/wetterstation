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
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_system.h"
#include "lwip/err.h"
#include "lwip/sys.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include <lwip/inet.h>
#include "spi_flash_mmap.h"
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
#include "rtc_wdt.h"
#include "esp_check.h"
#include "sensors/ds3231.h"
#include "common/tools.h"
#include "nvs_handle.hpp"
#include <memory>
#include "esp_timer.h"

#define FB2_VERSION "1.0.0.0"

/*
 * Pindefinitionen:
 *
 * SHT40, SDA -> 19 (I2C-0) (SHT40 und BMP390)
 * SHT40, SCL -> 20 (I2C-0) (SHT40 und BMP390)
 *
 *
 * BOARD_LED -> 15
 *
 * WAKE-UP-Schalter -> 11 (Pull-Up, Active-Low)
 *
 * Batteriespannung über 2x100 kOhm Spannungsteiler an GPIO2
 */

// SX1262 LoRa
#define PIN_NUM_MISO GPIO_NUM_21
#define PIN_NUM_MOSI GPIO_NUM_22
#define PIN_NUM_CLK GPIO_NUM_23
#define PIN_NUM_CS GPIO_NUM_9

// SHT40 und BMP390 am I2C-Bus 0
#define PIN_SDA_BUS0 GPIO_NUM_19
#define PIN_SCL_BUS0 GPIO_NUM_20

// GPIOs
#define BOARD_LED GPIO_NUM_17
#define WAKEUP_INPUT_PIN GPIO_NUM_7

// Interner ADC (ADC1 Channel1)
#define ADC_V_BATT ADC_CHANNEL_1

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
}

#define NUM_SENSOR_DATA 10

logger::logger()
{
  mADCHandle = NULL;
  mLoggerResetMode = RESET_MODE_POWERON;
  mStartTime = GetTime_us();
  // Log auf eigene Funktion umbiegen
  Sht = new SHT40;
  //Bmp = new BMP390;
  switch (esp_sleep_get_wakeup_cause())
  {
  case ESP_SLEEP_WAKEUP_TIMER:
  {
    // Modus nicht mehr vorgesehen! (macht jetzt der ULP)
    mLoggerResetMode = RESET_MODE_TIMER;
    ESP_LOGI(TAG, "Wake up from ESP sleep-timer. SleepCounter=%d", SleepCounter);
    break;
  }
  case ESP_SLEEP_WAKEUP_EXT0:
  {
    // Es wurde die Aktivierungstaste am Logger gedrückt, jetzt wird das WLAN aktiviert und
    // die Logs können verwaltet werden
    mLoggerResetMode = RESET_MODE_KEY;
    ESP_LOGI(TAG, "Wake up from activation key. SleepCounter=%d", SleepCounter);
    break;
  }
  case ESP_SLEEP_WAKEUP_ULP:
  {
    // Der ESP wurde durch den ULP aufgeweckt. Jetzt sind Messdaten zum Abspeichern vorhanden!
    mLoggerResetMode = RESET_MODE_ULP;
    ESP_LOGI(TAG, "Wake up from ULP processor (sensor data ready). SleepCounter=%d", SleepCounter);
    break;
  }
  default:
  {
    // Log-Modus aktivieren direkt nach dem Einschalten
    mLoggerResetMode = RESET_MODE_POWERON;
    ESP_LOGI(TAG, "Power on reset\n");

    // Init RTC-Variablen beim ersten Boot
    gettimeofday(&sleep_enter_time, NULL);
    SleepCounter = 0;
    LogCounter = 0;
  }
  }
}

void logger::Run()
{
  bool CRC_Err = false;
  uint32_t SerialNo = 0;
  esp_err_t ret;

  // Initialisierung GPIO
  ESP_LOGI(TAG, "GPIO init...");
  InitGPIO();

    for (int i = 0; i < 20; i++)
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
  else
  {
    Sht->ReadSerial(SerialNo, CRC_Err);
    ESP_LOGI(TAG, "SHT40 Seriennummer: %lu", SerialNo);
  }

  SD.PC = 0;

    // Sensordaten von der Hardware auslesen
    std::string res = ReadSensorData();
  GoSleep();
}

logger::~logger()
{
  delete Sht;
  delete Bmp;
}

#define SLEEP_TIME 120

void logger::GoSleep()
{
  ESP_LOGI(TAG, "Configuring sleep mode...");

  // SD-Karte abschalten
  
  ESP_LOGI(TAG, "Weckzeit einstellen nach %d s", SLEEP_TIME);
  ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(SLEEP_TIME * 1000000));

  rtc_gpio_pullup_en(WAKEUP_INPUT_PIN);
  rtc_gpio_pulldown_dis(WAKEUP_INPUT_PIN);

  ESP_ERROR_CHECK(esp_sleep_enable_ext1_wakeup(WAKEUP_INPUT_PIN, ESP_EXT1_WAKEUP_ANY_LOW));
  gpio_set_level(BOARD_LED, 0);

  // rtc_gpio_isolate(BOARD_LED);
  // rtc_gpio_isolate(SWITCH_BRIDGE_AND_CLOCK);

  // Pins auf Input
  gpio_set_direction(PIN_NUM_CS, GPIO_MODE_INPUT);
  gpio_set_direction(PIN_NUM_MISO, GPIO_MODE_INPUT);
  gpio_set_direction(PIN_NUM_MOSI, GPIO_MODE_INPUT);
  gpio_set_direction(PIN_NUM_CLK, GPIO_MODE_INPUT);
  gpio_set_direction(PIN_SDA_BUS0, GPIO_MODE_INPUT);
  gpio_set_direction(PIN_SCL_BUS0, GPIO_MODE_INPUT);

  // ADC aus
  if (mADCHandle != NULL)
    adc_oneshot_del_unit(mADCHandle);

  SleepCounter++;
  ESP_LOGI(TAG, "Going to sleep now!");

  //gpio_deep_sleep_hold_en();
  ESP_LOGI(TAG, "Starting ULP program...");
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
  bool iCRCErr;
  uint16_t ADCVal;
  int RetryCounter = 0;
  SensorData tmp;

  gpio_set_level(BOARD_LED, 1);

  tmp.uptime_s = GetSecondsAfterStart();
  // ESP_LOGI(TAG,"Alive time: %d s",tmp.alive_timer_s );

  // SHT40 (wir verwenden die Temperatur vom SHT, die ist genauer!)
  RetryCounter = 0;
  if (!SD.SkipSHT40)
  {
    do
    {
      ret = Sht->Read(tmp.Temp_deg, tmp.Hum_per, iCRCErr);
      if (ret != ESP_OK)
      {
        SD.SHT40Err++;
        ESP_LOGE(TAG, "Fehler beim Lesen der Temperatur/Luftfeuchtigkeit des SHT40: %d, Versuch %d", ret, RetryCounter + 1);
      }
      else if (iCRCErr)
      {
        SD.SHT40Err++;
        ESP_LOGE(TAG, "Fehler beim Lesen des SHT40 CRC (T/L), Versuch %d", RetryCounter + 1);
        ret = ESP_ERR_INVALID_CRC;
      }
      RetryCounter++;
      if (ret != ESP_OK)
        vTaskDelay(100 / portTICK_PERIOD_MS);
    } while (ret != ESP_OK && RetryCounter < MaxRetries);

    if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "SHT40: Wiederholtes Lesen des Sensors fehlgeschlagen! Sensor wird ab jetzt übersprungen!");
      res += "Reading SHT40(temperature, humidity) error. Skipping sensor. ";
      SD.SkipSHT40 = true;
    }
    else
    {
      // ESP_LOGI("SHT40", "SHT40 Temperatur: %.2f°C Luftfeuchte: %.2f %%", tmp.Temp_deg, tmp.Hum_per);
    }
  }

  if (SD.SkipSHT40)
  {
    tmp.Hum_per = -1;
    tmp.Temp_deg = -100;
    ESP_LOGI(TAG, "SHT40 wird übersprungen...");
    res += "Skipping SHT40 sensor. ";
  }

  // Vorher noch kurz die Batteriespannung abfragen. Am besten mit allen eingeschalteten Geräten, dann wird die Spannung unter
  // Belastung gemessen.
  tmp.VBatt_V = GetVBatt();

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
  // Referenzspannung 1.1V, Spannungsteiler 100K/100K, Abschwächung 12dB (Faktor 4) = 8.8 V bei Vollausschlag (4095)
  // Evtl. braucht es noch Offset/Gain als Kalibrierung für einen vernünftigen Wert. Alternativ an den ADS1115 anschließen...
  // tmp.VBatt_V = AdcMean / 4095 * 8.8; // -> Das ist die Theorie
  VBatt_V = AdcMean / 4095 * 6.6566; // -> ... und das die Praxis
  return VBatt_V;
}

esp_err_t logger::InitGPIO()
{
  rtc_gpio_hold_dis(BOARD_LED);
  // Outputs
  const uint64_t OutputBitMask = (1ULL << BOARD_LED) | (1ULL << PIN_NUM_CS) ;

  gpio_config_t ConfigOutput = {};
  ConfigOutput.pin_bit_mask = OutputBitMask;
  ConfigOutput.mode = GPIO_MODE_INPUT_OUTPUT; // damit man auch den aktuell eingestellten Wert zurücklesen kann!
  ConfigOutput.pull_up_en = GPIO_PULLUP_DISABLE;
  ConfigOutput.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ConfigOutput.intr_type = GPIO_INTR_DISABLE;
  ESP_RETURN_ON_ERROR(gpio_config(&ConfigOutput), TAG, "Fehler bei Init output GPIO");

  // Inputs
  const uint64_t InputBitMask = (1ULL << WAKEUP_INPUT_PIN);
  gpio_config_t ConfigInput = {};
  ConfigInput.pin_bit_mask = InputBitMask;
  ConfigInput.mode = GPIO_MODE_INPUT;
  ConfigInput.pull_up_en = GPIO_PULLUP_ENABLE;
  ConfigInput.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ConfigInput.intr_type = GPIO_INTR_DISABLE;
  ESP_RETURN_ON_ERROR(gpio_config(&ConfigInput), TAG, "Fehler bei Init input GPIO");
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
