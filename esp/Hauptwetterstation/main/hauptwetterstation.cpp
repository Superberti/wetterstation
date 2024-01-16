#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include <sys/time.h>
#include <math.h>
#include "esp_sleep.h"
#include "driver/rtc_io.h"
#include "soc/rtc.h"
#include "sht40.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_random.h"
#include "tools/json.hpp"
#include <esp_sleep.h>
#include <driver/rtc_io.h>
#include "lorastructs.h"
#include "esp_wifi.h"
#include "hauptwetterstation.h"
#include "bmp390.h"
extern "C"
{
#include <u8g2.h>
#include <u8g2_esp32_hal.h>
}

using json = nlohmann::json;

/*
 * Die Hauptwetterstation. Verwendet wird ein ESP32-DevKit-V4-Board
 * mit 433 MHz-SX1278-LoRa-Modul.
 *
 * Angeschlossene Sensorik:
 * BMP390 - Luftdruck
 * SHT40 - Temperatur und Luftfeuchte
 * Davis Anemometer (Windgeschwindigkeit und Winrichtung) am 12-Bit-ADC (ADS1015)
 * Luxmeter (bis 200000 lx) am ADS1015
 * Davis Niederschlagsmesser (Regenwippe)
 * Gewittersensor AS3935
 *
 * Sonstiges:
 * Display: Nokia 5110-Display (Philips PCD8544)
 */

/// BMP390 Luftdrucksensor angeschlossen
#define USE_BMP390
#define BMP390_SENSOR_ADDR 0x77 // Adresse BMP390 wenn SDO auf high, auf low = 0x76

// An welchem Ort befindet sich der Sensor? (s. lorastructs.h)
#define LORA_ORT ORT_SYMPATEC
// #define LORA_ORT ORT_ARBEITSZIMMER

#define TAG "HAUPT-WS"

// Temperatursensor, Luftdruck und ADC am I2C-Bus0
#define PIN_SDA_BUS0 GPIO_NUM_13
#define PIN_SCL_BUS0 GPIO_NUM_14

// GPIOs (vorläufig)
#define LORA_SEND_LED GPIO_NUM_15
#define ERROR_LED GPIO_NUM_16
#define READ_SENSOR_LED GPIO_NUM_18
#define RAIN_COUNTER_INPUT GPIO_NUM_20
#define LIGHTNING_INPUT GPIO_NUM_21

// Nokia 5110-Display am SPI

#define DISPLAY_DIN GPIO_NUM_23 // MOSI
#define DISPLAY_CLK GPIO_NUM_18 // SCK
#define DISPLAY_CE GPIO_NUM_15  // Chip Enable
#define DISPLAY_DC GPIO_NUM_27  // Umschaltung Config/Daten
#define DISPLAY_RST GPIO_NUM_26 // Reset

u8g2_t u8g2; // a structure which will contain all the data for one display

extern "C"
{
  void app_main()
  {
    app_main_cpp();
  }
}

void app_main_cpp()
{
  esp_err_t ret;
  float iTemp_deg, iHum_per;
  double iPress_mBar, t;
  bool iCRCErr;
  int64_t StartTime = GetTime_us();
  struct timeval now;

  // Initialisierung I2C und GPIO
  InitGPIO();

  gpio_set_level(ERROR_LED, 1);
  InitI2C(I2C_NUM_0, PIN_SDA_BUS0, PIN_SCL_BUS0);

  gettimeofday(&now, NULL);

  // Init Temperatursensor
  uint32_t iSerial = 0;
  SHT40 iTempSensor(I2C_NUM_0);
  ret = iTempSensor.ReadSerial(iSerial, iCRCErr);
  ESP_LOGI(TAG, "SHT40 Seriennummer: %lu", iSerial);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Lesen der Seriennummer des SHT40: %d", ret);
  if (iCRCErr)
    ESP_LOGE(TAG, "Fehler beim Lesen des SHT40 CRC (Seriennummer)");

  InitNokia_u8g2();
  int64_t EndTime = GetTime_us();
  ESP_LOGI(TAG, "SSD1306 Init fertig nach %.1f ms", (EndTime - StartTime) / 1000.0);

  // Temperatur und Luftdruck BMP390

  BMP390 Bmp(I2C_NUM_0, BMP390_SENSOR_ADDR);
  ret = Bmp.Init();
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Initialisieren des BMP390: %d", ret);
  else
  {
    ESP_LOGI(TAG, "BMP390 Init OK");
    /*
    ret = Bmp.ReadTempAndPress(t, iPress_mBar);
    if (ret != ESP_OK)
      ESP_LOGE(TAG, "Fehler beim Auslesen des BMP390: %d", ret);
    else
      ESP_LOGI(TAG, "Aktuelle Temperatur: %.2f°C. Luftdruck(raw): %.1f mbar", t, iPress_mBar);
      */
  }

  SX1278_LoRa LoRa(HeltecESPLoRa);
  // Parameter s. InitLoRa
  ret = InitLoRa(LoRa);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Initialisieren des LoRa Moduls: %d", ret);
  EndTime = GetTime_us();
  gpio_set_level(ERROR_LED, 0);
  ESP_LOGI(TAG, "LoRa Init fertig nach %.1f ms", (EndTime - StartTime) / 1000.0);

  // vTaskDelay(2000 / portTICK_PERIOD_MS);

  u8g2_ClearDisplay(&u8g2);
  u8g2_SetFont(&u8g2, u8g2_font_crox1h_tf);

  uint8_t LoraBuf[255];
  uint16_t iCBORBuildSize;
  ret = Bmp.StartReadTempAndPress();
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Starten des BMP390: %d", ret);

  int counter = 0;
  for (;;)
  {
    ret = Bmp.ReadTempAndPressAsync(t, iPress_mBar);
    if (ret != ESP_OK)
      ESP_LOGE(TAG, "Fehler beim Lesen des BMP390: %d", ret);
    iPress_mBar = Bmp.SeaLevelForAltitude(602, iPress_mBar);
    ESP_LOGI("BMP390", "Druck: %.2f mbar Temp: %.2f°C", iPress_mBar, t);

    ret = iTempSensor.Read(iTemp_deg, iHum_per, iCRCErr);
    ESP_LOGI("SHT40", "Temp.: %.2f LF: %.2f %%", iTemp_deg, iHum_per);
    if (ret != ESP_OK)
      ESP_LOGE(TAG, "Fehler beim Lesen der Temperatur/Luftfeuchtigkeit des SHT40: %d", ret);
    if (iCRCErr)
      ESP_LOGE(TAG, "Fehler beim Lesen des SHT40 CRC (T/L)");

    iCBORBuildSize = 0;
    BuildCBORBuf(LoraBuf, sizeof(LoraBuf), iCBORBuildSize, counter, iTemp_deg, iHum_per, (float)iPress_mBar);
    if (iCBORBuildSize > sizeof(LoraBuf))
      ESP_LOGE(TAG, "LoRa Buffer overflow...:%d", iCBORBuildSize);

    // LoRa-Paket senden
    int RetryCounter = 0;
    bool SendOK = false;
    const int MaxRetries = 5;
    char DisplayBuf[256] = {};
    while (!SendOK && RetryCounter < MaxRetries)
    {
      gpio_set_level(LORA_SEND_LED, 1);
      int64_t ts = GetTime_us();
      ret = LoRa.SendLoraMsg(CMD_CBORDATA, LoraBuf, iCBORBuildSize, counter);
      SendOK = ret == ESP_OK;
      int64_t te = GetTime_us();
      gpio_set_level(LORA_SEND_LED, 0);
      ESP_LOGI(TAG, "Zeit fuer LoRa: %.1f ms. Paketgroesse: %u", double(te - ts) / 1000.0, iCBORBuildSize);
      if (!SendOK)
      {
        RetryCounter++;
        ESP_LOGE(TAG, "Fehler beim Senden eines LoRa-Paketes: %d Versuch: %d", ret, RetryCounter);
        ESP_LOGE(TAG, "Resette LORA-Modul...");
        LoRa.Reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
        ret = InitLoRa(LoRa);
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

    EndTime = GetTime_us();
    ESP_LOGI(TAG, "LoRa gesendet nach %.1f ms", (EndTime - StartTime) / 1000.0);

    u8g2_ClearBuffer(&u8g2);
    // sprintf(DisplayBuf, "[%lu] Vbatt: %.2f V", counter, iVBatt_V);
    // u8g2_DrawStr(&u8g2, 2, 14, DisplayBuf);

    sprintf(DisplayBuf, "Temp: %.2f%cC[%.2f]", iTemp_deg, 176, t);
    u8g2_DrawStr(&u8g2, 2, 28, DisplayBuf);

    sprintf(DisplayBuf, "Feuchte: %.2f %%", iHum_per);
    u8g2_DrawStr(&u8g2, 2, 42, DisplayBuf);

    sprintf(DisplayBuf, "Druck: %.2f mBar", iPress_mBar);
    u8g2_DrawStr(&u8g2, 2, 56, DisplayBuf);

    u8g2_SendBuffer(&u8g2);

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    counter++;
  }

  // Hier kommt man eigentlich nicht hin...
  u8g2_SetPowerSave(&u8g2, 1);
  ESP_LOGI(TAG, "Schalte LoRa-Modem ab...");
  LoRa.Close();
}

esp_err_t InitGPIO()
{
  // Outputs
  uint64_t OutputBitMask = (1ULL << LORA_SEND_LED) | (1ULL << ERROR_LED) | (1ULL << READ_SENSOR_LED);
  gpio_config_t ConfigOutput = {};
  ConfigOutput.pin_bit_mask = OutputBitMask;
  ConfigOutput.mode = GPIO_MODE_OUTPUT;
  ConfigOutput.pull_up_en = GPIO_PULLUP_DISABLE;
  ConfigOutput.pull_down_en = GPIO_PULLDOWN_ENABLE;
  ConfigOutput.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&ConfigOutput);

  // Inputs
  return ESP_OK;
  uint64_t InputBitMask = (1ULL << RAIN_COUNTER_INPUT);
  gpio_config_t ConfigInput = {};
  ConfigInput.pin_bit_mask = InputBitMask;
  ConfigInput.mode = GPIO_MODE_INPUT;
  ConfigInput.pull_up_en = GPIO_PULLUP_ENABLE;
  ConfigInput.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ConfigInput.intr_type = GPIO_INTR_DISABLE;
  gpio_config(&ConfigInput);

  InputBitMask = (1ULL << LIGHTNING_INPUT);
  ConfigInput.pin_bit_mask = InputBitMask;
  ConfigInput.mode = GPIO_MODE_INPUT;
  ConfigInput.pull_up_en = GPIO_PULLUP_ENABLE;
  ConfigInput.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ConfigInput.intr_type = GPIO_INTR_POSEDGE;
  gpio_config(&ConfigInput);

  // install gpio isr service
  gpio_install_isr_service(0);
  // hook isr handler for specific gpio pin
  gpio_isr_handler_add(LIGHTNING_INPUT, gpio_isr_handler, (void *)LIGHTNING_INPUT);
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
  // Input-IRQ Gewittersensor
  uint32_t gpio_num = (uint32_t)arg;
  //xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

esp_err_t InitI2C(i2c_port_t aPort, int aSDA_Pin, int aSCL_Pin)
{
  if (aPort > 1)
    return ESP_ERR_INVALID_ARG;
  esp_err_t ret;

  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = aSDA_Pin;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = aSCL_Pin;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 100000; // Fast Mode, 1000000;  // Fast Mode Plus=1MHz
  conf.clk_flags = 0;
  ret = i2c_param_config(aPort, &conf);
  if (ret != ESP_OK)
    return ret;
  return i2c_driver_install(aPort, conf.mode, 0, 0, 0);
}

esp_err_t InitLoRa(SX1278_LoRa &aLoRa)
{
  // Sendefrequenz: 434,54 MHz
  // Preambellänge: 14
  // Bandbreite 500 kHz
  // Achtung: Damit auch der neuere SX1262 verwendet wird, muss man sich über die Sync-Words ein paar mehr
  // Gedanken machen. Der SX1262 hat 2 Bytes Sync, nicht nur 1 Byte. Damit sich trotzdem beide verstehen, muss man
  // folgendes beachten:
  // SX1276: Jedes Nibble unterschiedlich von 1-7, also z.B. 0x37 => 0xYZ (Y!=Z, Z und Y <8 && >0)
  // SX1262: 0Y4Z4, also hier 0x3474
  // Sync-Byte: 0x37
  // Spreading-Factor: 8 = 256 Chips/symbol
  // Coding-Rate: 6 = 4/6 = 1,5-facher FEC-Overhead
  // Tx-Power 2-17
  return aLoRa.SetupModule(LORA_ADDR_GWHS, 434.54e6, 14, LoRaBase::LoRaBandwidth::LORA_BW_500, 0x37, LoRaBase::SpreadingFactor::SF8, LoRaBase::LoRaCodingRate::LORA_CR_4_6, 15);
}

int64_t GetTime_us()
{
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  return (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
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
    gpio_set_level(ERROR_LED, toggle);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

esp_err_t BuildCBORBuf(uint8_t *aBuf, uint16_t aMaxBufSize, uint16_t &aCBORBuildSize, uint32_t aPC, float aTemp_deg, float aHum_per, float aPress_mBar)
{
  json j =
      {
          {POS_TAG, LORA_ORT},
          {DATA_TAG,
           {
               {PC_TAG, aPC},
               {TEMP_TAG, {{VAL_TAG, aTemp_deg}}},
               {HUM_TAG, {{VAL_TAG, aHum_per}}},
               {PRESS_TAG, {{VAL_TAG, aPress_mBar}}},
               //{VOL_TAG, {{VAL_TAG, iVBatt_V}}}
           }}};

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

void InitNokia_u8g2()
{
  u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
  u8g2_esp32_hal.clk = DISPLAY_CLK;
  u8g2_esp32_hal.mosi = DISPLAY_DIN;
  u8g2_esp32_hal.cs = DISPLAY_CE;
  u8g2_esp32_hal.reset = DISPLAY_RST;
  u8g2_esp32_hal.dc = DISPLAY_DC;
  u8g2_esp32_hal_init(u8g2_esp32_hal);

  /*
    gpio_set_level(PIN_DISP_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_DISP_RESET, 1);
  */

  // Nokia 5110 Display

  u8g2_Setup_pcd8544_84x48_1(&u8g2, U8G2_R0, u8g2_esp32_i2c_byte_cb, u8g2_esp32_gpio_and_delay_cb);

  ESP_LOGI(TAG, "Nokia5110 display init...");
  u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

  ESP_LOGI(TAG, "u8g2_SetPowerSave");
  u8g2_SetPowerSave(&u8g2, 0); // wake up display
  ESP_LOGI(TAG, "u8g2_ClearBuffer");
  u8g2_ClearBuffer(&u8g2);
  u8g2_SendBuffer(&u8g2);
  ESP_LOGI(TAG, "Nokia5110 display initialized!");
}