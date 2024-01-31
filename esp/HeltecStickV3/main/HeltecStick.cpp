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
#include "HeltecStick.h"
#include "bmp390.h"
#include "ads1015.h"
#include "rtc_wdt.h"
#include "esp_check.h"

extern "C"
{
#include <u8g2.h>
#include <u8g2_esp32_hal.h>
}

using json = nlohmann::json;

/*
 * Heltec LoRa-Wireless-StickV3 mit SX162 und UC1609C LCD-Display
 *
 *
 * Lora ist auf SPI2
 * Lora SX162, ChipSelect -> 8;
 * Lora SX162, Reset -> 12;
 * Lora SX162, Miso -> 11;
 * Lora SX162, Mosi -> 10;
 * Lora SX162, Clock -> 9;
 * Lora SX162, DIO1 -> 14;
 *
 * Display ist auf SPI3
 * SCK -> 33
 * SDA -> 34
 * RST -> 38
 * CD -> 36
 * CS -> 37
 *
 * Board-LED -> 35
 */

// An welchem Ort befindet sich der Sensor? (s. lorastructs.h)
#define LORA_ORT ORT_SYMPATEC

#define TAG "HELTEC-V3"
#define BOARD_LED GPIO_NUM_35

// ERM19264 192x64, UC1609C driver Display
#define DISPLAY_DIN GPIO_NUM_34 // MOSI
#define DISPLAY_CLK GPIO_NUM_33 // SCK
#define DISPLAY_CE GPIO_NUM_37  // Chip Enable (CS)
#define DISPLAY_DC GPIO_NUM_36  // Umschaltung Config/Daten
#define DISPLAY_RST GPIO_NUM_38 // Reset

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

  ESP_LOGI(TAG, "Starte Heltec-Stick-V3 V1.0");

  esp_err_t ret;
  int64_t StartTime = GetTime_us();

  // Initialisierung I2C und GPIO
  ESP_LOGI(TAG, "GPIO init...");
  InitGPIO();

  for (int i = 0; i < 10; i++)
  {
    gpio_set_level(BOARD_LED, 1);
    vTaskDelay(pdMS_TO_TICKS(100));
    gpio_set_level(BOARD_LED, 0);
    vTaskDelay(pdMS_TO_TICKS(100));
  }

  // LoRa-Modul
  SX1262_LoRa LoRa(HeltecWirelessStick_V3);
  // Parameter s. InitLoRa
  ret = InitLoRa(LoRa);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Initialisieren des LoRa Moduls: %d", ret);
  else
  {
    ESP_LOGI(TAG, "LoRa-Modul Init OK!");
  }

  InitDisplay_u8g2();
  u8g2_ClearDisplay(&u8g2);
  u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);

  int64_t EndTime = GetTime_us();
  ESP_LOGI(TAG, "Init fertig nach %.1f ms", (EndTime - StartTime) / 1000.0);

  uint8_t LoraBuf[255];
  uint16_t iCBORBuildSize=0;
  int counter=0;
  for (;;)
  {
    // LoRa-Paket senden
    int RetryCounter = 0;
    bool SendOK = false;
    const int MaxRetries = 5;
    StartTime = GetTime_us();
    while (!SendOK && RetryCounter < MaxRetries)
    {
      gpio_set_level(BOARD_LED, 1);

      ret = LoRa.SendLoraMsg(CMD_CBORDATA, LoraBuf, iCBORBuildSize, counter);
      SendOK = ret == ESP_OK;
      
      gpio_set_level(BOARD_LED, 0);
      // ESP_LOGI(TAG, "Zeit fuer LoRa: %.1f ms. Paketgroesse: %u", double(te - ts) / 1000.0, iCBORBuildSize);
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

// Bei dem verwendeten Font (6 breit x 10 hoch) kommen wir beim Nokia Display auf
// 4 Zeilen a 14 Zeichen
#define DISPLAY_CHARS_PER_LINE 30
#define LINE_HEIGHT 11
#define DISPLAY_LINES 4
#define DISPLAY_BUF_LINES 15

    // Anzuzeigender Bereich des gesamten Display-Buffers
    int ViewLineStart = 0;
    int ViewLineEnd = 3;

    char DisplayBuf[DISPLAY_BUF_LINES][DISPLAY_CHARS_PER_LINE + 1] = {};
    snprintf(DisplayBuf[0], DISPLAY_CHARS_PER_LINE, "Counter: %d", counter);
    snprintf(DisplayBuf[1], DISPLAY_CHARS_PER_LINE, "Hello WORLD");
    snprintf(DisplayBuf[2], DISPLAY_CHARS_PER_LINE, "Everybody");
    

    u8g2_ClearBuffer(&u8g2);
    for (int i = ViewLineStart; i <= ViewLineEnd; i++)
    {
      u8g2_DrawStr(&u8g2, 0, LINE_HEIGHT * (i - ViewLineStart + 1) - 1, DisplayBuf[i]);
    }
    u8g2_SendBuffer(&u8g2);

    vTaskDelay(5000 / portTICK_PERIOD_MS);
    counter++;
  } // Ende Hauptschleife

  // Hier kommt man eigentlich nicht hin...
  u8g2_SetPowerSave(&u8g2, 1);
  ESP_LOGI(TAG, "Schalte LoRa-Modem ab...");
  LoRa.Close();
}

esp_err_t InitGPIO()
{
  // Outputs
  const uint64_t OutputBitMask = (1ULL << BOARD_LED);

  gpio_config_t ConfigOutput = {};
  ConfigOutput.pin_bit_mask = OutputBitMask;
  ConfigOutput.mode = GPIO_MODE_OUTPUT;
  ConfigOutput.pull_up_en = GPIO_PULLUP_DISABLE;
  ConfigOutput.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ConfigOutput.intr_type = GPIO_INTR_DISABLE;
  ESP_RETURN_ON_ERROR(gpio_config(&ConfigOutput), TAG, "Fehler bei Init output GPIO");
  return ESP_OK;
}

esp_err_t InitLoRa(LoRaBase &aLoRa)
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
    gpio_set_level(BOARD_LED, toggle);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

esp_err_t BuildCBORBuf(uint8_t *aBuf, uint16_t aMaxBufSize, uint16_t &aCBORBuildSize)
{
  json j =
      {
          {POS_TAG, LORA_ORT},
          {DATA_TAG,
           {
               {PC_TAG, 42},
               {TEMP_TAG, {{VAL_TAG, 24.87}}},
               {HUM_TAG, {{VAL_TAG, 55.98}}},
               {PRESS_TAG, {{VAL_TAG, 1024.23}}},
               {ILLU_TAG, {{VAL_TAG, 1540}}},
               {FLASH_TAG, {{VAL_TAG, 3}}},
               {WINDSPEED_TAG, {{VAL_TAG, 12.87}}},
               {WINDDIR_TAG, {{VAL_TAG, 176.9}}},
               {RAIN_TAG, {{VAL_TAG, 0.8776}}},
               {HUM_DET_TAG, {{VAL_TAG, 1}}},
               {SENS_ERR_TAG, {{VAL_TAG, 99}}},
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

void InitDisplay_u8g2()
{
  u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
  u8g2_esp32_hal.clk = DISPLAY_CLK;
  u8g2_esp32_hal.mosi = DISPLAY_DIN;
  u8g2_esp32_hal.cs = DISPLAY_CE;
  u8g2_esp32_hal.reset = DISPLAY_RST;
  u8g2_esp32_hal.dc = DISPLAY_DC;
  u8g2_esp32_hal_init(u8g2_esp32_hal);
  // Nokia 5110 Display

  u8g2_Setup_uc1609_slg19264_f(&u8g2, U8G2_MIRROR_VERTICAL, u8g2_esp32_spi_byte_cb, u8g2_esp32_gpio_and_delay_cb);

  ESP_LOGI(TAG, "Display init...");
  u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

  ESP_LOGI(TAG, "u8g2_SetPowerSave");
  u8g2_SetPowerSave(&u8g2, 0); // wake up display
  u8g2_SetContrast (&u8g2,160);
  ESP_LOGI(TAG, "u8g2_ClearBuffer");
  u8g2_ClearBuffer(&u8g2);
  u8g2_SendBuffer(&u8g2);
  ESP_LOGI(TAG, "Display initialized!");
}