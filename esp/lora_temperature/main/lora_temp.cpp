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
#include "lora_temp.h"
extern "C"
{
#include <u8g2.h>
#include <u8g2_esp32_hal.h>
}

using json = nlohmann::json;

/*
 * Boardauswahl: Ich verwende bisher mit diesem Code 3 verschiedene Boards. Bitte hier definieren
 *
 * LILYGO_T3  // SX1276 mit OLED-Display und ESP32
 * HELTEC_ESP_LORA
 * HELTEC_STICK_V3  // SX1262 mit (ESP32-S3FN8) (erst mal nicht funktionsfähig, da neuer LoRa-Treiber notwendig!)
 */

// Welches Board wird verwendet?
#define LILYGO_T3

// Nur definieren, falls der SHT40 nicht angeschlossen ist, man aber trotzdem ein paar Daten haben möchte
// #define FAKE_SHT40

// ADC-Batteriespannungsmessung benutzen?
#define USE_ADC

// An welchem Ort befindet sich der Sensor? (s. lorastructs.h)
#define LORA_ORT ORT_GEWAECHSHAUS
// #define LORA_ORT ORT_ARBEITSZIMMER

// Tiny-CBOR benutzen, sondern die JSON/CBOR-lib von N. Lohmann

#define TAG "LORA_TEMP"

// SDA - GPIO21 (DISPLAY)
#define PIN_SDA_DISPL GPIO_NUM_21

// SCL - GPIO22 (DISPLAY)
#define PIN_SCL_DISPL GPIO_NUM_22

// Temperatursensor SHT40
#define PIN_SDA_TEMP GPIO_NUM_13
#define PIN_SCL_TEMP GPIO_NUM_14

#define SDCARD_MOSI GPIO_NUM_15
#define SDCARD_MISO GPIO_NUM_2
#define SDCARD_SCLK GPIO_NUM_14
#define SDCARD_CS GPIO_NUM_13

#define BOARD_LED GPIO_NUM_25
#define LED_ON HIGH

static RTC_DATA_ATTR struct timeval sleep_enter_time;
static RTC_DATA_ATTR uint32_t SleepCounter;
u8g2_t u8g2; // a structure which will contain all the data for one display
static bool FirstBoot = true;

extern "C"
{
  void app_main()
  {
    app_main_cpp();
  }
}

void app_main_cpp()
{
#ifdef USE_ADC
  // ADC
  //-------------ADC1 Init---------------//
  adc_oneshot_unit_handle_t adc1_handle;
  adc_oneshot_unit_init_cfg_t init_config1;
  init_config1.unit_id = ADC_UNIT_1;
  init_config1.ulp_mode = ADC_ULP_MODE_DISABLE;

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
#endif
  esp_err_t ret;
  float iTemp_deg, iHum_per, iPress_mBar;
  bool iCRCErr;
  int64_t StartTime = GetTime_us();
  // Strom-Reset oder Wakeup-Timer?
  switch (esp_sleep_get_wakeup_cause())
  {
  case ESP_SLEEP_WAKEUP_TIMER:
  {
    FirstBoot = false;
    // ESP_LOGI(TAG, "Wake up from timer. Time spent in deep sleep: %dms\n", sleep_time_ms);
    ESP_LOGI(TAG, "Wake up from timer.");
    break;
  }
  default:
  {
    FirstBoot = true;
    ESP_LOGI(TAG, "Not a deep sleep reset\n");

    // Init RTC-Variablen beim ersten Boot
    gettimeofday(&sleep_enter_time, NULL);
    SleepCounter = 0;
  }
  }
#ifdef LILYGO_T3
  bool UseDisplay = FirstBoot;
#else
  bool UseDisplay = false;
#endif
  ESP_LOGI(TAG, "Using display %d", UseDisplay);
  struct timeval now;
  gettimeofday(&now, NULL);
  // int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

#ifdef USE_ADC
  adc_oneshot_chan_cfg_t AdcConfig;
#endif

#if defined(LILYGO_T3)
  SX1278_LoRa LoRa(LilygoT3);
#ifdef USE_ADC
  AdcConfig.atten = ADC_ATTEN_DB_11;
#endif
#elif defined(HELTEC_ESP_LORA) // Nur die beiden Heltec-Boards haben die Reset-Leitung an einem RTC-Pin des ESP32!
  SX1278_LoRa LoRa(HeltecESPLoRa);
  rtc_gpio_hold_dis((gpio_num_t)LoRa.PinConfig->Reset);
#ifdef USE_ADC
  AdcConfig.atten = ADC_ATTEN_DB_0;
#endif
#elif defined(HELTEC_STICK_V3)
  SX1278_LoRa LoRa(HeltecWirelessStick_V3);
  rtc_gpio_hold_dis((gpio_num_t)LoRa.PinConfig->Reset);
#ifdef USE_ADC
  AdcConfig.atten = ADC_ATTEN_DB_0;
#endif
#endif

  // gpio_num_t LoraReset = (gpio_num_t)LoRa.PinConfig->Reset;
  gpio_num_t LoraLed = (gpio_num_t)LoRa.PinConfig->Led;

#ifdef USE_ADC
  adc_channel_t AdcChannel = (adc_channel_t)LoRa.PinConfig->AdcChannel;

  //-------------ADC1 Config---------------//
  AdcConfig.bitwidth = ADC_BITWIDTH_DEFAULT;
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, AdcChannel, &AdcConfig));
#endif
  rtc_gpio_hold_dis(LoraLed);
  gpio_reset_pin(LoraLed);
  /* Set the GPIO as a push/pull output */
  gpio_set_direction(LoraLed, GPIO_MODE_OUTPUT);
  gpio_set_level(LoraLed, 1);

  // Init Temperatursensor
  uint32_t iSerial = 0;
#ifndef FAKE_SHT40
  SHT40 iTempSensor(I2C_NUM_0, PIN_SDA_TEMP, PIN_SCL_TEMP);
  ret = iTempSensor.Init();
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Initialisieren des SHT40: %d", ret);
  ret = iTempSensor.ReadSerial(iSerial, iCRCErr);
  ESP_LOGI(TAG, "SHT40 Seriennummer: %lu", iSerial);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Lesen der Seriennummer des SHT40: %d", ret);
  if (iCRCErr)
    ESP_LOGE(TAG, "Fehler beim Lesen des SHT40 CRC (Seriennummer)");
#endif
  if (UseDisplay)
    InitSSD1306_u8g2();
  int64_t EndTime = GetTime_us();
  ESP_LOGI(TAG, "SSD1306 Init fertig nach %.1f ms", (EndTime - StartTime) / 1000.0);
  gpio_set_level(LoraLed, 0);

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
  ret = LoRa.SetupModule(LORA_ADDR_GWHS, 434.54e6, 14, 500E3, 0x37, 8, 6);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Initialisieren des LoRa Moduls: %d", ret);
  EndTime = GetTime_us();
  ESP_LOGI(TAG, "LoRa Init fertig nach %.1f ms", (EndTime - StartTime) / 1000.0);

  // vTaskDelay(2000 / portTICK_PERIOD_MS);
  if (UseDisplay)
  {
    u8g2_ClearDisplay(&u8g2);
    u8g2_SetFont(&u8g2, u8g2_font_crox1h_tf);
  }
  uint8_t LoraBuf[255];
  uint16_t iCBORBuildSize;
  // uint32_t iPC = 0;
  // Beim ersten booten 30s lang das Dispay anzeigen, dann in den Deep-Sleep
  // und nur alle 30s aufwachen...
  int Loops = FirstBoot ? 6 : 1;
  for (int i = 0; i < Loops; i++)
  {
#ifndef FAKE_SHT40
    ret = iTempSensor.Read(iTemp_deg, iHum_per, iCRCErr);
    ESP_LOGI(TAG, "Temp.: %.2f LF: %.2f %%", iTemp_deg, iHum_per);
    if (ret != ESP_OK)
      ESP_LOGE(TAG, "Fehler beim Lesen der Temperatur/Luftfeuchtigkeit des SHT40: %d", ret);
    if (iCRCErr)
      ESP_LOGE(TAG, "Fehler beim Lesen des SHT40 CRC (T/L)");
#else
    iTemp_deg = 20 + sin((SleepCounter * 6) / 360.0 * 2 * 3.141592) * 15;
    iHum_per = 50 + cos((SleepCounter * 3) / 360.0 * 2 * 3.141592) * 30;
#endif
    double AdcMean = 0;
    int AdcValue = 0;
#ifdef USE_ADC
    // Batteriespannung
    for (int i = 0; i < 64; i++)
    {
      ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, AdcChannel, &AdcValue));
      AdcMean += AdcValue;
    }
    AdcMean /= 64.0;
#endif

#ifdef LILYGO_T3
    // Referenzspannung 1.1V, Spannungsteiler 100K/100K, Abschwächung 11dB (Faktor 3.55) = 7.81 V bei Vollausschlag (4095)
    // Der Kalibrierwert -0.448 muss für jedes Board neu nachgemessen werden, da die Referenzspannung des ESP32-ADCs einfach
    // nur grottig ist...
    double iVBatt_V = AdcMean / 4095 * 7.81 - 0.448;
#else
    double iVBatt_V = AdcMean / 4095 * 7.81 - 0.448;
#endif

    ESP_LOGI(TAG, "ADC raw value: %.0f = %.2f V", AdcMean, iVBatt_V);

    // iTemp_deg = 20 + 20 * sin(3.141592 * (SleepCounter / 360.0));
    // iHum_per = 60 + 30 * sin(0.8 + 3.141592 * (SleepCounter / 360.0));
    iPress_mBar = 1000 + 30 * cos(3.141592 * (SleepCounter / 360.0));
    iCBORBuildSize = 0;
    BuildCBORBuf(LoraBuf, sizeof(LoraBuf), iCBORBuildSize, SleepCounter, iTemp_deg, iHum_per, iPress_mBar, iVBatt_V);
    if (iCBORBuildSize > sizeof(LoraBuf))
      ESP_LOGE(TAG, "LoRa Buffer overflow...:%d", iCBORBuildSize);

    // LoRa-Paket senden
    int RetryCounter = 0;
    bool SendOK = false;
    const int MaxRetries = 5;
    char DisplayBuf[256] = {};
    while (!SendOK && RetryCounter < MaxRetries)
    {
      gpio_set_level(LoraLed, 1);
      int64_t ts = GetTime_us();
      ret = LoRa.SendLoraMsg(CMD_CBORDATA, LoraBuf, iCBORBuildSize, SleepCounter);
      SendOK = ret == ESP_OK;
      int64_t te = GetTime_us();
      gpio_set_level(LoraLed, 0);
      ESP_LOGI(TAG, "Zeit fuer LoRa: %.1f ms. Paketgroesse: %u", double(te - ts) / 1000.0, iCBORBuildSize);
      if (!SendOK)
      {
        RetryCounter++;
        ESP_LOGE(TAG, "Fehler beim Senden eines LoRa-Paketes: %d Versuch: %d", ret, RetryCounter);
        ESP_LOGE(TAG, "Resette LORA-Modul...");
        LoRa.Reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
        ret = LoRa.SetupModule(LORA_ADDR_GWHS, 434.54e6, 14, 500E3, 0x3d, 10, 7);
        if (ret != ESP_OK)
          ESP_LOGI(TAG, "Fehler beim Initialisieren des LoRa Moduls: %d", ret);
        else
          ESP_LOGI(TAG, "Re-Init LORA erfolgreich");
      }
    }
    if (!SendOK)
    {
      ESP_LOGE(TAG, "Wiederholtes Senden fehlgeschlagen!");
      continue; // schlafen legen, vielleicht klappt es ja beim nächsten mal...
    }

    EndTime = GetTime_us();
    ESP_LOGI(TAG, "LoRa gesendet nach %.1f ms", (EndTime - StartTime) / 1000.0);

    if (UseDisplay)
    {
      u8g2_ClearBuffer(&u8g2);
      sprintf(DisplayBuf, "Paket Nr.: %lu", SleepCounter);
      u8g2_DrawStr(&u8g2, 2, 14, DisplayBuf);

      sprintf(DisplayBuf, "Temp: %.2f%cC", iTemp_deg, 176);
      u8g2_DrawStr(&u8g2, 2, 28, DisplayBuf);

      // sprintf(DisplayBuf, "Wach: %.1f ms", (EndTime - StartTime) / 1000.0);
      // u8g2_DrawStr(&u8g2, 2, 42, DisplayBuf);

      sprintf(DisplayBuf, "Feuchte: %.2f %%", iHum_per);
      u8g2_DrawStr(&u8g2, 2, 42, DisplayBuf);

      // sprintf(DisplayBuf, "Druck: %.2f mBar", iPress_mBar);
      // u8g2_DrawStr(&u8g2, 2, 56, DisplayBuf);

      // sprintf(DisplayBuf, "Sleep: %.3f s", sleep_time_ms / 1000.0);
      // u8g2_DrawStr(&u8g2, 2, 56, DisplayBuf);

      sprintf(DisplayBuf, "Vbatt: %.2f V", iVBatt_V);
      u8g2_DrawStr(&u8g2, 2, 56, DisplayBuf);

      u8g2_SendBuffer(&u8g2);
    }
    if (FirstBoot)
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    SleepCounter++;
  }

  // vTaskDelay(2000 / portTICK_PERIOD_MS);
  if (UseDisplay)
    u8g2_SetPowerSave(&u8g2, 1);

  ESP_LOGI(TAG, "Schalte LoRa-Modem ab...");
  LoRa.Close();

  // Wenn man nur ca. alle 2 Minuten sendet, dann sollte ein 18650-Akku ein Jahr halten
  const int wakeup_time_sec = 105 + (esp_random() % 30);

  ESP_LOGI(TAG, "Weckzeit einstellen nach %d s\n", wakeup_time_sec);
  esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

  // LoRa-Modul während des Sleeps in den Reset schicken.
  // gpio_set_level(LoraReset, 0);

  // RTC-GPIO-Pins isolieren, damit die während des Deep-Sleeps nicht in der Gegen rumfloaten
  rtc_gpio_isolate(LoraLed);

// Der LoRa-Reset-Pin ist nur bei den Heltec-Modulen an einen RTC-GPIO angeschlossen!
#if defined(HELTEC_ESP_LORA)
  rtc_gpio_isolate((gpio_num_t)LoRa.PinConfig->Reset);
#elif defined(HELTEC_STICK_V3)
  rtc_gpio_isolate((gpio_num_t)LoRa.PinConfig->Reset);
#endif

  // Pins auf Input
  gpio_set_direction(LoraLed, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t)LoRa.PinConfig->ChipSelect, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t)LoRa.PinConfig->Reset, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t)LoRa.PinConfig->DIO0, GPIO_MODE_INPUT);
  gpio_set_direction(PIN_SCL_TEMP, GPIO_MODE_INPUT);
  gpio_set_direction(PIN_SDA_TEMP, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t)LoRa.PinConfig->Clock, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t)LoRa.PinConfig->Miso, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t)LoRa.PinConfig->Mosi, GPIO_MODE_INPUT);

  // Peripeherie-Spezialitäten Lilygo_T3
#ifdef LILYGO_T3
#define DISPL_I2C_SDA GPIO_NUM_21
#define DISPL_I2C_SCL GPIO_NUM_22
#define ADC_PIN GPIO_NUM_36
  gpio_set_direction(DISPL_I2C_SDA, GPIO_MODE_INPUT);
  gpio_set_direction(DISPL_I2C_SCL, GPIO_MODE_INPUT);
  gpio_set_direction(SDCARD_MOSI, GPIO_MODE_INPUT);
  gpio_set_direction(SDCARD_MISO, GPIO_MODE_INPUT);
  gpio_set_direction(SDCARD_SCLK, GPIO_MODE_INPUT);
  gpio_set_direction(SDCARD_CS, GPIO_MODE_INPUT);
  gpio_set_direction(BOARD_LED, GPIO_MODE_INPUT);
  gpio_set_direction(ADC_PIN, GPIO_MODE_INPUT);
#endif

  gettimeofday(&sleep_enter_time, NULL);
  EndTime = GetTime_us();
  ESP_LOGI(TAG, "Gehe in Tiefschlaf nach %.1f ms", (EndTime - StartTime) / 1000.0);
  esp_deep_sleep_start();
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
    gpio_set_level(GPIO_NUM_25, toggle);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

esp_err_t BuildCBORBuf(uint8_t *aBuf, uint16_t aMaxBufSize, uint16_t &aCBORBuildSize, uint32_t aPC, float aTemp_deg, float aHum_per, float aPress_mBar, float iVBatt_V)
{
  json j =
      {
          {POS_TAG, LORA_ORT},
          {DATA_TAG,
           {{PC_TAG, aPC},
            {TEMP_TAG, {{VAL_TAG, aTemp_deg}}},
            {HUM_TAG, {{VAL_TAG, aHum_per}}},
            {PRESS_TAG, {{VAL_TAG, aPress_mBar}}},
            {VOL_TAG, {{VAL_TAG, iVBatt_V}}}}}};

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

void InitSSD1306_u8g2()
{
  u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
  u8g2_esp32_hal.sda = PIN_SDA_DISPL;
  u8g2_esp32_hal.scl = PIN_SCL_DISPL;
  u8g2_esp32_hal_init(u8g2_esp32_hal);

  /*
    gpio_set_level(PIN_DISP_RESET, 0);
    vTaskDelay(pdMS_TO_TICKS(50));
    gpio_set_level(PIN_DISP_RESET, 1);
  */

  u8g2_Setup_ssd1306_i2c_128x64_noname_f(
      &u8g2,
      U8G2_R0,
      u8g2_esp32_i2c_byte_cb,
      u8g2_esp32_gpio_and_delay_cb); // init u8g2 structure
  u8x8_SetI2CAddress(&u8g2.u8x8, 0x78);

  ESP_LOGI(TAG, "u8g2_InitDisplay");
  u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

  ESP_LOGI(TAG, "u8g2_SetPowerSave");
  u8g2_SetPowerSave(&u8g2, 0); // wake up display
  ESP_LOGI(TAG, "u8g2_ClearBuffer");
  u8g2_ClearBuffer(&u8g2);
  u8g2_SendBuffer(&u8g2);
  /*
    ESP_LOGI(TAG, "u8g2_DrawBox");
    u8g2_DrawBox(&u8g2, 0, 26, 80, 6);
    u8g2_DrawFrame(&u8g2, 0, 26, 100, 6);

    ESP_LOGI(TAG, "u8g2_SetFont");
    u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
    ESP_LOGI(TAG, "u8g2_DrawStr");
    u8g2_DrawStr(&u8g2, 2, 17, "Hi Oliver!");
    ESP_LOGI(TAG, "u8g2_SendBuffer");
    u8g2_SendBuffer(&u8g2);
  */
  ESP_LOGI(TAG, "SSD 1306 display initialized!");
}