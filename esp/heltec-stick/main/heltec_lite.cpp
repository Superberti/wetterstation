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
#include "driver/i2c.h"
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
#include "heltec_lite.h"
#include "bmp390.h"

using json = nlohmann::json;

/*
 * Boardauswahl: Ich verwende bisher mit diesem Code 3 verschiedene Boards. Bitte hier definieren
 *
 * LilygoT3  // SX1276 mit OLED-Display und ESP32
 * HeltecESPLoRa
 * HeltecWirelessStick_V3  // SX1262 mit (ESP32-S3FN8)
 */

// Welches Board wird verwendet?
static const LoRaBoardTypes BoardType = HeltecWirelessStick_V3;

// Nur definieren, falls der SHT40 nicht angeschlossen ist, man aber trotzdem ein paar Daten haben möchte
#define FAKE_SHT40

/// BMP390 Luftdrucksensor angeschlossen
// #define USE_BMP390
#define BMP390_SENSOR_ADDR 0x77 // Adresse BMP390 wenn SDO auf high, auf low = 0x76

// ADC-Batteriespannungsmessung benutzen?
#define USE_ADC

// An welchem Ort befindet sich der Sensor? (s. lorastructs.h)
#define LORA_ORT ORT_SYMPATEC
// #define LORA_ORT ORT_ARBEITSZIMMER

#define TAG "HELTEC_TEMP"

// Temperatursensor SHT40
#define PIN_SDA_TEMP GPIO_NUM_13
#define PIN_SCL_TEMP GPIO_NUM_14

#define SDCARD_MOSI GPIO_NUM_15
#define SDCARD_MISO GPIO_NUM_2
#define SDCARD_SCLK GPIO_NUM_14
#define SDCARD_CS GPIO_NUM_13

#define LED_ON HIGH

static RTC_DATA_ATTR struct timeval sleep_enter_time;
static RTC_DATA_ATTR uint32_t SleepCounter;

static bool FirstBoot = true;
static int BoardLED = -1;
static int AdcChannel = -1;

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
  if (BoardType == HeltecWirelessStick_V3)
    init_config1.clk_src = ADC_RTC_CLK_SRC_RC_FAST;

  ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
#endif
  esp_err_t ret;
  float iTemp_deg, iHum_per;
  double iPress_mBar, t;
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

  bool UseDisplay = false;

  struct timeval now;
  gettimeofday(&now, NULL);
  // int sleep_time_ms = (now.tv_sec - sleep_enter_time.tv_sec) * 1000 + (now.tv_usec - sleep_enter_time.tv_usec) / 1000;

#ifdef USE_ADC
  adc_oneshot_chan_cfg_t AdcConfig;
#endif

  LoRaBase *LoRa;
  if (BoardType == LilygoT3)
  {
    UseDisplay = FirstBoot;
    BoardLED = 25;
    AdcChannel = 7;
    LoRa = new SX1278_LoRa(BoardType);

#ifdef USE_ADC
    AdcConfig.atten = ADC_ATTEN_DB_11;
#endif
  }
  else if (BoardType == HeltecESPLoRa)
  {
    // Nur die beiden Heltec-Boards haben die Reset-Leitung an einem RTC-Pin des ESP32!
    LoRa = new SX1278_LoRa(BoardType);
    rtc_gpio_hold_dis((gpio_num_t)LoRa->PinConfig->Reset);
    BoardLED = 25;
#ifdef USE_ADC
    AdcConfig.atten = ADC_ATTEN_DB_0;
#endif
  }
  else if (BoardType == HeltecWirelessStick_V3)
  {
    LoRa = new SX1262_LoRa(HeltecWirelessStick_V3);
    ESP_LOGI(TAG, "Lora reset pin: %d", LoRa->PinConfig->Reset);
    rtc_gpio_hold_dis((gpio_num_t)LoRa->PinConfig->Reset);
    BoardLED = 35;
    AdcChannel = 0;
#ifdef USE_ADC
    AdcConfig.atten = ADC_ATTEN_DB_0;
#endif
  }
  ESP_LOGI(TAG, "Using display %d", UseDisplay);

#ifdef USE_ADC
  //-------------ADC1 Config---------------//
  AdcConfig.bitwidth = ADC_BITWIDTH_DEFAULT;
  ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, (adc_channel_t)AdcChannel, &AdcConfig));
#endif

  if (BoardType != HeltecWirelessStick_V3)
    rtc_gpio_hold_dis((gpio_num_t)BoardLED);

  gpio_reset_pin((gpio_num_t)BoardLED);
  /* Set the GPIO as a push/pull output */
  gpio_set_direction((gpio_num_t)BoardLED, GPIO_MODE_OUTPUT);
  gpio_set_level((gpio_num_t)BoardLED, 1);

  // Init Temperatursensor
  uint32_t iSerial = 0;
  bool i2c_init_done = false;
#ifndef FAKE_SHT40
  SHT40 iTempSensor(I2C_NUM_0, PIN_SDA_TEMP, PIN_SCL_TEMP);
  ret = iTempSensor.Init(true);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Initialisieren des SHT40: %d", ret);
  i2c_init_done = true;
  ret = iTempSensor.ReadSerial(iSerial, iCRCErr);
  ESP_LOGI(TAG, "SHT40 Seriennummer: %lu", iSerial);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Lesen der Seriennummer des SHT40: %d", ret);
  if (iCRCErr)
    ESP_LOGE(TAG, "Fehler beim Lesen des SHT40 CRC (Seriennummer)");
#endif
  int64_t EndTime = GetTime_us();
  // ESP_LOGI(TAG, "SSD1306 Init fertig nach %.1f ms", (EndTime - StartTime) / 1000.0);
  gpio_set_level((gpio_num_t)BoardLED, 0);

// Temperatur und Luftdruck BMP390
#ifdef USE_BMP390
  BMP390 Bmp(I2C_NUM_0, BMP390_SENSOR_ADDR, PIN_SDA_TEMP, PIN_SCL_TEMP);
  ret = Bmp.Init(!i2c_init_done); // I2C wurde schon vom SHT40 initialisiert
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
#endif

  // BlinkLED(2000, 10);
  //  Parameter s. InitLoRa
  ret = InitLoRa(*LoRa);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Initialisieren des LoRa Moduls: %d", ret);
  EndTime = GetTime_us();
  ESP_LOGI(TAG, "LoRa Init fertig nach %.1f ms", (EndTime - StartTime) / 1000.0);

  uint8_t LoraBuf[255];
  uint16_t iCBORBuildSize;
  // uint32_t iPC = 0;
  // Beim ersten booten 30s lang das Dispay anzeigen, dann in den Deep-Sleep
  // und nur alle 30s aufwachen...
  int Loops = FirstBoot ? 6 : 1;
  for (int i = 0; i < Loops; i++)
  {
#ifdef USE_BMP390
    ret = Bmp.StartReadTempAndPress();
    if (ret != ESP_OK)
      ESP_LOGE(TAG, "Fehler beim Starten des BMP390: %d", ret);
#endif
#ifndef FAKE_SHT40
    ret = iTempSensor.Read(iTemp_deg, iHum_per, iCRCErr);
    ESP_LOGI("SHT40", "Temp.: %.2f LF: %.2f %%", iTemp_deg, iHum_per);
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
      ESP_ERROR_CHECK(adc_oneshot_read(adc1_handle, (adc_channel_t)AdcChannel, &AdcValue));
      AdcMean += AdcValue;
    }
    AdcMean /= 64.0;
#endif

#ifdef USE_BMP390
    ret = Bmp.ReadTempAndPressAsync(t, iPress_mBar);
    iPress_mBar = Bmp.SeaLevelForAltitude(602, iPress_mBar);
    if (ret != ESP_OK)
      ESP_LOGE(TAG, "Fehler beim Lesen des BMP390: %d", ret);
    ESP_LOGI("BMP390", "Druck: %.2f mbar Temp: %.2f°C", iPress_mBar, t);
#endif

    double iVBatt_V = 0;
    if (BoardType == LilygoT3)
    {
      // Referenzspannung 1.1V, Spannungsteiler 100K/100K, Abschwächung 11dB (Faktor 3.55) = 7.81 V bei Vollausschlag (4095)
      // Der Kalibrierwert -0.448 muss für jedes Board neu nachgemessen werden, da die Referenzspannung des ESP32-ADCs einfach
      // nur grottig ist...
      iVBatt_V = AdcMean / 4095 * 7.81 - 0.448;
    }
    else if (BoardType == HeltecWirelessStick_V3)
    {
      iVBatt_V = AdcMean / 4095 * 7.81 * 100.0 / (100 + 390);
      /*
        #define VBAT_PIN 1

  void VBAT_Init() {
  pinMode(VBAT_PIN, INPUT);
  adcAttachPin(VBAT_PIN);
  analogReadResolution(10);

  pinMode(37, OUTPUT); // ADC_Ctrl
  }

  #define BATTERY_SAMPLES 20

  float readBattVoltage() {
  digitalWrite(37, LOW); // ADC_Ctrl

  uint32_t raw = 0;
  for (int i = 0; i < BATTERY_SAMPLES; i++) {
  raw += analogRead(VBAT_PIN);
  }
  raw = raw / BATTERY_SAMPLES;

  digitalWrite(37, HIGH); // ADC_Ctrl

  return 5.42 * (3.3 / 1024.0) * raw;
  }
      */
    }
    else
    {
      iVBatt_V = AdcMean / 4095 * 7.81 - 0.448;
    }
    ESP_LOGI(TAG, "ADC raw value: %.0f = %.2f V", AdcMean, iVBatt_V);

#ifndef USE_BMP390
    iPress_mBar = 1000 + 30 * cos(3.141592 * (SleepCounter / 360.0));
#endif
    iCBORBuildSize = 0;
    BuildCBORBuf(LoraBuf, sizeof(LoraBuf), iCBORBuildSize, SleepCounter, iTemp_deg, iHum_per, (float)iPress_mBar, iVBatt_V);
    if (iCBORBuildSize > sizeof(LoraBuf))
      ESP_LOGE(TAG, "LoRa Buffer overflow...:%d", iCBORBuildSize);

    // LoRa-Paket senden
    int RetryCounter = 0;
    bool SendOK = false;
    const int MaxRetries = 5;
    while (!SendOK && RetryCounter < MaxRetries)
    {
      gpio_set_level((gpio_num_t)BoardLED, 1);
      int64_t ts = GetTime_us();
      ret = LoRa->SendLoraMsg(CMD_CBORDATA, LoraBuf, iCBORBuildSize, SleepCounter);
      SendOK = ret == ESP_OK;
      int64_t te = GetTime_us();
      gpio_set_level((gpio_num_t)BoardLED, 0);
      ESP_LOGI(TAG, "Zeit fuer LoRa: %.1f ms. Paketgroesse: %u", double(te - ts) / 1000.0, iCBORBuildSize);
      if (!SendOK)
      {
        RetryCounter++;
        ESP_LOGE(TAG, "Fehler beim Senden eines LoRa-Paketes: %d Versuch: %d", ret, RetryCounter);
        ESP_LOGE(TAG, "Resette LORA-Modul...");
        LoRa->Reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
        ret = InitLoRa(*LoRa);
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

    if (FirstBoot)
      vTaskDelay(5000 / portTICK_PERIOD_MS);
    SleepCounter++;
  }

  ESP_LOGI(TAG, "Schalte LoRa-Modem ab...");
  LoRa->Close();

  // Wenn man nur ca. alle 2 Minuten sendet, dann sollte ein 18650-Akku ein Jahr halten
  const int wakeup_time_sec = 105 + (esp_random() % 30);

  ESP_LOGI(TAG, "Weckzeit einstellen nach %d s\n", wakeup_time_sec);
  esp_sleep_enable_timer_wakeup(wakeup_time_sec * 1000000);

  // LoRa-Modul während des Sleeps in den Reset schicken.
  // gpio_set_level(LoraReset, 0);

  // RTC-GPIO-Pins isolieren, damit die während des Deep-Sleeps nicht in der Gegen rumfloaten
  if (BoardType != HeltecWirelessStick_V3)
    rtc_gpio_isolate((gpio_num_t)BoardLED); // HELTEC_STICK_V3 hat die LED auf GPIO35, das ist kein RTC-Pin!

  // Der LoRa-Reset-Pin ist nur bei den Heltec-Modulen an einen RTC-GPIO angeschlossen!

  if (BoardType == HeltecESPLoRa)
    rtc_gpio_isolate((gpio_num_t)LoRa->PinConfig->Reset);
  else if (BoardType == HeltecWirelessStick_V3)
    rtc_gpio_isolate((gpio_num_t)LoRa->PinConfig->Reset);

  // Pins auf Input
  gpio_set_direction((gpio_num_t)BoardLED, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t)LoRa->PinConfig->ChipSelect, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t)LoRa->PinConfig->Reset, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t)LoRa->PinConfig->DIO0, GPIO_MODE_INPUT);
  gpio_set_direction(PIN_SCL_TEMP, GPIO_MODE_INPUT);
  gpio_set_direction(PIN_SDA_TEMP, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t)LoRa->PinConfig->Clock, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t)LoRa->PinConfig->Miso, GPIO_MODE_INPUT);
  gpio_set_direction((gpio_num_t)LoRa->PinConfig->Mosi, GPIO_MODE_INPUT);

  // Peripeherie-Spezialitäten Lilygo_T3
  if (BoardType == LilygoT3)
  {
    // #define DISPL_I2C_SDA GPIO_NUM_21
    // #define DISPL_I2C_SCL GPIO_NUM_22
    // #define ADC_PIN GPIO_NUM_36
    gpio_set_direction((gpio_num_t)21, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)22, GPIO_MODE_INPUT);
    gpio_set_direction(SDCARD_MOSI, GPIO_MODE_INPUT);
    gpio_set_direction(SDCARD_MISO, GPIO_MODE_INPUT);
    gpio_set_direction(SDCARD_SCLK, GPIO_MODE_INPUT);
    gpio_set_direction(SDCARD_CS, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)BoardLED, GPIO_MODE_INPUT);
    gpio_set_direction((gpio_num_t)36, GPIO_MODE_INPUT);
  }
  else if (BoardType == HeltecWirelessStick_V3)
  {
    gpio_set_direction((gpio_num_t)1, GPIO_MODE_INPUT);
  }
  gettimeofday(&sleep_enter_time, NULL);
  EndTime = GetTime_us();
  ESP_LOGI(TAG, "Gehe in Tiefschlaf nach %.1f ms", (EndTime - StartTime) / 1000.0);
  esp_deep_sleep_start();
}

esp_err_t InitLoRa(LoRaBase &aLoRa)
{
  // Sendefrequenz: 434,54 MHz
  // Preambellänge: 14
  // Bandbreite 500 kHz
  // Achtung: Damit auch der neuere SX1262 verwendet werden kann, muss man sich über die Sync-Words ein paar mehr
  // Gedanken machen. Der SX1262 hat 2 Bytes Sync, nicht nur 1 Byte. Damit sich trotzdem beide verstehen, muss man
  // folgendes beachten:
  // SX1276: Jedes Nibble unterschiedlich von 1-7, also z.B. 0x37 => 0xYZ (Y!=Z, Z und Y <8 && >0)
  // SX1262: 0Y4Z4, also hier 0x3474
  // Sync-Byte: 0x37 (SX1276) oder 0x3474 (SX1262)
  // Spreading-Factor: 8 = 256 Chips/symbol
  // Coding-Rate: 6 = 4/6 = 1,5-facher FEC-Overhead
  // Tx-Power 2-17
  return aLoRa.SetupModule(LORA_ADDR_SYMPATEC_2, 434.54e6, 14, LoRaBase::LoRaBandwidth::LORA_BW_500, 0x3474, LoRaBase::SpreadingFactor::SF8, LoRaBase::LoRaCodingRate::LORA_CR_4_6, 15);
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
    gpio_set_level((gpio_num_t)BoardLED, toggle);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

void BlinkLED(uint32_t aBlinkTime_ms, uint32_t aBlinkFrq_Hz)
{
  if (aBlinkFrq_Hz == 0)
    return;
  int64_t StartTime = GetTime_us();
  int wait_ms = 500 / aBlinkFrq_Hz;
  int toggle = 0;
  while (GetTime_us() - StartTime < aBlinkTime_ms * 1000)
  {
    toggle = 1 - toggle;
    gpio_set_level((gpio_num_t)BoardLED, toggle);
    vTaskDelay(pdMS_TO_TICKS(wait_ms));
  }
  gpio_set_level((gpio_num_t)BoardLED, 0);
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
