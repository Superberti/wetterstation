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
#include "ads1015.h"

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

/*
 * Pindefinitionen:
 * BMP390 und ADS1015, SCL -> 12 (I2C-0)
 * BMP390 und ADS1015, SDA -> 13 (I2C-0)
 * Lautsprecher -> 32
 * Lora SX1278, ChipSelect -> 18;
 * Lora SX1278, Reset -> 14;
 * Lora SX1278, Miso -> 19;
 * Lora SX1278, Mosi -> 27;
 * Lora SX1278, Clock -> 5;
 * Lora SX1278, DIO0 -> 33;
 * Lora SX1278, DIO1 -> 34;
 * Nokia5110, DIN -> 23 (MOSI)
 * Nokia5110, CLK -> 22 (SCK)
 * Nokia5110, CE -> 21 (CE)
 * Nokia5110, DC -> 25 (Umschaltung Config/Daten)
 * Nokia5110, RST -> 26 (Reset)
 * SHT40, SCL -> 11 (I2C-1)
 * SHT40, SDA -> 10 (I2C-1)
 * RAIN_INPUT -> 36
 * LIGHTNING_INPUT -> 39
 * WINDSPEED_INPUT -> 34
 */

/// BMP390 Luftdrucksensor angeschlossen
#define USE_BMP390
#define BMP390_SENSOR_ADDR 0x77 // Adresse BMP390 wenn SDO auf high, auf low = 0x76

// An welchem Ort befindet sich der Sensor? (s. lorastructs.h)
#define LORA_ORT ORT_CARPORT
// #define LORA_ORT ORT_ARBEITSZIMMER

#define TAG "HAUPT-WS"

// Temperatursensor, Luftdruck und ADC am I2C-Bus0
#define PIN_SDA_BUS0 GPIO_NUM_13
#define PIN_SCL_BUS0 GPIO_NUM_14

#define PIN_SDA_BUS1 GPIO_NUM_10
#define PIN_SCL_BUS1 GPIO_NUM_11

// GPIOs (vorläufig)
//#define LORA_SEND_LED GPIO_NUM_15
//#define ERROR_LED GPIO_NUM_16
//#define READ_SENSOR_LED GPIO_NUM_18

// Inputs (Counter)
#define RAIN_INPUT GPIO_NUM_36
#define LIGHTNING_INPUT GPIO_NUM_39
#define WINDSPEED_INPUT GPIO_NUM_34

// Nokia 5110-Display am SPI
#define DISPLAY_DIN GPIO_NUM_23 // MOSI
#define DISPLAY_CLK GPIO_NUM_22 // SCK
#define DISPLAY_CE GPIO_NUM_21  // Chip Enable
#define DISPLAY_DC GPIO_NUM_25  // Umschaltung Config/Daten
#define DISPLAY_RST GPIO_NUM_26 // Reset

// Input-Pins am ADC
#define ADC_INPUT_LUXMETER AIN0
#define ADC_INPUT_WINDDIR AIN1

u8g2_t u8g2; // a structure which will contain all the data for one display

extern "C"
{
  void app_main()
  {
    app_main_cpp();
  }
}

// Sensorik, global
SHT40 Sht(I2C_NUM_0);
BMP390 Bmp(I2C_NUM_0, BMP390_SENSOR_ADDR);
ADS1015 Ads(I2C_NUM_0, ADC1015_ADDR_VDD);

// Counter-Globals für die IRQs
volatile uint32_t gWindspeedCounter = 0;
volatile uint32_t gRainCounter = 0;
volatile uint32_t gFlashCounter = 0;

void app_main_cpp()
{
  SensorData SD = {};
  SensorStatus SST = {};

  esp_err_t ret;
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

  ret = Sht.ReadSerial(iSerial, iCRCErr);
  ESP_LOGI(TAG, "SHT40 Seriennummer: %lu", iSerial);
  if (ret != ESP_OK)
  {
    SST.SkipSHT40 = true;
    ESP_LOGE(TAG, "Fehler beim Lesen der Seriennummer des SHT40: %d. Sensor ausgeschaltet!", ret);
  }
  if (iCRCErr)
  {
    SST.SkipSHT40 = true;
    ESP_LOGE(TAG, "Fehler beim Lesen des SHT40 CRC (Seriennummer). Sensor ausgeschaltet");
  }

  // Temperatur und Luftdruck BMP390
  ret = Bmp.Init();
  if (ret != ESP_OK)
  {
    SST.SkipBMP390 = true;
    ESP_LOGE(TAG, "Fehler beim Initialisieren des BMP390: %d", ret);
  }
  else
  {
    ESP_LOGI(TAG, "BMP390 Init OK");
  }

  // ADC testweise auslesen
  uint16_t ADCVal;
  ret = Ads.ReadADC(AIN0, FSR_4_096, SPEED_128, ADCVal);
  if (ret != ESP_OK)
  {
    SST.SkipADS1015 = true;
    ESP_LOGE(TAG, "Fehler beim Initialisieren des ADS1015: %d", ret);
  }
  else
  {
    ESP_LOGI(TAG, "ADS1015 Init OK. ADCVal = %d", ADCVal);
  }

  // LoRa-Modul
  SX1278_LoRa LoRa(HeltecESPLoRa);
  // Parameter s. InitLoRa
  ret = InitLoRa(LoRa);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Initialisieren des LoRa Moduls: %d", ret);
  else
  {
    ESP_LOGI(TAG, "ALoRa Modul OK!");
  }
  gpio_set_level(ERROR_LED, 0);
  InitNokia_u8g2();
  u8g2_ClearDisplay(&u8g2);
  u8g2_SetFont(&u8g2, u8g2_font_6x10_tf);

  int64_t EndTime = GetTime_us();
  ESP_LOGI(TAG, "Init fertig nach %.1f ms", (EndTime - StartTime) / 1000.0);

  uint8_t LoraBuf[255];
  uint16_t iCBORBuildSize;
  ret = Bmp.StartReadTempAndPress();
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Starten des BMP390: %d", ret);

  SD.PC = 0;
  SD.SensorErrCount = 0;

  int64_t StartHourTime = GetTime_us();
  int64_t StartWindTime = StartTime;
  int64_t CurrentTime = StartTime;
  int64_t TimePast;
  for (;;)
  {
    // Hauptschleife läuft ca. jede Sekunde durch
    GetSensorData(SD, SST);

    // Counter auslesen
    uint32_t FlashCounter, RainCounter, WindspeedCounter;
    gpio_intr_disable(RAIN_INPUT);
    RainCounter = gRainCounter;
    gpio_intr_enable(RAIN_INPUT);

    gpio_intr_disable(WINDSPEED_INPUT);
    WindspeedCounter = gWindspeedCounter;
    gpio_intr_enable(WINDSPEED_INPUT);

    gpio_intr_disable(LIGHTNING_INPUT);
    FlashCounter = gFlashCounter;
    gpio_intr_enable(LIGHTNING_INPUT);

    CurrentTime = GetTime_us();

    // Die Windgeschwindigkeit wird erst ausgewertet, wenn der Windspeedzähler mehr als 10 Impulse gezählt hat,
    // damit die Genauigkeit nicht zu schlecht wird. Hat sich der Counter 10 s lang nicht gerührt, dann gehen wir von Flaute aus.
    if (SD.PC > 0)
    {
      // Eine Tick-Frequenz von einem Hertz enstpricht einer Windgeschwindigkeit vonn (500/499) m/s = 1,002004008 m/s
      TimePast = CurrentTime - StartWindTime;
      float TimePast_s = TimePast / 1.0E6;
      if (WindspeedCounter > 10 || TimePast >= 1E6 * 10)
      {
        StartWindTime = CurrentTime;
        // Windgeschwindigkeit auswerten
        gpio_intr_disable(WINDSPEED_INPUT);
        gWindspeedCounter = 0;
        gpio_intr_enable(WINDSPEED_INPUT);
        SD.WindSpeed_m_s = (WindspeedCounter / TimePast_s) * (500.0 / 499.0);
        ESP_LOGI(TAG, "Windgeschw.: %.1f m/s", SD.WindSpeed_m_s);
      }
    }

    // Jede Stunde die Counter speichern und zurücksetzen
    if (CurrentTime - StartHourTime > 1E6 * 3600)
    {
      ESP_LOGI(TAG, "Zähler werden ausgewertet und zurückgesetzt.");
      StartHourTime = CurrentTime;
      // Zähler zurücksetzen
      gpio_intr_disable(RAIN_INPUT);
      gRainCounter = 0;
      gpio_intr_enable(RAIN_INPUT);

      gpio_intr_disable(LIGHTNING_INPUT);
      gFlashCounter = 0;
      gpio_intr_enable(LIGHTNING_INPUT);

      // DAVIS Regenmesser: Ein Zähler der Regenwippe entsprechen 4,22 cm³, was einer Regenmenge 0,2 mm/m² entspricht
      // Wir geben das als mm / (m² * h) aus
      SD.Rain_mm_qm_h = RainCounter * 0.02;
      ESP_LOGI(TAG, "Regenmenge: %.3f mm/m²h", SD.Rain_mm_qm_h);
      // Blitze pro Stunde
      SD.Flashes_h = FlashCounter;
      ESP_LOGI(TAG, "Blitze: %d Blitze pro Stunde", SD.Flashes_h);
    }

    iCBORBuildSize = 0;
    BuildCBORBuf(LoraBuf, sizeof(LoraBuf), iCBORBuildSize, SD);
    if (iCBORBuildSize > sizeof(LoraBuf))
      ESP_LOGE(TAG, "LoRa Buffer overflow...:%d", iCBORBuildSize);

    // LoRa-Paket senden
    int RetryCounter = 0;
    bool SendOK = false;
    const int MaxRetries = 5;

    while (!SendOK && RetryCounter < MaxRetries)
    {
      gpio_set_level(LORA_SEND_LED, 1);

      ret = LoRa.SendLoraMsg(CMD_CBORDATA, LoraBuf, iCBORBuildSize, SD.PC);
      SendOK = ret == ESP_OK;
      int64_t te = GetTime_us();
      gpio_set_level(LORA_SEND_LED, 0);
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
#define DISPLAY_CHARS_PER_LINE 14
#define LINE_HEIGHT 11
#define DISPLAY_LINES 4
#define DISPLAY_BUF_LINES 15

    // Anzuzeigender Bereich des gesamten Display-Buffers
    int ViewLineStart = 0;
    int ViewLineEnd = 3;

    char DisplayBuf[DISPLAY_BUF_LINES][DISPLAY_CHARS_PER_LINE + 1] = {};
    snprintf(DisplayBuf[0], DISPLAY_CHARS_PER_LINE, "T: %.2f%cC", SD.Temp_deg, 176);
    snprintf(DisplayBuf[1], DISPLAY_CHARS_PER_LINE, "H: %.2f %%", SD.Hum_per);
    snprintf(DisplayBuf[2], DISPLAY_CHARS_PER_LINE, "P: %.2f mBar", SD.Press_mBar);
    snprintf(DisplayBuf[3], DISPLAY_CHARS_PER_LINE, "W: %.2f m/s", SD.WindSpeed_m_s);
    snprintf(DisplayBuf[4], DISPLAY_CHARS_PER_LINE, "R: %.2f mm", SD.Rain_mm_qm_h);
    snprintf(DisplayBuf[5], DISPLAY_CHARS_PER_LINE, "B: %.2f lux", SD.Brightness_lux);
    snprintf(DisplayBuf[6], DISPLAY_CHARS_PER_LINE, "SErr: %lu", SST.SHT40Err);
    snprintf(DisplayBuf[7], DISPLAY_CHARS_PER_LINE, "BErr: %lu", SST.BMP390Err);
    snprintf(DisplayBuf[8], DISPLAY_CHARS_PER_LINE, "AErr: %lu", SST.ADS1015Err);
    snprintf(DisplayBuf[9], DISPLAY_CHARS_PER_LINE, "DErr: %lu", SST.NokiaDisplayErr);

    u8g2_ClearBuffer(&u8g2);

    for (int i = ViewLineStart; i <= ViewLineEnd; i++)
    {
      u8g2_DrawStr(&u8g2, 0, LINE_HEIGHT*(i-ViewLineStart), DisplayBuf[i]);
    }
    u8g2_SendBuffer(&u8g2);

    vTaskDelay(1000 / portTICK_PERIOD_MS);
    SD.PC++;
  }

  // Hier kommt man eigentlich nicht hin...
  u8g2_SetPowerSave(&u8g2, 1);
  ESP_LOGI(TAG, "Schalte LoRa-Modem ab...");
  LoRa.Close();
}

void GetSensorData(SensorData &aData, SensorStatus &SST)
{
  const int MaxRetries = 5;
  esp_err_t ret = ESP_OK;
  double p, t;
  bool iCRCErr;
  uint16_t ADCVal;
  int RetryCounter = 0;

  // BMP390
  if (!SST.SkipBMP390)
  {
    do
    {
      ret = Bmp.ReadTempAndPressAsync(t, p);
      if (ret != ESP_OK)
      {
        SST.BMP390Err++;
        ESP_LOGE(TAG, "Fehler beim Lesen des BMP390: %d, Versuch %d", ret, RetryCounter + 1);
        Bmp.StartReadTempAndPress();
      }
      aData.Temp_deg = t;
      aData.Press_mBar = Bmp.SeaLevelForAltitude(210, p);
      RetryCounter++;
      if (ret != ESP_OK)
        vTaskDelay(100 / portTICK_PERIOD_MS);
    } while (ret != ESP_OK && RetryCounter < MaxRetries);
  }
  if (ret != ESP_OK || SST.SkipBMP390)
  {
    if (SST.SkipBMP390)
      ESP_LOGI(TAG, "BMP390 wird übersprungen...");
    else
    {
      ESP_LOGE(TAG, "BMP390: Wiederholtes Lesen des Sensors fehlgeschlagen! Sensor wird ab jetzt übersprungen!");
      SST.SkipBMP390 = true;
    }
    aData.Temp_deg = -100;
    aData.Press_mBar = -1;
  }
  else
  {
    ESP_LOGI("BMP390", "Druck: %.2f mbar Temp: %.2f°C", aData.Press_mBar, aData.Temp_deg);
  }

  // SHT40 (wir verwenden die Temperatur vom SHT, die ist genauer!)
  RetryCounter = 0;
  if (!SST.SkipSHT40)
  {
    do
    {
      ret = Sht.Read(aData.Temp_deg, aData.Hum_per, iCRCErr);
      if (ret != ESP_OK)
      {
        SST.SHT40Err++;
        ESP_LOGE(TAG, "Fehler beim Lesen der Temperatur/Luftfeuchtigkeit des SHT40: %d, Versuch %d", ret, RetryCounter + 1);
      }
      else if (iCRCErr)
      {
        SST.SHT40Err++;
        ESP_LOGE(TAG, "Fehler beim Lesen des SHT40 CRC (T/L), Versuch %d", RetryCounter + 1);
        ret = ESP_ERR_INVALID_CRC;
      }
      RetryCounter++;
      if (ret != ESP_OK)
        vTaskDelay(100 / portTICK_PERIOD_MS);
    } while (ret != ESP_OK && RetryCounter < MaxRetries);
  }
  if (ret != ESP_OK || SST.SkipSHT40)
  {
    if (SST.SkipSHT40)
      ESP_LOGI(TAG, "SHT40 wird übersprungen...");
    else
    {
      ESP_LOGE(TAG, "SHT40: Wiederholtes Lesen des Sensors fehlgeschlagen! Sensor wird ab jetzt übersprungen!");
      SST.SkipSHT40 = true;
    }
    aData.Temp_deg = -100;
    aData.Hum_per = -1;
  }
  else
  {
    ESP_LOGI("SHT40", "Temp.: %.2f LF: %.2f %%", aData.Temp_deg, aData.Hum_per);
  }

  ADC_MP InputPin;

  if (!SST.SkipADS1015)
  {
    for (int i = 4; i < 6; i++) // 2 Eingänge des ADCs lesen
    {
      InputPin = (ADC_MP)i;
      RetryCounter = 0;
      do
      {
        ret = Ads.ReadADC(InputPin, FSR_4_096, SPEED_128, ADCVal);
        if (ret != ESP_OK)
        {
          SST.ADS1015Err++;
          ESP_LOGE(TAG, "Fehler beim Lesen des ADS1015: %d, Versuch %d", ret, RetryCounter + 1);
        }
        RetryCounter++;
        if (ret != ESP_OK)
          vTaskDelay(100 / portTICK_PERIOD_MS);
      } while (ret != ESP_OK && RetryCounter < MaxRetries);
      if (ret != ESP_OK)
      {
        ESP_LOGE(TAG, "ADS1015: Wiederholtes Lesen des Sensors fehlgeschlagen! Sensor wird ab jetzt übersprungen!");
        SST.SkipADS1015 = true;
        if (InputPin == ADC_INPUT_LUXMETER)
          aData.Brightness_lux = -1;
        else if (InputPin == ADC_INPUT_WINDDIR)
          aData.WindDir_deg = -1;
      }
      else
      {
        ESP_LOGI("ADS1015", "Pin %d, ADC-Wert: %d", i, ADCVal);
        if (InputPin == ADC_INPUT_LUXMETER)
        {
          aData.Brightness_lux = ADCVal / 2500.0; // Milliwatt pro Quadratmeter?
        }
        else if (InputPin == ADC_INPUT_WINDDIR)
        {
          // Sensor geht von 0-5 V, 0=0°, 5 V=360°
          // Verwendet wird ein Spannungsteiler 1:1, d.h. es kommen max. 2,5V am Eingang an
          // FullScale ist bei 4,096 V erreicht = 4096 ADC-Einheiten (12 Bit-ADC), also 1 mV == 1 ADC
          aData.WindDir_deg = std::min(359.999, 360.0 * ADCVal / 2500.0);
        }
      }
    }
  }
  else
  {
    aData.Brightness_lux = -1;
    aData.WindDir_deg = -1;
    ESP_LOGI(TAG, "ADS1015 wird übersprungen...");
  }
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
  uint64_t InputBitMask = (1ULL << RAIN_INPUT) | (1ULL << LIGHTNING_INPUT) | (1ULL << WINDSPEED_INPUT);
  gpio_config_t ConfigInput = {};
  ConfigInput.pin_bit_mask = InputBitMask;
  ConfigInput.mode = GPIO_MODE_INPUT;
  ConfigInput.pull_up_en = GPIO_PULLUP_ENABLE;
  ConfigInput.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ConfigInput.intr_type = GPIO_INTR_POSEDGE;
  gpio_config(&ConfigInput);

  // install gpio isr service
  gpio_install_isr_service(0);
  // hook isr handler for specific gpio pin
  gpio_isr_handler_add(RAIN_INPUT, gpio_isr_handler, (void *)RAIN_INPUT);
  gpio_isr_handler_add(LIGHTNING_INPUT, gpio_isr_handler, (void *)LIGHTNING_INPUT);
  gpio_isr_handler_add(WINDSPEED_INPUT, gpio_isr_handler, (void *)WINDSPEED_INPUT);
}

static void IRAM_ATTR gpio_isr_handler(void *arg)
{
  // Input-IRQ Windgeschw., Regen und Blitze
  gpio_num_t InputPin = (gpio_num_t)((uint32_t)arg);
  // xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
  switch (InputPin)
  {
  case RAIN_INPUT:
    gRainCounter++;
    break;
  case LIGHTNING_INPUT:
    gFlashCounter++;
    break;
  case WINDSPEED_INPUT:
    gWindspeedCounter++;
    break;
  default:
    break;
  }
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

esp_err_t BuildCBORBuf(uint8_t *aBuf, uint16_t aMaxBufSize, uint16_t &aCBORBuildSize, const SensorData &aData)
{
  json j =
      {
          {POS_TAG, LORA_ORT},
          {DATA_TAG,
           {
               {PC_TAG, aData.PC},
               {TEMP_TAG, {{VAL_TAG, aData.Temp_deg}}},
               {HUM_TAG, {{VAL_TAG, aData.Hum_per}}},
               {PRESS_TAG, {{VAL_TAG, aData.Press_mBar}}},
               {ILLU_TAG, {{VAL_TAG, aData.Brightness_lux}}},
               {FLASH_TAG, {{VAL_TAG, aData.Flashes_h}}},
               {WINDSPEED_TAG, {{VAL_TAG, aData.WindSpeed_m_s}}},
               {WINDDIR_TAG, {{VAL_TAG, aData.WindDir_deg}}},
               {RAIN_TAG, {{VAL_TAG, aData.Rain_mm_qm_h}}},
               {HUM_DET_TAG, {{VAL_TAG, aData.HumDetected}}},
               {SENS_ERR_TAG, {{VAL_TAG, aData.SensorErrCount}}},
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