// Wettermodul auf Lilygo T3_V1.6.1
// SX1276 mit OLED-Display und ESP32

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
#include "driver/rtc_io.h"
#include "soc/rtc.h"
#include "sensors/sht40.h"
#include "sensors/sht35.h"
#include "esp_random.h"
#include "tools/json.hpp"
#include <driver/rtc_io.h>
#include "lorastructs.h"
#include "esp_wifi.h"
#include "sensors/bmp390.h"
#include "tools/tools.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_timer.h"
extern "C"
{
#include <u8g2/csrc/u8g2.h>
#include <u8g2/u8g2_esp32_hal.h>
}
#include "lora_temp.h"

using json = nlohmann::json;

// I2C-Speed
#define I2C_FREQ_HZ 400000

/// BMP390 Luftdrucksensor angeschlossen
#define BMP390_SENSOR_ADDR 0x77 // Adresse BMP390 wenn SDO auf high, auf low = 0x76

// An welchem Ort befindet sich der Sensor? (s. lorastructs.h)
#define LORA_ORT ORT_FAHRRADSCHUPPEN

#define TAG "LORA_TTGO_T3"

// SDA - GPIO21 (DISPLAY und zweiter SHT40)
#define PIN_SDA_BUS1 GPIO_NUM_21

// SCL - GPIO22 (DISPLAY und zweiter SHT40)
#define PIN_SCL_BUS1 GPIO_NUM_22

// Temperatursensor SHT40
#define PIN_SDA_BUS0 GPIO_NUM_13
#define PIN_SCL_BUS0 GPIO_NUM_14

#define SDCARD_MOSI GPIO_NUM_15
#define SDCARD_MISO GPIO_NUM_2
#define SDCARD_SCLK GPIO_NUM_14
#define SDCARD_CS GPIO_NUM_13

#define BOARD_LED GPIO_NUM_25
#define LED_ON HIGH

// Display-Einschalt-Schalter
#define DISPLAY_ON_SWITCH GPIO_NUM_36

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
  float iTemp_deg, iHum_per, iTemp2_deg, iHum2_per;
  float iPress_mBar, t;
  int64_t StartTime = esp_timer_get_time();
  InitGPIO();

  LoRa_PinConfiguration Lilygo = {};
  Lilygo.ChipSelect = GPIO_NUM_18;
  Lilygo.Reset = GPIO_NUM_23;
  Lilygo.Miso = GPIO_NUM_19;
  Lilygo.Mosi = GPIO_NUM_27;
  Lilygo.Clock = GPIO_NUM_5;
  Lilygo.DIO0 = GPIO_NUM_26;
  Lilygo.DIO1 = GPIO_NUM_33;
  Lilygo.Busy = GPIO_NUM_32;
  Lilygo.SPIChannel = SPI2_HOST;

  SX1278_LoRa LoRa(Lilygo);

  // Init I2C
  ESP_LOGI(TAG, "I2C init...");
  i2c_master_bus_handle_t i2c_bus_h_0, i2c_bus_h_1;
  ret = InitI2C(I2C_NUM_0, PIN_SDA_BUS0, PIN_SCL_BUS0, &i2c_bus_h_0);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim InitI2C(0): %d.", ret);

  ret = InitI2C(I2C_NUM_1, PIN_SDA_BUS1, PIN_SCL_BUS1, &i2c_bus_h_1);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim InitI2C(1): %d.", ret);

  uint32_t iSerial = 0;
  SHT40 iTempSensor;
  ret = iTempSensor.Init(i2c_bus_h_0, SHT40_ADDR, I2C_FREQ_HZ);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Initialisieren des SHT40: %d", ret);
  ret = iTempSensor.ReadSerial(iSerial);
  ESP_LOGI(TAG, "SHT40 Seriennummer: %lu", iSerial);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Lesen der Seriennummer des SHT40: %d", ret);

  SHT35 iTempSensor_SHT35;
  ret = iTempSensor_SHT35.Init(i2c_bus_h_1, SHT40_ADDR, I2C_FREQ_HZ);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Initialisieren des SHT35: %d", ret);

  InitSSD1306_u8g2(i2c_bus_h_1);
  int64_t EndTime = esp_timer_get_time();
  ESP_LOGI(TAG, "SSD1306 Init fertig nach %.1f ms", (EndTime - StartTime) / 1000.0);
  gpio_set_level(BOARD_LED, 0);

  // Temperatur und Luftdruck BMP390
  BMP390 Bmp;
  ret = Bmp.Init(i2c_bus_h_0, BMP390_ADDRESS, I2C_FREQ_HZ);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Initialisieren des BMP390: %d", ret);
  else
  {
    ESP_LOGI(TAG, "BMP390 Init OK");
  }

  // Parameter s. InitLoRa
  ret = InitLoRa(LoRa);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Initialisieren des LoRa Moduls: %d", ret);
  EndTime = esp_timer_get_time();
  ESP_LOGI(TAG, "LoRa Init fertig nach %.1f ms", (EndTime - StartTime) / 1000.0);

  // vTaskDelay(2000 / portTICK_PERIOD_MS);
  u8g2_ClearDisplay(&u8g2);
  u8g2_SetFont(&u8g2, u8g2_font_crox1h_tf);
  uint8_t LoraBuf[255];
  uint16_t iCBORBuildSize;
  uint32_t iPC = 0;
  vTaskDelay(100 / portTICK_PERIOD_MS);
  const double UpdateTime_ys = 20E6; // Alle 20 s messen
  // Das Display (OLED) schaltet sich nach 60 s aus.
  const double DisplayOnTime_ys = 60E6; // Das Display ist 60 s an
  int64_t RunTime, DisplayRunTime;
  StartTime = esp_timer_get_time();
  int64_t DisplayStartTime = esp_timer_get_time();
  bool first = true;
  bool DisplayOn = true;
  int DisplayOnSwitch = 1;
  for (;;)
  {
    DisplayOnSwitch = gpio_get_level(DISPLAY_ON_SWITCH);
    RunTime = esp_timer_get_time() - StartTime;
    DisplayRunTime = esp_timer_get_time() - DisplayStartTime;
    if (DisplayRunTime > DisplayOnTime_ys && DisplayOn)
    {
      DisplayOn = false;
      u8g2_SetPowerSave(&u8g2, 1); // Display aus
    }
    if (DisplayOnSwitch==0)
    {
      // Display-Einschalter wurde gedrückt
      DisplayOn = true;
      u8g2_SetPowerSave(&u8g2, 0); // Display an
      DisplayStartTime = esp_timer_get_time();
      first=true; // Sofort messen!
    }
    if (RunTime < UpdateTime_ys && !first)
    {
      vTaskDelay(100 / portTICK_PERIOD_MS);
      continue;
    }
    first = false;
    bool SHT40OK=true, SHT35OK=true, BMP390OK=true;
    StartTime = esp_timer_get_time();
    ret = Bmp.StartReadTempAndPress();
    if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Fehler beim Starten des BMP390: %d", ret);
      BMP390OK=false;
    }

    ret = iTempSensor.Read(iTemp_deg, iHum_per);
    ESP_LOGI("SHT40", "Temp.: %.2f LF: %.2f %%", iTemp_deg, iHum_per);
    if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Fehler beim Lesen der Temperatur/Luftfeuchtigkeit des SHT40: %d", ret);
      SHT40OK=false;
      iTemp_deg=0;
      iHum_per=0;
    }

    ret = iTempSensor_SHT35.Read(iTemp2_deg, iHum2_per);
    ESP_LOGI("SHT35", "Temp.: %.2f LF: %.2f %%", iTemp2_deg, iHum2_per);
    if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Fehler beim Lesen der Temperatur/Luftfeuchtigkeit des SHT35: %d", ret);
      SHT35OK=false;
      iTemp2_deg=0;
      iHum2_per=0;
    }

    ret = Bmp.ReadTempAndPressAsync(t, iPress_mBar);
    // Jerstedt 210 m üNN
    iPress_mBar = Bmp.SeaLevelForAltitude(210, iPress_mBar);
    if (ret != ESP_OK)
    {
      ESP_LOGE(TAG, "Fehler beim Lesen des BMP390: %d", ret);
      BMP390OK=false;
      t=0;
      iPress_mBar=0;
    }
    ESP_LOGI("BMP390", "Druck: %.2f mbar Temp: %.2f°C", iPress_mBar, t);

    iCBORBuildSize = 0;
    BuildCBORBuf(LoraBuf, sizeof(LoraBuf), iCBORBuildSize, iPC, iTemp2_deg, iTemp_deg, iHum2_per, iHum_per, (float)iPress_mBar);
    if (iCBORBuildSize > sizeof(LoraBuf))
      ESP_LOGE(TAG, "LoRa Buffer overflow...:%d", iCBORBuildSize);

    // LoRa-Paket senden
    int RetryCounter = 0;
    bool SendOK = false;
    const int MaxRetries = 5;
    char DisplayBuf[256] = {};
    while (!SendOK && RetryCounter < MaxRetries)
    {
      gpio_set_level(BOARD_LED, 1);
      int64_t ts = esp_timer_get_time();
      ret = LoRa.SendLoraMsg(CMD_CBORDATA, LoraBuf, iCBORBuildSize, iPC);
      SendOK = ret == ESP_OK;
      int64_t te = esp_timer_get_time();
      gpio_set_level(BOARD_LED, 0);
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

    if (DisplayOn)
    {
      u8g2_ClearBuffer(&u8g2);

      sprintf(DisplayBuf, "Paket Nr.: %lu", iPC);
      u8g2_DrawStr(&u8g2, 2, 14, DisplayBuf);

      sprintf(DisplayBuf, "Temp: %.2f%cC %.2f%cC", iTemp_deg, 176, iTemp2_deg, 176);
      u8g2_DrawStr(&u8g2, 2, 28, DisplayBuf);

      sprintf(DisplayBuf, "Feuchte: %.2f %% %.2f %%", iHum_per, iHum2_per);
      u8g2_DrawStr(&u8g2, 2, 42, DisplayBuf);

      sprintf(DisplayBuf, "Druck: %.2f mBar", iPress_mBar);
      u8g2_DrawStr(&u8g2, 2, 56, DisplayBuf);

      u8g2_SendBuffer(&u8g2);
    }
    iPC++;
  }
}

esp_err_t InitI2C(i2c_port_t aPort, gpio_num_t aSDA_Pin, gpio_num_t aSCL_Pin, i2c_master_bus_handle_t *aBusHandle)
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

esp_err_t BuildCBORBuf(uint8_t *aBuf, uint16_t aMaxBufSize, uint16_t &aCBORBuildSize, uint32_t aPC, float aTemp_deg, float aTemp2_deg, float aHum_per, float aHum2_per, float aPress_mBar)
{
  json j =
      {
          {POS_TAG, LORA_ORT},
          {DATA_TAG,
           {{PC_TAG, aPC},
            {TEMP_TAG, {{VAL_TAG, aTemp_deg}}},
            {HUM_TAG, {{VAL_TAG, aHum_per}}},
            {TEMP_TAG2, {{VAL_TAG, aTemp2_deg}}},
            {HUM_TAG2, {{VAL_TAG, aHum2_per}}},
            {PRESS_TAG, {{VAL_TAG, aPress_mBar}}},
            {VOL_TAG, {{VAL_TAG, 0.0}}}}}};

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

void InitSSD1306_u8g2(i2c_master_bus_handle_t aBusHandle)
{
  u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
  u8g2_esp32_hal.sda = PIN_SDA_BUS1;
  u8g2_esp32_hal.scl = PIN_SCL_BUS1;
  u8g2_esp32_hal.I2CBusHandle = aBusHandle;
  u8g2_esp32_hal_init(u8g2_esp32_hal);

  // gpio_set_level(PIN_DISP_RESET, 0);
  // vTaskDelay(pdMS_TO_TICKS(50));
  // gpio_set_level(PIN_DISP_RESET, 1);

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

esp_err_t InitGPIO()
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

  const uint64_t InputBitMask = (1ULL << DISPLAY_ON_SWITCH);
  gpio_config_t ConfigInput = {};
  ConfigInput.pin_bit_mask = InputBitMask;
  ConfigInput.mode = GPIO_MODE_INPUT;
  ConfigInput.pull_up_en = GPIO_PULLUP_ENABLE;
  ConfigInput.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ConfigInput.intr_type = GPIO_INTR_DISABLE;
  ESP_RETURN_ON_ERROR(gpio_config(&ConfigInput), TAG, "Fehler bei Init input GPIO");

  return ESP_OK;
}