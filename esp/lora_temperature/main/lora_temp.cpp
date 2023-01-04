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
extern "C"
{
#include <u8g2.h>
#include <u8g2_esp32_hal.h>
}
#include "cbor.h"
#include <esp_sleep.h>
#include <driver/rtc_io.h>

#include "lorastructs.h"
#include "esp_wifi.h"
#include "lora_temp.h"

#define LED_PIN GPIO_NUM_25
#define TAG "LORA_TEMP"

// SDA - GPIO21
#define PIN_SDA GPIO_NUM_21

// SCL - GPIO22
#define PIN_SCL GPIO_NUM_22

// aus Lilygo-Beispielen
#define I2C_SDA GPIO_NUM_21
#define I2C_SCL GPIO_NUM_22
/*
#define RADIO_SCLK_PIN GPIO_NUM_5
#define RADIO_MISO_PIN GPIO_NUM_19
#define RADIO_MOSI_PIN GPIO_NUM_27
#define RADIO_CS_PIN GPIO_NUM_18
#define RADIO_DIO0_PIN GPIO_NUM_26
#define RADIO_RST_PIN GPIO_NUM_23
#define RADIO_DIO1_PIN GPIO_NUM_33
#define RADIO_BUSY_PIN GPIO_NUM_32
*/
#define SDCARD_MOSI GPIO_NUM_15
#define SDCARD_MISO GPIO_NUM_2
#define SDCARD_SCLK GPIO_NUM_14
#define SDCARD_CS GPIO_NUM_13

#define BOARD_LED GPIO_NUM_25
#define LED_ON HIGH

#define ADC_PIN GPIO_NUM_35

u8g2_t u8g2; // a structure which will contain all the data for one display

void InitSSD1306_u8g2()
{
  u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
  u8g2_esp32_hal.sda = PIN_SDA;
  u8g2_esp32_hal.scl = PIN_SCL;
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

  ESP_LOGI(TAG, "u8g2_DrawBox");
  u8g2_DrawBox(&u8g2, 0, 26, 80, 6);
  u8g2_DrawFrame(&u8g2, 0, 26, 100, 6);

  ESP_LOGI(TAG, "u8g2_SetFont");
  u8g2_SetFont(&u8g2, u8g2_font_ncenB10_tr);
  ESP_LOGI(TAG, "u8g2_DrawStr");
  u8g2_DrawStr(&u8g2, 2, 17, "Hi Oliver!");
  ESP_LOGI(TAG, "u8g2_SendBuffer");
  u8g2_SendBuffer(&u8g2);

  ESP_LOGI(TAG, "SSD 1306 display initialized!");
}

extern "C"
{
  void app_main()
  {
    app_main_cpp();
  }
}

void app_main_cpp()
{
  gpio_reset_pin(LED_PIN);
  /* Set the GPIO as a push/pull output */
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_PIN, 1);
  InitSSD1306_u8g2();
  gpio_set_level(LED_PIN, 0);
  SX1278_LoRa LoRa;
  esp_err_t ret = LoRa.SetupModule(LORA_ADDR_GWHS, 434.54e6, 14, 500E3, 0x3d);
  if (ret != ESP_OK)
    ESP_LOGE(TAG, "Fehler beim Initialisieren des LoRa Moduls: %d", ret);

  vTaskDelay(2000 / portTICK_PERIOD_MS);
  u8g2_ClearDisplay(&u8g2);
  u8g2_SetFont(&u8g2, u8g2_font_crox1h_tf);
  uint8_t LoraBuf[255];
  uint16_t iCBORBuildSize;
  uint32_t iPC = 0;
  float iTemp_deg, iHum_per, iPress_mBar;
  for (;;)
  {
    iTemp_deg = 20 + 20 * sin(3.141592 * (iPC / 360.0));
    iHum_per = 60 + 30 * sin(0.8 + 3.141592 * (iPC / 360.0));
    iPress_mBar = 1000 + 30 * cos(3.141592 * (iPC / 360.0));
    iCBORBuildSize=0;
    BuildCBORBuf(LoraBuf, sizeof(LoraBuf), iCBORBuildSize, iPC, iTemp_deg, iHum_per, iPress_mBar);
    if (iCBORBuildSize > sizeof(LoraBuf))
      ESP_LOGE(TAG, "LoRa Buffer overflow...:%d",iCBORBuildSize);

    // LoRa-Paket senden
    int RetryCounter = 0;
    bool SendOK = false;
    const int MaxRetries = 5;
    char DisplayBuf[256]={};
    while (!SendOK && RetryCounter < MaxRetries)
    {
      RetryCounter=0;
      gpio_set_level(LED_PIN, 1);
      int64_t ts = GetTime_us();
      ret = LoRa.SendLoraMsg(CMD_CBORDATA, LoraBuf, iCBORBuildSize, iPC);
      SendOK=ret==ESP_OK;
      int64_t te = GetTime_us();
      gpio_set_level(LED_PIN, 0);
      ESP_LOGI(TAG, "Zeit fuer LoRa: %.1f ms. Paketgroesse: %u", double(te - ts) / 1000.0, iCBORBuildSize);
      if (!SendOK)
      {
        RetryCounter++;
        ESP_LOGE(TAG, "Fehler beim Senden eines LoRa-Paketes: %d Versuch: %d", ret, RetryCounter);
        ESP_LOGE(TAG, "Resette LORA-Modul...");
        LoRa.Reset();
        vTaskDelay(pdMS_TO_TICKS(100));
        ret = LoRa.SetupModule(LORA_ADDR_GWHS, 434.54e6, 14, 500E3, 0x3d);
        if (ret != ESP_OK)
          error(TAG, "Fehler beim Initialisieren des LoRa Moduls: %d", ret);
        else
          ESP_LOGI(TAG, "Re-Init LORA erfolgreich");
      }
    } 
    if (!SendOK)
      ESP_LOGE(TAG, "Wiederholtes Senden fehlgeschlagen!");

    u8g2_ClearBuffer(&u8g2);
    sprintf(DisplayBuf, "Paket Nr.: %lu",iPC);
    u8g2_DrawStr(&u8g2, 2, 14, DisplayBuf);

    sprintf(DisplayBuf, "Temp: %.2f%cC",iTemp_deg,176);
    u8g2_DrawStr(&u8g2, 2, 28, DisplayBuf);

    sprintf(DisplayBuf, "Feuchte: %.2f %%",iHum_per);
    u8g2_DrawStr(&u8g2, 2, 42, DisplayBuf);

    sprintf(DisplayBuf, "Druck: %.2f mBar",iPress_mBar);
    u8g2_DrawStr(&u8g2, 2, 56, DisplayBuf);


    u8g2_SendBuffer(&u8g2);
    iPC++;
    vTaskDelay(10000 / portTICK_PERIOD_MS);
  }

  gpio_set_level(LED_PIN, 0);
  u8g2_SetPowerSave(&u8g2, 1);
  LoRa.Close();

  esp_wifi_stop();
  rtc_gpio_isolate(LED_PIN);
  esp_sleep_disable_wakeup_source(ESP_SLEEP_WAKEUP_ALL);
  esp_deep_sleep_start();
  /*
  uint8_t s_led_state = 0;
  int c=0;
  char text[20];
  ssd1306_clear_screen(&dev, false);
  while (1)
  {
    gpio_set_level(LED_PIN, s_led_state);
    // Toggle the LED state
    s_led_state = !s_led_state;
    sprintf(text,"Hallo %d",c);
    ssd1306_display_text(&dev, 5, text, strlen(text), false);
    vTaskDelay(200 / portTICK_PERIOD_MS);
    c++;
  }*/
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
    gpio_set_level(LED_PIN, toggle);
    vTaskDelay(pdMS_TO_TICKS(100));
  }
}

esp_err_t BuildCBORBuf(uint8_t *aBuf, uint16_t aMaxBufSize, uint16_t &aCBORBuildSize, uint32_t aPC, float aTemp_deg, float aHum_per, float aPress_mBar)
{
  // Achtung: In einer Map müssesn stets zwei Einträge paarweise stehen, sonst
  // schlägt der Encoder fehl!
  CborEncoder encoder, me0, me1, arr;
  cbor_encoder_init(&encoder, aBuf, aMaxBufSize, 0);
  // cbor_encode_text_stringz(&encoder, "WS");

  // Paketzähler
  cbor_encoder_create_map(&encoder, &me0, CborIndefiniteLength);
  cbor_encode_text_stringz(&me0, PC_TAG);
  cbor_encode_int(&me0, aPC);

  // Temperaturen
  cbor_encode_text_stringz(&me0, TEMP_TAG);
  cbor_encoder_create_array(&me0, &arr, CborIndefiniteLength);

  cbor_encoder_create_map(&arr, &me1, CborIndefiniteLength);
  cbor_encode_text_stringz(&me1, POS_TAG);
  cbor_encode_text_stringz(&me1, ORT_GEWAECHSHAUS);
  cbor_encode_text_stringz(&me1, VAL_TAG);
  cbor_encode_float(&me1, aTemp_deg);
  cbor_encoder_close_container(&arr, &me1);
  /*
      cbor_encoder_create_map(&arr, &me1, CborIndefiniteLength);
      cbor_encode_text_stringz(&me1, POS_TAG);
      cbor_encode_text_stringz(&me1, ORT_SCHUPPEN_SONNE);
      cbor_encode_text_stringz(&me1, VAL_TAG);
      cbor_encode_float(&me1, temp2);
      cbor_encoder_close_container(&arr, &me1);
  */
  cbor_encoder_close_container(&me0, &arr);

  // Luftfeuchtigkeit
  cbor_encode_text_stringz(&me0, HUM_TAG);
  cbor_encoder_create_array(&me0, &arr, CborIndefiniteLength);

  cbor_encoder_create_map(&arr, &me1, CborIndefiniteLength);
  cbor_encode_text_stringz(&me1, POS_TAG);
  cbor_encode_text_stringz(&me1, ORT_GEWAECHSHAUS);
  cbor_encode_text_stringz(&me1, VAL_TAG);
  cbor_encode_float(&me1, aHum_per);
  cbor_encoder_close_container(&arr, &me1);
  /*
      cbor_encoder_create_map(&arr, &me1, CborIndefiniteLength);
      cbor_encode_text_stringz(&me1, POS_TAG);
      cbor_encode_text_stringz(&me1, ORT_SCHUPPEN_SONNE);
      cbor_encode_text_stringz(&me1, VAL_TAG);
      cbor_encode_float(&me1, hum2);
      cbor_encoder_close_container(&arr, &me1);

      
  */
  cbor_encoder_close_container(&me0, &arr);
  
  // Luftdruck
  cbor_encode_text_stringz(&me0, PRESS_TAG);
  cbor_encoder_create_map(&me0, &me1, CborIndefiniteLength);
  cbor_encode_text_stringz(&me1, POS_TAG);
  cbor_encode_text_stringz(&me1, ORT_GEWAECHSHAUS);
  cbor_encode_text_stringz(&me1, VAL_TAG);
  cbor_encode_float(&me1, aPress_mBar);
  cbor_encoder_close_container(&me0, &me1);
  /*
      // Beleuchtungsstaerke
      cbor_encode_text_stringz(&me0, ILLU_TAG);
      cbor_encoder_create_map(&me0, &me1, CborIndefiniteLength);
      cbor_encode_text_stringz(&me1, POS_TAG);
      cbor_encode_text_stringz(&me1, ORT_GEWAECHSHAUS);
      cbor_encode_text_stringz(&me1, VAL_TAG);
      cbor_encode_float(&me1, 42);
      cbor_encoder_close_container(&me0, &me1);

      // Lüftergeschwindigkeit
      cbor_encode_text_stringz(&me0, COOL_TAG);
      cbor_encoder_create_map(&me0, &me1, CborIndefiniteLength);
      cbor_encode_text_stringz(&me1, POS_TAG);
      cbor_encode_text_stringz(&me1, ORT_GEWAECHSHAUS);
      cbor_encode_text_stringz(&me1, VAL_TAG);
      cbor_encode_float(&me1, 42);
      cbor_encoder_close_container(&me0, &me1);
  */
  cbor_encoder_close_container(&encoder, &me0);

  aCBORBuildSize = cbor_encoder_get_buffer_size(&encoder, aBuf);
  // ESP_LOGI(TAG, "CBOR erstellt, Groesse: %d", len);
  return ESP_OK;
}