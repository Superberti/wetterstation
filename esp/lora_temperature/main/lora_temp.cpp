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

#define RADIO_SCLK_PIN GPIO_NUM_5
#define RADIO_MISO_PIN GPIO_NUM_19
#define RADIO_MOSI_PIN GPIO_NUM_27
#define RADIO_CS_PIN GPIO_NUM_18
#define RADIO_DIO0_PIN GPIO_NUM_26
#define RADIO_RST_PIN GPIO_NUM_23
#define RADIO_DIO1_PIN GPIO_NUM_33
#define RADIO_BUSY_PIN GPIO_NUM_32

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
  u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
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
  //vTaskDelay(5000 / portTICK_PERIOD_MS);
  CborEncoder root_encoder;
  CborParser root_parser;
  CborValue it;
  uint8_t buf[100];

  // Initialize the outermost cbor encoder
  cbor_encoder_init(&root_encoder, buf, sizeof(buf), 0);

  // Create an array containing several items
  CborEncoder array_encoder;
  CborEncoder map_encoder;

  gpio_reset_pin(LED_PIN);
  /* Set the GPIO as a push/pull output */
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_PIN, 1);
  InitSSD1306_u8g2();
  SX1278_LoRa LoRa;
  esp_err_t ret = LoRa.SetupModule();
  if (ret != ESP_OK)
    error(TAG, "Fehler beim Initialisieren des LoRa Moduls: %d", ret);

  vTaskDelay(5000 / portTICK_PERIOD_MS);
  gpio_set_level(LED_PIN, 0);
  u8g2_SetPowerSave(&u8g2, 1);
  LoRa.lora_sleep();
/*
  gpio_reset_pin(RADIO_CS_PIN);
  gpio_set_direction(RADIO_CS_PIN, GPIO_MODE_INPUT);
  //pinMode(RADIO_CS_PIN, INPUT);

  gpio_reset_pin(RADIO_RST_PIN);
  gpio_set_direction(RADIO_RST_PIN, GPIO_MODE_INPUT);
  //pinMode(RADIO_RST_PIN, INPUT);

  gpio_reset_pin(RADIO_DIO0_PIN);
  gpio_set_direction(RADIO_DIO0_PIN, GPIO_MODE_INPUT);
  //pinMode(RADIO_DIO0_PIN, INPUT);

  gpio_reset_pin(RADIO_CS_PIN);
  gpio_set_direction(RADIO_CS_PIN, GPIO_MODE_INPUT);
  //pinMode(RADIO_CS_PIN, INPUT);

  gpio_reset_pin(RADIO_RST_PIN);
  gpio_set_direction(RADIO_RST_PIN, GPIO_MODE_INPUT);
  //pinMode(RADIO_RST_PIN, INPUT);

  gpio_reset_pin(I2C_SDA);
  gpio_set_direction(I2C_SDA, GPIO_MODE_INPUT);
  //pinMode(I2C_SDA, INPUT);

  gpio_reset_pin(I2C_SCL);
  gpio_set_direction(I2C_SCL, GPIO_MODE_INPUT);
  //pinMode(I2C_SCL, INPUT);

  gpio_reset_pin(RADIO_SCLK_PIN);
  gpio_set_direction(RADIO_SCLK_PIN, GPIO_MODE_INPUT);
  //pinMode(RADIO_SCLK_PIN, INPUT);

  gpio_reset_pin(RADIO_MISO_PIN);
  gpio_set_direction(RADIO_MISO_PIN, GPIO_MODE_INPUT);
  //pinMode(RADIO_MISO_PIN, INPUT);

  gpio_reset_pin(RADIO_MOSI_PIN);
  gpio_set_direction(RADIO_MOSI_PIN, GPIO_MODE_INPUT);
  //pinMode(RADIO_MOSI_PIN, INPUT);

  gpio_reset_pin(SDCARD_MOSI);
  gpio_set_direction(SDCARD_MOSI, GPIO_MODE_INPUT);
  //pinMode(SDCARD_MOSI, INPUT);

  gpio_reset_pin(SDCARD_MISO);
  gpio_set_direction(SDCARD_MISO, GPIO_MODE_INPUT);
  //pinMode(SDCARD_MISO, INPUT);

  gpio_reset_pin(SDCARD_SCLK);
  gpio_set_direction(SDCARD_SCLK, GPIO_MODE_INPUT);
  //pinMode(SDCARD_SCLK, INPUT);

  gpio_reset_pin(SDCARD_CS);
  gpio_set_direction(SDCARD_CS, GPIO_MODE_INPUT);
  //pinMode(SDCARD_CS, INPUT);

  //gpio_reset_pin(BOARD_LED);
  //gpio_set_direction(BOARD_LED, GPIO_MODE_INPUT);
  //pinMode(BOARD_LED, INPUT);

  gpio_reset_pin(ADC_PIN);
  gpio_set_direction(ADC_PIN, GPIO_MODE_INPUT);
  //pinMode(ADC_PIN, INPUT);
*/
  
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