#include <stdio.h>
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "spi_flash_mmap.h"
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include <u8g2.h>
#include "../../../shared/u8g2_esp32_hal.h"

#define LED_PIN GPIO_NUM_25
#define TAG "LORA_TEMP"

// SDA - GPIO21
#define PIN_SDA 21

// SCL - GPIO22
#define PIN_SCL 22
u8g2_t u8g2;  // a structure which will contain all the data for one display

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

void app_main(void)
{
  gpio_reset_pin(LED_PIN);
  /* Set the GPIO as a push/pull output */
  gpio_set_direction(LED_PIN, GPIO_MODE_OUTPUT);
  gpio_set_level(LED_PIN, 1);
  InitSSD1306_u8g2();
  gpio_set_level(LED_PIN, 0);
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
