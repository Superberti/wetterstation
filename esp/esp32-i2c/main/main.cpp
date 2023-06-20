#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "esp_task_wdt.h"
#include "main.h"
#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include <string.h>
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_common.h"
#include <sys/time.h>
#include <math.h>
#include "esp_sleep.h"
#include "bmp390.h"

static const char *TAG = "BMP390 test";

#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL /*!< GPIO number used for I2C master clock (GPIO19)*/
#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA /*!< GPIO number used for I2C master data  (GPIO18)*/
#define I2C_MASTER_NUM 0                        /*!< I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip */
#define I2C_MASTER_FREQ_HZ 400000               /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0             /*!< I2C master doesn't need buffer */
#define I2C_MASTER_TIMEOUT_MS 1000

#define BMP390_SENSOR_ADDR 0x77 /*!< Slave address of the BMP390 sensor */

void app_main_cpp(void)
{
  ESP_LOGI(TAG, "BMP390 test started.");
  xTaskCreate(&main_task, "main_task", 4096, NULL, 5, NULL);
  for (;;)
  {
    vTaskDelay(pdMS_TO_TICKS(5000));
    // esp_task_wdt_reset();
  }
}

extern "C"
{
  void app_main()
  {
    app_main_cpp();
  }
}

void main_task(void *par)
{
  BMP390 bmp(I2C_NUM_0, BMP390_SENSOR_ADDR, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);
  esp_err_t stat = bmp.Init();
  bool SensorOK = true;
  if (stat != ESP_OK)
  {
    SensorOK = false;
    ESP_LOGI(TAG, "Initialisierung BMP390 fehlgeschlagen: code %d.", stat);
  }

  double t, p;
  for (;;)
  {
    if (SensorOK)
    {
      stat = bmp.ReadTempAndPress(t, p);
      if (stat != ESP_OK)
      {
        ESP_LOGI(TAG, "Lesefehler BMP390: code %d.", stat);
      }
      else
        ESP_LOGI(TAG, "Temp: %.2f°C Druck: %.1f mbar", t, p);
    }
    ESP_LOGI(TAG, "ping.");
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}