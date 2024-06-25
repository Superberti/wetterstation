/*
 * SPDX-FileCopyrightText: 2023 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
/* ULP-RISC-V example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.

   This code runs on ULP-RISC-V  coprocessor
*/

#include <stdint.h>
#include <stdbool.h>
#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"
#include "ulp_i2c_bitbang.h"
#include "esp_err.h"
#include "tools.h"
#include "../CommonTypes.h"
#include "../SensorTypes.h"
#include "ulp_sht40.h"
#include "ulp_ads1015.h"

/* this variable will be exported as a public symbol, visible from main CPU: */
uint32_t DataReady = 0;
uint32_t Cycles = 0;

struct RawSensorData SensorDataBuffer[ESP_WAKEUP_MULT] = {0};

int main(void)
{
  ulp_riscv_gpio_init(BOARD_LED);
  ulp_riscv_gpio_output_enable(BOARD_LED);
  ulp_riscv_gpio_set_output_mode(BOARD_LED, RTCIO_MODE_OUTPUT);
  ulp_riscv_gpio_output_level(BOARD_LED, 1);

  ulp_riscv_gpio_init(SWITCH_BRIDGE_AND_CLOCK);
  ulp_riscv_gpio_output_enable(SWITCH_BRIDGE_AND_CLOCK);
  ulp_riscv_gpio_set_output_mode(SWITCH_BRIDGE_AND_CLOCK, RTCIO_MODE_OUTPUT);

  // I2C-0 (Uhr und ADC)
  ulp_riscv_gpio_init(PIN_SDA_BUS0);
  ulp_riscv_gpio_init(PIN_SCL_BUS0);
  ulp_riscv_gpio_output_enable(PIN_SDA_BUS0);
  ulp_riscv_gpio_output_enable(PIN_SCL_BUS0);
  ulp_riscv_gpio_input_enable(PIN_SDA_BUS0);
  ulp_riscv_gpio_input_enable(PIN_SCL_BUS0);
  ulp_riscv_gpio_set_output_mode(PIN_SDA_BUS0, RTCIO_MODE_OUTPUT_OD);
  ulp_riscv_gpio_set_output_mode(PIN_SCL_BUS0, RTCIO_MODE_OUTPUT_OD);

  // I2C-1 (Temperatursensor und Luftfeuchte)
  ulp_riscv_gpio_init(PIN_SDA_BUS1);
  ulp_riscv_gpio_init(PIN_SCL_BUS1);
  ulp_riscv_gpio_output_enable(PIN_SDA_BUS1);
  ulp_riscv_gpio_output_enable(PIN_SCL_BUS1);
  ulp_riscv_gpio_input_enable(PIN_SDA_BUS1);
  ulp_riscv_gpio_input_enable(PIN_SCL_BUS1);
  ulp_riscv_gpio_set_output_mode(PIN_SDA_BUS1, RTCIO_MODE_OUTPUT_OD);
  ulp_riscv_gpio_set_output_mode(PIN_SCL_BUS1, RTCIO_MODE_OUTPUT_OD);

  soft_i2c_config_t ads_config = {PIN_SCL_BUS0, PIN_SDA_BUS0, ADC1x15_ADDR_VDD};
  soft_i2c_config_t sht_config = {PIN_SCL_BUS1, PIN_SDA_BUS1, SHT40_ADDR};

  uint16_t temp, hum;
  uint32_t aSerial = 0;
  uint16_t adc;
  bool CRC_Err=false;
  
  ulp_riscv_gpio_output_level(SWITCH_BRIDGE_AND_CLOCK, 1);
  //delay(1000 * ULP_RISCV_CYCLES_PER_US);
  // Sensoren abfragen
  esp_err_t ret_sht = SHT40_Read(&sht_config, SHT40_CMD_HPM, &temp, &hum, &CRC_Err);
  //esp_err_t ret_sht = SHT40_ReadSerial(&sht_config, &aSerial, &CRC_Err);
  esp_err_t ret_adc = ADS1x15_ReadADC(&ads_config, false, AIN0_AND_AIN1, FSR_0_256, ADS_1115_SPEED_128, &adc);

  if (ret_sht == ESP_OK && CRC_Err)
    ret_sht = ESP_ERR_INVALID_CRC;
  SensorDataBuffer[Cycles % ESP_WAKEUP_MULT].Temp = temp;
  SensorDataBuffer[Cycles % ESP_WAKEUP_MULT].Hum = hum;
  SensorDataBuffer[Cycles % ESP_WAKEUP_MULT].Press_ADC = adc;
  SensorDataBuffer[Cycles % ESP_WAKEUP_MULT].err_sht = ret_sht;
  SensorDataBuffer[Cycles % ESP_WAKEUP_MULT].err_adc = ret_adc;
  //delay(1000*1000 * ULP_RISCV_CYCLES_PER_US);
  ulp_riscv_gpio_output_level(SWITCH_BRIDGE_AND_CLOCK, 0);

  Cycles++;
  if (Cycles != 0 && Cycles % ESP_WAKEUP_MULT == 0)
  {
    DataReady = 1;
    ulp_riscv_wakeup_main_processor(); // Weck den großen auf!
  }

  /* ulp_riscv_halt() is called automatically when main exits */
  ulp_riscv_gpio_output_level(BOARD_LED, 0);
  return 0;
}
