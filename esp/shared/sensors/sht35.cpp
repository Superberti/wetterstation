/*!
 * SHT35 Sensor
 */

#include <stdio.h>
#include "esp_log.h"
#include <stdint.h>
#include "sht35.h"
#include <math.h>
#include "freertos/FreeRTOS.h"

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */

#define DEV_TIMEOUT 100

SHT35::SHT35()
{
  mDevHandle = NULL;
  mBusHandle = NULL;
}

esp_err_t SHT35::Init(i2c_master_bus_handle_t aBusHandle, uint8_t aI2CAddr, uint32_t aI2CSpeed_Hz)
{
  mBusHandle = aBusHandle;
  i2c_device_config_t conf;
  conf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  conf.device_address = aI2CAddr;
  conf.scl_speed_hz = aI2CSpeed_Hz;
  conf.flags.disable_ack_check=false;
  return i2c_master_bus_add_device(mBusHandle, &conf, &mDevHandle);
}

SHT35::~SHT35(void)
{
  if (mDevHandle != NULL)
    i2c_master_bus_rm_device(mDevHandle);
}

// SHT35-Sensor auslesen
esp_err_t SHT35::Read(float &aTemp, float &aHum)
{
  esp_err_t ret;
  uint8_t StartCmd[2] = {SHT35_CMD_START_MSB, SHT35_CMD_START_LSB};
  ret = i2c_master_transmit(mDevHandle, StartCmd, 2, DEV_TIMEOUT);
  if (ret != ESP_OK)
  {
    ESP_LOGE("SHT35::Read", "I2C error no.: %d", ret);
    return ret;
  }
  int iWaitTime_ms = 100;
  vTaskDelay(pdMS_TO_TICKS(iWaitTime_ms));

  // Belegung Datenblock in bytes:
  // Temperatur_high, Temperatur_low, Temperatur_crc, Luftfeuchte_high, Luftfeuchte_low, Luftfeuchte_crc
  uint8_t rb[6] = {0};
  ret = i2c_master_receive(mDevHandle, rb, 6, DEV_TIMEOUT);
  if (ret != ESP_OK)
  {
    ESP_LOGE("SHT35::Read", "I2C error no.: %d", ret);
    return ret;
  }

  aTemp = -45 + 175 * (double)(rb[0] * 256 + rb[1]) / 65535.0;
  aHum = 100.0*(double)(rb[3]*256+rb[4])/65535.0;
  // Theoretisch können auch größere Werte als 0..100 % bei der Luftfeuchtigkeit entstehen, deshalb hier begrenzen
  aHum = std::max(0.0, std::min((double)aHum, 100.0));

  // Checksumme Temperatur
  uint8_t crc = ComputeChecksum(rb, 2);
  if (crc != rb[2])
  {
    ESP_LOGE("SHT35:", "Falscher Temperatur-CRC: [0x%x] <> [0x%x]\r\n", crc, rb[2]);
    return ESP_ERR_INVALID_CRC;
  }
  // Checksumme Luftfeuchte
  crc = ComputeChecksum(rb + 3, 2);
  if (crc != rb[5])
  {
    ESP_LOGE("SHT35:", "Falscher Luftfeuchte-CRC: [0x%x] <> [0x%x]\r\n", crc, rb[5]);
    return ESP_ERR_INVALID_CRC;
  }

  return ret;
}

uint8_t SHT35::ComputeChecksum(uint8_t *bytes, int len)
{
  uint8_t crc = 0xff; // Startwert

  if (bytes != NULL && len > 0)
  {
    for (int i = 0; i < len; i++)
    {
      crc = Crc8b(crc ^ bytes[i]);
    }
  }
  return crc;
}

uint8_t SHT35::Crc8b(uint8_t aData)
{
  for (int j = 0; j < 8; ++j)
  {
    if ((aData & 0x80) != 0)
    {
      aData = (aData << 1) ^ poly;
    }
    else
    {
      aData <<= 1;
    }
  }
  return aData;
}


