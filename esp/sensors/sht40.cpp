/*
 * SHT40 Sensor
 */

#include <stdio.h>
#include "esp_log.h"
#include <stdint.h>
#include "sht40.h"
#include <math.h>
#include "freertos/FreeRTOS.h"

#define DEV_TIMEOUT 100

SHT40::SHT40()
{
  mDevHandle = NULL;
  mBusHandle = NULL;
}

esp_err_t SHT40::Init(i2c_master_bus_handle_t aBusHandle, uint8_t aI2CAddr, uint32_t aI2CSpeed_Hz, SHT40_COMMAND aReadMode)
{
  mReadMode = aReadMode;
  mBusHandle = aBusHandle;
  i2c_device_config_t conf;
  conf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  conf.device_address = aI2CAddr;
  conf.scl_speed_hz = aI2CSpeed_Hz;
  return i2c_master_bus_add_device(mBusHandle, &conf, &mDevHandle);
}

SHT40::~SHT40(void)
{
  if (mDevHandle != NULL)
    i2c_master_bus_rm_device(mDevHandle);
}

// SHT40-Sensor auslesen
esp_err_t SHT40::Read(float &aTemp, float &aHum, bool &rCRC_Err)
{
  int ret;
  rCRC_Err = false;
  uint8_t mode = (uint8_t)mReadMode;
  ret = i2c_master_transmit(mDevHandle, &mode, 1, DEV_TIMEOUT);
  if (ret != ESP_OK)
  {
    ESP_LOGE("SHT40::Read", "I2C error no.: %d", ret);
    return ret;
  }
  int iWaitTime_ms = 100;
  switch (mReadMode)
  {
  case SHT40_CMD_HPM_H_20_100:
  case SHT40_CMD_HPM_H_110_100:
  case SHT40_CMD_HPM_H_200_100:
    iWaitTime_ms = 110;
    break;
  case SHT40_CMD_HPM_H_20_1000:
  case SHT40_CMD_HPM_H_110_1000:
  case SHT40_CMD_HPM_H_200_1000:
    iWaitTime_ms = 1010;
    break;
  case SHT40_CMD_LPM:
    iWaitTime_ms = DELAY_LPM_MS;
    break;
  case SHT40_CMD_MPM:
    iWaitTime_ms = DELAY_MPM_MS;
    break;
  case SHT40_CMD_HPM:
    iWaitTime_ms = DELAY_HPM_MS;
    break;
  default:
    iWaitTime_ms = 100;
    break;
  }
  vTaskDelay(iWaitTime_ms / portTICK_PERIOD_MS);
  // Belegung Datenblock in bytes:
  // Temperatur_high, Temperatur_low, Temperatur_crc, Luftfeuchte_high, Luftfeuchte_low, Luftfeuchte_crc
  uint8_t rb[6] = {0};
  ret = i2c_master_receive(mDevHandle, rb, 6, DEV_TIMEOUT);
  if (ret != ESP_OK)
  {
    ESP_LOGE("SHT40::Read", "I2C error no.: %d", ret);
    return ret;
  }
  aTemp = -45 + 175 * (double)(rb[0] * 256 + rb[1]) / 65535.0;
  aHum = -6 + 125 * (double)(rb[3] * 256 + rb[4]) / 65535.0;
  // Theoretisch können auch größere Werte als 0..100 % bei der Luftfeuchtigkeit entstehen, deshalb hier begrenzen
  aHum = std::max(0.0, std::min((double)aHum, 100.0));

  // Checksumme Temperatur
  uint8_t crc = ComputeChecksum(rb, 2);
  if (crc != rb[2])
  {
    ESP_LOGE("SHT40:", "Falscher Temperatur-CRC: [0x%x] <> [0x%x]\r\n", crc, rb[2]);
    rCRC_Err = true;
  }
  // Checksumme Luftfeuchte
  crc = ComputeChecksum(rb + 3, 2);
  if (crc != rb[5])
  {
    ESP_LOGE("SHT40:", "Falscher Luftfeuchte-CRC: [0x%x] <> [0x%x]\r\n", crc, rb[5]);
    rCRC_Err = true;
  }

  return ret;
}

esp_err_t SHT40::ReadSerial(uint32_t &aSerialNo, bool &rCRC_Err)
{
  int ret;
  rCRC_Err = false;
  uint8_t cmd = (uint8_t)SHT40_CMD_READ_SERIAL;
  ret = i2c_master_transmit(mDevHandle, &cmd, 1, DEV_TIMEOUT);
  if (ret != ESP_OK)
  {
    ESP_LOGE("SHT40::ReadSerial", "I2C-Fehler: %d", ret);
    return ret;
  }
  // Ohne diese Pause kann die Seriennummer nicht gelesen werden. Es gibt dann ein Timeout
  vTaskDelay(10 / portTICK_PERIOD_MS);
  // Belegung Datenblock in bytes:
  // [2 * 8-bit data; 8-bit CRC; 2 * 8-bit data; 8-bit CRC]
  uint8_t rb[6] = {0};
  ret = i2c_master_receive(mDevHandle, rb, 6, DEV_TIMEOUT);
  if (ret != ESP_OK)
  {
    ESP_LOGE("SHT40::ReadSerial", "I2C-Fehler: %d", ret);
    return ret;
  }

  aSerialNo = (rb[0] << 24) + (rb[1] << 16) + (rb[3] << 8) + rb[4];

  // Checksumme Serial Block0
  uint8_t crc = ComputeChecksum(rb, 2);
  if (crc != rb[2])
  {
    ESP_LOGE("SHT40:", "Falscher Serial(0)-CRC: [0x%x] <> [0x%x]\r\n", crc, rb[2]);
    rCRC_Err = true;
  }

  // Checksumme Serial Block1
  crc = ComputeChecksum(rb + 3, 2);
  if (crc != rb[5])
  {
    ESP_LOGE("SHT40:", "Falscher Serial(1)-CRC: [0x%x] <> [0x%x]\r\n", crc, rb[5]);
    rCRC_Err = true;
  }

  return ret;
}

uint8_t SHT40::ComputeChecksum(uint8_t *bytes, int len)
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

uint8_t SHT40::Crc8b(uint8_t aData)
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
