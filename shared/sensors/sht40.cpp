/*!
 * SHT35 Sensor
 */

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <stdint.h>
#include "sht40.h"
#include <math.h>

#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */

SHT40::SHT40(int aPort, int aSDA_Pin, int aSCL_Pin, SHT40_COMMAND aReadMode)
{
  mReadMode = aReadMode;
  mPort = aPort;
  mSDA_Pin=aSDA_Pin;
  mSCL_Pin=aSCL_Pin;
}

SHT40::~SHT40(void)
{
}

esp_err_t SHT40::Init()
{
  if (mPort > 1)
    return ESP_ERR_INVALID_ARG;

  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = mSDA_Pin;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = mSCL_Pin;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = 100000; // Fast Mode, 1000000;  // Fast Mode Plus=1MHz
  conf.clk_flags=0;
  i2c_param_config(mPort, &conf);
  return i2c_driver_install(mPort, conf.mode, 0, 0, 0);
}

// SHT340-Sensor auslesen
esp_err_t SHT40::Read(float &aTemp, float &aHum, bool &rCRC_Err)
{
  int ret;
  rCRC_Err = false;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHT40_ADDR) << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, (uint8_t)mReadMode, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(mPort, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
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
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHT40_ADDR) << 1 | I2C_MASTER_READ, ACK_CHECK_EN);

  i2c_master_read(cmd, rb, sizeof(rb) - 1, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, rb + 5, I2C_MASTER_NACK); // Beim letzten Byte gibt's ein NACK

  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(mPort, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);

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
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHT40_ADDR << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, SHT40_CMD_READ_SERIAL, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(mPort, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    ESP_LOGE("SHT40:", "Fehler beim command ReadSerial: %d", ret);
    return ret;
  }
  // Ohne diese Pause kann die Seriennummer nicht gelesen werden. Es gibt dann ein Timeout
  vTaskDelay(10 / portTICK_PERIOD_MS);
  // Belegung Datenblock in bytes:
  // [2 * 8-bit data; 8-bit CRC; 2 * 8-bit data; 8-bit CRC]
  uint8_t rb[6] = {0};
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (SHT40_ADDR) << 1 | I2C_MASTER_READ, ACK_CHECK_EN);

  i2c_master_read(cmd, rb, sizeof(rb) - 1, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, rb + 5, I2C_MASTER_NACK); // Beim letzten Byte gibt's ein NACK

  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(mPort, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    ESP_LOGE("SHT40:", "Fehler beim Lesen der Seriennummer: %d", ret);
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
