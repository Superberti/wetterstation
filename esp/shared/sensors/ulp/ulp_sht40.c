#include <stdint.h>
#include <stdbool.h>
#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"
#include "esp_err.h"
#include "ulp_sht40.h"

static const uint8_t poly = 0x31; // x8 + x5 + x4 + 1

uint8_t Crc8b(uint8_t aData)
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

uint8_t ComputeChecksum(uint8_t *bytes, int len)
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

esp_err_t SHT40_ReadSerial(soft_i2c_config_t *cfg, uint32_t *aSerialNo, bool *rCRC_Err)
{
  *aSerialNo = -1;
  esp_err_t ret = 0;
  *rCRC_Err = false;
  uint8_t cmd = SHT40_CMD_READ_SERIAL;

  ret = i2c_bb_master_write(cfg, &cmd, 1);
  if (ret != ESP_OK)
  {
    // ESP_LOGE("SHT40::ReadSerial", "I2C-Fehler: %d", ret);
    return ret;
  }
  // Ohne diese Pause kann die Seriennummer nicht gelesen werden. Es gibt dann ein Timeout
  delay(1000 * ULP_RISCV_CYCLES_PER_US);

  //   Belegung Datenblock in bytes:
  //   [2 * 8-bit data; 8-bit CRC; 2 * 8-bit data; 8-bit CRC]
  uint8_t rb[6] = {0};
  ret = i2c_bb_master_read(cfg, rb, sizeof(rb));
  if (ret != ESP_OK)
  {
    // ESP_LOGE("SHT40::ReadSerial", "I2C-Fehler: %d", ret);
    return ret;
  }

  *aSerialNo = (rb[0] << 24) + (rb[1] << 16) + (rb[3] << 8) + rb[4];

  // Checksumme Serial Block0
  uint8_t crc = ComputeChecksum(rb, 2);
  if (crc != rb[2])
  {
    // ESP_LOGE("SHT40:", "Falscher Serial(0)-CRC: [0x%x] <> [0x%x]\r\n", crc, rb[2]);
    *rCRC_Err = true;
  }

  // Checksumme Serial Block1
  crc = ComputeChecksum(rb + 3, 2);
  if (crc != rb[5])
  {
    // ESP_LOGE("SHT40:", "Falscher Serial(1)-CRC: [0x%x] <> [0x%x]\r\n", crc, rb[5]);
    *rCRC_Err = true;
  }
  return ret;
}



// SHT40-Sensor auslesen
esp_err_t SHT40_Read(soft_i2c_config_t *cfg, SHT40_COMMAND aReadMode, uint16_t *aTemp, uint16_t *aHum, bool *rCRC_Err)
{
  int ret;
  *rCRC_Err = false;
  uint8_t mode = (uint8_t)aReadMode;
  ret = i2c_bb_master_write(cfg, &mode, 1);
  if (ret != ESP_OK)
  {
    // ESP_LOGE("SHT40::Read", "I2C error no.: %d", ret);
    return ret;
  }
  int iWaitTime_ms = 100;
  switch (aReadMode)
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
  delay(iWaitTime_ms * 1000 * ULP_RISCV_CYCLES_PER_US);

  // Belegung Datenblock in bytes:
  // Temperatur_high, Temperatur_low, Temperatur_crc, Luftfeuchte_high, Luftfeuchte_low, Luftfeuchte_crc
  uint8_t rb[6] = {0};
  ret = i2c_bb_master_read(cfg, rb, 6);
  if (ret != ESP_OK)
  {
    return ret;
  }
  *aTemp = rb[0] * 256 + rb[1];
  *aHum = rb[3] * 256 + rb[4];
  // Checksumme Temperatur
  uint8_t crc = ComputeChecksum(rb, 2);
  if (crc != rb[2])
  {
    *rCRC_Err = true;
  }
  // Checksumme Luftfeuchte
  crc = ComputeChecksum(rb + 3, 2);
  if (crc != rb[5])
  {
    *rCRC_Err = true;
  }

  return ret;
}