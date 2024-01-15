/*!
 * SHT35 Sensor
 */

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <stdint.h>
#include "ads1015.h"
#include <math.h>

#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */

ADS1015::ADS1015(i2c_port_t aPort, uint8_t aI2CAddr, int aSDA_Pin, int aSCL_Pin)
{
  mOpened = false;
  mPort = aPort;
  mSDA_Pin = aSDA_Pin;
  mSCL_Pin = aSCL_Pin;
  mInitI2C = aInitI2C;
  mI2CAddr = aI2CAddr;
}

ADS1015::~ADS1015(void)
{
  Close();
}

esp_err_t ADS1015::Init(bool aContinuousMode, ADC_MP aInputMux, FSC_RANGE aFullScale, ADC_SPEED aSpeed);
{
  esp_err_t ret;
  if (mInitI2C && !mOpened)
  {
    if (mPort > 1)
      return ESP_ERR_INVALID_ARG;

    mOpened = true;
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;
    conf.sda_io_num = mSDA_Pin;
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
    conf.scl_io_num = mSCL_Pin;
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000; // Fast Mode, 1000000;  // Fast Mode Plus=1MHz
    conf.clk_flags = 0;
    ret = i2c_param_config(mPort, &conf);
    if (ret != ESP_OK)
      return ret;
    ret = i2c_driver_install(mPort, conf.mode, 0, 0, 0);
    if (ret != ESP_OK)
      return ret;
  }

  // Init ADS1015 (die 0x3 am Ende schaltet den Komperator ab...)
  )
  uint16_t Config=(aInputMux << 12) + (aFullScale << 9) + ((uint8_t)(!aContinuousMode) << 8) + (aSpeed << 5) + 0x3;
  return WriteRegister16(REG_CONFIG, Config);
}

void ADS1015::Close()
{
  if (mInitI2C && mOpened)
  {
    mOpened = false;
    i2c_driver_delete(mPort);
  }
}

esp_err_t ADS1015::ReadRegister16(ADC_REGISTER aReg, uint16_t &aRegVal)
{
  if (!mOpened)
    return ESP_ERR_NOT_ALLOWED;
  esp_err_t ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, mI2CAddr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(mPort, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    ESP_LOGE("ADS1015::ReadRegister", "I2C error no.: %d", ret);
    return ret;
  }
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, mI2CAddr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
  uint8_t data[2];
  i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(mPort, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  aRegVal = data[0] * 0xff + data[1];
  if (ret != ESP_OK)
    ESP_LOGE("ADS1015::ReadRegister", "I2C error no.: %d", ret);
  return ret;
}

esp_err_t ADS1015::WriteRegister16(ADC_REGISTER aReg, uint16_t aRegVal)
{
  if (!mOpened)
    return ESP_ERR_NOT_ALLOWED;
  esp_err_t ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, mI2CAddr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, aReg, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, (aRegVal >> 8), ACK_CHECK_EN);   // Registerwert Hi
  i2c_master_write_byte(cmd, (aRegVal & 0xFF), ACK_CHECK_EN); // Registerwert Lo
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(mPort, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
    ESP_LOGE("ADS1015::WriteRegister", "I2C error no.: %d", ret);
  return ret;
}

esp_err_t ADS1015::ReadADC(uint16_t &aADCValue)
{
}

// SHT40-Sensor auslesen
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
