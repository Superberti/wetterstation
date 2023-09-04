/*
 * BMP390 Sensor
 */

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <stdint.h>
#include "bmp390.h"
#include <math.h>

#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */

BMP390::BMP390(int aPort, uint8_t aI2CAddr, int aSDA_Pin, int aSCL_Pin)
{
  mPort = aPort;
  mSDA_Pin = aSDA_Pin;
  mSCL_Pin = aSCL_Pin;
  mI2CAddr = aI2CAddr;
}

BMP390::~BMP390(void)
{
}

esp_err_t BMP390::Init(bool aDoI2CInit)
{
  esp_err_t status = ESP_OK;
  if (aDoI2CInit)
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
    conf.clk_flags = 0;
    i2c_param_config((i2c_port_t)mPort, &conf);
    status = i2c_driver_install((i2c_port_t)mPort, conf.mode, 0, 0, 0);
    if (status != ESP_OK)
      return status;
  }

  if (Read8(BMP390_REGISTER_CHIP_ID) != BMP390_CHIP_ID)
    return ESP_ERR_INVALID_VERSION;
  Reset();
  status = ReadCalibData();
  if (status != ESP_OK)
    return status;

  //  Oversampling setzen. Bit 0..2: Druck, Bit 3..5 Temperatur
  return WriteRegister(BMP390_REGISTER_OSR, (SAMPLING_X2 << 3) | SAMPLING_X32);
}

esp_err_t BMP390::WriteRegister(uint8_t reg_addr, uint8_t value)
{
  esp_err_t ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, mI2CAddr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
    ESP_LOGE("BMP390::WriteRegister", "I2C error no.: %d", ret);
  return ret;
}

esp_err_t BMP390::ReadRegister(uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  esp_err_t ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, mI2CAddr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    ESP_LOGE("BMP390::ReadRegister", "I2C error no.: %d", ret);
    return ret;
  }
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, mI2CAddr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
  i2c_master_read(cmd, data, len, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
    ESP_LOGE("BMP390::ReadRegister", "I2C error no.: %d", ret);
  return ret;
}

uint8_t BMP390::Read8(uint8_t reg, esp_err_t *rError)
{
  uint8_t value;
  esp_err_t res = ReadRegister(reg, &value, sizeof(value));
  if (rError)
    *rError = res;
  return value;
}

esp_err_t BMP390::ReadCalibData()
{
  esp_err_t status = ReadRegister(BMP390_REGISTER_NVM_START, (uint8_t *)&calib_data, sizeof(calib_data));
  if (status != ESP_OK)
    return status;
  PAR_T1 = calib_data.NVM_PAR_T1 / pow(2, -8);
  PAR_T2 = calib_data.NVM_PAR_T2 / pow(2, 30);
  PAR_T3 = calib_data.NVM_PAR_T3 / pow(2, 48);

  PAR_P1 = (calib_data.NVM_PAR_P1 - pow(2, 14)) / pow(2, 20);
  PAR_P2 = (calib_data.NVM_PAR_P2 - pow(2, 14)) / pow(2, 29);
  PAR_P3 = calib_data.NVM_PAR_P3 / pow(2, 32);
  PAR_P4 = calib_data.NVM_PAR_P4 / pow(2, 37);
  PAR_P5 = calib_data.NVM_PAR_P5 / pow(2, -3);
  PAR_P6 = calib_data.NVM_PAR_P6 / pow(2, 6);
  PAR_P7 = calib_data.NVM_PAR_P7 / pow(2, 8);
  PAR_P8 = calib_data.NVM_PAR_P8 / pow(2, 15);
  PAR_P9 = calib_data.NVM_PAR_P9 / pow(2, 48);
  PAR_P10 = calib_data.NVM_PAR_P10 / pow(2, 48);
  PAR_P11 = calib_data.NVM_PAR_P11 / pow(2, 65);
  return status;
}

esp_err_t BMP390::ReadTempAndPress(double &rTemp_C, double &rPress_mbar)
{
  esp_err_t Status;
  Status = StartReadTempAndPress();
  if (Status != ESP_OK)
    return Status;
  return ReadTempAndPressAsync(rTemp_C, rPress_mbar, true);
}

esp_err_t BMP390::StartReadTempAndPress()
{
  return WriteRegister(BMP390_REGISTER_PWR_CTRL, (MODE_FORCED << 4) | 0x03);
}

esp_err_t BMP390::ReadTempAndPressAsync(double &rTemp_C, double &rPress_mbar, bool aWaitForData)
{
  if (!aWaitForData)
  {
    if (!DataReady())
      return ESP_ERR_INVALID_RESPONSE; // nix zum Abholen da
  }
  else
  {
    int tc = 0;
    while (!DataReady())
    {
      tc++;
      vTaskDelay(10 / portTICK_PERIOD_MS);
      if (tc > 50)
        return ESP_ERR_TIMEOUT;
    }
  }
  esp_err_t Status = ReadTemperature(rTemp_C);
  if (Status != ESP_OK)
    return Status;
  Status = ReadPressure(rPress_mbar);
  return Status;
}

bool BMP390::DataReady()
{
  uint8_t Status = 0;
  Status = Read8(BMP390_REGISTER_INT_STATUS);
  return Status & (1 << 3);
}

esp_err_t BMP390::ReadTemperature(double &rTemp_C)
{
  // Temperaturwert lesen
  esp_err_t Status;
  uint32_t adc_T;
  uint8_t adc[3];
  Status = ReadRegister(BMP390_REGISTER_TEMP_DATA_START, adc, sizeof(adc));
  if (Status != ESP_OK)
    return Status;
  adc_T = adc[0] + (adc[1] << 8) + (adc[2] << 16);
  //ESP_LOGI("BMP390", "ADC Temperatur: %lu.", adc_T);
  double partial_data1 = (double)(adc_T - PAR_T1);
  double partial_data2 = (double)(partial_data1 * PAR_T2);
  // Update the compensated temperature in calib structure since this is
  // needed for pressure calculation */
  t_lin = partial_data2 + (partial_data1 * partial_data1) * PAR_T3;
  rTemp_C = t_lin;

  return Status;
}

esp_err_t BMP390::ReadPressure(double &rPress_mbar)
{
  esp_err_t Status = ESP_OK;
  uint32_t adc_P;
  uint8_t adc[3];
  Status = ReadRegister(BMP390_REGISTER_PRESS_DATA_START, adc, sizeof(adc));
  if (Status != ESP_OK)
    return Status;
  adc_P = adc[0] + (adc[1] << 8) + (adc[2] << 16);
  //ESP_LOGI("BMP390", "ADC Druck: %lu.", adc_P);
  /* Temporary variables used for compensation */
  double partial_data1;
  double partial_data2;
  double partial_data3;
  double partial_data4;
  double partial_out1;
  double partial_out2;
  /* Calibration data */
  partial_data1 = PAR_P6 * t_lin;
  partial_data2 = PAR_P7 * (t_lin * t_lin);
  partial_data3 = PAR_P8 * (t_lin * t_lin * t_lin);
  partial_out1 = PAR_P5 + partial_data1 + partial_data2 + partial_data3;
  partial_data1 = PAR_P2 * t_lin;
  partial_data2 = PAR_P3 * (t_lin * t_lin);
  partial_data3 = PAR_P4 * (t_lin * t_lin * t_lin);
  partial_out2 = (double)adc_P * (PAR_P1 + partial_data1 + partial_data2 + partial_data3);
  partial_data1 = (double)adc_P * (double)adc_P;
  partial_data2 = PAR_P9 + PAR_P10 * t_lin;
  partial_data3 = partial_data1 * partial_data2;
  partial_data4 = partial_data3 + ((double)adc_P * (double)adc_P * (double)adc_P) * PAR_P11;
  rPress_mbar = (partial_out1 + partial_out2 + partial_data4) / 100.0; // Wird im Datenblatt in Pascal berechnet
  return Status;
}

esp_err_t BMP390::ReadAltitude(double &aAlt, double seaLevelhPa)
{
  double pressure;
  esp_err_t Status = ReadPressure(pressure); // in Si units for Pascal
  pressure /= 100;

  aAlt = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return Status;
}

double BMP390::SeaLevelForAltitude(double altitude, double atmospheric)
{
  // Equation taken from BMP180 datasheet (page 17):
  // http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  // http://forums.adafruit.com/viewtopic.php?f=22&t=58064
  return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

double BMP390::WaterBoilingPoint(double pressure)
{
  // Magnusformular for calculation of the boiling point of water at a given
  // pressure
  return (234.175 * log(pressure / 6.1078)) /
         (17.08085 - log(pressure / 6.1078));
}

void BMP390::Reset(void)
{
  WriteRegister(BMP390_REGISTER_CMD, BMP390_CMD_SOFTRESET);
  vTaskDelay(10 / portTICK_PERIOD_MS);
}

uint8_t BMP390::GetStatus(void)
{
  return Read8(BMP390_REGISTER_STATUS);
}
