/*
 * BMP390 Sensor
 */

#include <stdio.h>
#include "esp_log.h"
#include <stdint.h>
#include "bmp390.h"
#include <math.h>
#include "freertos/FreeRTOS.h"

#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */
#define DEV_TIMEOUT 100

BMP390::BMP390()
{
}

BMP390::~BMP390(void)
{
  if (mDevHandle != NULL)
    i2c_master_bus_rm_device(mDevHandle);
}

esp_err_t BMP390::Init(i2c_master_bus_handle_t aBusHandle, uint8_t aI2CAddr, uint32_t aI2CSpeed_Hz)
{
  esp_err_t ret = ESP_OK;
  mBusHandle = aBusHandle;
  i2c_device_config_t conf;
  conf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  conf.device_address = aI2CAddr;
  conf.scl_speed_hz = aI2CSpeed_Hz;
  conf.flags.disable_ack_check=false;
  ret = i2c_master_bus_add_device(mBusHandle, &conf, &mDevHandle);
  if (ret != ESP_OK)
    return ret;

  uint8_t ChipID = Read8(BMP390_REGISTER_CHIP_ID, &ret);
  if (ret != ESP_OK)
    return ret;

  if (ChipID != BMP390_CHIP_ID)
  {
    ESP_LOGE("BMP390::Init", "Unbekannte Chip-ID: %d", ChipID);
    return ESP_ERR_INVALID_VERSION;
  }
  Reset();
  ret = ReadCalibData();
  if (ret != ESP_OK)
    return ret;

  //  Oversampling setzen. Bit 0..2: Druck, Bit 3..5 Temperatur
  return WriteRegister(BMP390_REGISTER_OSR, (SAMPLING_X2 << 3) | SAMPLING_X32);
}

esp_err_t BMP390::WriteRegister(uint8_t reg_addr, uint8_t value)
{
  esp_err_t ret;
  uint8_t data[2] = {reg_addr, value};
  ret = i2c_master_transmit(mDevHandle, data, sizeof(data), DEV_TIMEOUT);
  if (ret != ESP_OK)
    ESP_LOGE("BMP390::WriteRegister", "I2C error no.: %d", ret);
  return ret;
}

esp_err_t BMP390::ReadRegister(uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  esp_err_t ret;
  uint8_t RegAddr = reg_addr;
  ret = i2c_master_transmit_receive(mDevHandle, &RegAddr, 1, data, len, DEV_TIMEOUT);
  if (ret != ESP_OK)
    ESP_LOGE("BMP390::ReadRegister", "I2C error no.: %d", ret);

  return ret;
}

uint8_t BMP390::Read8(uint8_t reg, esp_err_t *rError)
{
  uint8_t value=0;
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

esp_err_t BMP390::ReadTempAndPress(float &rTemp_C, float &rPress_mbar)
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

esp_err_t BMP390::ReadTempAndPressAsync(float &rTemp_C, float &rPress_mbar, bool aWaitForData)
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
      vTaskDelay(pdMS_TO_TICKS(1));
      if (tc > 500)
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

esp_err_t BMP390::ReadTemperature(float &rTemp_C)
{
  // Temperaturwert lesen
  esp_err_t Status;
  uint32_t adc_T;
  uint8_t adc[3];
  Status = ReadRegister(BMP390_REGISTER_TEMP_DATA_START, adc, sizeof(adc));
  if (Status != ESP_OK)
    return Status;
  adc_T = adc[0] + (adc[1] << 8) + (adc[2] << 16);
  // ESP_LOGI("BMP390", "ADC Temperatur: %lu.", adc_T);
  float partial_data1 = (float)(adc_T - PAR_T1);
  float partial_data2 = (float)(partial_data1 * PAR_T2);
  // Update the compensated temperature in calib structure since this is
  // needed for pressure calculation */
  t_lin = partial_data2 + (partial_data1 * partial_data1) * PAR_T3;
  rTemp_C = t_lin;

  return Status;
}

esp_err_t BMP390::ReadPressure(float &rPress_mbar)
{
  esp_err_t Status = ESP_OK;
  uint32_t adc_P;
  uint8_t adc[3];
  Status = ReadRegister(BMP390_REGISTER_PRESS_DATA_START, adc, sizeof(adc));
  if (Status != ESP_OK)
    return Status;
  adc_P = adc[0] + (adc[1] << 8) + (adc[2] << 16);
  // ESP_LOGI("BMP390", "ADC Druck: %lu.", adc_P);
  /* Temporary variables used for compensation */
  float partial_data1;
  float partial_data2;
  float partial_data3;
  float partial_data4;
  float partial_out1;
  float partial_out2;
  /* Calibration data */
  partial_data1 = PAR_P6 * t_lin;
  partial_data2 = PAR_P7 * (t_lin * t_lin);
  partial_data3 = PAR_P8 * (t_lin * t_lin * t_lin);
  partial_out1 = PAR_P5 + partial_data1 + partial_data2 + partial_data3;
  partial_data1 = PAR_P2 * t_lin;
  partial_data2 = PAR_P3 * (t_lin * t_lin);
  partial_data3 = PAR_P4 * (t_lin * t_lin * t_lin);
  partial_out2 = (float)adc_P * (PAR_P1 + partial_data1 + partial_data2 + partial_data3);
  partial_data1 = (float)adc_P * (float)adc_P;
  partial_data2 = PAR_P9 + PAR_P10 * t_lin;
  partial_data3 = partial_data1 * partial_data2;
  partial_data4 = partial_data3 + ((float)adc_P * (float)adc_P * (float)adc_P) * PAR_P11;
  rPress_mbar = (partial_out1 + partial_out2 + partial_data4) / 100.0; // Wird im Datenblatt in Pascal berechnet
  return Status;
}

esp_err_t BMP390::ReadAltitude(float &aAlt, float seaLevelhPa)
{
  float pressure;
  esp_err_t Status = ReadPressure(pressure); // in Si units for Pascal
  pressure /= 100;

  aAlt = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return Status;
}

float BMP390::SeaLevelForAltitude(float altitude, float atmospheric)
{
  // Equation taken from BMP180 datasheet (page 17):
  // http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  // http://forums.adafruit.com/viewtopic.php?f=22&t=58064
  return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

float BMP390::WaterBoilingPoint(float pressure)
{
  // Magnusformular for calculation of the boiling point of water at a given
  // pressure
  return (234.175 * log(pressure / 6.1078)) /
         (17.08085 - log(pressure / 6.1078));
}

void BMP390::Reset(void)
{
  WriteRegister(BMP390_REGISTER_CMD, BMP390_CMD_SOFTRESET);
  vTaskDelay(pdMS_TO_TICKS(1));
}

uint8_t BMP390::GetStatus(void)
{
  return Read8(BMP390_REGISTER_STATUS);
}
