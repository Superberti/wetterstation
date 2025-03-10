/*!
 * VEML7700 Sensor
 */

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <stdint.h>
#include "veml7700.h"
#include <math.h>

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */


VEML7700::VEML7700()
  : mGain(VEML7700_GAIN_1)
  , mIntegrationTime(VEML7700_IT_100MS)
{

}

VEML7700::~VEML7700(void)
{

}

uint16_t VEML7700::swapByteOrder(uint16_t us)
{
  return (us >> 8) | (us << 8);
}

uint32_t VEML7700::swapByteOrder(uint32_t ui)
{
  return  (ui >> 24) | ((ui<<8) & 0x00FF0000) | ((ui>>8) & 0x0000FF00) | (ui << 24);
}

bool VEML7700::init(uint8_t aGain, uint8_t aIntegrationTime)
{
  mGain=aGain;
  mIntegrationTime=aIntegrationTime;
  ConfigRegister cr;
  cr.val=0;
  cr.bits.ALS_GAIN=aGain;
  cr.bits.ALS_IT=aIntegrationTime;
  cr.bits.ALS_INT_ENT=0;
  cr.bits.ALS_PERS=VEML7700_PERS_1;
  cr.bits.ALS_SD=0;
  //ESP_LOGI("VEML7700::init", "Schreibe: %d",cr.val);
  esp_err_t ret=WriteRegister(VEML7700_ALS_CONFIG,cr.val);
  ret=ReadRegister(VEML7700_ALS_CONFIG,(uint8_t*)&cr.val,sizeof(cr));
  //ESP_LOGI("VEML7700::init", "Lese: %d",cr.val);
  vTaskDelay(3 / portTICK_RATE_MS);
  return ret==ESP_OK;
}


esp_err_t VEML7700::WriteRegister(uint8_t reg_addr, uint16_t value)
{
  //uint8_t* pData=(uint8_t*)&value;
  esp_err_t ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, VEML7700_ADDRESS << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
  // VEML will zuerst das LSB
  i2c_master_write_byte(cmd, value & 0xff, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, value >> 8, ACK_CHECK_EN);

  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret!=ESP_OK)
    ESP_LOGE("VEML7700::WriteRegister", "I2C error no.: %d",ret);
  return ret;
}

esp_err_t VEML7700::ReadRegister(uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  esp_err_t ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, VEML7700_ADDRESS << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, VEML7700_ADDRESS << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
  i2c_master_read(cmd, data, len,  I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    ESP_LOGE("VEML7700::ReadRegister", "I2C error no.: %d",ret);
  }
  return ret;

}



double VEML7700::normalize_resolution(double value)
{
  // adjust for gain (1x is normalized)
  switch (mGain)
  {
  case VEML7700_GAIN_2:
    value /= 2.0;
    break;
  case VEML7700_GAIN_1_4:
    value *= 4;
    break;
  case VEML7700_GAIN_1_8:
    value *= 8;
    break;
  }

  // adjust for integrationtime (100ms is normalized)
  switch (mIntegrationTime)
  {
  case VEML7700_IT_25MS:
    value *= 4;
    break;
  case VEML7700_IT_50MS:
    value *= 2;
    break;
  case VEML7700_IT_200MS:
    value /= 2.0;
    break;
  case VEML7700_IT_400MS:
    value /= 4.0;
    break;
  case VEML7700_IT_800MS:
    value /= 8.0;
    break;
  }

  return value;
}

/*!
 *    @brief Read the calibrated lux value. See app note lux table on page 5
 *    @returns Floating point Lux data (ALS multiplied by 0.0576)
 */
esp_err_t VEML7700::readLux(double & aLux)
{
  uint16_t als;
  esp_err_t Status=readALS(als);
  //printf("ALS: %d\r\n",als);
  aLux = (normalize_resolution(als) * 0.0576); // see app note lux table on page 5
  return Status;
}

/*!
 *    @brief Read the lux value with correction for non-linearity at high-lux
 * settings
 *    @returns Floating point Lux data (ALS multiplied by 0.0576 and corrected
 * for high-lux settings)
 */
esp_err_t VEML7700::readLuxNormalized(double & aLux)
{
  double lux;
  esp_err_t Status = readLux(lux);

  // user-provided correction for non-linearities at high lux/white values:
  // https://forums.adafruit.com/viewtopic.php?f=19&t=152997&p=758582#p759346
  if ((mGain == VEML7700_GAIN_1_8) && (mIntegrationTime == VEML7700_IT_25MS))
  {
    lux = 6.0135e-13 * pow(lux, 4) - 9.3924e-9 * pow(lux, 3) +
          8.1488e-5 * pow(lux, 2) + 1.0023 * lux;
  }

  aLux = lux;
  return Status;
}

/*!
 *    @brief Read the raw ALS data
 *    @returns 16-bit data value from the ALS register
 */
esp_err_t VEML7700::readALS(uint16_t & aALS)
{
  return ReadRegister(VEML7700_ALS_DATA,(uint8_t*)&aALS,sizeof(aALS));
}

/*!
 *    @brief Read the white light data
 *    @returns Floating point 'white light' data multiplied by 0.0576
 */
esp_err_t VEML7700::readWhite(double & aWhite)
{
  // white_corrected= 2E-15*pow(VEML_white,4) + 4E-12*pow(VEML_white,3) +
  // 9E-06*pow(VEML_white,)2 + 1.0179*VEML_white - 11.052;
  uint16_t iALS;
  esp_err_t Status=ReadRegister(VEML7700_WHITE_DATA,(uint8_t*)&iALS,sizeof(iALS));
  aWhite = normalize_resolution(iALS) * 0.0576; // Unclear if this is the right multiplier
  return Status;
}

/*!
 *    @brief Read the 'white light' value with correction for non-linearity at
 * high-lux settings
 *    @returns Floating point 'white light' data multiplied by 0.0576 and
 * corrected for high-lux settings
 */
esp_err_t VEML7700::readWhiteNormalized(double & aWhite)
{
  double white;
  esp_err_t Status = readWhite(white);

  // user-provided correction for non-linearities at high lux values:
  // https://forums.adafruit.com/viewtopic.php?f=19&t=152997&p=758582#p759346
  if ((mGain == VEML7700_GAIN_1_8) && (mIntegrationTime == VEML7700_IT_25MS))
  {
    white = 2E-15 * pow(white, 4) + 4E-12 * pow(white, 3) +
            9E-06 * pow(white, 2) + 1.0179 * white - 11.052;
  }

  aWhite = white;
  return Status;
}







