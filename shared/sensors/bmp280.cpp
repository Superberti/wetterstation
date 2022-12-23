/*!
 * BMP280 Sensor
 */

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <stdint.h>
#include "bmp280.h"
#include <math.h>

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */

/*!
 * @brief  BMP280 constructor using i2c
 * @param  *theWire
 *         optional wire
 */
BMP280::BMP280()
{

}

BMP280::~BMP280(void)
{

}

/*!
 *  Initialises the sensor.
 *  @param addr
 *         The I2C address to use (default = 0x77)
 *  @param chipid
 *         The expected chip ID (used to validate connection).
 *  @return True if the init was successful, otherwise false.
 */
bool BMP280::init(uint8_t addr, uint8_t chipid)
{
  _i2caddr = addr;
  esp_err_t err;
  if (read8(BMP280_REGISTER_CHIPID,&err) != chipid)
    return false;

  readCoefficients();
  // WriteRegister(BMP280_REGISTER_CONTROL, 0x3F); /* needed? */
  setSampling();
  vTaskDelay(100 / portTICK_RATE_MS);
  return true;
}

/*!
 * Sets the sampling config for the device.
 * @param mode
 *        The operating mode of the sensor.
 * @param tempSampling
 *        The sampling scheme for temp readings.
 * @param pressSampling
 *        The sampling scheme for pressure readings.
 * @param filter
 *        The filtering mode to apply (if any).
 * @param duration
 *        The sampling duration.
 */
void BMP280::setSampling(sensor_mode mode,
                         sensor_sampling tempSampling,
                         sensor_sampling pressSampling,
                         sensor_filter filter,
                         standby_duration duration)
{
  _measReg.mode = mode;
  _measReg.osrs_t = tempSampling;
  _measReg.osrs_p = pressSampling;

  _configReg.filter = filter;
  _configReg.t_sb = duration;

  WriteRegister(BMP280_REGISTER_CONFIG, _configReg.get());
  WriteRegister(BMP280_REGISTER_CONTROL, _measReg.get());
}


/**************************************************************************/
/*!
    @brief  Writes an 8 bit value over I2C/SPI
*/
/**************************************************************************/
esp_err_t BMP280::WriteRegister(uint8_t reg_addr, uint8_t value)
{
  esp_err_t ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, _i2caddr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, value, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret!=ESP_OK)
    ESP_LOGE("BMP280::WriteRegister", "I2C error no.: %d",ret);
  return ret;
}

esp_err_t BMP280::ReadRegister(uint8_t reg_addr, uint8_t *data, uint16_t len)
{
  esp_err_t ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, _i2caddr << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, reg_addr, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    ESP_LOGE("BMP280::ReadRegister", "I2C error no.: %d",ret);
    return ret;
  }
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, _i2caddr << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
  i2c_master_read(cmd, data, len,  I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_NUM_0, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret!=ESP_OK)
    ESP_LOGE("BMP280::ReadRegister", "I2C error no.: %d",ret);
  return ret;
}

/*!
 *  @brief  Reads an 8 bit value over I2C/SPI
 *  @param  reg
 *          selected register
 *  @return value from selected register
 */
uint8_t BMP280::read8(uint8_t reg, esp_err_t* rError)
{
  uint8_t value;
  esp_err_t res=ReadRegister(reg, &value, sizeof(value));
  if (rError)
    *rError=res;
  return value;
}

/*!
 *  @brief  Reads a 16 bit value over I2C/SPI
 */
uint16_t BMP280::read16(uint8_t reg, esp_err_t* rError)
{
  uint16_t value;
  esp_err_t res=ReadRegister(reg, (uint8_t*)&value, sizeof(value));
  if (rError)
    *rError=res;
  return value;
}

/*!
 *  @brief  Reads a 24 bit value over I2C/SPI
 */
uint32_t BMP280::read24(uint8_t reg, esp_err_t* rError)
{
  uint8_t tmp[3];
  esp_err_t res=ReadRegister(reg, tmp, sizeof(tmp));
  if (rError)
    *rError=res;
  return (tmp[0]<<16) | (tmp[1]<<8) | tmp[2];
}


/*!
 *  @brief  Reads the factory-set coefficients
 */
void BMP280::readCoefficients()
{
  _bmp280_calib.dig_T1 = read16(BMP280_REGISTER_DIG_T1);
  _bmp280_calib.dig_T2 = (int16_t)read16(BMP280_REGISTER_DIG_T2);
  _bmp280_calib.dig_T3 = (int16_t)read16(BMP280_REGISTER_DIG_T3);

  _bmp280_calib.dig_P1 = read16(BMP280_REGISTER_DIG_P1);
  _bmp280_calib.dig_P2 = (int16_t)read16(BMP280_REGISTER_DIG_P2);
  _bmp280_calib.dig_P3 = (int16_t)read16(BMP280_REGISTER_DIG_P3);
  _bmp280_calib.dig_P4 = (int16_t)read16(BMP280_REGISTER_DIG_P4);
  _bmp280_calib.dig_P5 = (int16_t)read16(BMP280_REGISTER_DIG_P5);
  _bmp280_calib.dig_P6 = (int16_t)read16(BMP280_REGISTER_DIG_P6);
  _bmp280_calib.dig_P7 = (int16_t)read16(BMP280_REGISTER_DIG_P7);
  _bmp280_calib.dig_P8 = (int16_t)read16(BMP280_REGISTER_DIG_P8);
  _bmp280_calib.dig_P9 = (int16_t)read16(BMP280_REGISTER_DIG_P9);
}

/*!
 * Reads the temperature from the device.
 * @return The temperature in degress celcius.
 */
esp_err_t BMP280::ReadTemperature(double & aTemp)
{
  int32_t var1, var2;
  esp_err_t Status=ESP_OK;
  int32_t adc_T = read24(BMP280_REGISTER_TEMPDATA, &Status);
  adc_T >>= 4;

  var1 = ((((adc_T >> 3) - ((int32_t)_bmp280_calib.dig_T1 << 1))) *
          ((int32_t)_bmp280_calib.dig_T2)) >>
         11;

  var2 = (((((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1)) *
            ((adc_T >> 4) - ((int32_t)_bmp280_calib.dig_T1))) >>
           12) *
          ((int32_t)_bmp280_calib.dig_T3)) >>
         14;

  t_fine = var1 + var2;

  double T = (t_fine * 5 + 128) >> 8;
  aTemp = T / 100;
  return Status;
}

/*!
 * Reads the barometric pressure from the device.
 * @return Barometric pressure in Pa.
 */
esp_err_t BMP280::ReadPressure(double & aPress)
{
  esp_err_t Status=ESP_OK;
  int64_t var1, var2, p;

  // Must be done first to get the t_fine variable set up
  double dum;
  Status=ReadTemperature(dum);
  if (Status!=ESP_OK)
    return Status;

  int32_t adc_P = read24(BMP280_REGISTER_PRESSUREDATA, &Status);
  adc_P >>= 4;

  var1 = ((int64_t)t_fine) - 128000;
  var2 = var1 * var1 * (int64_t)_bmp280_calib.dig_P6;
  var2 = var2 + ((var1 * (int64_t)_bmp280_calib.dig_P5) << 17);
  var2 = var2 + (((int64_t)_bmp280_calib.dig_P4) << 35);
  var1 = ((var1 * var1 * (int64_t)_bmp280_calib.dig_P3) >> 8) +
         ((var1 * (int64_t)_bmp280_calib.dig_P2) << 12);
  var1 =
    (((((int64_t)1) << 47) + var1)) * ((int64_t)_bmp280_calib.dig_P1) >> 33;

  if (var1 == 0)
  {
    return 0; // avoid exception caused by division by zero
  }
  p = 1048576 - adc_P;
  p = (((p << 31) - var2) * 3125) / var1;
  var1 = (((int64_t)_bmp280_calib.dig_P9) * (p >> 13) * (p >> 13)) >> 25;
  var2 = (((int64_t)_bmp280_calib.dig_P8) * p) >> 19;

  p = ((p + var1 + var2) >> 8) + (((int64_t)_bmp280_calib.dig_P7) << 4);
  aPress = (double)p / 256;
  return Status;
}

/*!
 * @brief Calculates the approximate altitude using barometric pressure and the
 * supplied sea level hPa as a reference.
 * @param seaLevelhPa
 *        The current hPa at sea level.
 * @return The approximate altitude above sea level in meters.
 */
esp_err_t BMP280::readAltitude(double & aAlt, double seaLevelhPa)
{
  double pressure;
  esp_err_t Status = ReadPressure(pressure); // in Si units for Pascal
  pressure /= 100;

  aAlt = 44330 * (1.0 - pow(pressure / seaLevelhPa, 0.1903));

  return Status;
}

/*!
 * Calculates the pressure at sea level (QFH) from the specified altitude,
 * and atmospheric pressure (QFE).
 * @param  altitude      Altitude in m
 * @param  atmospheric   Atmospheric pressure in hPa
 * @return The approximate pressure in hPa
 */
double BMP280::seaLevelForAltitude(double altitude, double atmospheric)
{
  // Equation taken from BMP180 datasheet (page 17):
  // http://www.adafruit.com/datasheets/BST-BMP180-DS000-09.pdf

  // Note that using the equation from wikipedia can give bad results
  // at high altitude.  See this thread for more information:
  // http://forums.adafruit.com/viewtopic.php?f=22&t=58064
  return atmospheric / pow(1.0 - (altitude / 44330.0), 5.255);
}

/*!
    @brief  calculates the boiling point  of water by a given pressure
    @param pressure pressure in hPa
    @return temperature in °C
*/

double BMP280::waterBoilingPoint(double pressure)
{
  // Magnusformular for calculation of the boiling point of water at a given
  // pressure
  return (234.175 * log(pressure / 6.1078)) /
         (17.08085 - log(pressure / 6.1078));
}

/*!
 *  @brief  Take a new measurement (only possible in forced mode)
 *  !!!todo!!!
 */


/*!
 *  @brief  Resets the chip via soft reset
 */
void BMP280::reset(void)
{
  WriteRegister(BMP280_REGISTER_SOFTRESET, MODE_SOFT_RESET_CODE);
}

/*!
    @brief  Gets the most recent sensor event from the hardware status register.
    @return Sensor status as a byte.
 */
uint8_t BMP280::getStatus(void)
{
  return read8(BMP280_REGISTER_STATUS);
}



