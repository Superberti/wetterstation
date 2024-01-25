/*!
 * SHT35 Sensor
 */

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <stdint.h>
#include "ads1015.h"
#include <math.h>
#include <sys/time.h>
#include "esp_log.h"

#define ACK_CHECK_EN 0x1  /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0 /*!< I2C master will not check ack from slave */

ADS1015::ADS1015(i2c_port_t aPort, uint8_t aI2CAddr)
{
  mPort = aPort;
  mI2CAddr = aI2CAddr;
}

ADS1015::~ADS1015(void)
{
}

esp_err_t ADS1015::ConvReady(bool &aConvReady)
{
  esp_err_t ret;
  uint16_t Config = 0;
  ret = ReadRegister16(REG_CONFIG, Config);
  // ESP_LOGI("ADS1015", "Lese Config-Register: %d", (int)Config);
  aConvReady = (Config & (1 << 15)) > 0;
  return ret;
}

int64_t ADS1015::GetTime_us()
{
  struct timeval tv_now;
  gettimeofday(&tv_now, NULL);
  return (int64_t)tv_now.tv_sec * 1000000L + (int64_t)tv_now.tv_usec;
}

esp_err_t ADS1015::ReadADC(ADC_MP aInputMux, FSC_RANGE aFullScale, ADC_SPEED aSpeed, uint16_t &aADCValue)
{
  esp_err_t ret;
  
  // Init ADS1015 (die 0x3 am Ende schaltet den Komperator ab...)
  uint16_t Config = (1 << 15) | (aInputMux << 12) | (aFullScale << 9) | (1 << 8) | (aSpeed << 5) | 0x3;

  // Konfiguration schreiben und Konversion starten
  ret = WriteRegister16(REG_CONFIG, Config);
  if (ret != ESP_OK)
    return ret;

  int64_t StartTime = GetTime_us();

  bool cvr = false;

  // Achtung: Abweichend vom Datenblatt braucht der ADC im "Single-Shot-Modus"
  // deutlich länger, als im kontinuierlichen Modus. Bei 128_SPS sind das ca. 500 ms (und nicht 4 ms lt. DB)!
  // bei 250_SPS sind es ca. 250 ms usw.

  // Timeout nach spätestens 600 ms.Länger dauert kein Sample!
  while (GetTime_us() - StartTime < 5000000)
  {
    ret = ConvReady(cvr);
    if (ret != ESP_OK)
      return ret;
    if (cvr)
    {
      // Konversionsergebnis vom ADC lesen
      double TimePast = (GetTime_us() - StartTime) / 1.0E6;
      ESP_LOGI("ADS1015", "Zeit für ADC: %.3f s", TimePast);
      ret = ReadRegister16(REG_CONVERSION, aADCValue);
      aADCValue = (aADCValue >> 4); // Die vier ersten LSBs haben keine Bedeutung!
      return ret;
    }

    vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  return ESP_ERR_TIMEOUT;
}

esp_err_t ADS1015::ReadRegister16(ADC_REGISTER aReg, uint16_t &aRegVal)
{
  esp_err_t ret;
  uint8_t data[2] = {};
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, (mI2CAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, aReg, ACK_CHECK_EN);
  i2c_master_start(cmd); // I2C-Restart
  i2c_master_write_byte(cmd, (mI2CAddr << 1) | I2C_MASTER_READ, ACK_CHECK_EN);
  i2c_master_read(cmd, data, 2, I2C_MASTER_LAST_NACK);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(mPort, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  aRegVal = (data[0] << 8) | data[1];
  if (ret != ESP_OK)
    ESP_LOGE("ADS1015::ReadRegister", "I2C error no.: %d", ret);

  return ret;
}

esp_err_t ADS1015::WriteRegister16(ADC_REGISTER aReg, uint16_t aRegVal)
{
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
