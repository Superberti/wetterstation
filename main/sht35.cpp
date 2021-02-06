/*!
 * SHT35 Sensor
 */

#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include <stdint.h>
#include "sht35.h"
#include <math.h>

#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */

SHT35::SHT35(int aPort)
{
  mPort=aPort;
}

SHT35::~SHT35(void)
{

}

uint8_t SHT35::ComputeChecksum(uint8_t* bytes, int len)
{
  uint8_t crc = 0xff;// Startwert

  if (bytes != NULL && len > 0)
  {
    for (int i=0; i<len; i++)
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

bool SHT35::init()
{
  double t,h;
  bool crc_err;
  esp_err_t res=ReadSHT35(t,h,crc_err);
  if (res!=ESP_OK || crc_err)
    return false;
  return true;
}

// SHT35-Sensor
esp_err_t SHT35::ReadSHT35(double & aTemp, double & aHum, bool & rCRC_Err)
{
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, SHT35_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, SHT35_CMD_START_MSB, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, SHT35_CMD_START_LSB, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(mPort, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    return ret;
  }
  vTaskDelay(100 / portTICK_RATE_MS);
  // Belegung Datenblock in bytes:
  // Temperatur_high, Temperatur_low, Temperatur_crc, Luftfeuchte_high, Luftfeuchte_low, Luftfeuchte_crc
  uint8_t rb[6]= {0};
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, SHT35_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);

  i2c_master_read(cmd,rb,sizeof(rb)-1,I2C_MASTER_ACK);
  i2c_master_read_byte(cmd,rb+5,I2C_MASTER_NACK);  // Beim letzten Byte gibt's ein NACK


  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(mPort, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  aTemp=-45+175*(double)(rb[0]*256+rb[1])/65535.0;
  aHum=100.0*(double)(rb[3]*256+rb[4])/65535.0;
  //disp_buf(rb,6);
  // Checksumme Temperatur
  uint8_t crc=ComputeChecksum(rb,2);
  if (crc!=rb[2])
  {
    ESP_LOGE("ReadSHT35:","Falscher Temperatur-CRC: [0x%x] <> [0x%x]\r\n",crc,rb[2]);
    rCRC_Err=1;
  }

  // Checksumme Luftfeuchte
  crc=ComputeChecksum(rb+3,2);
  if (crc!=rb[5])
  {
    ESP_LOGE("ReadSHT35:","Falscher Luftfeuchte-CRC: [0x%x] <> [0x%x]\r\n",crc,rb[5]);
    rCRC_Err=1;
  }

  return ret;
}

