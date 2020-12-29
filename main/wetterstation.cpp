/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"
#include "sdkconfig.h"
#include "wetterstation.h"
#include "bmp280.h"

static const char *TAG = "Wetterstation";

#define _I2C_NUMBER(num) I2C_NUM_##num
#define I2C_NUMBER(num) _I2C_NUMBER(num)

#define I2C_MASTER_SCL_IO 19               /*!< gpio number for I2C master clock */
#define I2C_MASTER_SDA_IO 18               /*!< gpio number for I2C master data  */
#define I2C_MASTER_NUM I2C_NUMBER(0)       /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000          /*!< I2C master clock frequency */
#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */


#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
//#define ACK_VAL 0x0                             /*!< I2C ack value */
//#define NACK_VAL 0x1                            /*!< I2C nack value */
#define SHT35_ADDR 0x44   /* I2C-Adresse SHT35 */
#define BME280_ADDR 0x77  /* I2C-Adresse BME280 (SDO auf Vdd) */

// Kein Clock-Stretching, hohe Genauigkeit. S. Datenblatt Seite 10
#define SHT35_CMD_START_MSB 0x24
#define SHT35_CMD_START_LSB 0x00

#define BH1750_SENSOR_ADDR 0x23
// Hohe Genauigkeit, einmal messen
#define BH1750_CMD_START 0x23

extern "C"
{
  void app_main(void)
  {
    ESP_LOGI(TAG, "Starte ESP32-Wetterstation...");
    //uint8_t TestData[]= {0xBE,0xEF};
    //uint8_t crc=ComputeChecksum(TestData,sizeof(TestData));
    //ESP_LOGI(TAG, "CRC: 0x%x",crc);
    ESP_ERROR_CHECK(i2c_master_init());
    //ESP_LOGI(TAG, "Starte Hauptschleife...");
    int c=0;
    double temp=0,hum=0;
    uint8_t CrcErr=0;
    BMP280 bmp;
    bool bmp_init_ok = bmp.init();
    if (!bmp_init_ok)
      ESP_LOGE(TAG, "Fehler bei der Initialisierung vom BMP280!");
    double BMP_temp, BMP_pres, BMP_pres_raw;
    for(;;)
    {
      ESP_ERROR_CHECK(ReadSHT35(&temp, &hum, &CrcErr));
      if (CrcErr)
        printf("WARNUNG: CRC-Datenfehler aufgetreten, Daten könnten falsch sein.\r\n");
      printf("SHT35 Temperatur: %.2f°C Luftfeuchte: %.2f %%\r\n",temp,hum);
      if (bmp_init_ok)
      {
        BMP_temp=bmp.ReadTemperature();
        BMP_pres_raw=bmp.ReadPressure();
        BMP_pres=bmp.seaLevelForAltitude(210, BMP_pres_raw);
        printf("BMP280 Temperatur: %.2f°C Luftdruck: %.3f bar [raw: %.3f]\r\n",BMP_temp,BMP_pres/100000,BMP_pres_raw/100000);
      }

      vTaskDelay(5000 / portTICK_RATE_MS);

      c++;
    }
  }
}

esp_err_t ReadBH1750(double* aLux)
{
  uint8_t data_h, data_l;
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, BH1750_CMD_START, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (ret != ESP_OK)
  {
    return ret;
  }
  vTaskDelay(30 / portTICK_RATE_MS);
  cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | I2C_MASTER_READ, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, &data_h, I2C_MASTER_ACK);
  i2c_master_read_byte(cmd, &data_l, I2C_MASTER_NACK);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);
  if (aLux!=NULL)
    *aLux=(data_h << 8 | data_l) / 1.2;

  return ret;
}

// CRC8-Berechnung
const uint8_t poly = 0x31; // x8 + x5 + x4 + 1

uint8_t ComputeChecksum(uint8_t* bytes, int len)
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

// SHT35-Sensor
esp_err_t ReadSHT35(double * aTemp, double * aHum, uint8_t*rCRC_Err)
{
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, SHT35_ADDR << 1 | I2C_MASTER_WRITE, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, SHT35_CMD_START_MSB, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, SHT35_CMD_START_LSB, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
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
  ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, 1000 / portTICK_RATE_MS);
  i2c_cmd_link_delete(cmd);

  *aTemp=-45+175*(double)(rb[0]*256+rb[1])/65535.0;
  *aHum=100.0*(double)(rb[3]*256+rb[4])/65535.0;
  //disp_buf(rb,6);
  // Checksumme Temperatur
  uint8_t crc=ComputeChecksum(rb,2);
  if (crc!=rb[2] && rCRC_Err!=NULL)
  {
    printf("Falscher Temperatur-CRC: [0x%x] <> [0x%x]\r\n",crc,rb[2]);
    *rCRC_Err=1;
  }

  // Checksumme Luftfeuchte
  crc=ComputeChecksum(rb+3,2);
  if (crc!=rb[5] && rCRC_Err!=NULL)
  {
    printf("Falscher Luftfeuchte-CRC: [0x%x] <> [0x%x]\r\n",crc,rb[5]);
    *rCRC_Err=1;
  }
  if (rCRC_Err!=NULL && *rCRC_Err)
    disp_buf(rb,6);
  return ret;
}

/**
 * @brief i2c master initialization
 */
esp_err_t i2c_master_init(void)
{
  int i2c_master_port = I2C_MASTER_NUM;
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;
  conf.sda_io_num = I2C_MASTER_SDA_IO;
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
  conf.scl_io_num = I2C_MASTER_SCL_IO;
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
  conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
  i2c_param_config(i2c_master_port, &conf);
  return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
}


void disp_buf(uint8_t *buf, int len)
{
  int i;
  for (i = 0; i < len; i++)
  {
    printf("%02x ", buf[i]);
    if ((i + 1) % 16 == 0)
    {
      printf("\n");
    }
  }
  printf("\n");
}


