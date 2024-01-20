#ifndef HAUPTWETTERSTATION_H
#define HAUPTWETTERSTATION_H

#include "lora.h"

// Erfasste Sensordaten
struct SensorData
{
  uint32_t PC; 
  float Temp_deg;
  float Hum_per; 
  float Press_mBar; 
  float WindSpeed_m_s; 
  uint16_t WindDir_deg;
  uint16_t Flashes_h;
  float Rain_mm_qm_h; 
  float Brightness_lux;
  bool HumDetected; 
  uint32_t SensorErrCount;
};

// Sensor-Fehlerzähler und Status
struct SensorStatus
{
  uint32_t SHT40Err;
  uint32_t BMP390Err;
  uint32_t ADS1015Err;
  uint32_t NokiaDisplayErr;

  bool SkipSHT40;
  bool SkipBMP390;
  bool SkipADS1015;
  bool SkipNokiaDisplay;
};

void app_main_cpp();
void task_tx(void *p);
void error(const char *format, ...);
void ParseLoraPacket(uint8_t *buf, uint8_t len);
esp_err_t InitLoRa(SX1278_LoRa &aLoRa);
int64_t GetTime_us();
void InitNokia_u8g2();
esp_err_t BuildCBORBuf(uint8_t *aBuf, uint16_t aMaxBufSize, uint16_t &aCBORBuildSize, const SensorData & aData);
esp_err_t InitI2C(i2c_port_t aPort, int aSDA_Pin, int aSCL_Pin);
esp_err_t InitGPIO();
static void IRAM_ATTR gpio_isr_handler(void *arg);
void GetSensorData(SensorData & aData, SensorStatus & aStatus);

#endif