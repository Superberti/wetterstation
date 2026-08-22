/*!
 * Ansteuerung eines SHT40 Temperatur und Luftfeuchtigkeitssensor.
 * Umgesetzt auf ESP32, I2C-Interface
 */
#ifndef __SHT40_H__
#define __SHT40_H__

#include "driver/i2c_master.h"
#include "SensorTypes.h"

class SHT40
{
private:
  SHT40_COMMAND mReadMode;
  i2c_master_bus_handle_t mBusHandle;
  i2c_master_dev_handle_t mDevHandle;
  // CRC8-Berechnung
  static const uint8_t poly = 0x31; // x8 + x5 + x4 + 1

public:
  SHT40();
  esp_err_t Init(i2c_master_bus_handle_t aBusHandle, uint8_t aI2CAddr, uint32_t aI2CSpeed_Hz, SHT40_COMMAND aReadMode = SHT40_CMD_HPM);
  ~SHT40(void);
  esp_err_t Read(float &aTemp, float &aHum);
  esp_err_t ReadSerial(uint32_t &aSerialNo);
  static uint8_t ComputeChecksum(uint8_t *bytes, int len);
  static uint8_t Crc8b(uint8_t aData);
};

#endif
