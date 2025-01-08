/*!
 * Ansteuerung eines SHT35 Temperatur und Luftfeuchtigkeitssensor.
 * Umgesetzt auf ESP32, I2C-Interface
 */
#ifndef __SHT35_H__
#define __SHT35_H__

#include "driver/i2c_master.h"
#include "SensorTypes.h"

/*!
 *  I2C ADDRESS/BITS/SETTINGS
 */
#define SHT35_ADDR (0x44) /**< The default I2C address for the sensor. */
// Kein Clock-Stretching, hohe Genauigkeit. S. Datenblatt Seite 10
#define SHT35_CMD_START_MSB 0x24
#define SHT35_CMD_START_LSB 0x00


class SHT35
{
private:
  i2c_master_bus_handle_t mBusHandle;
  i2c_master_dev_handle_t mDevHandle;
  // CRC8-Berechnung
  static const uint8_t poly = 0x31; // x8 + x5 + x4 + 1

public:
  SHT35();
  esp_err_t Init(i2c_master_bus_handle_t aBusHandle, uint8_t aI2CAddr, uint32_t aI2CSpeed_Hz);
  ~SHT35(void);
  esp_err_t Read(float &aTemp, float &aHum);
  static uint8_t ComputeChecksum(uint8_t *bytes, int len);
  static uint8_t Crc8b(uint8_t aData);
};


#endif
