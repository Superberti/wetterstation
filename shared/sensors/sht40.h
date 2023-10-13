/*!
 * Ansteuerung eines SHT40 Temperatur und Luftfeuchtigkeitssensor.
 * Umgesetzt auf ESP32, I2C-Interface
 */
#ifndef __SHT40_H__
#define __SHT40_H__

#include "driver/i2c.h"
/*!
 *  I2C ADDRESS/BITS/SETTINGS
 */
#define SHT40_ADDR (0x44) // Standard I2C-Adresse

#define DELAY_LPM_MS 2
#define DELAY_MPM_MS 5
#define DELAY_HPM_MS 10

// Kommandos s. Datenblatt Seite 12
enum SHT40_COMMAND
{
  // Hohe Präzision, 20 mW Heater, 100 ms on-time
  SHT40_CMD_HPM_H_20_100=0x15,

  // Hohe Präzision, 20 mW Heater, 1000 ms on-time
  SHT40_CMD_HPM_H_20_1000=0x1E,

  // Hohe Präzision, 110 mW Heater, 100 ms on-time
  SHT40_CMD_HPM_H_110_100=0x24,

  // Hohe Präzision, 110 mW Heater, 1000 ms on-time
  SHT40_CMD_HPM_H_110_1000=0x2F,

  // Hohe Präzision, 200 mW Heater, 100 ms on-time
  SHT40_CMD_HPM_H_200_100=0x32,

  // Hohe Präzision, 200 mW Heater, 1000 ms on-time
  SHT40_CMD_HPM_H_200_1000=0x39,

  // Seriennummer auslesen
  SHT40_CMD_READ_SERIAL=0x89,

  // Reset
  SHT40_CMD_SOFT_RESET=0x95,

  // Niedrige Präzision, kein Heater, Messzeit=1,6 ms
  SHT40_CMD_LPM=0xE0,

  // Mittlere Präzision, kein Heater, Messzeit=4,5 ms
  SHT40_CMD_MPM=0xF6,
  
  // Hohe Präzision, kein Heater, Messzeit=8,3 ms
  SHT40_CMD_HPM=0xFD,

};

class SHT40
{
private:
  SHT40_COMMAND mReadMode;
  i2c_port_t mPort;
  // CRC8-Berechnung
  const uint8_t poly = 0x31; // x8 + x5 + x4 + 1
  uint8_t ComputeChecksum(uint8_t* bytes, int len);
  uint8_t Crc8b(uint8_t aData);
  int mSDA_Pin ;
  int mSCL_Pin;
public:

  SHT40(i2c_port_t aPort, int aSDA_Pin, int aSCL_Pin, SHT40_COMMAND aReadMode=SHT40_CMD_HPM);
  ~SHT40(void);

  esp_err_t Init(bool aDoI2CInit);
  void Close();
  esp_err_t Read(float & aTemp, float & aHum, bool & rCRC_Err);
  esp_err_t ReadSerial(uint32_t & aSerialNo, bool &rCRC_Err);

};

#endif
