/*!
 * Ansteuerung eines SHT35 Temperatur und Luftfeuchtigkeitssensor.
 * Umgesetzt auf ESP32, I2C-Interface
 */
#ifndef __SHT35_H__
#define __SHT35_H__


/*!
 *  I2C ADDRESS/BITS/SETTINGS
 */
#define SHT35_ADDR (0x44) /**< The default I2C address for the sensor. */
// Kein Clock-Stretching, hohe Genauigkeit. S. Datenblatt Seite 10
#define SHT35_CMD_START_MSB 0x24
#define SHT35_CMD_START_LSB 0x00

class SHT35
{
public:

  SHT35(i2c_port_t aPort);

  ~SHT35(void);

  bool init();
  esp_err_t ReadSHT35(double & aTemp, double & aHum, bool & rCRC_Err);

private:
  i2c_port_t mPort;
  // CRC8-Berechnung
  const uint8_t poly = 0x31; // x8 + x5 + x4 + 1
  uint8_t ComputeChecksum(uint8_t* bytes, int len);
  uint8_t Crc8b(uint8_t aData);
};

#endif
