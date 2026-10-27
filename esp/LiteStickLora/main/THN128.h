

#ifndef THN128_H_
#define THN128_H_

#include <stdbool.h>
#include <stdint.h>

/* Timing micro's in micro seconds */
#define T_RX_TOLERANCE_US 400
#define T_PREAMBLE_SPACE_US 3000
#define T_SYNC_US 5500
#define T_BIT_US 1450
#define T_SPACE_FRAMES_MS 100

#define T_SYNC_H_MIN (T_SYNC_US - T_RX_TOLERANCE_US)
#define T_SYNC_H_MAX (T_SYNC_US + T_RX_TOLERANCE_US)

#define T_SYNC_L_MIN_0 (T_SYNC_US + T_BIT_US - T_RX_TOLERANCE_US)
#define T_SYNC_L_MAX_0 (T_SYNC_US + T_BIT_US + T_RX_TOLERANCE_US)
#define T_SYNC_L_MIN_1 (T_SYNC_US - T_RX_TOLERANCE_US)
#define T_SYNC_L_MAX_1 (T_SYNC_US + T_RX_TOLERANCE_US)

#define T_BIT_SHORT_MIN (T_BIT_US - T_RX_TOLERANCE_US)
#define T_BIT_SHORT_MAX (T_BIT_US + T_RX_TOLERANCE_US)
#define T_BIT_LONG_MIN ((T_BIT_US * 2) - T_RX_TOLERANCE_US)
#define T_BIT_LONG_MAX ((T_BIT_US * 2) + T_RX_TOLERANCE_US)

struct OregonTHN128Data_t
{
  uint32_t rawData;       /*!< Raw data */
  uint8_t rollingAddress; /*!< Rolling address */
  uint8_t channel;        /*!< Channel */
  float temperature;      /*!< Temperature in degrees */
  bool lowBattery;        /*!< Low battery indication */
};

struct OregonTHN128Data2_t
{
  uint32_t rawData;       /*!< Raw data */
  uint8_t rollingAddress; /*!< Rolling address */
  uint8_t channel;        /*!< Channel */
  int16_t temperature;      /*!< Temperature in degrees */
  bool lowBattery;        /*!< Low battery indication */
};

struct OregonV1Bits 
{
	// 32-Bit Wortbreite der gesamten Übertragung
	uint8_t crc;	// 8 Bit crc
	uint8_t NotUsed : 1;
	uint8_t NegativeFlag : 1;	// Temperatur ist negativ
	uint8_t Unknown : 1;
	uint8_t BatteryLow : 1;		// Batterie leer
	uint8_t Temp_10_Deg : 4;	// 10 Grad
	uint8_t Temp_1_Deg : 4;		// 1 Grad
	uint8_t Temp_01_Deg : 4;	// 1/10 Grad
	uint8_t Channel : 4;			// Kanal: 1=0, 2=4, 3=8
	uint8_t RollingCode : 4;	// Zufallscode zur Senderfestlegung
};

union OregonV1Data
{
	struct OregonV1Bits Bitfield;
	uint32_t RawData;
};

class THN128
{
protected:
  uint8_t calcCrc(uint32_t rawData);
  uint8_t calcCrc2(uint32_t rawData);
public:
  void OregonTHN128_TempToString(char *temperatureStr, uint8_t temperatureStrLen, int16_t temperature);
  uint32_t OregonTHN128_DataToRaw(OregonTHN128Data_t &data);
  uint32_t OregonTHN128_DataToRaw2(OregonTHN128Data2_t *data);
  bool OregonTHN128_RawToData(uint32_t rawData, OregonTHN128Data_t &data);
};

#endif /* ERRIEZ_OREGON_THN128_H_ */
