

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
  int16_t temperature;    /*!< Temperature in degrees*10 */
  bool lowBattery;        /*!< Low battery indication */
};

class THN128
{
protected:
  uint8_t calcCrc(uint32_t rawData);
public:
  bool OregonTHN128_CheckCRC(uint32_t rawData);
  void OregonTHN128_TempToString(char *temperatureStr, uint8_t temperatureStrLen, int16_t temperature);
  uint32_t OregonTHN128_DataToRaw(OregonTHN128Data_t &data);
  bool OregonTHN128_RawToData(uint32_t rawData, OregonTHN128Data_t &data);
};

#endif /* ERRIEZ_OREGON_THN128_H_ */
