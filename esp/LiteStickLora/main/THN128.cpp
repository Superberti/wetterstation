

#include <string.h>
#include <stdio.h>
#include <cmath>
#include "THN128.h"

/*------------------------------------------------------------------------------------------------*/
/*                                     Public functions                                           */
/*------------------------------------------------------------------------------------------------*/

uint8_t THN128::calcCrc(uint32_t rawData)
{
  uint8_t byte0 = ((rawData >> 24) & 0xff);
  uint8_t byte1 = ((rawData >> 16) & 0xff);
  uint8_t byte2 = ((rawData >> 8) & 0xff);
  // Nibbles vertauschen
  byte0 = (byte0 << 4) | (byte0 >> 4);
  byte1 = (byte1 << 4) | (byte1 >> 4);
  byte2 = (byte2 << 4) | (byte2 >> 4);
  uint16_t crc = byte0 + byte1 + byte2;
  /* Return 8-bit CRC */
  uint8_t crc8 = (crc >> 8) + (crc & 0xff);
  return (crc8 << 4) | (crc8 >> 4);
}

void THN128::OregonTHN128_TempToString(char *temperatureStr, uint8_t temperatureStrLen, int16_t temperature)
{
  bool tempNegative = false;
  int tempAbs;

  /* Convert temperature without using float to string */
  tempAbs = temperature;
  if (temperature < 0)
  {
    tempNegative = true;
    tempAbs *= -1;
  }

  snprintf(temperatureStr, temperatureStrLen, "%s%d.%d",
           tempNegative ? "-" : "", (tempAbs / 10), tempAbs % 10);
}

uint32_t THN128::OregonTHN128_DataToRaw(OregonTHN128Data_t &data)
{
  OregonV1Data v1data;
  v1data.Bitfield.RollingCode = data.rollingAddress;
  v1data.Bitfield.Channel = (data.channel - 1) << 2;
  uint16_t temp = std::round(fabs(data.temperature * 10));
  int hunderter = temp / 100;
  int zehner = (temp / 10) % 10;
  int einer = temp % 10;

  v1data.Bitfield.Temp_01_Deg = einer;
  v1data.Bitfield.Temp_1_Deg = zehner;
  v1data.Bitfield.Temp_10_Deg = hunderter;
  v1data.Bitfield.BatteryLow = data.lowBattery;
  v1data.Bitfield.Unknown = 0;
  v1data.Bitfield.NegativeFlag = data.temperature < 0.0 ? 1 : 0;
  v1data.Bitfield.NotUsed = 0;
  v1data.Bitfield.crc = calcCrc(v1data.RawData);
  return v1data.RawData;
}

bool THN128::OregonTHN128_RawToData(uint32_t rawData, OregonTHN128Data_t &data)
{
  return true;
}
