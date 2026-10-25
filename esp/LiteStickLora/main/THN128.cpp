

#include <string.h>
#include <stdio.h>
#include "THN128.h"

/*!
 * \defgroup Bit data macro's
 * @{
 */
/*! Set rolling address */
#define SET_ROL_ADDR(x) (((x) & 0x07) << 0)
/*! Get rolling address */
#define GET_ROL_ADDR(x) (((x) & 0x07) << 0)

/*! Set channel */
#define SET_CHANNEL(x) ((((x) - 1) & 0x03) << 6)
/*! Get channel */
#define GET_CHANNEL(x) ((((x) >> 6) & 0x03) + 1)

/*! Set temperature */
#define SET_TEMP(x) ((((((uint32_t)(x) / 100) % 10)) << 16) | \
                     ((((uint32_t)(x) / 10) % 10) << 12) |    \
                     (((x) % 10) << 8))
/*! Get temperature */
#define GET_TEMP(x) (((((x) >> 16) & 0x0f) * 100) + \
                     ((((x) >> 12) & 0x0f) * 10) +  \
                     (((x) >> 8) & 0x0f))

/*! Sign bit */
#define SIGN_BIT (1UL << 21)

/*! Low battery bit */
#define LOW_BAT_BIT (1UL << 23)

/*! Set CRC */
#define SET_CRC(x) ((uint32_t)(x) << 24)
/*! Get CRC */
#define GET_CRC(x) ((x) >> 24)

/*! @} */

/*!
 * \brief Calculate CRC
 * \param rawData
 *      Input data
 * \return
 *      8-bit checksum
 */
uint8_t THN128::calcCrc(uint32_t rawData)
{
  uint16_t crc;

  /* Add Bytes 0, 1 and 2 */
  crc = ((rawData >> 16) & 0xff) + ((rawData >> 8) & 0xff) + ((rawData >> 0) & 0xff);
  /* Add most significant Byte of the CRC to the final CRC */
  crc = (crc >> 8) + (crc & 0xff);

  /* Return 8-bit CRC */
  return (uint8_t)crc;
}

/*!
 * \brief Verify checksum
 * \param rawData
 *      32-bit raw data input
 * \return
 *      true: Success, false: error
 */
bool THN128::OregonTHN128_CheckCRC(uint32_t rawData)
{
  return calcCrc(rawData) == GET_CRC(rawData);
}

/*------------------------------------------------------------------------------------------------*/
/*                                     Public functions                                           */
/*------------------------------------------------------------------------------------------------*/
/*!
 * \brief Convert temperature to string
 * \param temperatureStr
 *      Character buffer
 * \param temperatureStrLen
 *      Size of character buffer
 * \param temperature
 *      Input temperature
 */
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

/*!
 * \brief Convert data structure to 32-bit raw data
 * \param data
 *      Input
 * \return
 *      Output
 */
uint32_t THN128::OregonTHN128_DataToRaw(OregonTHN128Data_t &data)
{
  uint32_t rawData;

  /* Rolling address 0..7 */
  rawData = SET_ROL_ADDR(data.rollingAddress);

  /* Set channel 1..3 */
  rawData |= SET_CHANNEL(data.channel);

  /* Set temperature -999..999 */
  if (data.temperature < 0)
  {
    rawData |= SIGN_BIT;
    rawData |= SET_TEMP(data.temperature * -1);
  }
  else
  {
    rawData |= SET_TEMP(data.temperature);
  }

  /* Low battery bit */
  if (data.lowBattery)
  {
    rawData |= LOW_BAT_BIT;
  }

  /* Calculate CRC */
  rawData |= SET_CRC(calcCrc(rawData));

  /* Return 32-bit raw data */
  return rawData;
}

/*!
 * \brief Cnonvert 32-bit raw data to OregonTHN128Data_t structure
 * \param rawData
 *      32-bit input
 * \param data
 *      output
 * \return
 *      CRC true: Success, false: error
 */
bool THN128::OregonTHN128_RawToData(uint32_t rawData, OregonTHN128Data_t &data)
{
  memset(&data, 0, sizeof(OregonTHN128Data_t));

  /* Set data structure */
  data.rawData = rawData;
  data.rollingAddress = GET_ROL_ADDR(rawData);
  data.channel = GET_CHANNEL(rawData);
  data.temperature = GET_TEMP(rawData);
  if (rawData & SIGN_BIT)
  {
    data.temperature *= -1;
  }
  data.lowBattery = (rawData & LOW_BAT_BIT) ? true : false;

  /* Return CRC success or failure */
  return calcCrc(rawData) == GET_CRC(rawData);
}
