

#include <string.h>
#include <stdio.h>
#include <cmath>
#include "THN128.h"

/*! Set rolling address */
#define SET_ROL_ADDR(x)     (((x) & 0x07) << 0)
/*! Get rolling address */
#define GET_ROL_ADDR(x)     (((x) & 0x07) << 0)

/*! Set channel */
#define SET_CHANNEL(x)      ((((x) - 1) & 0x03) << 6)
/*! Get channel */
#define GET_CHANNEL(x)      ((((x) >> 6) & 0x03) + 1)

/*! Set temperature */
#define SET_TEMP(x)         ((((((uint32_t)(x) / 100) % 10)) << 16) | \
                            ((((uint32_t)(x) / 10) % 10) << 12) | \
                            (((x) % 10) << 8))
/*! Get temperature */
#define GET_TEMP(x)         (((((x) >> 16) & 0x0f) * 100) + \
                            ((((x) >> 12) & 0x0f) * 10) + \
                            (((x) >> 8) & 0x0f))

/*! Sign bit */
#define SIGN_BIT            (1UL << 21)

/*! Low battery bit */
#define LOW_BAT_BIT         (1UL << 23)

/*! Set CRC */
#define SET_CRC(x)          ((uint32_t)(x) << 24)
/*! Get CRC */
#define GET_CRC(x)          ((x) >> 24)
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

uint8_t THN128::calcCrc2(uint32_t rawData)
{
  uint16_t crc;

    /* Add Bytes 0, 1 and 2 */
    crc = ((rawData >> 16) & 0xff) + ((rawData >> 8) & 0xff) + ((rawData >> 0) & 0xff);
    /* Add most significant Byte of the CRC to the final CRC */
    crc = (crc >> 8) + (crc & 0xff);

    /* Return 8-bit CRC */
    return (uint8_t)crc;
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

uint32_t THN128::OregonTHN128_DataToRaw2(OregonTHN128Data2_t *data)
{
    uint32_t rawData;

    /* Rolling address 0..7 */
    rawData = SET_ROL_ADDR(data->rollingAddress);

    /* Set channel 1..3 */
    rawData |= SET_CHANNEL(data->channel);

    /* Set temperature -999..999 */
    if (data->temperature < 0) {
        rawData |= SIGN_BIT;
        rawData |= SET_TEMP(data->temperature * -1);
    } else {
        rawData |= SET_TEMP(data->temperature);
    }

    /* Low battery bit */
    if (data->lowBattery) {
        rawData |= LOW_BAT_BIT;
    }

    /* Calculate CRC */
    rawData |= SET_CRC(calcCrc(rawData));

    /* Return 32-bit raw data */
    return rawData;
}

bool THN128::OregonTHN128_RawToData(uint32_t rawData, OregonTHN128Data_t &data)
{
  return true;
}
/*
// Sendefunktion für das THN128 / RMR112 v1.0 Protokoll
void oregon_v1_send(rmt_channel_handle_t tx_channel, uint8_t channel, uint8_t rolling_code, float temperature, bool low_battery) {
    uint8_t packet[4] = {0};

    // 1. Kanal vorbereiten (0 = Ch1, 1 = Ch2, 2 = Ch3)
    if (channel < 1 || channel > 3) channel = 1;
    uint8_t ch_bits = channel - 1;

    // Byte 0: Rolling Code & Kanal
    packet[0] = (rolling_code & 0x0F) | ((ch_bits & 0x03) << 6);

    // Temperatur zerlegen und im BCD-Format speichern
    bool is_negative = (temperature < 0);
    if (is_negative) temperature = -temperature;

    int temp_int = (int)(temperature * 10.0f + 0.5f); // z.B. 23.5 -> 235
    uint8_t digit_sub  = temp_int % 10;          // 5
    uint8_t digit_ones = (temp_int / 10) % 10;   // 3
    uint8_t digit_tens = (temp_int / 100) % 10;  // 2

    // Byte 1: Nachkomma (Bit 0..3) & Einerstelle (Bit 4..7)
    packet[1] = (digit_sub & 0x0F) | ((digit_ones & 0x0F) << 4);

    // Byte 2: Zehnerstelle, Vorzeichen & Batterie
    packet[2] = (digit_tens & 0x0F);
    if (is_negative) packet[2] |= (1 << 5);       // Vorzeichen-Bit
    if (low_battery) packet[2] |= (1 << 7);       // Low Battery Flag

    // Byte 3: Prüfsumme (Einfache Addition der ersten 3 Bytes)
    packet[3] = packet[0] + packet[1] + packet[2];

    

    // Die 4 Bytes (32 Bits) senden - LSB-first pro Byte
    for (int b = 0; b < 4; b++) {
        for (int i = 0; i < 8; i++) {
            bool bit = (packet[b] >> i) & 0x01;
            append_manchester_bit(tx_buffer, &idx, bit);
        }
    }


    // Payload abschicken
}

// Hilfsfunktion: Fügt ein einzelnes Bit im Manchester-Format zum RMT-Buffer hinzu
static void append_manchester_bit(rmt_symbol_word_t *buffer, size_t *idx, bool bit) {
    if (bit) {
        // Logisch 1: Invertiertes Manchester = LOW -> HIGH
        buffer[(*idx)++] = (rmt_symbol_word_t) { .duration0 = HALF_BIT_US, .level0 = 0, .duration1 = HALF_BIT_US, .level1 = 1 };
    } else {
        // Logisch 0: Invertiertes Manchester = HIGH -> LOW
        buffer[(*idx)++] = (rmt_symbol_word_t) { .duration0 = HALF_BIT_US, .level0 = 1, .duration1 = HALF_BIT_US, .level1 = 0 };
    }
}
*/