// Gemeinsam benutzte Typen der Sensoren (ULP und ESP32)
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C"
{
#endif

/*
 *  I2C-Adresse SHT40
 */
#define SHT40_ADDR (0x44)

/*
 * I2C-Adresse ADS1x15. Der ADDR-Pin muss mit GND,VDD oder SCL verbunden werden (notfalls auch SDA).
 * Dadurch ergeben sich vier Adressen:
 */
#define ADC1x15_ADDR_GND (0x48)
#define ADC1x15_ADDR_VDD (0x49)
#define ADC1x15_ADDR_SCL (0x4A)
#define ADC1x15_ADDR_SDA (0x4B)

/*
 *  I2C-Adresse DS3231
 */
#define DS3231_ADDR (0x68)

#define DELAY_LPM_MS 2
#define DELAY_MPM_MS 5
#define DELAY_HPM_MS 10

  // Kommandos s. Datenblatt Seite 12
  typedef enum
  {
    // Hohe Präzision, 20 mW Heater, 100 ms on-time
    SHT40_CMD_HPM_H_20_100 = 0x15,

    // Hohe Präzision, 20 mW Heater, 1000 ms on-time
    SHT40_CMD_HPM_H_20_1000 = 0x1E,

    // Hohe Präzision, 110 mW Heater, 100 ms on-time
    SHT40_CMD_HPM_H_110_100 = 0x24,

    // Hohe Präzision, 110 mW Heater, 1000 ms on-time
    SHT40_CMD_HPM_H_110_1000 = 0x2F,

    // Hohe Präzision, 200 mW Heater, 100 ms on-time
    SHT40_CMD_HPM_H_200_100 = 0x32,

    // Hohe Präzision, 200 mW Heater, 1000 ms on-time
    SHT40_CMD_HPM_H_200_1000 = 0x39,

    // Seriennummer auslesen
    SHT40_CMD_READ_SERIAL = 0x89,

    // Reset
    SHT40_CMD_SOFT_RESET = 0x95,

    // Niedrige Präzision, kein Heater, Messzeit=1,6 ms
    SHT40_CMD_LPM = 0xE0,

    // Mittlere Präzision, kein Heater, Messzeit=4,5 ms
    SHT40_CMD_MPM = 0xF6,

    // Hohe Präzision, kein Heater, Messzeit=8,3 ms
    SHT40_CMD_HPM = 0xFD,

  } SHT40_COMMAND;

  /**
   * Alarms
   */
  typedef enum
  {
    DS3231_ALARM_NONE = 0, //!< No alarms
    DS3231_ALARM_1,        //!< First alarm
    DS3231_ALARM_2,        //!< Second alarm
    DS3231_ALARM_BOTH      //!< Both alarms
  } ds3231_alarm_t;

  /**
   * First alarm rate
   */
  typedef enum
  {
    DS3231_ALARM1_EVERY_SECOND = 0,
    DS3231_ALARM1_MATCH_SEC,
    DS3231_ALARM1_MATCH_SECMIN,
    DS3231_ALARM1_MATCH_SECMINHOUR,
    DS3231_ALARM1_MATCH_SECMINHOURDAY,
    DS3231_ALARM1_MATCH_SECMINHOURDATE
  } ds3231_alarm1_rate_t;

  /**
   * Second alarm rate
   */
  typedef enum
  {
    DS3231_ALARM2_EVERY_MIN = 0,
    DS3231_ALARM2_MATCH_MIN,
    DS3231_ALARM2_MATCH_MINHOUR,
    DS3231_ALARM2_MATCH_MINHOURDAY,
    DS3231_ALARM2_MATCH_MINHOURDATE
  } ds3231_alarm2_rate_t;

  /**
   * Squarewave frequency
   */
  typedef enum
  {
    DS3231_SQWAVE_1HZ = 0x00,
    DS3231_SQWAVE_1024HZ = 0x08,
    DS3231_SQWAVE_4096HZ = 0x10,
    DS3231_SQWAVE_8192HZ = 0x18
  } ds3231_sqwave_freq_t;

  // Vorverstärker Spannungsbereich (muss aber immer <= Versorgungsspannung sein!)
  typedef enum
  {
    FSR_6_144 = 0, // 6,144 V
    FSR_4_096,
    FSR_2_048, // default
    FSR_1_024,
    FSR_0_512,
    FSR_0_256
  } FSC_RANGE;

  // Pin-Multiplexing
  typedef enum
  {
    AIN0_AND_AIN1 = 0, // Differentiell, Pin AIN0 und 1, default
    AIN0_AND_AIN3,
    AIN1_AND_AIN3,
    AIN2_AND_AIN3,
    AIN0, // Single-Ended
    AIN1,
    AIN2,
    AIN3
  } ADC_MP;

  // ADC-Konversionsrate beim ADS1015
  typedef enum
  {
    ADS_1015_SPEED_128 = 0, // 128 Samples per second
    ADS_1015_SPEED_250,
    ADS_1015_SPEED_490,
    ADS_1015_SPEED_920,
    ADS_1015_SPEED_1600, // default
    ADS_1015_SPEED_2400,
    ADS_1015_SPEED_3300
  } ADC_SPEED_1015;

  // ADC-Konversionsrate beim ADS1115 (langsamer)
  typedef enum
  {
    ADS_1115_SPEED_8 = 0, // 128 Samples per second
    ADS_1115_SPEED_16,
    ADS_1115_SPEED_32,
    ADS_1115_SPEED_64,
    ADS_1115_SPEED_128, // default
    ADS_1115_SPEED_250,
    ADS_1115_SPEED_475,
    ADS_1115_SPEED_860,
  } ADC_SPEED_1115;

  typedef enum
  {
    REG_CONVERSION = 0,
    REG_CONFIG = 1,
    REG_THRESH_LO = 2,
    REG_THRESH_HI = 3,
  } ADC_REGISTER;

#ifdef __cplusplus
}
#endif