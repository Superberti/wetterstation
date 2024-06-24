/*
 * ds3231 Echtzeituhr
 */

/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2015 Richard A Burton <richardaburton@gmail.com>
 * Copyright (c) 2016 Bhuvanchandra DV <bhuvanchandra.dv@gmail.com>
 * Copyright (c) 2018 Ruslan V. Uss <unclerus@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdio.h>
#include "esp_log.h"
#include <stdint.h>
#include "ds3231.h"
#include <math.h>
#include <time.h>
#include <string.h>

#define DS3231_STAT_OSCILLATOR 0x80
#define DS3231_STAT_32KHZ 0x08
#define DS3231_STAT_ALARM_2 0x02
#define DS3231_STAT_ALARM_1 0x01

#define DS3231_CTRL_OSCILLATOR 0x80
#define DS3231_CTRL_TEMPCONV 0x20
#define DS3231_CTRL_ALARM_INTS 0x04
#define DS3231_CTRL_ALARM2_INT 0x02
#define DS3231_CTRL_ALARM1_INT 0x01

#define DS3231_ALARM_WDAY 0x40
#define DS3231_ALARM_NOTSET 0x80

#define DS3231_ADDR_TIME 0x00
#define DS3231_ADDR_ALARM1 0x07
#define DS3231_ADDR_ALARM2 0x0b
#define DS3231_ADDR_CONTROL 0x0e
#define DS3231_ADDR_STATUS 0x0f
#define DS3231_ADDR_AGING 0x10
#define DS3231_ADDR_TEMP 0x11

#define DS3231_12HOUR_FLAG 0x40
#define DS3231_12HOUR_MASK 0x1f
#define DS3231_PM_FLAG 0x20
#define DS3231_MONTH_MASK 0x1f

#define DEV_TIMEOUT 100

enum
{
  DS3231_SET = 0,
  DS3231_CLEAR,
  DS3231_REPLACE
};

static const int days_per_month[] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};
static const int days_per_month_leap_year[] = {31, 29, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

DS3231::DS3231()
{
  mDevHandle = NULL;
  mBusHandle = NULL;
}

esp_err_t DS3231::Init(i2c_master_bus_handle_t aBusHandle, uint8_t aI2CAddr, uint32_t aI2CSpeed_Hz)
{
  mBusHandle = aBusHandle;
  i2c_device_config_t conf;
  conf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
  conf.device_address = aI2CAddr;
  conf.scl_speed_hz = aI2CSpeed_Hz;
  return i2c_master_bus_add_device(mBusHandle, &conf, &mDevHandle);
}

esp_err_t DS3231::WriteReg(uint8_t aReg, uint8_t *aWriteBuf, uint8_t aWriteSize)
{
  esp_err_t ret;
  uint8_t *WriteData = new uint8_t[aWriteSize + 1];
  WriteData[0] = aReg;
  memcpy(WriteData + 1, aWriteBuf, aWriteSize);
  ret = i2c_master_transmit(mDevHandle, WriteData, aWriteSize + 1, DEV_TIMEOUT);
  delete[] WriteData;
  if (ret != ESP_OK)
    ESP_LOGE("DS3231::WriteRegister", "I2C error no.: %d", ret);
  return ret;
}

esp_err_t DS3231::ReadReg(uint8_t aReg, uint8_t *aReadBuf, uint8_t aReadSize)
{
  if (mDevHandle == NULL)
    return ESP_ERR_INVALID_ARG;
  esp_err_t ret;
  uint8_t RegAddr = aReg;
  ret = i2c_master_transmit_receive(mDevHandle, &RegAddr, 1, aReadBuf, aReadSize, DEV_TIMEOUT);
  if (ret != ESP_OK)
    ESP_LOGE("ADS1015::ReadRegister", "I2C error no.: %d", ret);

  return ret;
}

DS3231::~DS3231(void)
{
  if (mDevHandle != NULL)
    i2c_master_bus_rm_device(mDevHandle);
}

uint8_t DS3231::bcd2dec(uint8_t val)
{
  return (val >> 4) * 10 + (val & 0x0f);
}

uint8_t DS3231::dec2bcd(uint8_t val)
{
  return ((val / 10) << 4) + (val % 10);
}

// Function to convert year, month, and day to days since January 1st
int DS3231::days_since_january_1st(int year, int month, int day)
{
  int days = day - 1;
  const int *ptr = days_per_month;

  // Handle leap year
  if ((year % 4 == 0 && year % 100 != 0) || (year % 400 == 0))
    ptr = days_per_month_leap_year;

  // Add days from previous months
  for (int i = 0; i < month; i++)
  {
    days += ptr[i];
  }

  return days;
}

esp_err_t DS3231::GetTime(struct tm *time)
{
  uint8_t data[7];

  /* read time */
  esp_err_t res = ReadReg(DS3231_ADDR_TIME, data, 7);
  if (res != ESP_OK)
    return res;

  /* convert to unix time structure */
  time->tm_sec = bcd2dec(data[0]);
  time->tm_min = bcd2dec(data[1]);
  if (data[2] & DS3231_12HOUR_FLAG)
  {
    /* 12H */
    time->tm_hour = bcd2dec(data[2] & DS3231_12HOUR_MASK) - 1;
    /* AM/PM? */
    if (data[2] & DS3231_PM_FLAG)
      time->tm_hour += 12;
  }
  else
    time->tm_hour = bcd2dec(data[2]); /* 24H */
  time->tm_wday = bcd2dec(data[3]) - 1;
  time->tm_mday = bcd2dec(data[4]);
  time->tm_mon = bcd2dec(data[5] & DS3231_MONTH_MASK) - 1;
  time->tm_year = bcd2dec(data[6]) + 100;
  time->tm_isdst = 0;
  time->tm_yday = days_since_january_1st(time->tm_year, time->tm_mon, time->tm_mday);
  //ESP_LOGI("DS3231","Reading year (since 1900):%d ",time->tm_year);

  // apply a time zone (if you are not using localtime on the rtc or you want to check/apply DST)
  // applyTZ(time);

  return ESP_OK;
}

esp_err_t DS3231::SetTime(struct tm *time)
{
  uint8_t data[7];

  /* time/date data */
  data[0] = dec2bcd(time->tm_sec);
  data[1] = dec2bcd(time->tm_min);
  data[2] = dec2bcd(time->tm_hour);
  /* The week data must be in the range 1 to 7, and to keep the start on the
   * same day as for tm_wday have it start at 1 on Sunday. */
  data[3] = dec2bcd(time->tm_wday + 1);
  data[4] = dec2bcd(time->tm_mday);
  data[5] = dec2bcd(time->tm_mon + 1);
  data[6] = dec2bcd(time->tm_year - 100);
  //ESP_LOGI("DS3231","Setting year (since 1900):%d ",time->tm_year );
  return WriteReg(DS3231_ADDR_TIME, data, 7);
}

esp_err_t DS3231::SetAlarm(ds3231_alarm_t alarms, struct tm *time1,
                           ds3231_alarm1_rate_t option1, struct tm *time2, ds3231_alarm2_rate_t option2)
{

  int i = 0;
  uint8_t data[7];

  /* alarm 1 data */
  if (alarms != DS3231_ALARM_2)
  {
    data[i++] = (option1 >= DS3231_ALARM1_MATCH_SEC ? dec2bcd(time1->tm_sec) : DS3231_ALARM_NOTSET);
    data[i++] = (option1 >= DS3231_ALARM1_MATCH_SECMIN ? dec2bcd(time1->tm_min) : DS3231_ALARM_NOTSET);
    data[i++] = (option1 >= DS3231_ALARM1_MATCH_SECMINHOUR ? dec2bcd(time1->tm_hour) : DS3231_ALARM_NOTSET);
    data[i++] = (option1 == DS3231_ALARM1_MATCH_SECMINHOURDAY ? (dec2bcd(time1->tm_wday + 1) & DS3231_ALARM_WDAY) : (option1 == DS3231_ALARM1_MATCH_SECMINHOURDATE ? dec2bcd(time1->tm_mday) : DS3231_ALARM_NOTSET));
  }

  /* alarm 2 data */
  if (alarms != DS3231_ALARM_1)
  {
    data[i++] = (option2 >= DS3231_ALARM2_MATCH_MIN ? dec2bcd(time2->tm_min) : DS3231_ALARM_NOTSET);
    data[i++] = (option2 >= DS3231_ALARM2_MATCH_MINHOUR ? dec2bcd(time2->tm_hour) : DS3231_ALARM_NOTSET);
    data[i++] = (option2 == DS3231_ALARM2_MATCH_MINHOURDAY ? (dec2bcd(time2->tm_wday + 1) & DS3231_ALARM_WDAY) : (option2 == DS3231_ALARM2_MATCH_MINHOURDATE ? dec2bcd(time2->tm_mday) : DS3231_ALARM_NOTSET));
  }

  return WriteReg((alarms == DS3231_ALARM_2 ? DS3231_ADDR_ALARM2 : DS3231_ADDR_ALARM1), data, i);
}

/* Get a byte containing just the requested bits
 * pass the register address to read, a mask to apply to the register and
 * an uint* for the output
 * you can test this value directly as true/false for specific bit mask
 * of use a mask of 0xff to just return the whole register byte
 * returns true to indicate success
 */
esp_err_t DS3231::GetFlag(uint8_t addr, uint8_t mask, uint8_t *flag)
{
  uint8_t data;

  /* get register */
  esp_err_t res = ReadReg(addr, &data, 1);
  if (res != ESP_OK)
    return res;

  /* return only requested flag */
  *flag = (data & mask);
  return ESP_OK;
}

/* Set/clear bits in a byte register, or replace the byte altogether
 * pass the register address to modify, a byte to replace the existing
 * value with or containing the bits to set/clear and one of
 * DS3231_SET/DS3231_CLEAR/DS3231_REPLACE
 * returns true to indicate success
 */
esp_err_t DS3231::SetFlag(uint8_t addr, uint8_t bits, uint8_t mode)
{
  uint8_t data;

  /* get status register */
  esp_err_t res = ReadReg(addr, &data, 1);
  if (res != ESP_OK)
    return res;
  /* clear the flag */
  if (mode == DS3231_REPLACE)
    data = bits;
  else if (mode == DS3231_SET)
    data |= bits;
  else
    data &= ~bits;

  return WriteReg(addr, &data, 1);
}

esp_err_t DS3231::GetOscillatorStopFlag(bool *flag)
{
  uint8_t f=0;
  esp_err_t res = GetFlag(DS3231_ADDR_STATUS, DS3231_STAT_OSCILLATOR, &f);
  *flag = (f ? true : false);
  return res;
}

esp_err_t DS3231::ClearOscillatorStopFlag()
{
  return SetFlag(DS3231_ADDR_STATUS, DS3231_STAT_OSCILLATOR, DS3231_CLEAR);
}

esp_err_t DS3231::GetAlarmFlags(ds3231_alarm_t *alarms)
{
  return GetFlag(DS3231_ADDR_STATUS, DS3231_ALARM_BOTH, (uint8_t *)alarms);
}

esp_err_t DS3231::ClearAlarmFlags(ds3231_alarm_t alarms)
{
  return SetFlag(DS3231_ADDR_STATUS, alarms, DS3231_CLEAR);
}

esp_err_t DS3231::EnableAlarmInts(ds3231_alarm_t alarms)
{
  return SetFlag(DS3231_ADDR_CONTROL, DS3231_CTRL_ALARM_INTS | alarms, DS3231_SET);
}

esp_err_t DS3231::DisableAlarmInts(ds3231_alarm_t alarms)
{
  /* Just disable specific alarm(s) requested
   * does not disable alarm interrupts generally (which would enable the squarewave)
   */
  return SetFlag(DS3231_ADDR_CONTROL, alarms, DS3231_CLEAR);
}

esp_err_t DS3231::Enable32khz()
{
  return SetFlag(DS3231_ADDR_STATUS, DS3231_STAT_32KHZ, DS3231_SET);
}

esp_err_t DS3231::Disable32khz()
{
  return SetFlag(DS3231_ADDR_STATUS, DS3231_STAT_32KHZ, DS3231_CLEAR);
}

esp_err_t DS3231::EnableSquarewave()
{
  return SetFlag(DS3231_ADDR_CONTROL, DS3231_CTRL_ALARM_INTS, DS3231_CLEAR);
}

esp_err_t DS3231::DisableSquarewave()
{
  return SetFlag(DS3231_ADDR_CONTROL, DS3231_CTRL_ALARM_INTS, DS3231_SET);
}

esp_err_t DS3231::SetSquarewaveFreq(ds3231_sqwave_freq_t freq)
{
  uint8_t flag = 0;
  esp_err_t res = GetFlag(DS3231_ADDR_CONTROL, 0xff, &flag);
  if (res != ESP_OK)
    return res;
  flag &= ~DS3231_SQWAVE_8192HZ;
  flag |= freq;
  return SetFlag(DS3231_ADDR_CONTROL, flag, DS3231_REPLACE);
}

esp_err_t DS3231::GetSquarewaveFreq(ds3231_sqwave_freq_t *freq)
{
  uint8_t flag = 0;
  esp_err_t res = GetFlag(DS3231_ADDR_CONTROL, 0xff, &flag);
  flag &= DS3231_SQWAVE_8192HZ;
  *freq = (ds3231_sqwave_freq_t)flag;
  return res;
}

esp_err_t DS3231::GetRawTemp(int16_t *temp)
{
  uint8_t data[2];
  esp_err_t res = ReadReg(DS3231_ADDR_TEMP, data, sizeof(data));
  *temp = (int16_t)(int8_t)data[0] << 2 | data[1] >> 6;
  return res;
}

esp_err_t DS3231::GetTempInteger(int8_t *temp)
{
  int16_t t_int;
  esp_err_t res = GetRawTemp(&t_int);
  *temp = t_int >> 2;
  return res;
}

esp_err_t DS3231::GetTempFloat(float *temp)
{
  int16_t t_int;
  esp_err_t res = GetRawTemp(&t_int);
  *temp = t_int * 0.25;
  return res;
}

esp_err_t DS3231::SetAgingOffset(int8_t age)
{
  uint8_t age_u8 = (uint8_t)age;
  esp_err_t res = WriteReg(DS3231_ADDR_AGING, &age_u8, sizeof(uint8_t));
  if (res != ESP_OK)
    return res;
  /**
   * To see the effects of the aging register on the 32kHz output
   * frequency immediately, a manual conversion should be started
   * after each aging register change.
   */
  return SetFlag(DS3231_ADDR_CONTROL, DS3231_CTRL_TEMPCONV, DS3231_SET);
}

esp_err_t DS3231::GetAgingOffset(int8_t *age)
{
  uint8_t age_u8;
  esp_err_t res = ReadReg(DS3231_ADDR_AGING, &age_u8, sizeof(uint8_t));
  *age = (int8_t)age_u8;
  return res;
}
