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

/*
 * Ansteuerung einer DS3231 Echtzeituhr
 * Umgesetzt auf ESP32 in C++, I2C-Interface
 * O. Rutsch, Sympatec GmbH
 */
#ifndef __DS3231_H__
#define __DS3231_H__

#include "driver/i2c_master.h"
#include "SensorTypes.h"


class DS3231
{
private:
  i2c_master_bus_handle_t mBusHandle;
  i2c_master_dev_handle_t mDevHandle;

  uint8_t bcd2dec(uint8_t val);
  uint8_t dec2bcd(uint8_t val);
  int days_since_january_1st(int year, int month, int day);
  esp_err_t WriteReg(uint8_t aReg, uint8_t* aWriteBuf, uint8_t aWriteSize);
  esp_err_t ReadReg(uint8_t aReg, uint8_t* aReadBuf, uint8_t aReadSize);
  esp_err_t GetFlag(uint8_t addr, uint8_t mask, uint8_t *flag);
  esp_err_t SetFlag(uint8_t addr, uint8_t bits, uint8_t mode);

public:
  DS3231();
  esp_err_t Init(i2c_master_bus_handle_t aBusHandle, uint8_t aI2CAddr, uint32_t aI2CSpeed_Hz);
  ~DS3231(void);
  /**
 * @brief Set the time on the RTC
 *
 * Timezone agnostic, pass whatever you like.
 * I suggest using GMT and applying timezone and DST when read back.
 *
 * @return ESP_OK to indicate success
 */
esp_err_t SetTime(struct tm *time);

/**
 * @brief Get the time from the RTC, populates a supplied tm struct
 *
 * @param dev Device descriptor
 * @param[out] time RTC time
 * @return ESP_OK to indicate success
 */
esp_err_t GetTime(struct tm *time);

/**
 * @brief Set alarms
 *
 * `alarm1` works with seconds, minutes, hours and day of week/month, or fires every second.
 * `alarm2` works with minutes, hours and day of week/month, or fires every minute.
 *
 * Not all combinations are supported, see `DS3231_ALARM1_*` and `DS3231_ALARM2_*` defines
 * for valid options you only need to populate the fields you are using in the `tm` struct,
 * and you can set both alarms at the same time (pass `DS3231_ALARM_1`/`DS3231_ALARM_2`/`DS3231_ALARM_BOTH`).
 *
 * If only setting one alarm just pass 0 for `tm` struct and `option` field for the other alarm.
 * If using ::DS3231_ALARM1_EVERY_SECOND/::DS3231_ALARM2_EVERY_MIN you can pass 0 for `tm` struct.
 *
 * If you want to enable interrupts for the alarms you need to do that separately.
 *
 * @return ESP_OK to indicate success
 */
esp_err_t SetAlarm(ds3231_alarm_t alarms, struct tm *time1,
        ds3231_alarm1_rate_t option1, struct tm *time2, ds3231_alarm2_rate_t option2);

/**
 * @brief Check if oscillator has previously stopped
 *
 * E.g. no power/battery or disabled
 * sets flag to true if there has been a stop
 *
 * @param dev Device descriptor
 * @param[out] flag Stop flag
 * @return ESP_OK to indicate success
 */
esp_err_t GetOscillatorStopFlag(bool *flag);

/**
 * @brief Clear the oscillator stopped flag
 *
 * @param dev Device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t ClearOscillatorStopFlag();

/**
 * @brief Check which alarm(s) have past
 *
 * Sets alarms to `DS3231_ALARM_NONE`/`DS3231_ALARM_1`/`DS3231_ALARM_2`/`DS3231_ALARM_BOTH`
 *
 * @param dev Device descriptor
 * @param[out] alarms Alarms
 * @return ESP_OK to indicate success
 */
esp_err_t GetAlarmFlags(ds3231_alarm_t *alarms);

/**
 * @brief Clear alarm past flag(s)
 *
 * Pass `DS3231_ALARM_1`/`DS3231_ALARM_2`/`DS3231_ALARM_BOTH`
 *
 * @param dev Device descriptor
 * @param alarms Alarms
 * @return ESP_OK to indicate success
 */
esp_err_t ClearAlarmFlags(ds3231_alarm_t alarms);

/**
 * @brief enable alarm interrupts (and disables squarewave)
 *
 * Pass `DS3231_ALARM_1`/`DS3231_ALARM_2`/`DS3231_ALARM_BOTH`.
 *
 * If you set only one alarm the status of the other is not changed
 * you must also clear any alarm past flag(s) for alarms with
 * interrupt enabled, else it will trigger immediately.
 *
 * @param dev Device descriptor
 * @param alarms Alarms
 * @return ESP_OK to indicate success
 */
esp_err_t EnableAlarmInts(ds3231_alarm_t alarms);

/**
 * @brief Disable alarm interrupts
 *
 * Does not (re-)enable squarewave
 *
 * @param dev Device descriptor
 * @param alarms Alarm
 * @return ESP_OK to indicate success
 */
esp_err_t DisableAlarmInts(ds3231_alarm_t alarms);

/**
 * @brief Enable the output of 32khz signal
 *
 * **Supported only by DS3231**
 *
 * @param dev Device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t Enable32khz();

/**
 * @brief Disable the output of 32khz signal
 *
 * **Supported only by DS3231**
 *
 * @param dev Device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t Disable32khz();

/**
 * @brief Enable the squarewave output
 *
 * Disables alarm interrupt functionality.
 *
 * @param dev Device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t EnableSquarewave();

/**
 * @brief Disable the squarewave output
 *
 * Which re-enables alarm interrupts, but individual alarm interrupts also
 * need to be enabled, if not already, before they will trigger.
 *
 * @param dev Device descriptor
 * @return ESP_OK to indicate success
 */
esp_err_t DisableSquarewave();

/**
 * @brief Set the frequency of the squarewave output
 *
 * Does not enable squarewave output.
 *
 * @param dev Device descriptor
 * @param freq Squarewave frequency
 * @return ESP_OK to indicate success
 */
esp_err_t SetSquarewaveFreq(ds3231_sqwave_freq_t freq);

/**
 * @brief Get the frequency of the squarewave output
 *
 * Does not enable squarewave output.
 *
 * @param dev Device descriptor
 * @param freq Squarewave frequency to store the output
 * @return ESP_OK to indicate success
 */
esp_err_t GetSquarewaveFreq(ds3231_sqwave_freq_t* freq);

/**
 * @brief Get the raw temperature value
 *
 * **Supported only by DS3231**
 *
 * @param dev Device descriptor
 * @param[out] temp Raw temperature value
 * @return ESP_OK to indicate success
 */
esp_err_t GetRawTemp(int16_t *temp);

/**
 * @brief Get the temperature as an integer
 *
 * **Supported only by DS3231**
 *
 * @param dev Device descriptor
 * @param[out] temp Temperature, degrees Celsius
 * @return ESP_OK to indicate success
 */
esp_err_t GetTempInteger(int8_t *temp);

/**
 * @brief Get the temperature as a float
 *
 * **Supported only by DS3231**
 *
 * @param dev Device descriptor
 * @param[out] temp Temperature, degrees Celsius
 * @return ESP_OK to indicate success
 */
esp_err_t GetTempFloat(float *temp);


/**
 * @brief Set the aging offset register to a new value.
 *
 * Positive aging values add capacitance to the array,
 * slowing the oscillator frequency. Negative values remove
 * capacitance from the array, increasing the oscillator
 * frequency.
 *
 * **Supported only by DS3231**
 *
 * @param dev Device descriptor
 * @param age Aging offset (in range [-128, 127]) to be set
 * @return ESP_OK to indicate success
 */
esp_err_t SetAgingOffset(int8_t age);


/**
 * @brief Get the aging offset register.
 *
 * **Supported only by DS3231**
 *
 * @param dev Device descriptor
 * @param[out] age Aging offset in range [-128, 127]
 * @return ESP_OK to indicate success
 */
esp_err_t GetAgingOffset(int8_t *age);

};

#endif
