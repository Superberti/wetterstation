/*
 * Ansteuerung eines ADS1015-12bit-ADC.
 * Umgesetzt auf ESP32, I2C-Interface
 */
#ifndef __ADS1015_H__
#define __ADS1015_H__

#include "driver/i2c.h"
/*!
 * I2C-Adresse. Der ADDR-Pin muss mit GND,VDD oder SCL verbunden werden (notfalls auch SDA). 
 * Dadurch ergeben sich vier Adressen:
 */
#define ADC1015_ADDR_GND (0x48)
#define ADC1015_ADDR_VDD (0x49)
#define ADC1015_ADDR_SCL (0x4A)
#define ADC1015_ADDR_SDA (0x4B)

// Vorverstärker Spannungsbereich (muss aber immer <= Versorgungsspannung sein!)
enum FSC_RANGE
{
  FSR_6_144=0,  // 6,144 V
  FSR_4_096,
  FSR_2_048,  // default
  FSR_1_024,
  FSR_0_512,
  FSR_0_256
};

// Pin-Multiplexing
enum ADC_MP
{
  AIN0_AND_AIN1=0,  // Differentiell, Pin AIN0 und 1, default
  AIN0_AND_AIN3,
  AIN1_AND_AIN3,
  AIN2_AND_AIN3,
  AIN0, // Single-Ended
  AIN1,
  AIN2,
  AIN3
};

// ADC-Konversionsrate
enum ADC_SPEED
{
  SPEED_128=0,  // 128 Samples per second
  SPEED_250,
  SPEED_490,
  SPEED_920,
  SPEED_1600, // default
  SPEED_2400,
  SPEED_3300
};

enum ADC_REGISTER
{
  REG_CONVERSION = 0,
  REG_CONFIG,
  REG_THRESH_LO,
  REG_THRESH_HI,
};

class ADS1015
{
private:
  i2c_port_t mPort;
  uint8_t mI2CAddr;
  esp_err_t ReadRegister16(ADC_REGISTER aReg, uint16_t & aRegVal);
  esp_err_t WriteRegister16(ADC_REGISTER aReg, uint16_t aRegVal);
  int64_t GetTime_us();
  esp_err_t ConvReady(bool &aConvReady);
public:

  ADS1015(i2c_port_t aPort, uint8_t aI2CAddr);
  ~ADS1015(void);

  esp_err_t ReadADC(ADC_MP aInputMux, FSC_RANGE aFullScale, ADC_SPEED aSpeed, uint16_t & aADCValue);
};

#endif
