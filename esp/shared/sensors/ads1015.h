/*
 * Ansteuerung eines ADS1015-12bit oder ADS1115-16bit-ADC.
 * Umgesetzt auf ESP32, I2C-Interface
 */
#ifndef __ADS1015_H__
#define __ADS1015_H__

#include "driver/i2c_master.h"
#include "SensorTypes.h"

class ADS1x15
{
private:
  i2c_master_bus_handle_t mBusHandle;
  i2c_master_dev_handle_t mDevHandle;
  esp_err_t ReadRegister16(ADC_REGISTER aReg, uint16_t &aRegVal);
  esp_err_t WriteRegister16(ADC_REGISTER aReg, uint16_t aRegVal);
  esp_err_t ConvReady(bool &aConvReady);

public:
  ADS1x15();
  esp_err_t Init(i2c_master_bus_handle_t aBusHandle, uint8_t aI2CAddr, uint32_t aI2CSpeed_Hz, bool ReInit=false);
  void Close();
  ~ADS1x15(void);

  esp_err_t ReadADC_1015(ADC_MP aInputMux, FSC_RANGE aFullScale, ADC_SPEED_1015 aSpeed, uint16_t &aADCValue);
  esp_err_t ReadADC_1115(ADC_MP aInputMux, FSC_RANGE aFullScale, ADC_SPEED_1115 aSpeed, uint16_t &aADCValue);
};

#endif
