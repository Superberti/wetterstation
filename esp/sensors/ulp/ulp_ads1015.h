#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "../CommonTypes.h"
#include "../SensorTypes.h"
#include "ulp_i2c_bitbang.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t ADS1x15_ReadADC(soft_i2c_config_t *cfg, bool IsADS1015, ADC_MP aInputMux, FSC_RANGE aFullScale, uint8_t aSpeed, uint16_t *aADCValue);


#ifdef __cplusplus
}
#endif