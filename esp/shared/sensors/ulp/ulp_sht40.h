#pragma once

#include <stdint.h>
#include "esp_err.h"
#include "../CommonTypes.h"
#include "../SensorTypes.h"
#include "ulp_i2c_bitbang.h"
#include "tools.h"

#ifdef __cplusplus
extern "C" {
#endif

esp_err_t SHT40_ReadSerial(soft_i2c_config_t *cfg, uint32_t *aSerialNo, bool *rCRC_Err);
esp_err_t SHT40_Read(soft_i2c_config_t *cfg, SHT40_COMMAND aReadMode, uint16_t *aTemp, uint16_t *aHum, bool *rCRC_Err);


#ifdef __cplusplus
}
#endif