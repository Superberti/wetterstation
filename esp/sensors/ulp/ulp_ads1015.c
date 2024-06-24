#include <stdint.h>
#include <stdbool.h>
#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"
#include "esp_err.h"
#include "ulp_ads1015.h"
#include "../SensorTypes.h"
#include "tools.h"

esp_err_t ADS1x15_ReadRegister16(soft_i2c_config_t *cfg, ADC_REGISTER aReg, uint16_t *aRegVal);
esp_err_t ADS1x15_WriteRegister16(soft_i2c_config_t *cfg, ADC_REGISTER aReg, uint16_t aRegVal);
esp_err_t ADS1x15_ConvReady(soft_i2c_config_t *cfg, bool *aConvReady);

esp_err_t ADS1x15_ConvReady(soft_i2c_config_t *cfg, bool *aConvReady)
{
  esp_err_t ret;
  uint16_t Config = 0;
  ret = ADS1x15_ReadRegister16(cfg, REG_CONFIG, &Config);
  *aConvReady = (Config & (1 << 15)) > 0;
  return ret;
}

esp_err_t ADS1x15_ReadADC(soft_i2c_config_t *cfg, bool IsADS1015, ADC_MP aInputMux, FSC_RANGE aFullScale, uint8_t aSpeed, uint16_t *aADCValue)
{
  esp_err_t ret;

  // Init ADS1x15 (die 0x3 am Ende schaltet den Komperator ab...)
  uint16_t Config = (1 << 15) | (aInputMux << 12) | (aFullScale << 9) | (1 << 8) | (aSpeed << 5) | 0x3;

  // Konfiguration schreiben und Konversion starten
  ret = ADS1x15_WriteRegister16(cfg, REG_CONFIG, Config);
  if (ret != ESP_OK)
    return ret;

  bool cvr = false;

  // Achtung: Abweichend vom Datenblatt braucht der ADC im "Single-Shot-Modus"
  // deutlich länger (Fälschung?), als im kontinuierlichen Modus. Bei 128_SPS sind das ca. 500 ms (und nicht 4 ms lt. DB)!
  // bei 250_SPS sind es ca. 250 ms usw.
  // Timeout nach spätestens 600 ms.Länger dauert kein Sample!
  uint32_t MaxDelay = 600 * 1000 * ULP_RISCV_CYCLES_PER_US;
  uint32_t interval;
  uint32_t StartTime = ULP_RISCV_GET_CCOUNT();
  do
  {
    interval = ULP_RISCV_GET_CCOUNT() - StartTime; // underflow is well defined
    ret = ADS1x15_ConvReady(cfg, &cvr);
    if (ret != ESP_OK)
      return ret;
    if (cvr)
    {
      // Konversionsergebnis vom ADC lesen
      ret = ADS1x15_ReadRegister16(cfg, REG_CONVERSION, aADCValue);
      if (IsADS1015)
        *aADCValue = (*aADCValue >> 4); // Die vier ersten LSBs haben beim ADS1015 (12-Bit) keine Bedeutung!
      return ret;
    }
  } while (interval < MaxDelay);
  return ESP_ERR_TIMEOUT;
}

esp_err_t ADS1x15_ReadRegister16(soft_i2c_config_t *cfg, ADC_REGISTER aReg, uint16_t *aRegVal)
{
  esp_err_t ret;
  uint8_t RegAddr = aReg;
  uint8_t data[2] = {};
  ret = i2c_bb_master_write_read(cfg, &RegAddr, 1, data, 2);
  *aRegVal = (data[0] << 8) | data[1];
  return ret;
}

esp_err_t ADS1x15_WriteRegister16(soft_i2c_config_t *cfg, ADC_REGISTER aReg, uint16_t aRegVal)
{
  esp_err_t ret;
  uint8_t data[3] = {aReg, (uint8_t)(aRegVal >> 8), (uint8_t)(aRegVal & 0xFF)};
  ret = i2c_bb_master_write(cfg, data, 3);
  return ret;
}