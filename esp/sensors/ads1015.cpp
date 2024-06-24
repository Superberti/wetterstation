/*!
 * SHT35 Sensor
 */

#include <stdio.h>
#include <stdint.h>
#include "ads1015.h"
#include <math.h>
#include <sys/time.h>
#include "freertos/FreeRTOS.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "tools.h"

#define DEV_TIMEOUT 100

ADS1x15::ADS1x15()
{
  mDevHandle = NULL;
  mBusHandle = NULL;
}

esp_err_t ADS1x15::Init(i2c_master_bus_handle_t aBusHandle, uint8_t aI2CAddr, uint32_t aI2CSpeed_Hz, bool ReInit)
{
  mBusHandle = aBusHandle;
  if (mDevHandle == NULL || ReInit)
  {
    i2c_device_config_t conf;
    conf.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    conf.device_address = aI2CAddr;
    conf.scl_speed_hz = aI2CSpeed_Hz;
    return i2c_master_bus_add_device(mBusHandle, &conf, &mDevHandle);
  }
  else
    return ESP_OK;
}

ADS1x15::~ADS1x15(void)
{
  Close();
}

void ADS1x15::Close()
{
  if (mDevHandle != NULL)
  {
    i2c_master_bus_rm_device(mDevHandle);
    mDevHandle = NULL;
  }
}

esp_err_t ADS1x15::ConvReady(bool &aConvReady)
{
  esp_err_t ret;
  uint16_t Config = 0;
  ret = ReadRegister16(REG_CONFIG, Config);
  // ESP_LOGI("ADS1015", "Lese Config-Register: %d", (int)Config);
  aConvReady = (Config & (1 << 15)) > 0;
  return ret;
}

esp_err_t ADS1x15::ReadADC_1015(ADC_MP aInputMux, FSC_RANGE aFullScale, ADC_SPEED_1015 aSpeed, uint16_t &aADCValue)
{
  esp_err_t ret;

  // Init ADS1015 (die 0x3 am Ende schaltet den Komperator ab...)
  uint16_t Config = (1 << 15) | (aInputMux << 12) | (aFullScale << 9) | (1 << 8) | (aSpeed << 5) | 0x3;

  // Konfiguration schreiben und Konversion starten
  ret = WriteRegister16(REG_CONFIG, Config);
  if (ret != ESP_OK)
    return ret;

  int64_t StartTime = GetTime_us();

  bool cvr = false;

  // Achtung: Abweichend vom Datenblatt braucht der ADC im "Single-Shot-Modus"
  // deutlich länger (Fälschung?), als im kontinuierlichen Modus. Bei 128_SPS sind das ca. 500 ms (und nicht 4 ms lt. DB)!
  // bei 250_SPS sind es ca. 250 ms usw.
  // Timeout nach spätestens 600 ms.Länger dauert kein Sample!
  while (GetTime_us() - StartTime < 600000)
  {
    ret = ConvReady(cvr);
    if (ret != ESP_OK)
      return ret;
    if (cvr)
    {
      // Konversionsergebnis vom ADC lesen
      //double TimePast = (GetTime_us() - StartTime) / 1.0E6;
      //ESP_LOGI("ADS1015", "Zeit für ADC: %.3f s", TimePast);
      ret = ReadRegister16(REG_CONVERSION, aADCValue); 
      aADCValue = (aADCValue >> 4); // Die vier ersten LSBs haben keine Bedeutung!
      return ret;
    }
    else
      vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  return ESP_ERR_TIMEOUT;
}

esp_err_t ADS1x15::ReadADC_1115(ADC_MP aInputMux, FSC_RANGE aFullScale, ADC_SPEED_1115 aSpeed, uint16_t &aADCValue)
{
  esp_err_t ret;

  // Init ADS1015 (die 0x3 am Ende schaltet den Komperator ab...)
  uint16_t Config = (1 << 15) | (aInputMux << 12) | (aFullScale << 9) | (1 << 8) | (aSpeed << 5) | 0x3;

  // Konfiguration schreiben und Konversion starten
  ret = WriteRegister16(REG_CONFIG, Config);
  if (ret != ESP_OK)
    return ret;

  int64_t StartTime = GetTime_us();

  bool cvr = false;

  // Timeout nach spätestens 600 ms.Länger dauert kein Sample!
  while (GetTime_us() - StartTime < 600000)
  {
    ret = ConvReady(cvr);
    if (ret != ESP_OK)
      return ret;
    if (cvr)
    {
      // Konversionsergebnis vom ADC lesen
      //double TimePast = (GetTime_us() - StartTime) / 1.0E6;
      //ESP_LOGI("ADS1015", "Zeit für ADC: %.3f s", TimePast);
      ret = ReadRegister16(REG_CONVERSION, aADCValue);
      return ret;
    }
    else
      vTaskDelay(10 / portTICK_PERIOD_MS);
  }
  return ESP_ERR_TIMEOUT;
}

esp_err_t ADS1x15::ReadRegister16(ADC_REGISTER aReg, uint16_t &aRegVal)
{
  if (mDevHandle == NULL)
    return ESP_ERR_INVALID_ARG;
  esp_err_t ret;
  uint8_t RegAddr = aReg;
  uint8_t data[2] = {};
  ret = i2c_master_transmit_receive(mDevHandle, &RegAddr, 1, data, 2, DEV_TIMEOUT);
  aRegVal = (data[0] << 8) | data[1];
  if (ret != ESP_OK)
    ESP_LOGE("ADS1015::ReadRegister", "I2C error no.: %d", ret);

  return ret;
}

esp_err_t ADS1x15::WriteRegister16(ADC_REGISTER aReg, uint16_t aRegVal)
{
  esp_err_t ret;
  uint8_t data[3] = {aReg, (uint8_t)(aRegVal >> 8), (uint8_t)(aRegVal & 0xFF)};
  ret = i2c_master_transmit(mDevHandle, data, 3, DEV_TIMEOUT);
  if (ret != ESP_OK)
    ESP_LOGE("ADS1015::WriteRegister", "I2C error no.: %d", ret);
  return ret;
}
