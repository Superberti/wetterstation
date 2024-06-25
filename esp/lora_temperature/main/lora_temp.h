#ifndef LORA_TEMP_H
#define LORA_TEMP_H

#include "lora.h"

void app_main_cpp();
void task_tx(void *p);
void error(const char *format, ...);
void ParseLoraPacket(uint8_t *buf, uint8_t len);
esp_err_t InitLoRa(SX1278_LoRa & aLoRa);
void InitSSD1306_u8g2();
esp_err_t BuildCBORBuf(uint8_t *aBuf , uint16_t aMaxBufSize, uint16_t & aCBORBuildSize, uint32_t aPC, float aTemp_deg, float aHum_per, float aPress_mBar, float iVBatt_V);
esp_err_t InitI2C(i2c_port_t aPort, gpio_num_t aSDA_Pin, gpio_num_t aSCL_Pin, i2c_master_bus_handle_t * aBusHandle);
#endif