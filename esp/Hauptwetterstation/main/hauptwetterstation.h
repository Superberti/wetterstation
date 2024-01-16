#ifndef HAUPTWETTERSTATION_H
#define HAUPTWETTERSTATION_H

#include "lora.h"

void app_main_cpp();
void task_tx(void *p);
void error(const char *format, ...);
void ParseLoraPacket(uint8_t *buf, uint8_t len);
esp_err_t InitLoRa(SX1278_LoRa & aLoRa);
int64_t GetTime_us();
void InitNokia_u8g2();
esp_err_t BuildCBORBuf(uint8_t *aBuf , uint16_t aMaxBufSize, uint16_t & aCBORBuildSize, uint32_t aPC, float aTemp_deg, float aHum_per, float aPress_mBar);
esp_err_t InitI2C(i2c_port_t aPort, int aSDA_Pin, int aSCL_Pin);
esp_err_t InitGPIO();
static void IRAM_ATTR gpio_isr_handler(void *arg);
#endif