#ifndef HELTEC_V3_H
#define HELTEC_V3_H

#include "lora.h"

void app_main_cpp();
void task_tx(void *p);
void error(const char *format, ...);
void ParseLoraPacket(uint8_t *buf, uint8_t len);
esp_err_t InitLoRa(LoRaBase &aLoRa);
int64_t GetTime_us();
void InitDisplay_u8g2();
esp_err_t BuildCBORBuf(uint8_t *aBuf, uint16_t aMaxBufSize, uint16_t &aCBORBuildSize);
esp_err_t InitGPIO();

#endif