#ifndef LORA_TEMP_H
#define LORA_TEMP_H

#include "lora.h"

void app_main_cpp();
esp_err_t SendLoraMsg(SX1278_LoRa & aLoRa, uint8_t* aBuf, uint16_t aSize, uint32_t aTag);
void task_tx(void *p);
void error(const char *format, ...);
void ParseLoraPacket(uint8_t *buf, uint8_t len);
void InitLora();
int64_t GetTime_us();
void InitSSD1306();

#endif