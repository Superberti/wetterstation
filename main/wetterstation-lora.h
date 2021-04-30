#ifndef WETTERSTATION_LORA_H
#define WETTERSTATION_LORA_H

extern "C"
{
#include "../components/lora/include/lora.h"
}

void task_tx(void *p);
#endif