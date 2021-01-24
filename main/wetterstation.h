#ifndef WETTERSTATION_H_INCLUDED
#define WETTERSTATION_H_INCLUDED

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
// Prototypen:

void disp_buf(uint8_t *buf, int len);
esp_err_t i2c_master_init(void);

static void event_handler(void* arg, esp_event_base_t event_base, int event_id, void* event_data);
static void wifi_init_sta(void);
static void get_device_service_name(char *service_name, size_t max);
esp_err_t custom_prov_data_handler(uint32_t session_id, const uint8_t *inbuf, ssize_t inlen,
                                   uint8_t **outbuf, ssize_t *outlen, void *priv_data);
static void mqtt_app_start(void);
static esp_err_t mqtt_event_handler_cb(esp_mqtt_event_handle_t event);
static void mqtt_event_handler(void *handler_args, esp_event_base_t base, int32_t event_id, void *event_data);

void SetLEDColor(uint8_t aLEDNum, uint8_t r, uint8_t g, uint8_t b);
void SetLEDColors(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2);


#endif // WETTERSTATION_H_INCLUDED
