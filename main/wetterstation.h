#ifndef WETTERSTATION_H_INCLUDED
#define WETTERSTATION_H_INCLUDED

#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include <string>
#include <stdarg.h>
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
void led_task_init(void);
void led_cmd_task(void * arg);
void i2c_master_reset();

struct LEDData
{
  uint8_t r;
  uint8_t g;
  uint8_t b;
  uint8_t LEDNum;
};

// Die Sensoren der Wetterstation. Bis zu 16 Stück, ansonsten muss der
// Error-Status erweitert werden
enum SensorType
{
  eSHT35_0,
  eSHT35_1,
  eBMP280,
  eVEML7700,
};

void SetSensorErr(uint16_t & aSensorErr, SensorType aSensorType, bool aStatus);
bool GetSensorErr(const uint16_t aSensorErr, SensorType aSensorType);
std::string NRFCommand(std::string aCmd);
void NRFLog(std::string aLog);
int nrf_vprintf(const char *fmt, va_list args);

#endif // WETTERSTATION_H_INCLUDED
