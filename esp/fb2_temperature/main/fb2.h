#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include "driver/i2c_master.h"
#include "esp_event.h"
#include "esp_adc/adc_oneshot.h"
#include "time.h"
#include <string>
#include <vector>

// Bestimmung des Zustandes beim Aufwachen
enum LoggerResetMode
{
  // Logger wurde neu gestartet durch das Anlegen der Betriebsspannung (Web-Interface aktiv)
  RESET_MODE_POWERON,
  // Logger wurde periodisch neu gestartet durch die RTC (Standard im Loggerbetrieb, Web-Interface NICHT aktiv)
  RESET_MODE_TIMER,
  // Logger wurde neu gestartet, da der Aktivierungs-Knopf gedrückt wurde (Web-Interface aktiv)
  RESET_MODE_KEY,
  // Hauptprozessor wurde durch den ULP aufgeweckt
  RESET_MODE_ULP,
};

// Sensor-Fehlerzähler und Status
struct SensorData
{
  uint32_t PC;
  uint32_t SHT40Err;
  uint32_t BMP390Err;
  uint32_t uptime_s;

  bool SkipSHT40;
  bool SkipBMP390;

  float Temp_deg;
  float Hum_per;
  float Press_mBar;
  float VBatt_V;
};

void app_main_cpp();
void error(const char *format, ...);

// Forwards
class SHT40;
class BMP390;

class logger
{
protected:
  
  static const int mSleepIntervalTime_s;
  LoggerResetMode mLoggerResetMode;
  int64_t mStartTime; // gültig nach dem Auslesen des Uhrenchips
  // Sensorik, global
  SensorData SD = {};
  i2c_master_bus_handle_t i2c_bus_h_0;
  adc_oneshot_unit_handle_t mADCHandle;

  SHT40 *Sht;
  BMP390 *Bmp;
  esp_err_t InitI2C(i2c_port_t aPort, gpio_num_t aSDA_Pin, gpio_num_t aSCL_Pin, i2c_master_bus_handle_t *aBusHandle);
  esp_err_t InitGPIO();
  std::string ReadSensorData();
  void GoSleep();
  float GetVBatt();
  uint32_t GetSecondsAfterStart();
public:
  logger();
  ~logger();
  void Run();
};

#endif