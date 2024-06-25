/*
 * Ansteuerung eines BMP390 Luftdrucksensors.
 * Umgesetzt auf ESP32, neuer I2C-Treiber aus dem IDF 5.2.2
 * O. Rutsch, 24.06.24
 */
#ifndef __BMP390_H__
#define __BMP390_H__

#include "driver/i2c_master.h"
#include "SensorTypes.h"

#define BMP390_ADDRESS (0x77) // Adresse, wenn OSD auf Masse. Bei 3.3 V ist die Adresse 0x76
#define BMP390_CHIP_ID (0x60) // Feste Chip-ID

// Chip-Register
enum
{
  BMP390_REGISTER_CHIP_ID = 0x00,
  BMP390_REGISTER_REV_ID = 0x01,
  BMP390_REGISTER_ERROR = 0x02,
  BMP390_REGISTER_STATUS = 0x03,
  BMP390_REGISTER_PRESS_DATA_START = 0x04, // 24-Bit Druck

  BMP390_REGISTER_TEMP_DATA_START = 0x07, // 24-Bit Temperatur

  BMP390_REGISTER_SENSORTIME0 = 0x0C,
  BMP390_REGISTER_SENSORTIME1 = 0x0D,
  BMP390_REGISTER_SENSORTIME2 = 0x0E,
  BMP390_REGISTER_EVENT = 0x10,
  BMP390_REGISTER_INT_STATUS = 0x11,
  BMP390_REGISTER_FIFO_LENGTH0 = 0x12,
  BMP390_REGISTER_FIFO_LENGTH1 = 0x13,
  BMP390_REGISTER_FIFO_DATA = 0x14,
  BMP390_REGISTER_FIFO_WTM0 = 0x15,
  BMP390_REGISTER_FIFO_WTM1 = 0x16,
  BMP390_REGISTER_FIFO_CONFIG1 = 0x17,
  BMP390_REGISTER_FIFO_CONFIG2 = 0x18,
  BMP390_REGISTER_INT_CTRL = 0x19,
  BMP390_REGISTER_IF_CONF = 0x1A,
  BMP390_REGISTER_PWR_CTRL = 0x1B,
  BMP390_REGISTER_OSR = 0x1C,
  BMP390_REGISTER_ODR = 0x1D,
  BMP390_REGISTER_CONFIG = 0x1F,
  BMP390_REGISTER_NVM_START = 0x31,
  BMP390_REGISTER_CMD = 0x7E,
};

// Kommandos in Verbindung mit Register BMP390_REGISTER_CMD
enum
{
  BMP390_CMD_FIFO_FLUSH = 0xB0,
  BMP390_CMD_SOFTRESET = 0xB6,
};

// Kalibrierdaten des Sensors im NVM
struct bmp390_calib_data
{
  // Temperatur
  uint16_t NVM_PAR_T1;
  uint16_t NVM_PAR_T2;
  int8_t NVM_PAR_T3;
  // Druck
  int16_t NVM_PAR_P1;
  int16_t NVM_PAR_P2;
  int8_t NVM_PAR_P3;
  int8_t NVM_PAR_P4;
  uint16_t NVM_PAR_P5;
  uint16_t NVM_PAR_P6;
  int8_t NVM_PAR_P7;
  int8_t NVM_PAR_P8;
  int16_t NVM_PAR_P9;
  int8_t NVM_PAR_P10;
  int8_t NVM_PAR_P11;
} __attribute__((packed));

/// @brief Bosch BMP390 Sensor
class BMP390
{
protected:
  i2c_master_bus_handle_t mBusHandle;
  i2c_master_dev_handle_t mDevHandle;
  // Berechnete Kalibrierfaktoren aus dem NVM
  float PAR_T1, PAR_T2, PAR_T3;
  float PAR_P1, PAR_P2, PAR_P3, PAR_P4, PAR_P5, PAR_P6, PAR_P7, PAR_P8, PAR_P9, PAR_P10, PAR_P11;
  float t_lin;
  bmp390_calib_data calib_data;

  /// @brief I2C-Register mit beliebiger Länge lesen
  /// @param reg_addr Registeradresse
  /// @param data Datenpointer
  /// @param len Länge der zu lesenden Daten
  /// @return 
  esp_err_t ReadRegister(uint8_t reg_addr, uint8_t *data, uint16_t len);

  /// @brief Kalibrierdaten des BMP390 auslesen und in PAR_xx speichern
  /// @param  
  /// @return 
  esp_err_t ReadCalibData(void);

  /// @brief Temperaturregister auslesen und Umrechnung in die Temperatur
  /// @param rTemp Ausgelesene Temperatur in °C
  /// @return Status
  esp_err_t ReadTemperature(float &rTemp);

  /// @brief Druckregister auslesen und Umrechnung in Druck
  /// @param rPress Absoluter Druck in mbar
  /// @return Status
  esp_err_t ReadPressure(float &rPress);

  /// @brief 8-Bit-Register schreiben
  /// @param reg_addr Registeradresse
  /// @param value Zu schreibender Wert
  /// @return Status
  esp_err_t WriteRegister(uint8_t reg_addr, uint8_t value);

  /// @brief 8-Bit-Register lesen
  /// @param reg_addr Registeradresse 
  /// @param rError Status, falls gewünscht
  /// @return 
  uint8_t Read8(uint8_t reg_addr, esp_err_t *rError = NULL);

public:
  
  /// @brief Oversampling-Einstellungen Druck und Temperatur
  enum sensor_sampling
  {
    SAMPLING_NONE = 0x00,
    SAMPLING_X2 = 0x01,
    SAMPLING_X4 = 0x02,
    SAMPLING_X8 = 0x03,
    SAMPLING_X16 = 0x04,
    SAMPLING_X32 = 0x05,
  };

  /// @brief Betriebsarten des Sensors
  enum power_mode
  {
    MODE_SLEEP = 0x00,  // Ruhezustand
    MODE_FORCED = 0x01, // One-Shot-Messung
    MODE_NORMAL = 0x03, // Dauermessung
  };

  /// @brief Konstruktor BMP390-Sensor
  /// @param aPort I2C-Port des ESP32
  /// @param aI2CAddr I2C-Adresse
  BMP390();
  ~BMP390(void);

  /// @brief BMP390-Sensor initialisieren und Kalibrierdaten lesen
  esp_err_t Init(i2c_master_bus_handle_t aBusHandle, uint8_t aI2CAddr, uint32_t aI2CSpeed_Hz);

  /// @brief Gibt es Messwerte, die gelesen werden können?
  /// @return Messwerte lesbar
  bool DataReady();

  /// @brief Temperatur und Druck im Force-Mode synchron lesen, reine Messzeit ca. 70 ms. Hier werden 500 gewartet...
  /// @param rTemp_C Temperatur
  /// @param rPress_mbar Druck
  /// @return Status
  esp_err_t ReadTempAndPress(float &rTemp_C, float &rPress_mbar);

  /// @brief Auslesevorgang starten
  /// @return Status
  esp_err_t StartReadTempAndPress();

  /// @brief Temperatur und Druck auslesen mit einem zuvor gestartetem Auslesevorgang (StartReadTempAndPress)
  /// @brief Falls noch keine Werte vorliegen, dann ESP_ERR_INVALID_RESPONSE und noch einmal probieren
  /// @param rTemp_C Temperatur
  /// @param rPress_mbar Druck
  /// @param rPress_mbar Wartet, bis die Daten anliegen. Timeout bei 1 s
  /// @return Status
  esp_err_t ReadTempAndPressAsync(float &rTemp_C, float &rPress_mbar, bool aWaitForData = true);

  /// @brief Sensor Soft-Reset
  /// @param  
  void Reset(void);

  /// @brief Statusregister auslesen
  /// @return Statusregister
  uint8_t GetStatus();

  /// @brief Höhe berechnen, falls der aktuelle Druck auf Meereshöhe bekannt ist
  /// @param aAlt Gemessener Absolutdruck
  /// @param seaLevelhPa Aktueller Druck auf Meereshöhe
  /// @return Status
  esp_err_t ReadAltitude(float &aAlt, float seaLevelhPa = 1013.25);

  /// @brief Luftdruck auf Meereshöhe umrechnen, falls die aktuelle Höhe bekannt ist
  /// @param altitude Aktuelle Höhe üNN
  /// @param atmospheric Gemessener Absolutdruck
  /// @return Druck relativ zur Meereshöhe
  float SeaLevelForAltitude(float altitude, float atmospheric);

  /// @brief Siedetemperatur von Wasser berechnen
  /// @param pressure Absolutdruck
  /// @return Siedepunkt Wasser in °C
  float WaterBoilingPoint(float pressure);
};

#endif
