/*
 * Ansteuerung eines BMP390 Luftdrucksensors.
 * Umgesetzt auf ESP32, nur I2C-Interface
 */
#ifndef __BMP390_H__
#define __BMP390_H__

#define BMP390_ADDRESS (0x77) /**< The default I2C address for the sensor. */
#define BMP390_CHIP_ID (0x60) /**< Default chip ID. */

// Chip-Register
enum
{
  BMP390_REGISTER_CHIP_ID = 0x00,
  BMP390_REGISTER_REV_ID = 0x01,
  BMP390_REGISTER_ERROR = 0x02,
  BMP390_REGISTER_STATUS = 0x03,
  BMP390_REGISTER_PRESS_DATA_START = 0x04,  // 24-Bit Druck
  //BMP390_REGISTER_DATA1 = 0x05,
  //BMP390_REGISTER_DATA2 = 0x06,
  BMP390_REGISTER_TEMP_DATA_START = 0x07,   // 24-Bit Temperatur
  //BMP390_REGISTER_DATA4 = 0x08,
  //BMP390_REGISTER_DATA5 = 0x09,
  BMP390_REGISTER_SENSORTIME0 = 0x0C,
  BMP390_REGISTER_SENSORTIME1 = 0x0C,
  BMP390_REGISTER_SENSORTIME2 = 0x0C,
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
typedef struct
{
  // Temperatur
  uint16_t NVM_PAR_T1;
  uint16_t NVM_PAR_T2;
  uint8_t NVM_PAR_T3;
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
} bmp390_calib_data;



/**
 * Driver for the Adafruit BMP390 barometric pressure sensor.
 */
class BMP390
{
public:
  /** Oversampling rate for the sensor. */
  enum sensor_sampling
  {
    /** No over-sampling. */
    SAMPLING_NONE = 0x00,
    /** 2x over-sampling. */
    SAMPLING_X2 = 0x01,
    /** 4x over-sampling. */
    SAMPLING_X4 = 0x02,
    /** 8x over-sampling. */
    SAMPLING_X8 = 0x03,
    /** 16x over-sampling. */
    SAMPLING_X16 = 0x04,
    /** 32x over-sampling. */
    SAMPLING_X32 = 0x05,
  };

  /** Operating mode for the sensor. */
  enum power_mode
  {
    /** Sleep mode. */
    MODE_SLEEP = 0x00,
    /** Forced mode. */
    MODE_FORCED = 0x01,
    /** Normal mode. */
    MODE_NORMAL = 0x03,
  };

  BMP390();

  ~BMP390(void);

  esp_err_t ReadTempAndPress(double & rTemp_C, double & rPress_mbar);
  esp_err_t StartReadTempAndPress();
  esp_err_t ReadTempAndPressAsync(double & rTemp_C, double & rPress_mbar);

  esp_err_t Init(uint8_t addr = BMP390_ADDRESS);
  void Reset(void);
  uint8_t GetStatus();

  esp_err_t ReadAltitude(double & aAlt, double seaLevelhPa = 1013.25);
  double SeaLevelForAltitude(double altitude, double atmospheric);
  double WaterBoilingPoint(double pressure);

private:
  // Berechnete Kalibrierfaktoren aus dem NVM
  double PAR_T1, PAR_T2, PAR_T3;
  double PAR_P1, PAR_P2, PAR_P3, PAR_P4, PAR_P5, PAR_P6, PAR_P7, PAR_P8, PAR_P9, PAR_P10, PAR_P11;
  double t_lin;

  esp_err_t ReadRegister(uint8_t reg_addr, uint8_t *data, uint16_t len);
  esp_err_t ReadCalibData(void);
  esp_err_t ReadTemperature(double & rTemp);
  esp_err_t ReadPressure(double & rPress);

  esp_err_t WriteRegister(uint8_t reg, uint8_t value);
  uint8_t read8(uint8_t reg, esp_err_t* rError=NULL);
  uint16_t read16(uint8_t reg, esp_err_t* rError=NULL);
  uint32_t read24(uint8_t reg, esp_err_t* rError=NULL);

  uint8_t _i2caddr;
  bool DataReady();
  bmp390_calib_data calib_data;
};

#endif
