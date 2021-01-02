/*!
 * Ansteuerung eines BMP280 Luftdrucksensors. Inspiriert von der Adafruit-Lib.
 * Umgesetzt auf ESP32, nur I2C-Interface
 */
#ifndef __VEML7700_H__
#define __VEML7700_H__


/*!
 *  I2C ADDRESS/BITS/SETTINGS
 */
#define VEML7700_ADDRESS (0x10) /**< The default I2C address for the sensor. */

#define VEML7700_ALS_CONFIG 0x00        ///< Light configuration register
#define VEML7700_ALS_THREHOLD_HIGH 0x01 ///< Light high threshold for irq
#define VEML7700_ALS_THREHOLD_LOW 0x02  ///< Light low threshold for irq
#define VEML7700_ALS_POWER_SAVE 0x03    ///< Power save regiester
#define VEML7700_ALS_DATA 0x04          ///< The light data output
#define VEML7700_WHITE_DATA 0x05        ///< The white light data output
#define VEML7700_INTERRUPTSTATUS 0x06   ///< What IRQ (if any)

#define VEML7700_INTERRUPT_HIGH 0x4000 ///< Interrupt status for high threshold
#define VEML7700_INTERRUPT_LOW 0x8000  ///< Interrupt status for low threshold

#define VEML7700_GAIN_1 0x00   ///< ALS gain 1x
#define VEML7700_GAIN_2 0x01   ///< ALS gain 2x
#define VEML7700_GAIN_1_8 0x02 ///< ALS gain 1/8x
#define VEML7700_GAIN_1_4 0x03 ///< ALS gain 1/4x

#define VEML7700_IT_100MS 0x00 ///< ALS intetgration time 100ms
#define VEML7700_IT_200MS 0x01 ///< ALS intetgration time 200ms
#define VEML7700_IT_400MS 0x02 ///< ALS intetgration time 400ms
#define VEML7700_IT_800MS 0x03 ///< ALS intetgration time 800ms
#define VEML7700_IT_50MS 0x08  ///< ALS intetgration time 50ms
#define VEML7700_IT_25MS 0x0C  ///< ALS intetgration time 25ms

#define VEML7700_PERS_1 0x00 ///< ALS irq persisance 1 sample
#define VEML7700_PERS_2 0x01 ///< ALS irq persisance 2 samples
#define VEML7700_PERS_4 0x02 ///< ALS irq persisance 4 samples
#define VEML7700_PERS_8 0x03 ///< ALS irq persisance 8 samples

#define VEML7700_POWERSAVE_MODE1 0x00 ///< Power saving mode 1
#define VEML7700_POWERSAVE_MODE2 0x01 ///< Power saving mode 2
#define VEML7700_POWERSAVE_MODE3 0x02 ///< Power saving mode 3
#define VEML7700_POWERSAVE_MODE4 0x03 ///< Power saving mode 4

union ConfigRegister
{
  uint16_t val;
  struct
  {
    uint8_t ALS_SD : 1;
    uint8_t ALS_INT_ENT : 1;
    uint8_t Res1 : 2;
    uint8_t ALS_PERS : 2;
    uint8_t ALS_IT : 4;
    uint8_t Res2 : 1;
    uint8_t ALS_GAIN : 2;
    uint8_t Res3 : 3;
  } bits;
};

class VEML7700
{
public:


  VEML7700();

  ~VEML7700(void);

  bool init(uint8_t aGain=VEML7700_GAIN_1, uint8_t aIntegrationTime=VEML7700_IT_100MS);

  double readLux();
  double readLuxNormalized();

  uint16_t readALS();
  double readWhite();
  double readWhiteNormalized();

private:
  uint8_t mGain, mIntegrationTime;
  uint16_t swapByteOrder(uint16_t us);
  uint32_t swapByteOrder(uint32_t ui);
  double normalize_resolution(double value);

  esp_err_t ReadRegister(uint8_t reg_addr, uint8_t *data, uint16_t len);
  esp_err_t WriteRegister(uint8_t reg, uint16_t value);

};

#endif
