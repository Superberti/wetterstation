
#ifndef __LORA_H__
#define __LORA_H__

#include "driver/spi_master.h"

enum LoRaBoardTypes
{
  LilygoT3,
  HeltecESPLoRa,
  HeltecWirelessStick_V3,
  DevKitC_V4  // Board der Hauptwetterstation
};

class LoRa_PinConfiguration
{
public:
  uint8_t ChipSelect;
  uint8_t Reset;
  uint8_t Miso;
  uint8_t Mosi;
  uint8_t Clock;
  uint8_t DIO0;
	uint8_t DIO1;
	uint8_t Busy;
  uint8_t Led;
  uint8_t AdcChannel;
  LoRa_PinConfiguration(LoRaBoardTypes aBoard);
};


class SX1278_LoRa
{
  struct Registers
  {
    static const uint8_t REG_FIFO = 0x00;
    static const uint8_t REG_OP_MODE = 0x01;
    static const uint8_t REG_FRF_MSB = 0x06;
    static const uint8_t REG_FRF_MID = 0x07;
    static const uint8_t REG_FRF_LSB = 0x08;
    static const uint8_t REG_PA_CONFIG = 0x09;
    static const uint8_t REG_LNA = 0x0c;
    static const uint8_t REG_FIFO_ADDR_PTR = 0x0d;
    static const uint8_t REG_FIFO_TX_BASE_ADDR = 0x0e;
    static const uint8_t REG_FIFO_RX_BASE_ADDR = 0x0f;
    static const uint8_t REG_FIFO_RX_CURRENT_ADDR = 0x10;
    static const uint8_t REG_IRQ_MASK_FLAGS = 0x11;
    static const uint8_t REG_IRQ_FLAGS = 0x12;
    static const uint8_t REG_RX_NB_BYTES = 0x13;
    static const uint8_t REG_PKT_SNR_VALUE = 0x19;
    static const uint8_t REG_PKT_RSSI_VALUE = 0x1a;
    static const uint8_t REG_HOP_CHANNEL = 0x1c;
    static const uint8_t REG_MODEM_CONFIG_1 = 0x1d;
    static const uint8_t REG_MODEM_CONFIG_2 = 0x1e;
    static const uint8_t REG_PREAMBLE_MSB = 0x20;
    static const uint8_t REG_PREAMBLE_LSB = 0x21;
    static const uint8_t REG_PAYLOAD_LENGTH = 0x22;
    static const uint8_t REG_MODEM_CONFIG_3 = 0x26;
    static const uint8_t REG_RSSI_WIDEBAND = 0x2c;
    static const uint8_t REG_DETECTION_OPTIMIZE = 0x31;
    static const uint8_t REG_DETECTION_THRESHOLD = 0x37;
    static const uint8_t REG_SYNC_WORD = 0x39;
    static const uint8_t REG_DIO_MAPPING_1 = 0x40;
    static const uint8_t REG_VERSION = 0x42;
  };

  struct TransceiverMode
  {
    static const uint8_t MODE_LONG_RANGE_MODE = 0x80;
    static const uint8_t MODE_SLEEP = 0x00;
    static const uint8_t MODE_STDBY = 0x01;
    static const uint8_t MODE_TX = 0x03;
    static const uint8_t MODE_RX_CONTINUOUS = 0x05;
    static const uint8_t MODE_RX_SINGLE = 0x06;
  };

  struct IrqMask
  {
    static const uint8_t IRQ_TX_DONE_MASK = 0x08;
    static const uint8_t IRQ_PAYLOAD_CRC_ERROR_MASK = 0x20;
    static const uint8_t IRQ_RX_DONE_MASK = 0x40;
  };

  struct Misc
  {
    static const uint8_t PA_BOOST = 0x80;
    static const uint8_t PA_OUTPUT_RFO_PIN = 0;
    static const uint8_t PA_OUTPUT_PA_BOOST_PIN = 1;
    static const uint8_t TIMEOUT_RESET = 100;
  };

  spi_device_handle_t mhSpi;
  bool mInitialized;
  bool mImplicit;
  long mFrequency;
  uint8_t mAddress;
  static const uint16_t mLoraBufSize=256;
  uint8_t mLoraBuf[mLoraBufSize];
  
  esp_err_t ExplicitHeaderMode(void);
  esp_err_t ImplicitHeaderMode(uint8_t size);
  esp_err_t Idle(void);
  esp_err_t WriteReg(uint8_t reg, uint8_t val);
  esp_err_t ReadReg(uint8_t reg, uint8_t *aInVal);

  esp_err_t SetTxPower(uint8_t level);
  esp_err_t SetFrequency(long frequency);
  esp_err_t SetSpreadingFactor(uint8_t sf);
  esp_err_t SetBandwidth(long sbw);
  esp_err_t SetCodingRate(uint8_t denominator);
  esp_err_t SetPreambleLength(uint16_t length);
  esp_err_t SetSyncWord(uint8_t sw);
  esp_err_t Init(void);
  esp_err_t EnableCrc(void);
  esp_err_t DisableCrc(void);
  
  bool Initialized(){return mInitialized;}

public:
  LoRa_PinConfiguration *PinConfig;
  void Reset(void);
  void Close(void);
  esp_err_t Receive(void);
  esp_err_t Sleep(void);
  esp_err_t SendPacket(uint8_t *buf, uint8_t size);
  esp_err_t ReceivePacket(uint8_t *buf, uint8_t size, uint8_t *BytesRead);
  uint8_t Received(void);
  uint8_t PacketRssi(void);
  float PacketSnr(void);
  
  esp_err_t DumpRegisters(void);
  esp_err_t SetupModule(uint8_t aAddress, long aFrq, uint16_t aPreambleLength, long aBandwidth, uint8_t aSyncByte, uint8_t aSpreadingFactor, uint8_t aCodingRate, uint8_t aTxPower);
  esp_err_t SendLoraMsg(LoraCommand aCmd, uint8_t* aBuf, uint16_t aSize, uint32_t aTag);

  SX1278_LoRa(LoRaBoardTypes aBoard);
  ~SX1278_LoRa();
};
#endif
