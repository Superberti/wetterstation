
#ifndef __LORA_H__
#define __LORA_H__

#include "driver/spi_master.h"

enum LoRaBoardTypes
{
  LilygoT3,
  HeltecESPLoRa,
  HeltecWirelessStick_V3
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
  spi_host_device_t SPIChannel;
  bool UseTXCO;
  LoRa_PinConfiguration(LoRaBoardTypes aBoard);
};

class LoRaBase
{
protected:
  spi_device_handle_t mhSpi;
  bool mInitialized;
  bool mImplicit;
  uint32_t mFrequency;
  uint8_t mAddress;
  static const uint16_t mLoraBufSize = 256;
  uint8_t mLoraBuf[mLoraBufSize];

  esp_err_t SPIBusInit();
  void WaitBusy();
  virtual esp_err_t SetFrequency(uint32_t frequency) = 0;

public:
  enum SpreadingFactor
  {
    SF5 = 5,
    SF6 = 6,
    SF7 = 7,
    SF8 = 8,
    SF9 = 9,
    SF10 = 10,
    SF11 = 11,
    SF12 = 12,
  };

  enum LoRaBandwidth
  {
    LORA_BW_7 = 0x00,   // (7.81 kHz real)
    LORA_BW_10 = 0x08,  // (10.42 kHz real)
    LORA_BW_15 = 0x01,  // (15.63 kHz real)
    LORA_BW_20 = 0x09,  // (20.83 kHz real)
    LORA_BW_31 = 0x02,  // (31.25 kHz real)
    LORA_BW_41 = 0x0A,  // (41.67 kHz real)
    LORA_BW_62 = 0x03,  // (62.50 kHz real)
    LORA_BW_125 = 0x04, // (125 kHz real)
    LORA_BW_250 = 0x05, // (250 kHz real)
    LORA_BW_500 = 0x06, // (500 kHz real)
  };

  enum LoRaCodingRate
  {
    LORA_CR_4_5 = 0x01,
    LORA_CR_4_6 = 0x02,
    LORA_CR_4_7 = 0x03,
    LORA_CR_4_8 = 0x04,
  };

  LoRa_PinConfiguration *PinConfig;
  void Reset();
  void CloseSPI();
  bool Initialized() { return mInitialized; }

  virtual void Close();
  virtual esp_err_t DumpRegisters() = 0;
  virtual esp_err_t SetupModule(uint8_t aAddress, uint32_t aFrq, uint16_t aPreambleLength, LoRaBandwidth aBandwidth,
                                uint16_t aSyncWord, SpreadingFactor aSpreadingFactor, LoRaCodingRate aCodingRate, int8_t aTxPower) = 0;
  virtual esp_err_t SendLoraMsg(LoraCommand aCmd, uint8_t *aBuf, uint16_t aSize, uint32_t aTag);
  virtual esp_err_t SendPacket(uint8_t *buf, uint8_t size) = 0;
  virtual esp_err_t ReceivePacket(uint8_t *buf, uint8_t size, uint8_t *BytesRead) = 0;
  virtual esp_err_t Sleep() = 0;
  LoRaBase(LoRaBoardTypes aBoard);
  virtual ~LoRaBase();
};

class SX1278_LoRa : public LoRaBase
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
  esp_err_t WriteReg(uint8_t reg, uint8_t val);
  esp_err_t ReadReg(uint8_t reg, uint8_t *aInVal);
  esp_err_t ExplicitHeaderMode();
  esp_err_t ImplicitHeaderMode(uint8_t size);
  esp_err_t Idle();
  esp_err_t SetTxPower(uint8_t level);
  esp_err_t SetFrequency(uint32_t frequency);
  esp_err_t SetSpreadingFactor(SpreadingFactor sf);
  esp_err_t SetBandwidth(LoRaBandwidth sbw);
  esp_err_t SetCodingRate(LoRaCodingRate aCR);
  esp_err_t SetPreambleLength(uint16_t length);
  esp_err_t SetSyncWord(uint8_t sw);
  esp_err_t Init();
  esp_err_t EnableCrc();
  esp_err_t DisableCrc();

public:
  esp_err_t Receive();
  esp_err_t Sleep();
  esp_err_t SendPacket(uint8_t *buf, uint8_t size);
  esp_err_t ReceivePacket(uint8_t *buf, uint8_t size, uint8_t *BytesRead);
  uint8_t Received();
  uint8_t PacketRssi();
  float PacketSnr();

  esp_err_t DumpRegisters();
  esp_err_t SetupModule(uint8_t aAddress, uint32_t aFrq, uint16_t aPreambleLength, LoRaBandwidth aBandwidth,
                        uint16_t aSyncWord, SpreadingFactor aSpreadingFactor, LoRaCodingRate aCodingRate, int8_t aTxPower);

  SX1278_LoRa(LoRaBoardTypes aBoard);
  //~SX1278_LoRa();
};

class SX1262_LoRa : public LoRaBase
{
  struct Opcodes
  {
    static const uint8_t NOP = 0x00;
    static const uint8_t OP_RESET_STATS = 0x00;
    static const uint8_t OP_CLR_IRQ_STATUS = 0x02;
    static const uint8_t OP_CLR_DEV_ERR = 0x07;
    static const uint8_t OP_SET_DIO_IRQ_PARAMS = 0x08;
    static const uint8_t OP_WRITE_REG = 0x0D;
    static const uint8_t OP_WRITE_BUF = 0x0E;
    static const uint8_t OP_GET_STATS = 0x10;
    static const uint8_t OP_GET_PKG_TYPE = 0x11;
    static const uint8_t OP_GET_IRQ_STATUS = 0x12;
    static const uint8_t OP_GET_RX_BUF_STAT = 0x13;
    static const uint8_t OP_GET_PKG_STAT = 0x14;
    static const uint8_t OP_GET_RSSI_INST = 0x15;
    static const uint8_t OP_GET_DEV_ERR = 0x17;
    static const uint8_t OP_READ_REG = 0x1D;
    static const uint8_t OP_READ_BUF = 0x1E;
    static const uint8_t OP_SET_RX = 0x82;
    static const uint8_t OP_SET_TX = 0x83;
    static const uint8_t OP_SET_STANDBY = 0x80;
    static const uint8_t OP_SET_SLEEP = 0x84;
    static const uint8_t OP_SET_RF_FRQ = 0x86;
    static const uint8_t OP_SET_CAD_PARAMS = 0x88;
    static const uint8_t OP_CALIBRATE = 0x89;
    static const uint8_t OP_SET_PKG_TYPE = 0x8A;
    static const uint8_t OP_SET_MOD_PARAMS = 0x8B;
    static const uint8_t OP_SET_PKG_PARAMS = 0x8C;
    static const uint8_t OP_SET_TX_PARAMS = 0x8E;
    static const uint8_t OP_SET_BUFFER_BASE_ADDR = 0x8F;
    static const uint8_t OP_SET_RXTX_FALLBACK_MODE = 0x93;
    static const uint8_t OP_SET_RX_DUTY_CYCLE = 0x94;
    static const uint8_t OP_SET_PA_CONFIG = 0x95;
    static const uint8_t OP_SET_REGUL_MODE = 0x96;
    static const uint8_t OP_SET_DIO3_AS_TCXO_CTRL = 0x97;
    static const uint8_t OP_CALIBRATE_IMAGE = 0x98;
    static const uint8_t OP_SET_DIO2_AS_RF_SWITCH_CTRL = 0x9D;
    static const uint8_t OP_STOP_TIMER_ON_PREAMBLE = 0x9F;
    static const uint8_t OP_SET_LORA_SYM_NUM_TO = 0xA0;
    static const uint8_t OP_GET_STATUS = 0xC0;
    static const uint8_t OP_SET_FS = 0xC1;
    static const uint8_t OP_SET_CAD = 0xC5;
    static const uint8_t OP_SET_TX_CONT_WAVE = 0xD1;
    static const uint8_t OP_SET_TX_INF_PREAMBLE = 0xD2;
  };

  struct Registers
  {
    static const uint16_t REG_WHITE_INIT_MSB = 0x06B8;
    static const uint16_t REG_WHITE_INIT_LSB = 0x06B9;
    static const uint16_t REG_CRC_INIT_MSB = 0x06BC;
    static const uint16_t REG_CRC_INIT_LSB = 0x06BD;
    static const uint16_t REG_CRC_POLY_MSB = 0x06BE;
    static const uint16_t REG_CRC_POLY_LSB = 0x06BF;
    static const uint16_t REG_FSK_SYNC_0 = 0x06C0;
    static const uint16_t REG_FSK_SYNC_1 = 0x06C1;
    static const uint16_t REG_FSK_SYNC_2 = 0x06C2;
    static const uint16_t REG_FSK_SYNC_3 = 0x06C3;
    static const uint16_t REG_FSK_SYNC_4 = 0x06C4;
    static const uint16_t REG_FSK_SYNC_5 = 0x06C5;
    static const uint16_t REG_FSK_SYNC_6 = 0x06C6;
    static const uint16_t REG_FSK_SYNC_7 = 0x06C7;
    static const uint16_t REG_FSK_NODE_ADDRESS = 0x06CD;
    static const uint16_t REG_FSK_BROADCAST_ADDRESS = 0x06CE;
    static const uint16_t REG_LORA_SYNC_MSB = 0x0740;
    static const uint16_t REG_LORA_SYNC_LSB = 0x0741;
    static const uint16_t REG_RANDOM_DATA_0 = 0x0819;
    static const uint16_t REG_RANDOM_DATA_1 = 0x081A;
    static const uint16_t REG_RANDOM_DATA_2 = 0x081B;
    static const uint16_t REG_RANDOM_DATA_3 = 0x081C;
    static const uint16_t REG_RX_GAIN = 0x08AC;
    static const uint16_t REG_OCP_CONFIG = 0x08E7;
    static const uint16_t REG_XTA_TRIM = 0x0911;
    static const uint16_t REG_XTB_TRIM = 0x0912;
  };

  enum RadioMode
  {
    RM_STANDBY,
    RM_SLEEP,
    RM_TX_ENABLE,
    RM_RX_ENABLE,
  };

  enum PacketType
  {
    PACKET_TYPE_GFSK = 0,
    PACKET_TYPE_LORA = 1
  };

  enum RampTime
  {
    SET_RAMP_10U = 0x00,
    SET_RAMP_20U = 0x01,
    SET_RAMP_40U = 0x02,
    SET_RAMP_80U = 0x03,
    SET_RAMP_200U = 0x04,
    SET_RAMP_800U = 0x05,
    SET_RAMP_1700U = 0x06,
    SET_RAMP_3400U = 0x07,
  };

  enum TXCO_Voltage
  {
    TCXO_CTRL_1_6V = 0x00,
    TCXO_CTRL_1_7V = 0x01,
    TCXO_CTRL_1_8V = 0x02,
    TCXO_CTRL_2_2V = 0x03,
    TCXO_CTRL_2_4V = 0x04,
    TCXO_CTRL_2_7V = 0x05,
    TCXO_CTRL_3_0V = 0x06,
    TCXO_CTRL_3_3V = 0x07,
  };

  struct IRQFlags
  {
    static const uint16_t TxDone = 1 << 0;
    static const uint16_t RxDone = 1 << 1;
    static const uint16_t PreambleDetected = 1 << 2;
    static const uint16_t SyncWordValid = 1 << 3;
    static const uint16_t HeaderValid = 1 << 4;
    static const uint16_t HeaderErr = 1 << 5;
    static const uint16_t CRCErr = 1 << 6;
    static const uint16_t CadDone = 1 << 7;
    static const uint16_t CadDetected = 1 << 8;
    static const uint16_t Timeout = 1 << 9;
  };

  uint8_t SPIRecBuf[262];
  uint8_t SPITransBuf[262];
  uint16_t mPreambleLength;

  esp_err_t SetRadioMode(RadioMode aRM, uint32_t aTimeout);
  esp_err_t SetPacketType(PacketType);
  esp_err_t SetTxParams(RampTime aRampTime, int8_t aTXPower_DB);
  esp_err_t SetModulationParams(SpreadingFactor aSF, LoRaBandwidth aBW, LoRaCodingRate aCR, bool aLowDataRateOpt);
  esp_err_t SetPacketParams(uint16_t aPreambleLength, bool aImplicitHeader, uint8_t aPayloadLength, bool aUseCRC);
  esp_err_t ChipCommand(uint8_t *aParams, uint8_t aParamLength);
  esp_err_t SetFrequency(uint32_t aFrequency);
  esp_err_t SetSyncWord(uint16_t aSyncWord);
  esp_err_t ReadReg(uint16_t aAddr, uint8_t *aParams, uint8_t aParamLength);
  esp_err_t WriteReg(uint16_t aAddr, uint8_t *aParams, uint8_t aParamLength);
  esp_err_t ReadBuffer(uint8_t aOffset, uint8_t *aParams, uint8_t aParamLength);
  esp_err_t WriteBuffer(uint8_t aOffset, uint8_t *aParams, uint8_t aParamLength);
  esp_err_t SetBufferBaseAddress(uint8_t aTxBaseAddr, uint8_t aRxBaseAddr);
  esp_err_t SetCadParams(uint8_t aSymbolNum, uint8_t aDetPeak, uint8_t aDetMin, uint8_t aExitMode, uint32_t aTimeout);
  esp_err_t GetIRQStatus(uint16_t &aStatus);
  esp_err_t ClearIRQStatus(uint16_t aIRQsToClear);
  esp_err_t SetIRQParams(uint16_t aIRQMask, uint16_t DIO1Mask, uint16_t DIO2Mask, uint16_t DIO3Mask);
  esp_err_t GetDeviceError(uint16_t &aErr);
  esp_err_t ClearDeviceError();
  esp_err_t SetDio2AsRfSwitchCtrl(bool aOn);
  esp_err_t SetDio3AsTcxoCtrl(TXCO_Voltage, uint32_t aTimeout);
  esp_err_t SetCalibrateSections(uint8_t aCP);
  esp_err_t Calibrate(double aFrq_MHz);

public:
  esp_err_t Receive();
  esp_err_t Sleep();
  esp_err_t SendPacket(uint8_t *buf, uint8_t size);
  esp_err_t ReceivePacket(uint8_t *buf, uint8_t size, uint8_t *BytesRead);
  uint8_t Received();
  uint8_t PacketRssi();
  float PacketSnr();

  esp_err_t DumpRegisters();
  esp_err_t SetupModule(uint8_t aAddress, uint32_t aFrq, uint16_t aPreambleLength, LoRaBandwidth aBandwidth,
                        uint16_t aSyncWord, SpreadingFactor aSpreadingFactor, LoRaCodingRate aCodingRate, int8_t aTxPower);

  SX1262_LoRa(LoRaBoardTypes aBoard);
};
#endif
