
#ifndef __LORA_H__
#define __LORA_H__

#include "driver/spi_master.h"

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

  struct PinConfiguration
  {
    // Pin definitions für Heltec-ESP-LoRa
    
    static const uint8_t CONFIG_CS_GPIO = 18;
    static const uint8_t CONFIG_RST_GPIO = 14;
    static const uint8_t CONFIG_MISO_GPIO = 19;
    static const uint8_t CONFIG_MOSI_GPIO = 27;
    static const uint8_t CONFIG_SCK_GPIO = 5;
    static const uint8_t CONFIG_DIO0_GPIO = 33;

/*
    // Pin definitions für ESP-DevKit mit externem LoRa-Modul
    static const uint8_t CONFIG_CS_GPIO = 12;
    static const uint8_t CONFIG_RST_GPIO = 13;
    static const uint8_t CONFIG_MISO_GPIO = 15;
    static const uint8_t CONFIG_MOSI_GPIO = 16;
    static const uint8_t CONFIG_SCK_GPIO = 2;
    */
  };

  spi_device_handle_t mhSpi;
  bool mInitialized;
  bool mImplicit;
  long mFrequency;

  
  esp_err_t lora_explicit_header_mode(void);
  esp_err_t lora_implicit_header_mode(uint8_t size);
  esp_err_t lora_idle(void);
  esp_err_t lora_write_reg(uint8_t reg, uint8_t val);
  esp_err_t lora_read_reg(uint8_t reg, uint8_t *aInVal);

  esp_err_t lora_set_tx_power(uint8_t level);
  esp_err_t lora_set_frequency(long frequency);
  esp_err_t lora_set_spreading_factor(uint8_t sf);
  esp_err_t lora_set_bandwidth(long sbw);
  esp_err_t lora_set_coding_rate(uint8_t denominator);
  esp_err_t lora_set_preamble_length(uint16_t length);
  esp_err_t lora_set_sync_word(uint8_t sw);
  esp_err_t lora_init(void);
  esp_err_t lora_enable_crc(void);
  esp_err_t lora_disable_crc(void);
  void lora_close(void);
  bool lora_initialized(){return mInitialized;}

public:
  void lora_reset(void);
  esp_err_t lora_receive(void);
  esp_err_t lora_sleep(void);
  esp_err_t lora_send_packet(uint8_t *buf, uint8_t size);
  esp_err_t lora_receive_packet(uint8_t *buf, uint8_t size, uint8_t *BytesRead);
  uint8_t lora_received(void);
  uint8_t lora_packet_rssi(void);
  float lora_packet_snr(void);
  
  esp_err_t lora_dump_registers(void);
  esp_err_t SetupModule();

  SX1278_LoRa();
  ~SX1278_LoRa();
};
#endif
