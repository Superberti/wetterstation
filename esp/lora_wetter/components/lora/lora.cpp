#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include <string.h>
#include "esp_log.h"

#include "include/lora.h"

SX1278_LoRa::SX1278_LoRa()
  : mhSpi(NULL)
  , mInitialized(false)
  , mImplicit(false)
{
}

SX1278_LoRa::~SX1278_LoRa()
{
  lora_close();
}

esp_err_t SX1278_LoRa::SetupModule()
{
  mInitialized = false;
  esp_err_t ret;
  ret = lora_init();
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "lora_init failed: %d", ret);
    return ret;
  }
  ret = lora_explicit_header_mode();
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "lora_explicit_header_mode failed: %d", ret);
    return ret;
  }
  ret = lora_set_frequency(434.54e6);
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "lora_set_frequency failed: %d", ret);
    return ret;
  }
  ret = lora_enable_crc();
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "lora_enable_crc failed: %d", ret);
    return ret;
  }

  ret = lora_set_preamble_length(14);
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "lora_set_preamble_length failed: %d", ret);
    return ret;
  }
  ret = lora_set_bandwidth(500E3);
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "lora_set_bandwidth failed: %d", ret);
    return ret;
  }
  ret = lora_set_sync_word(0x3d);
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "lora_set_sync_word failed: %d", ret);
    return ret;
  }
  mInitialized = true;
  return ret;
}

/**
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
esp_err_t SX1278_LoRa::lora_write_reg(uint8_t reg, uint8_t val)
{
  uint8_t out[2] = {(uint8_t)(0x80 | reg), val};
  uint8_t in[2];

  spi_transaction_t t={};
  t.flags = 0;
  t.length = 8 * sizeof(out);
  t.tx_buffer = out;
  t.rx_buffer = in;

  gpio_set_level((gpio_num_t)PinConfiguration::CONFIG_CS_GPIO, 0);
  esp_err_t ret = spi_device_transmit(mhSpi, &t);
  gpio_set_level((gpio_num_t)PinConfiguration::CONFIG_CS_GPIO, 1);
  return ret;
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
esp_err_t SX1278_LoRa::lora_read_reg(uint8_t reg, uint8_t *aInVal)
{
  uint8_t out[2] = {reg, 0xff};
  uint8_t in[2];

  spi_transaction_t t={};
  t.flags = 0;
  t.length = 8 * sizeof(out);
  t.tx_buffer = out;
  t.rx_buffer = in;

  gpio_set_level((gpio_num_t)PinConfiguration::CONFIG_CS_GPIO, 0);
  esp_err_t ret = spi_device_transmit(mhSpi, &t);
  gpio_set_level((gpio_num_t)PinConfiguration::CONFIG_CS_GPIO, 1);
  *aInVal = in[1];
  return ret;
}

/**
 * Perform physical reset on the Lora chip
 */
void SX1278_LoRa::lora_reset(void)
{
  gpio_set_level((gpio_num_t)PinConfiguration::CONFIG_RST_GPIO, 0);
  vTaskDelay(pdMS_TO_TICKS(1));
  gpio_set_level((gpio_num_t)PinConfiguration::CONFIG_RST_GPIO, 1);
  vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
esp_err_t SX1278_LoRa::lora_explicit_header_mode(void)
{
  uint8_t reg;
  esp_err_t ret = lora_read_reg(Registers::REG_MODEM_CONFIG_1, &reg);
  if (ret != ESP_OK)
    return ret;
  mImplicit = false;
  return lora_write_reg(Registers::REG_MODEM_CONFIG_1, reg & 0xfe);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
esp_err_t SX1278_LoRa::lora_implicit_header_mode(uint8_t size)
{
  uint8_t reg;
  mImplicit = true;
  esp_err_t ret = lora_read_reg(Registers::REG_MODEM_CONFIG_1, &reg);
  if (ret != ESP_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_MODEM_CONFIG_1, reg | 0x01);
  if (ret != ESP_OK)
    return ret;
  return lora_write_reg(Registers::REG_PAYLOAD_LENGTH, size);
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
esp_err_t SX1278_LoRa::lora_idle(void)
{
  return lora_write_reg(Registers::REG_OP_MODE, TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
esp_err_t SX1278_LoRa::lora_sleep(void)
{
  return lora_write_reg(Registers::REG_OP_MODE, TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_SLEEP);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
esp_err_t SX1278_LoRa::lora_receive(void)
{
  return lora_write_reg(Registers::REG_OP_MODE, TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_RX_CONTINUOUS);
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
esp_err_t SX1278_LoRa::lora_set_tx_power(uint8_t level)
{
  // RF9x module uses PA_BOOST pin
  if (level < 2)
    level = 2;
  else if (level > 17)
    level = 17;
  return lora_write_reg(Registers::REG_PA_CONFIG, Misc::PA_BOOST | (level - 2));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
esp_err_t SX1278_LoRa::lora_set_frequency(long frequency)
{
  mFrequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
  esp_err_t ret;
  ret = lora_write_reg(Registers::REG_FRF_MSB, (uint8_t)(frf >> 16));
  if (ret != ESP_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_FRF_MID, (uint8_t)(frf >> 8));
  if (ret != ESP_OK)
    return ret;
  return lora_write_reg(Registers::REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
esp_err_t SX1278_LoRa::lora_set_spreading_factor(uint8_t sf)
{
  esp_err_t ret;
  if (sf < 6)
    sf = 6;
  else if (sf > 12)
    sf = 12;

  if (sf == 6)
  {
    ret = lora_write_reg(Registers::REG_DETECTION_OPTIMIZE, 0xc5);
    if (ret != ESP_OK)
      return ret;
    ret = lora_write_reg(Registers::REG_DETECTION_THRESHOLD, 0x0c);
    if (ret != ESP_OK)
      return ret;
  }
  else
  {
    ret = lora_write_reg(Registers::REG_DETECTION_OPTIMIZE, 0xc3);
    if (ret != ESP_OK)
      return ret;
    ret = lora_write_reg(Registers::REG_DETECTION_THRESHOLD, 0x0a);
    if (ret != ESP_OK)
      return ret;
  }
  uint8_t reg;
  ret = lora_read_reg(Registers::REG_MODEM_CONFIG_2, &reg);
  if (ret != ESP_OK)
    return ret;
  return lora_write_reg(Registers::REG_MODEM_CONFIG_2, (reg & 0x0f) | ((sf << 4) & 0xf0));
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Bandwidth in Hz (up to 500000)
 */
esp_err_t SX1278_LoRa::lora_set_bandwidth(long sbw)
{
  uint8_t bw;

  if (sbw <= 7.8E3)
    bw = 0;
  else if (sbw <= 10.4E3)
    bw = 1;
  else if (sbw <= 15.6E3)
    bw = 2;
  else if (sbw <= 20.8E3)
    bw = 3;
  else if (sbw <= 31.25E3)
    bw = 4;
  else if (sbw <= 41.7E3)
    bw = 5;
  else if (sbw <= 62.5E3)
    bw = 6;
  else if (sbw <= 125E3)
    bw = 7;
  else if (sbw <= 250E3)
    bw = 8;
  else
    bw = 9;

  esp_err_t ret;
  uint8_t reg;
  ret = lora_read_reg(Registers::REG_MODEM_CONFIG_1, &reg);
  if (ret != ESP_OK)
    return ret;
  return lora_write_reg(Registers::REG_MODEM_CONFIG_1, (reg & 0x0f) | (bw << 4));
}

/**
 * Set coding rate 
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */
esp_err_t SX1278_LoRa::lora_set_coding_rate(uint8_t denominator)
{
  if (denominator < 5)
    denominator = 5;
  else if (denominator > 8)
    denominator = 8;

  uint8_t cr = denominator - 4;

  esp_err_t ret;
  uint8_t reg;
  ret = lora_read_reg(Registers::REG_MODEM_CONFIG_1, &reg);
  if (ret != ESP_OK)
    return ret;
  return lora_write_reg(Registers::REG_MODEM_CONFIG_1, (reg & 0xf1) | (cr << 1));
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
esp_err_t SX1278_LoRa::lora_set_preamble_length(uint16_t length)
{
  esp_err_t ret;
  ret = lora_write_reg(Registers::REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  if (ret != ESP_OK)
    return ret;
  return lora_write_reg(Registers::REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
esp_err_t SX1278_LoRa::lora_set_sync_word(uint8_t sw)
{
  return lora_write_reg(Registers::REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying packet CRC.
 */
esp_err_t SX1278_LoRa::lora_enable_crc(void)
{
  esp_err_t ret;
  uint8_t reg;
  ret = lora_read_reg(Registers::REG_MODEM_CONFIG_2, &reg);
  if (ret != ESP_OK)
    return ret;
  return lora_write_reg(Registers::REG_MODEM_CONFIG_2, reg | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
esp_err_t SX1278_LoRa::lora_disable_crc(void)
{
  esp_err_t ret;
  uint8_t reg;
  ret = lora_read_reg(Registers::REG_MODEM_CONFIG_2, &reg);
  if (ret != ESP_OK)
    return ret;
  return lora_write_reg(Registers::REG_MODEM_CONFIG_2, reg & 0xfb);
}

/**
 * Perform hardware initialization.
 */
esp_err_t SX1278_LoRa::lora_init(void)
{
  esp_err_t ret;

  /*
    * Configure CPU hardware to communicate with the radio chip
    */
  gpio_pad_select_gpio((gpio_num_t)PinConfiguration::CONFIG_RST_GPIO);
  gpio_set_direction((gpio_num_t)PinConfiguration::CONFIG_RST_GPIO, GPIO_MODE_OUTPUT);
  gpio_pad_select_gpio((gpio_num_t)PinConfiguration::CONFIG_CS_GPIO);
  gpio_set_direction((gpio_num_t)PinConfiguration::CONFIG_CS_GPIO, GPIO_MODE_OUTPUT);

  spi_bus_config_t bus = {};
  bus.miso_io_num = PinConfiguration::CONFIG_MISO_GPIO;
  bus.mosi_io_num = PinConfiguration::CONFIG_MOSI_GPIO;
  bus.sclk_io_num = PinConfiguration::CONFIG_SCK_GPIO;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = 0;
  
  ret = spi_bus_initialize(VSPI_HOST, &bus, 0);
  if (ret != ESP_OK)
    return ret;
  spi_device_interface_config_t dev = {};
  dev.clock_speed_hz = 9000000;
  dev.mode = 0;
  dev.spics_io_num = -1;
  dev.queue_size = 1;
  dev.flags = 0;
  dev.pre_cb = NULL;
  ret = spi_bus_add_device(VSPI_HOST, &dev, &mhSpi);
  if (ret != ESP_OK)
    return ret;

  /*
    * Perform hardware reset.
    */
  lora_reset();

  /*
    * Check version.
    */
  uint8_t version, reg;
  int i;
  for (i = 0; i < Misc::TIMEOUT_RESET; i++)
  {
    ret = lora_read_reg(Registers::REG_VERSION, &version);
    if (ret != ESP_OK)
      return ret;
    if (version == 0x12)
      break;
    vTaskDelay(2);
  }
  if (i == Misc::TIMEOUT_RESET)
    return ESP_ERR_TIMEOUT;

  /*
    * Default configuration.
    */
  ret = lora_sleep();
  if (ret != ESP_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_FIFO_RX_BASE_ADDR, 0);
  if (ret != ESP_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_FIFO_TX_BASE_ADDR, 0);
  if (ret != ESP_OK)
    return ret;
  ret = lora_read_reg(Registers::REG_LNA, &reg);
  if (ret != ESP_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_LNA, reg | 0x03);
  if (ret != ESP_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_MODEM_CONFIG_3, 0x04);
  if (ret != ESP_OK)
    return ret;
  ret = lora_set_tx_power(10);
  if (ret != ESP_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_IRQ_MASK_FLAGS, 0);
  if (ret != ESP_OK)
    return ret;
  return lora_idle();
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 * @return Error status.
 */
esp_err_t SX1278_LoRa::lora_send_packet(uint8_t *buf, uint8_t size)
{
  uint8_t reg;
  esp_err_t ret;
  /*
    * Transfer data to radio.
    */
  ret = lora_idle();
  if (ret != ESP_OK)
    return ret;
  //ESP_LOGI("lora","lora is idle");
  ret = lora_write_reg(Registers::REG_FIFO_ADDR_PTR, 0);
  if (ret != ESP_OK)
    return ret;
  //ESP_LOGI("lora","filling FIFO");
  for (int i = 0; i < size; i++)
  {
    ret = lora_write_reg(Registers::REG_FIFO, *buf++);
    if (ret != ESP_OK)
      return ret;
  }
  //ESP_LOGI("lora","FIFO is idle");
  ret = lora_write_reg(Registers::REG_PAYLOAD_LENGTH, size);
  if (ret != ESP_OK)
    return ret;

  /*
    * Start transmission and wait for conclusion.
    */
  //ESP_LOGI("lora","starting transmission");

  ret = lora_write_reg(Registers::REG_OP_MODE, TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_TX);
  if (ret != ESP_OK)
    return ret;
  vTaskDelay(1);
  // Überprüfen, ob der Chip wirklich in den Modus gewechselt hat
  ret = lora_read_reg(Registers::REG_OP_MODE, &reg);
  if (ret != ESP_OK)
    return ret;
  if (reg != (TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_TX))
  {
    ESP_LOGE("lora", "Umschalten in den TX-Modus hat nicht funktioniert! Reg: 0x%x", reg);
    return ESP_ERR_TIMEOUT;
  }
  //

  ret = lora_read_reg(Registers::REG_IRQ_FLAGS, &reg);
  if (ret != ESP_OK)
    return ret;
  //ESP_LOGI("lora","Reg: %d",reg);
  //ESP_LOGI("lora","waiting for end of transmission");
  while ((reg & IrqMask::IRQ_TX_DONE_MASK) == 0)
  {
    ret = lora_read_reg(Registers::REG_IRQ_FLAGS, &reg);
    if (reg & IrqMask::IRQ_TX_DONE_MASK)
      break;
    //ESP_LOGI("lora","Reg: %d",reg);
    if (ret != ESP_OK)
      return ret;
    vTaskDelay(2);
  };
  //ESP_LOGI("lora","Sending done");
  return lora_write_reg(Registers::REG_IRQ_FLAGS, IrqMask::IRQ_TX_DONE_MASK);
}

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return BytesRead Number of bytes received (zero if no packet available).
 * @return Error status.
 */
esp_err_t SX1278_LoRa::lora_receive_packet(uint8_t *buf, uint8_t size, uint8_t *BytesRead)
{
  esp_err_t ret;
  uint8_t len = 0;
  uint8_t reg;

  /*
    * Check interrupts.
    */
  uint8_t irq;
  ret = lora_read_reg(Registers::REG_IRQ_FLAGS, &irq);
  if (ret != ESP_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_IRQ_FLAGS, irq);
  if (ret != ESP_OK)
    return ret;
  if ((irq & IrqMask::IRQ_RX_DONE_MASK) == 0) // nichts zu lesen
  {
    *BytesRead = 0;
    return ESP_OK;
  }
  if (irq & IrqMask::IRQ_PAYLOAD_CRC_ERROR_MASK)
    return ESP_ERR_INVALID_CRC;

  // Kein CRC im Header aktiviert? Das wird nicht akzeptiert
  uint8_t hop;
  ret = lora_read_reg(Registers::REG_HOP_CHANNEL, &hop);
  if (ret != ESP_OK)
    return ret;
  if (!(hop & 0x40))
    return ESP_ERR_INVALID_CRC;

  /*
    * Find packet size.
    */
  ret = lora_read_reg(mImplicit ? Registers::REG_PAYLOAD_LENGTH : Registers::REG_RX_NB_BYTES, &len);
  if (ret != ESP_OK)
    return ret;

  /*
    * Transfer data from radio.
    */
  //ret = lora_idle();
  //if (ret != ESP_OK)
    //return ret;
  ret = lora_read_reg(Registers::REG_FIFO_RX_CURRENT_ADDR, &reg);
  if (ret != ESP_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_FIFO_ADDR_PTR, reg);
  if (ret != ESP_OK)
    return ret;
  if (len > size)
    len = size;
  for (int i = 0; i < len; i++)
  {
    ret = lora_read_reg(Registers::REG_FIFO, &reg);
    if (ret != ESP_OK)
      return ret;
    *buf++ = reg;
  }

  *BytesRead = len;
  return ret;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
uint8_t SX1278_LoRa::lora_received(void)
{
  uint8_t reg;
  lora_read_reg(Registers::REG_IRQ_FLAGS, &reg);
  if (reg & IrqMask::IRQ_RX_DONE_MASK)
    return 1;
  return 0;
}

/**
 * Return last packet's RSSI.
 */
uint8_t SX1278_LoRa::lora_packet_rssi(void)
{
  uint8_t reg;
  lora_read_reg(Registers::REG_PKT_RSSI_VALUE, &reg);
  return (reg - (mFrequency < 868E6 ? 164 : 157));
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float SX1278_LoRa::lora_packet_snr(void)
{
  uint8_t reg;
  lora_read_reg(Registers::REG_PKT_SNR_VALUE, &reg);
  return ((int8_t)reg) * 0.25;
}

/**
 * Shutdown hardware.
 */
void SX1278_LoRa::lora_close(void)
{
  lora_sleep();
  if (mhSpi!=NULL)
  {
    spi_bus_remove_device(mhSpi);
    spi_bus_free(VSPI_HOST);
    mhSpi=NULL;
  }
}

esp_err_t SX1278_LoRa::lora_dump_registers(void)
{
  esp_err_t ret;
  uint8_t i, reg;
  printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
  for (i = 0; i < 0x40; i++)
  {
    ret = lora_read_reg(i, &reg);
    if (ret != ESP_OK)
      return ret;
    printf("%02X ", reg);
    if ((i & 0x0f) == 0x0f)
      printf("\n");
  }
  printf("\n");
  return ESP_OK;
}


