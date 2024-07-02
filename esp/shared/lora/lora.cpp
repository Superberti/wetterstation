#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"
#include <string.h>
#include "esp_log.h"
#include "lorastructs.h"
#include "lora.h"
#include "../tools/tools.h"
#include "esp_check.h"


// Quarzfrequenz aller Module
#define XTAL_FRQ 32000000

LoRaBase::LoRaBase(const LoRa_PinConfiguration & aPinConfig)
    : mhSpi(NULL), mInitialized(false), mImplicit(false), mAddress(0)
{
  mPinConfig = aPinConfig;
}

esp_err_t LoRaBase::SPIBusInit()
{
  esp_err_t ret;

  // GPIO-Init

  // Outputs
  const uint64_t OutputBitMask = (1ULL << mPinConfig.Reset) | (1ULL << mPinConfig.ChipSelect);

  gpio_config_t ConfigOutput = {};
  ConfigOutput.pin_bit_mask = OutputBitMask;
  ConfigOutput.mode = GPIO_MODE_OUTPUT;
  ConfigOutput.pull_up_en = GPIO_PULLUP_DISABLE;
  ConfigOutput.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ConfigOutput.intr_type = GPIO_INTR_DISABLE;
  ESP_RETURN_ON_ERROR(gpio_config(&ConfigOutput), "LORA:", "Fehler bei Init output GPIO");

  // Inputs
  uint64_t InputBitMask = (1ULL << mPinConfig.Busy);
  gpio_config_t ConfigInput = {};
  ConfigInput.pin_bit_mask = InputBitMask;
  ConfigInput.mode = GPIO_MODE_INPUT;
  ConfigInput.pull_up_en = GPIO_PULLUP_DISABLE;
  ConfigInput.pull_down_en = GPIO_PULLDOWN_DISABLE;
  ConfigInput.intr_type = GPIO_INTR_DISABLE;
  ESP_RETURN_ON_ERROR(gpio_config(&ConfigInput), "LORA:", "Fehler bei Init input GPIO");

  spi_bus_config_t bus = {};
  bus.miso_io_num = mPinConfig.Miso;
  bus.mosi_io_num = mPinConfig.Mosi;
  bus.sclk_io_num = mPinConfig.Clock;
  bus.quadwp_io_num = -1;
  bus.quadhd_io_num = -1;
  bus.max_transfer_sz = 0;
  ESP_LOGI("LORA:", "Init LoRa SPI. Channel: %d", mPinConfig.SPIChannel);
  ret = spi_bus_initialize(mPinConfig.SPIChannel, &bus, 0 /*SPI_DMA_CH_AUTO*/); // ohne DMA max. 32 Bytes gleichzeitig per SPI!
  if (ret != ESP_OK)
    return ret;
  spi_device_interface_config_t dev = {};
  dev.clock_speed_hz = 9000000;
  dev.mode = 0;
  dev.spics_io_num = -1;
  dev.queue_size = 1;
  dev.flags = 0;
  dev.pre_cb = NULL;
  ESP_LOGI("LORA:", "AddDevice LoRa SPI. Channel: %d", mPinConfig.SPIChannel);
  ret = spi_bus_add_device(mPinConfig.SPIChannel, &dev, &mhSpi);
  if (ret != ESP_OK)
    return ret;
  return ESP_OK;
}

/**
 * Shutdown hardware.
 */
void LoRaBase::Close()
{
  Sleep();
  CloseSPI();
}

void LoRaBase::CloseSPI()
{
  if (mhSpi != NULL)
  {
    spi_bus_remove_device(mhSpi);
    spi_bus_free(mPinConfig.SPIChannel);
    gpio_isr_handler_remove(mPinConfig.DIO1);
    gpio_uninstall_isr_service();
    mhSpi = NULL;
  }
}

esp_err_t LoRaBase::SendLoraMsg(LoraCommand aCmd, uint8_t *aBuf, uint16_t aSize, uint32_t aTag)
{
  uint8_t MaxPayloadPerPaketSize = 255 - sizeof(LoraPacketHeader);
  uint8_t NumPackets = aSize / MaxPayloadPerPaketSize + 1;
  uint8_t LastPacketSize = aSize - (NumPackets - 1) * MaxPayloadPerPaketSize;
  LoraPacketHeader ph;
  ph.Address = mAddress;
  ph.Cmd = (uint8_t)aCmd;
  ph.NumPackets = NumPackets;
  ph.Tag = aTag;
  int BytesWritten = 0;
  esp_err_t ret;
  for (int i = 0; i < NumPackets; i++)
  {
    ph.PacketNumber = i;
    ph.PacketPayloadSize = i == (NumPackets - 1) ? LastPacketSize : MaxPayloadPerPaketSize;
    ph.TotalTransmissionSize = aSize;
    ph.PayloadCRC = Crc16(aBuf + BytesWritten, ph.PacketPayloadSize);
    memcpy(mLoraBuf, &ph, sizeof(ph)); // Header kopieren
    memcpy(mLoraBuf + sizeof(ph), aBuf + BytesWritten, ph.PacketPayloadSize);
    // ESP_LOGI("LORA:", "Sending LoRa packet %d/%d, %d bytes", i + 1, NumPackets, sizeof(ph) + ph.PacketPayloadSize);
    ret = SendPacket(mLoraBuf, sizeof(ph) + ph.PacketPayloadSize);
    if (ret != ESP_OK)
      return ret;

    BytesWritten += ph.PacketPayloadSize;
  }
  return ESP_OK;
}

/**
 * Perform physical reset on the Lora chip
 */
void LoRaBase::Reset(void)
{
  gpio_set_level(mPinConfig.Reset, 0);
  vTaskDelay(pdMS_TO_TICKS(10));
  gpio_set_level(mPinConfig.Reset, 1);
  vTaskDelay(pdMS_TO_TICKS(10));
}

void LoRaBase::WaitBusy()
{
  uint32_t timeout = 0;
  while (gpio_get_level(mPinConfig.Busy) == 1)
  {
    // ESP_LOGI("LoRa", "LoRa busy level: %d", Busy);
    timeout++;
    if (timeout > 1000)
    {
      ESP_LOGE("LoRa", "WaitBusy timeout!");
      break;
    }
    vTaskDelay(1);
  }
}

LoRaBase::~LoRaBase()
{
  Close();
}

// ----------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------

SX1278_LoRa::SX1278_LoRa(const LoRa_PinConfiguration & aPinConfig) : LoRaBase(aPinConfig)
{
}

/**
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
esp_err_t SX1278_LoRa::WriteReg(uint8_t reg, uint8_t val)
{
  uint8_t out[2] = {(uint8_t)(0x80 | reg), val};
  uint8_t in[2];

  spi_transaction_t t = {};
  t.flags = 0;
  t.length = 8 * sizeof(out);
  t.tx_buffer = out;
  t.rx_buffer = in;

  gpio_set_level(mPinConfig.ChipSelect, 0);
  esp_err_t ret = spi_device_transmit(mhSpi, &t);
  gpio_set_level(mPinConfig.ChipSelect, 1);
  return ret;
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
esp_err_t SX1278_LoRa::ReadReg(uint8_t reg, uint8_t *aInVal)
{
  uint8_t out[2] = {reg, 0xff};
  uint8_t in[2];

  spi_transaction_t t = {};
  t.flags = 0;
  t.length = 8 * sizeof(out);
  t.tx_buffer = out;
  t.rx_buffer = in;

  gpio_set_level(mPinConfig.ChipSelect, 0);
  esp_err_t ret = spi_device_transmit(mhSpi, &t);
  gpio_set_level(mPinConfig.ChipSelect, 1);
  *aInVal = in[1];
  return ret;
}

/**
 * Perform hardware initialization.
 */
esp_err_t SX1278_LoRa::Init()
{
  esp_err_t ret;
  ret = SPIBusInit();
  if (ret != ESP_OK)
    return ret;

  /*
   * Perform hardware reset.
   */
  Reset();

  /*
   * Check version.
   */
  uint8_t version, reg;
  int i;
  for (i = 0; i < Misc::TIMEOUT_RESET; i++)
  {
    ret = ReadReg(Registers::REG_VERSION, &version);
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
  ret = Sleep();
  if (ret != ESP_OK)
    return ret;
  ret = WriteReg(Registers::REG_FIFO_RX_BASE_ADDR, 0);
  if (ret != ESP_OK)
    return ret;
  ret = WriteReg(Registers::REG_FIFO_TX_BASE_ADDR, 0);
  if (ret != ESP_OK)
    return ret;
  ret = ReadReg(Registers::REG_LNA, &reg);
  if (ret != ESP_OK)
    return ret;
  ret = WriteReg(Registers::REG_LNA, reg | 0x03);
  if (ret != ESP_OK)
    return ret;
  ret = WriteReg(Registers::REG_MODEM_CONFIG_3, 0x04);
  if (ret != ESP_OK)
    return ret;
  ret = SetTxPower(10);
  if (ret != ESP_OK)
    return ret;
  ret = WriteReg(Registers::REG_IRQ_MASK_FLAGS, 0);
  if (ret != ESP_OK)
    return ret;
  return Idle();
}

esp_err_t SX1278_LoRa::SetupModule(uint8_t aAddress, uint32_t aFrq, uint16_t aPreambleLength, LoRaBandwidth aBandwidth,
                                   uint16_t aSyncWord, SpreadingFactor aSpreadingFactor, LoRaCodingRate aCodingRate, int8_t aTxPower)
{
  mAddress = aAddress;
  CloseSPI();
  mInitialized = false;
  esp_err_t ret;
  ret = Init();
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "Init failed: %d", ret);
    return ret;
  }
  ret = ExplicitHeaderMode();
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "ExplicitHeaderMode failed: %d", ret);
    return ret;
  }
  ret = SetFrequency(aFrq);
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "SetFrequency failed: %d", ret);
    return ret;
  }
  ret = EnableCrc();
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "EnableCrc failed: %d", ret);
    return ret;
  }

  ret = SetPreambleLength(aPreambleLength);
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "SetPreambleLength failed: %d", ret);
    return ret;
  }
  ret = SetBandwidth(aBandwidth);
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "SetBandwidth failed: %d", ret);
    return ret;
  }
  ret = SetSyncWord((uint8_t)aSyncWord);
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "SetSyncWord failed: %d", ret);
    return ret;
  }
  ret = SetSpreadingFactor(aSpreadingFactor);
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "SetSpreadingFactor failed: %d", ret);
    return ret;
  }
  ret = SetCodingRate(aCodingRate);
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "SetCodingRate failed: %d", ret);
    return ret;
  }
  ret = SetTxPower(aTxPower);
  if (ret != ESP_OK)
  {
    ESP_LOGE("lora", "SetTxPower failed: %d", ret);
    return ret;
  }
  mInitialized = true;
  return ret;
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
esp_err_t SX1278_LoRa::ExplicitHeaderMode(void)
{
  uint8_t reg;
  esp_err_t ret = ReadReg(Registers::REG_MODEM_CONFIG_1, &reg);
  if (ret != ESP_OK)
    return ret;
  mImplicit = false;
  return WriteReg(Registers::REG_MODEM_CONFIG_1, reg & 0xfe);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
esp_err_t SX1278_LoRa::ImplicitHeaderMode(uint8_t size)
{
  uint8_t reg;
  mImplicit = true;
  esp_err_t ret = ReadReg(Registers::REG_MODEM_CONFIG_1, &reg);
  if (ret != ESP_OK)
    return ret;
  ret = WriteReg(Registers::REG_MODEM_CONFIG_1, reg | 0x01);
  if (ret != ESP_OK)
    return ret;
  return WriteReg(Registers::REG_PAYLOAD_LENGTH, size);
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
esp_err_t SX1278_LoRa::Idle()
{
  return WriteReg(Registers::REG_OP_MODE, TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
esp_err_t SX1278_LoRa::Sleep()
{
  return WriteReg(Registers::REG_OP_MODE, TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_SLEEP);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
esp_err_t SX1278_LoRa::Receive()
{
  return WriteReg(Registers::REG_OP_MODE, TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_RX_CONTINUOUS);
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
esp_err_t SX1278_LoRa::SetTxPower(uint8_t level)
{
  // RF9x module uses PA_BOOST pin
  if (level < 2)
    level = 2;
  else if (level > 17)
    level = 17;
  return WriteReg(Registers::REG_PA_CONFIG, Misc::PA_BOOST | (level - 2));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
esp_err_t SX1278_LoRa::SetFrequency(uint32_t frequency)
{
  mFrequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / XTAL_FRQ;
  esp_err_t ret;
  ret = WriteReg(Registers::REG_FRF_MSB, (uint8_t)(frf >> 16));
  if (ret != ESP_OK)
    return ret;
  ret = WriteReg(Registers::REG_FRF_MID, (uint8_t)(frf >> 8));
  if (ret != ESP_OK)
    return ret;
  return WriteReg(Registers::REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
esp_err_t SX1278_LoRa::SetSpreadingFactor(SpreadingFactor aSF)
{
  uint8_t sf = (uint8_t)aSF;
  esp_err_t ret;
  if (sf < 6)
    sf = 6;
  else if (sf > 12)
    sf = 12;

  if (sf == 6)
  {
    ret = WriteReg(Registers::REG_DETECTION_OPTIMIZE, 0xc5);
    if (ret != ESP_OK)
      return ret;
    ret = WriteReg(Registers::REG_DETECTION_THRESHOLD, 0x0c);
    if (ret != ESP_OK)
      return ret;
  }
  else
  {
    ret = WriteReg(Registers::REG_DETECTION_OPTIMIZE, 0xc3);
    if (ret != ESP_OK)
      return ret;
    ret = WriteReg(Registers::REG_DETECTION_THRESHOLD, 0x0a);
    if (ret != ESP_OK)
      return ret;
  }
  uint8_t reg;
  ret = ReadReg(Registers::REG_MODEM_CONFIG_2, &reg);
  if (ret != ESP_OK)
    return ret;
  return WriteReg(Registers::REG_MODEM_CONFIG_2, (reg & 0x0f) | ((sf << 4) & 0xf0));
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Bandwidth in Hz (up to 500000)
 */
esp_err_t SX1278_LoRa::SetBandwidth(LoRaBandwidth sbw)
{
  uint8_t bw;
  switch (sbw)
  {
  case LORA_BW_7:
    bw = 0;
    break;
  case LORA_BW_10:
    bw = 1;
    break;
  case LORA_BW_15:
    bw = 2;
    break;
  case LORA_BW_20:
    bw = 3;
    break;
  case LORA_BW_31:
    bw = 4;
    break;
  case LORA_BW_41:
    bw = 5;
    break;
  case LORA_BW_62:
    bw = 6;
    break;
  case LORA_BW_125:
    bw = 7;
    break;
  case LORA_BW_250:
    bw = 8;
    break;
  case LORA_BW_500:
    bw = 9;
    break;
  default:
    bw = 8;
    break;
  }

  esp_err_t ret;
  uint8_t reg;
  ret = ReadReg(Registers::REG_MODEM_CONFIG_1, &reg);
  if (ret != ESP_OK)
    return ret;
  return WriteReg(Registers::REG_MODEM_CONFIG_1, (reg & 0x0f) | (bw << 4));
}

esp_err_t SX1278_LoRa::SetCodingRate(LoRaCodingRate aCR)
{
  uint8_t cr = (uint8_t)aCR;
  esp_err_t ret;
  uint8_t reg;
  ret = ReadReg(Registers::REG_MODEM_CONFIG_1, &reg);
  if (ret != ESP_OK)
    return ret;
  return WriteReg(Registers::REG_MODEM_CONFIG_1, (reg & 0xf1) | (cr << 1));
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
esp_err_t SX1278_LoRa::SetPreambleLength(uint16_t length)
{
  esp_err_t ret;
  ret = WriteReg(Registers::REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  if (ret != ESP_OK)
    return ret;
  return WriteReg(Registers::REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
esp_err_t SX1278_LoRa::SetSyncWord(uint8_t sw)
{
  return WriteReg(Registers::REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying packet CRC.
 */
esp_err_t SX1278_LoRa::EnableCrc(void)
{
  esp_err_t ret;
  uint8_t reg;
  ret = ReadReg(Registers::REG_MODEM_CONFIG_2, &reg);
  if (ret != ESP_OK)
    return ret;
  return WriteReg(Registers::REG_MODEM_CONFIG_2, reg | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
esp_err_t SX1278_LoRa::DisableCrc(void)
{
  esp_err_t ret;
  uint8_t reg;
  ret = ReadReg(Registers::REG_MODEM_CONFIG_2, &reg);
  if (ret != ESP_OK)
    return ret;
  return WriteReg(Registers::REG_MODEM_CONFIG_2, reg & 0xfb);
}

/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 * @return Error status.
 */
esp_err_t SX1278_LoRa::SendPacket(uint8_t *buf, uint8_t size)
{
  uint8_t reg;
  esp_err_t ret;
  /*
   * Transfer data to radio.
   */
  ret = Idle();
  if (ret != ESP_OK)
    return ret;
  // ESP_LOGI("lora","lora is idle");
  ret = WriteReg(Registers::REG_FIFO_ADDR_PTR, 0);
  if (ret != ESP_OK)
    return ret;
  // ESP_LOGI("lora","filling FIFO");
  for (int i = 0; i < size; i++)
  {
    ret = WriteReg(Registers::REG_FIFO, *buf++);
    if (ret != ESP_OK)
      return ret;
  }
  // ESP_LOGI("lora","FIFO is idle");
  ret = WriteReg(Registers::REG_PAYLOAD_LENGTH, size);
  if (ret != ESP_OK)
    return ret;

  /*
   * Start transmission and wait for conclusion.
   */
  // ESP_LOGI("lora","starting transmission");

  ret = WriteReg(Registers::REG_OP_MODE, TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_TX);
  if (ret != ESP_OK)
    return ret;
  vTaskDelay(1);
  // Überprüfen, ob der Chip wirklich in den Modus gewechselt hat
  ret = ReadReg(Registers::REG_OP_MODE, &reg);
  if (ret != ESP_OK)
    return ret;
  if (reg != (TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_TX))
  {
    ESP_LOGE("lora", "Umschalten in den TX-Modus hat nicht funktioniert! Reg: 0x%x", reg);
    return ESP_ERR_TIMEOUT;
  }
  //

  ret = ReadReg(Registers::REG_IRQ_FLAGS, &reg);
  if (ret != ESP_OK)
    return ret;
  // ESP_LOGI("lora","Reg: %d",reg);
  // ESP_LOGI("lora","waiting for end of transmission");
  while ((reg & IrqMask::IRQ_TX_DONE_MASK) == 0)
  {
    ret = ReadReg(Registers::REG_IRQ_FLAGS, &reg);
    if (reg & IrqMask::IRQ_TX_DONE_MASK)
      break;
    // ESP_LOGI("lora","Reg: %d",reg);
    if (ret != ESP_OK)
      return ret;
    vTaskDelay(2);
  };
  // ESP_LOGI("lora","Sending done");
  return WriteReg(Registers::REG_IRQ_FLAGS, IrqMask::IRQ_TX_DONE_MASK);
}

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return BytesRead Number of bytes received (zero if no packet available).
 * @return Error status.
 */
esp_err_t SX1278_LoRa::ReceivePacket(uint8_t *buf, uint8_t size, uint8_t *BytesRead)
{
  esp_err_t ret;
  uint8_t len = 0;
  uint8_t reg;

  /*
   * Check interrupts.
   */
  uint8_t irq;
  ret = ReadReg(Registers::REG_IRQ_FLAGS, &irq);
  if (ret != ESP_OK)
    return ret;
  ret = WriteReg(Registers::REG_IRQ_FLAGS, irq);
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
  ret = ReadReg(Registers::REG_HOP_CHANNEL, &hop);
  if (ret != ESP_OK)
    return ret;
  if (!(hop & 0x40))
    return ESP_ERR_INVALID_CRC;

  /*
   * Find packet size.
   */
  ret = ReadReg(mImplicit ? Registers::REG_PAYLOAD_LENGTH : Registers::REG_RX_NB_BYTES, &len);
  if (ret != ESP_OK)
    return ret;

  /*
   * Transfer data from radio.
   */
  // ret = lora_idle();
  // if (ret != ESP_OK)
  // return ret;
  ret = ReadReg(Registers::REG_FIFO_RX_CURRENT_ADDR, &reg);
  if (ret != ESP_OK)
    return ret;
  ret = WriteReg(Registers::REG_FIFO_ADDR_PTR, reg);
  if (ret != ESP_OK)
    return ret;
  if (len > size)
    len = size;
  for (int i = 0; i < len; i++)
  {
    ret = ReadReg(Registers::REG_FIFO, &reg);
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
uint8_t SX1278_LoRa::Received(void)
{
  uint8_t reg;
  ReadReg(Registers::REG_IRQ_FLAGS, &reg);
  if (reg & IrqMask::IRQ_RX_DONE_MASK)
    return 1;
  return 0;
}

/**
 * Return last packet's RSSI.
 */
uint8_t SX1278_LoRa::PacketRssi(void)
{
  uint8_t reg;
  ReadReg(Registers::REG_PKT_RSSI_VALUE, &reg);
  return (reg - (mFrequency < 868E6 ? 164 : 157));
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float SX1278_LoRa::PacketSnr(void)
{
  uint8_t reg;
  ReadReg(Registers::REG_PKT_SNR_VALUE, &reg);
  return ((int8_t)reg) * 0.25;
}

esp_err_t SX1278_LoRa::DumpRegisters(void)
{
  esp_err_t ret;
  uint8_t i, reg;
  printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
  for (i = 0; i < 0x40; i++)
  {
    ret = ReadReg(i, &reg);
    if (ret != ESP_OK)
      return ret;
    printf("%02X ", reg);
    if ((i & 0x0f) == 0x0f)
      printf("\n");
  }
  printf("\n");
  return ESP_OK;
}

// ----------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------

SX1262_LoRa::SX1262_LoRa(const LoRa_PinConfiguration & aPinConfig) : LoRaBase(aPinConfig)
{
}

esp_err_t SX1262_LoRa::ChipCommand(uint8_t *aParams, uint8_t aParamLength)
{
  // Byte0 = SX1262-Opcode
  const uint8_t MaxTrans = 32;
  esp_err_t ret = ESP_OK;
  spi_transaction_t t = {};
  t.flags = 0;
  // t.length = 8 * aParamLength;
  // t.tx_buffer = aParams;
  // t.rx_buffer = SPIRecBuf;

  uint8_t FullTransfers = aParamLength / MaxTrans;
  uint8_t RestBytes = aParamLength % MaxTrans;
  gpio_set_level(mPinConfig.ChipSelect, 0);
  uint8_t i = 0;
  for (i = 0; i < FullTransfers; i++)
  {
    t.rxlength = 0;
    t.length = 8 * MaxTrans;
    t.tx_buffer = aParams + i * MaxTrans;
    t.rx_buffer = SPIRecBuf + i * MaxTrans;
    ret = spi_device_transmit(mhSpi, &t);
    if (ret != ESP_OK)
    {
      gpio_set_level(mPinConfig.ChipSelect, 1);
      return ret;
    }
  }
  if (RestBytes > 0)
  {
    t.rxlength = 0;
    t.length = 8 * RestBytes;
    t.tx_buffer = aParams + i * MaxTrans;
    t.rx_buffer = SPIRecBuf + i * MaxTrans;
    ret = spi_device_transmit(mhSpi, &t);
    if (ret != ESP_OK)
    {
      gpio_set_level(mPinConfig.ChipSelect, 1);
      return ret;
    }
  }
  gpio_set_level(mPinConfig.ChipSelect, 1);
  if (aParams != NULL && aParamLength != 0)
    memcpy(aParams, SPIRecBuf, aParamLength); // Für Lesekommandos
  return ret;
}

esp_err_t SX1262_LoRa::SetRadioMode(RadioMode aRM, uint32_t aTimeout)
{
  esp_err_t ret;
  uint8_t Params[4];

  // Zu allererst immer in den Standby gehen!
  Params[0] = Opcodes::OP_SET_STANDBY;
  Params[1] = 0; // STDBY_RC
  ret = ChipCommand(Params, 2);
  if (ret != ESP_OK)
    return ret;

  switch (aRM)
  {
  case RM_STANDBY:
    // Ist schon im Standby
    break;
  case RM_SLEEP:
    Params[0] = Opcodes::OP_SET_SLEEP;
    Params[1] = 0;
    ret = ChipCommand(Params, 2);
    // Mindestens 500 µs Pause lt. Datenblatt
    vTaskDelay(1);
    break;
  case RM_RX_ENABLE:
    Params[0] = Opcodes::OP_SET_RX;
    // Timeout Bit 23:0
    Params[1] = (aTimeout >> 16) & 0xFF; // MSB
    Params[2] = (aTimeout >> 8) & 0xFF;
    Params[3] = aTimeout & 0xFF;
    ret = ChipCommand(Params, 4);
    break;
  case RM_TX_ENABLE:
    Params[0] = Opcodes::OP_SET_TX;
    // Timeout Bit 23:0, 64000 = 1 s
    Params[1] = (aTimeout >> 16) & 0xFF; // MSB
    Params[2] = (aTimeout >> 8) & 0xFF;
    Params[3] = aTimeout & 0xFF;
    ret = ChipCommand(Params, 4);
    break;
  }
  return ret;
}

esp_err_t SX1262_LoRa::SetPacketType(PacketType aPT)
{
  uint8_t Params[2];
  Params[0] = Opcodes::OP_SET_PKG_TYPE;
  Params[1] = (uint8_t)aPT;
  return ChipCommand(Params, sizeof(Params));
}

esp_err_t SX1262_LoRa::SetFrequency(uint32_t aFrequency)
{
  mFrequency = aFrequency;
  uint8_t Params[5];
  Params[0] = Opcodes::OP_SET_RF_FRQ;
  uint32_t Chip_FRQ = (uint32_t)(((uint64_t)aFrequency << 25) / XTAL_FRQ);
  Params[1] = (Chip_FRQ >> 24) & 0xFF;
  Params[2] = (Chip_FRQ >> 16) & 0xFF;
  Params[3] = (Chip_FRQ >> 8) & 0xFF;
  Params[4] = Chip_FRQ & 0xFF;
  return ChipCommand(Params, sizeof(Params));
}

esp_err_t SX1262_LoRa::Sleep()
{
  return SetRadioMode(RadioMode::RM_SLEEP, 0);
}

esp_err_t SX1262_LoRa::SetTxParams(RampTime aRampTime, int8_t aTXPower_DB)
{
  esp_err_t ret;
  uint8_t Params[5];
  Params[0] = Opcodes::OP_SET_PA_CONFIG;

  // s. Tabelle 13-21 Datenblatt
  Params[3] = 0x00;
  Params[4] = 0x01;
  if (aTXPower_DB <= 14)
  {
    Params[1] = 0x02;
    Params[2] = 0x02;
  }
  else if (aTXPower_DB <= 17)
  {
    Params[1] = 0x02;
    Params[2] = 0x03;
  }
  else if (aTXPower_DB <= 20)
  {
    Params[1] = 0x03;
    Params[2] = 0x05;
  }
  else
  {
    Params[1] = 0x04;
    Params[2] = 0x07;
  }
  ret = ChipCommand(Params, sizeof(Params));
  if (ret != ESP_OK)
    return ret;

  Params[0] = Opcodes::OP_SET_TX_PARAMS;
  Params[1] = (uint8_t)aRampTime;
  Params[2] = (uint8_t)aTXPower_DB;
  return ChipCommand(Params, 3);
}

esp_err_t SX1262_LoRa::SetModulationParams(SpreadingFactor aSF, LoRaBandwidth aBW, LoRaCodingRate aCR, bool aLowDataRateOpt)
{
  uint8_t Params[9] = {0};
  Params[0] = Opcodes::OP_SET_MOD_PARAMS;
  Params[1] = (uint8_t)aSF;
  Params[2] = (uint8_t)aBW;
  Params[3] = (uint8_t)aCR;
  Params[4] = (uint8_t)aLowDataRateOpt;
  return ChipCommand(Params, sizeof(Params));
}

esp_err_t SX1262_LoRa::SetPacketParams(uint16_t aPreambleLength, bool aImplicitHeader, uint8_t aPayloadLength, bool aUseCRC)
{
  uint8_t Params[10] = {0};
  Params[0] = Opcodes::OP_SET_PKG_PARAMS;
  // PreambleLength
  Params[1] = (aPreambleLength >> 8) & 0xFF;
  Params[2] = aPreambleLength & 0xFF;

  // HeaderType
  Params[3] = (uint8_t)aImplicitHeader;

  // Paketlänge (Nutzdaten)
  Params[4] = aPayloadLength;

  // CRC ja/nein
  Params[5] = (uint8_t)aUseCRC;

  // InvertIQ
  Params[6] = 0;

  return ChipCommand(Params, sizeof(Params));
}

esp_err_t SX1262_LoRa::SetCadParams(uint8_t aSymbolNum, uint8_t aDetPeak, uint8_t aDetMin, uint8_t aExitMode, uint32_t aTimeout)
{
  uint8_t Params[8] = {0};
  Params[0] = Opcodes::OP_SET_CAD_PARAMS;

  Params[1] = aSymbolNum;
  Params[2] = aDetPeak;
  Params[3] = aDetMin;
  Params[4] = aExitMode;

  // CAD-Timeout
  Params[5] = (aTimeout >> 16) & 0xFF;
  Params[6] = (aTimeout >> 8) & 0xFF;
  Params[7] = aTimeout & 0xFF;

  return ChipCommand(Params, sizeof(Params));
}

esp_err_t SX1262_LoRa::SetBufferBaseAddress(uint8_t aTxBaseAddr, uint8_t aRxBaseAddr)
{
  uint8_t Params[3];
  Params[0] = Opcodes::OP_SET_BUFFER_BASE_ADDR;
  Params[1] = aTxBaseAddr;
  Params[2] = aRxBaseAddr;
  return ChipCommand(Params, sizeof(Params));
}

esp_err_t SX1262_LoRa::SetSyncWord(uint16_t aSyncWord)
{
  uint8_t Params[2];
  Params[0] = (aSyncWord >> 8) & 0xFF;
  Params[1] = aSyncWord & 0xFF;
  return WriteReg(Registers::REG_LORA_SYNC_MSB, Params, 2);
}

esp_err_t SX1262_LoRa::ReadReg(uint16_t aAddr, uint8_t *aParams, uint8_t aParamLength)
{
  SPITransBuf[0] = Opcodes::OP_READ_REG;
  SPITransBuf[1] = (aAddr >> 8) & 0xFF;
  SPITransBuf[2] = aAddr & 0xFF;
  // Es müssen aParamLength+1 NOPs gesendet werden, damit 1 (oder mehrere) Register gelesen werden können
  memset(SPITransBuf + 3, 0, aParamLength + 1);
  esp_err_t ret = ChipCommand(SPITransBuf, aParamLength + 4);
  memcpy(aParams, SPITransBuf + 4, aParamLength);
  return ret;
}

esp_err_t SX1262_LoRa::WriteReg(uint16_t aAddr, uint8_t *aParams, uint8_t aParamLength)
{
  SPITransBuf[0] = Opcodes::OP_WRITE_REG;
  SPITransBuf[1] = (aAddr >> 8) & 0xFF;
  SPITransBuf[2] = aAddr & 0xFF;
  memcpy(SPITransBuf + 3, aParams, aParamLength);
  return ChipCommand(SPITransBuf, aParamLength + 3);
}

esp_err_t SX1262_LoRa::ReadBuffer(uint8_t aOffset, uint8_t *aParams, uint8_t aParamLength)
{
  SPITransBuf[0] = Opcodes::OP_READ_BUF;
  SPITransBuf[1] = aOffset;
  memset(SPITransBuf + 2, 0, aParamLength + 1);
  esp_err_t ret = ChipCommand(SPITransBuf, aParamLength + 3);
  memcpy(aParams, SPITransBuf + 3, aParamLength);
  return ret;
}

esp_err_t SX1262_LoRa::WriteBuffer(uint8_t aOffset, uint8_t *aParams, uint8_t aParamLength)
{
  SPITransBuf[0] = Opcodes::OP_WRITE_BUF;
  SPITransBuf[1] = aOffset;
  memcpy(SPITransBuf + 2, aParams, aParamLength);
  return ChipCommand(SPITransBuf, aParamLength + 2);
}

esp_err_t SX1262_LoRa::GetIRQStatus(uint16_t &aStatus)
{
  esp_err_t ret;
  uint8_t Params[4] = {0};
  Params[0] = Opcodes::OP_GET_IRQ_STATUS;
  ret = ChipCommand(Params, sizeof(Params));
  aStatus = (Params[2] << 8) + Params[3];
  // ESP_LOGI("LoRa", "GetIQRStatus: %d %d %d %d", Params[0], Params[1], Params[2], Params[3]);
  return ret;
}

esp_err_t SX1262_LoRa::ClearIRQStatus(uint16_t aIRQsToClear)
{
  uint8_t Params[3];
  Params[0] = Opcodes::OP_CLR_IRQ_STATUS;
  Params[1] = (aIRQsToClear >> 8) & 0xFF;
  Params[2] = aIRQsToClear & 0xFF;
  return ChipCommand(Params, sizeof(Params));
}

esp_err_t SX1262_LoRa::SetIRQParams(uint16_t aIRQMask, uint16_t DIO1Mask, uint16_t DIO2Mask, uint16_t DIO3Mask)
{
  uint8_t Params[9];
  Params[0] = Opcodes::OP_SET_DIO_IRQ_PARAMS;

  Params[1] = (aIRQMask >> 8) & 0xFF;
  Params[2] = aIRQMask & 0xFF;

  Params[3] = (DIO1Mask >> 8) & 0xFF;
  Params[4] = DIO1Mask & 0xFF;

  Params[5] = (DIO2Mask >> 8) & 0xFF;
  Params[6] = DIO2Mask & 0xFF;

  Params[7] = (DIO3Mask >> 8) & 0xFF;
  Params[8] = DIO3Mask & 0xFF;
  return ChipCommand(Params, sizeof(Params));
}

esp_err_t SX1262_LoRa::SetDio2AsRfSwitchCtrl(bool aOn)
{
  uint8_t Params[2];
  Params[0] = Opcodes::OP_SET_DIO2_AS_RF_SWITCH_CTRL;
  Params[1] = (uint8_t)aOn;
  return ChipCommand(Params, sizeof(Params));
}

esp_err_t SX1262_LoRa::SetDio3AsTcxoCtrl(TXCO_Voltage aU, uint32_t aTimeout)
{
  uint8_t Params[5];
  Params[0] = Opcodes::OP_SET_DIO3_AS_TCXO_CTRL;
  Params[1] = (uint8_t)aU;
  Params[2] = (aTimeout >> 16) & 0xFF;
  Params[3] = (aTimeout >> 8) & 0xFF;
  Params[4] = aTimeout & 0xFF;
  return ChipCommand(Params, sizeof(Params));
}

esp_err_t SX1262_LoRa::SetCalibrateSections(uint8_t aCP)
{
  uint8_t Params[2];
  Params[0] = Opcodes::OP_CALIBRATE;
  Params[1] = aCP;
  return ChipCommand(Params, sizeof(Params));
}

esp_err_t SX1262_LoRa::Calibrate(double aFrq_MHz)
{
  uint8_t Params[3];
  Params[0] = Opcodes::OP_CALIBRATE_IMAGE;
  if (aFrq_MHz > 900)
  {
    Params[1] = 0xE1;
    Params[2] = 0xE9;
  }
  else if (aFrq_MHz > 850)
  {
    Params[1] = 0xD7;
    Params[2] = 0xD8;
  }
  else if (aFrq_MHz > 770)
  {
    Params[1] = 0xC1;
    Params[2] = 0xC5;
  }
  else if (aFrq_MHz > 460)
  {
    Params[1] = 0x75;
    Params[2] = 0x81;
  }
  else if (aFrq_MHz > 425)
  {
    Params[1] = 0x6B;
    Params[2] = 0x6F;
  }
  esp_err_t ret = ChipCommand(Params, sizeof(Params));
  WaitBusy();
  return ret;
}

esp_err_t SX1262_LoRa::SetupModule(uint8_t aAddress, uint32_t aFrq, uint16_t aPreambleLength, LoRaBandwidth aBandwidth,
                                   uint16_t aSyncWord, SpreadingFactor aSpreadingFactor, LoRaCodingRate aCodingRate, int8_t aTxPower)

{
  mAddress = aAddress;
  CloseSPI();
  mPreambleLength = aPreambleLength;
  mInitialized = false;
  esp_err_t ret;
  ret = SPIBusInit();
  if (ret != ESP_OK)
    return ret;

  Reset();

  ret = SetRadioMode(RM_STANDBY, 0);
  if (ret != ESP_OK)
    return ret;

  if (mPinConfig.UseTXCO)
  {
    // Aufwachzeit TXCO in ms
    ESP_LOGI("LoRa", "Calibrating TXCO...");
    const uint32_t TXCO_WakeupTime = 5;
    SetDio3AsTcxoCtrl(TCXO_CTRL_1_8V, TXCO_WakeupTime << 6); // convert from ms to SX126x time base
    uint8_t CalibParam = 0x7F;
    SetCalibrateSections(CalibParam);
    Calibrate(433);
    ESP_LOGI("LoRa", "Calibrating done!");
  }

  SetDio2AsRfSwitchCtrl(true);

  // Testweise Register lesen
  uint8_t p[4] = {};
  ret = ReadReg(0x06BC, p, 4);
  if (ret != ESP_OK)
  {
    ESP_LOGE("LoRa", "Register lesen fehlgeschlagen");
    return ret;
  }
  ESP_LOGI("LoRa", "%d %d %d %d", p[0], p[1], p[2], p[3]);

  ret = SetPacketType(PACKET_TYPE_LORA);
  if (ret != ESP_OK)
    return ret;
  ret = SetFrequency(aFrq);
  if (ret != ESP_OK)
    return ret;
  ret = SetTxParams(SET_RAMP_10U, aTxPower);
  if (ret != ESP_OK)
    return ret;
  ret = SetBufferBaseAddress(0, 0);
  if (ret != ESP_OK)
    return ret;
  ret = SetModulationParams(aSpreadingFactor, aBandwidth, aCodingRate, false);
  if (ret != ESP_OK)
    return ret;
  /*
  ret = SetPacketParams(aPreambleLength, true, 0, true);
  if (ret != ESP_OK)
    return ret;
  */
  ret = SetSyncWord(aSyncWord);
  if (ret != ESP_OK)
    return ret;

  ret = SetIRQParams(IRQFlags::TxDone | IRQFlags::RxDone | IRQFlags::CRCErr | IRQFlags::HeaderErr | IRQFlags::HeaderValid | IRQFlags::SyncWordValid | IRQFlags::Timeout, 0, 0, 0);

  mInitialized = true;
  return ret;
}

esp_err_t SX1262_LoRa::GetDeviceError(uint16_t &aErr)
{
  uint8_t Params[4] = {0};
  memset(Params, 0, 4);
  Params[0] = Opcodes::OP_GET_DEV_ERR;
  esp_err_t ret = ChipCommand(Params, sizeof(Params));
  aErr = Params[2] * 256 + Params[3];
  return ret;
}

esp_err_t SX1262_LoRa::ClearDeviceError()
{
  uint8_t Params[2] = {0};
  Params[0] = Opcodes::OP_CLR_DEV_ERR;
  return ChipCommand(Params, sizeof(Params));
}

esp_err_t SX1262_LoRa::SendPacket(uint8_t *aBuf, uint8_t aSize)
{
  esp_err_t ret;
  ESP_LOGI("LoRa", "Sende LoRa Paket: %d bytes", aSize);
  ret = SetRadioMode(RadioMode::RM_STANDBY, 0);
  if (ret != ESP_OK)
    return ret;
  ret = ClearDeviceError();
  if (ret != ESP_OK)
    return ret;
  // ESP_LOGI("lora","lora is idle");
  ret = SetBufferBaseAddress(0, 0);
  if (ret != ESP_OK)
    return ret;
  // ESP_LOGI("lora","filling FIFO");
  ret = WriteBuffer(0, aBuf, aSize);
  if (ret != ESP_OK)
    return ret;
  /*
  uint8_t ReadBuf[256] = {};
  ret = ReadBuffer(0, ReadBuf, aSize);
  if (ret != ESP_OK)
    return ret;
  if (memcmp(aBuf, ReadBuf, aSize) != 0)
    ESP_LOGE("LoRa", "Vergleich Puffer Lesen/Schreiben fehlgeschlagen!");
  */
  ret = SetPacketParams(mPreambleLength, true, aSize, true);
  if (ret != ESP_OK)
    return ret;
  ret = ClearIRQStatus(0xFFFF);
  if (ret != ESP_OK)
    return ret;
  ret = SetIRQParams(IRQFlags::TxDone | IRQFlags::RxDone | IRQFlags::CRCErr | IRQFlags::HeaderErr | IRQFlags::HeaderValid | IRQFlags::SyncWordValid | IRQFlags::Timeout, 0, 0, 0);
  if (ret != ESP_OK)
    return ret;

  // ESP_LOGI("lora","starting transmission");

  ret = SetRadioMode(RadioMode::RM_TX_ENABLE, 64000); // = 1 s Sendetimeout
  if (ret != ESP_OK)
    return ret;

  // Wurde der Modus wirklich eingestellt?
  vTaskDelay(1);

  uint16_t iIRQStatus = 0;
  ret = GetIRQStatus(iIRQStatus);
  if (ret != ESP_OK)
    return ret;
  // ESP_LOGI("LoRa", "iIRQStatus: %d", iIRQStatus);

  // ESP_LOGI("LoRa", "Warte auf Paketende...");
  while ((iIRQStatus & IRQFlags::TxDone) == 0)
  {
    ret = GetIRQStatus(iIRQStatus);
    if (ret != ESP_OK)
      return ret;
    /*
    ret = GetDeviceError(ChipError);
    if (ret != ESP_OK)
      return ret;
    ESP_LOGI("LoRa", "iIRQStatus: %d", iIRQStatus);
    ESP_LOGI("LoRa", "ChipError: %d", ChipError);
    */
    if (iIRQStatus & IRQFlags::Timeout)
    {
      ESP_LOGE("LoRa", "Sendetimeout!");
      ClearIRQStatus(0xFFFF);
      return ESP_ERR_TIMEOUT;
    }

    vTaskDelay(1);
  };
  ClearIRQStatus(0xFFFF);
  return ret;
}

esp_err_t SX1262_LoRa::ReceivePacket(uint8_t *buf, uint8_t size, uint8_t *BytesRead)
{
  esp_err_t ret = 0;
  return ret;
}

esp_err_t SX1262_LoRa::Receive()
{
  esp_err_t ret = 0;
  return ret;
}

uint8_t SX1262_LoRa::Received()
{
  return 0;
}

uint8_t SX1262_LoRa::PacketRssi()
{
  return 0;
}

float SX1262_LoRa::PacketSnr()
{
  return 0.0;
}

esp_err_t SX1262_LoRa::DumpRegisters()
{
  esp_err_t ret = 0;
  return ret;
}
