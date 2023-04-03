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

LoRa_PinConfiguration::LoRa_PinConfiguration(LoRaBoardTypes aBoard)
{
  switch (aBoard)
  {
    case LilygoT3:
      ChipSelect=18;
      Reset=23;
      Miso=19;
      Mosi=27;
      Clock=5;
      DIO0=26;
	    DIO1=33;
	    Busy=32;
      Led=25;
      AdcChannel=7;
    break;
    case HeltecESPLoRa:
      ChipSelect=18;
      Reset=14;
      Miso=19;
      Mosi=27;
      Clock=5;
      DIO0=33;
	    DIO1=34;
	    Busy=0;
      Led=25;
      AdcChannel=0;
    break;
    case HeltecWirelessStick_V3:
      ChipSelect=8;
      Reset=12;
      Miso=11;
      Mosi=10;
      Clock=9;
      DIO0=26;
	    DIO1=14;
	    Busy=13;
      Led=35;
      AdcChannel=0;
    break;
  }
}

SX1278_LoRa::SX1278_LoRa(LoRaBoardTypes aBoard)
  : mhSpi(NULL)
  , mInitialized(false)
  , mImplicit(false)
  , mAddress(0)
{
  PinConfig=new LoRa_PinConfiguration(aBoard);
}

SX1278_LoRa::~SX1278_LoRa()
{
  Close();
}

/**
 * Perform hardware initialization.
 */
esp_err_t SX1278_LoRa::Init(void)
{
  esp_err_t ret;

  /*
    * Configure CPU hardware to communicate with the radio chip
    */
  gpio_reset_pin((gpio_num_t)PinConfig->Reset);
  gpio_set_direction((gpio_num_t)PinConfig->Reset, GPIO_MODE_OUTPUT);
  gpio_reset_pin((gpio_num_t)PinConfig->ChipSelect);
  gpio_set_direction((gpio_num_t)PinConfig->ChipSelect, GPIO_MODE_OUTPUT);
  Reset();

  spi_bus_config_t bus = {};
  bus.miso_io_num = PinConfig->Miso;
  bus.mosi_io_num = PinConfig->Mosi;
  bus.sclk_io_num = PinConfig->Clock;
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

esp_err_t SX1278_LoRa::SetupModule(uint8_t aAddress, long aFrq, uint16_t aPreambleLength, long aBandwidth, uint8_t aSyncByte, uint8_t aSpreadingFactor, uint8_t aCodingRate, uint8_t aTxPower)
{
  mAddress=aAddress;
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
  ret = SetSyncWord(aSyncByte);
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
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
esp_err_t SX1278_LoRa::WriteReg(uint8_t reg, uint8_t val)
{
  uint8_t out[2] = {(uint8_t)(0x80 | reg), val};
  uint8_t in[2];

  spi_transaction_t t={};
  t.flags = 0;
  t.length = 8 * sizeof(out);
  t.tx_buffer = out;
  t.rx_buffer = in;

  gpio_set_level((gpio_num_t)PinConfig->ChipSelect, 0);
  esp_err_t ret = spi_device_transmit(mhSpi, &t);
  gpio_set_level((gpio_num_t)PinConfig->ChipSelect, 1);
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

  spi_transaction_t t={};
  t.flags = 0;
  t.length = 8 * sizeof(out);
  t.tx_buffer = out;
  t.rx_buffer = in;

  gpio_set_level((gpio_num_t)PinConfig->ChipSelect, 0);
  esp_err_t ret = spi_device_transmit(mhSpi, &t);
  gpio_set_level((gpio_num_t)PinConfig->ChipSelect, 1);
  *aInVal = in[1];
  return ret;
}

/**
 * Perform physical reset on the Lora chip
 */
void SX1278_LoRa::Reset(void)
{
  gpio_set_level((gpio_num_t)PinConfig->Reset, 0);
  vTaskDelay(pdMS_TO_TICKS(1));
  gpio_set_level((gpio_num_t)PinConfig->Reset, 1);
  vTaskDelay(pdMS_TO_TICKS(10));
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
esp_err_t SX1278_LoRa::Idle(void)
{
  return WriteReg(Registers::REG_OP_MODE, TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
esp_err_t SX1278_LoRa::Sleep(void)
{
  return WriteReg(Registers::REG_OP_MODE, TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_SLEEP);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
esp_err_t SX1278_LoRa::Receive(void)
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
esp_err_t SX1278_LoRa::SetFrequency(long frequency)
{
  mFrequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
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
esp_err_t SX1278_LoRa::SetSpreadingFactor(uint8_t sf)
{
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
esp_err_t SX1278_LoRa::SetBandwidth(long sbw)
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
  ret = ReadReg(Registers::REG_MODEM_CONFIG_1, &reg);
  if (ret != ESP_OK)
    return ret;
  return WriteReg(Registers::REG_MODEM_CONFIG_1, (reg & 0x0f) | (bw << 4));
}

/**
 * Set coding rate 
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */
esp_err_t SX1278_LoRa::SetCodingRate(uint8_t denominator)
{
  if (denominator < 5)
    denominator = 5;
  else if (denominator > 8)
    denominator = 8;

  uint8_t cr = denominator - 4;

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
  //ESP_LOGI("lora","lora is idle");
  ret = WriteReg(Registers::REG_FIFO_ADDR_PTR, 0);
  if (ret != ESP_OK)
    return ret;
  //ESP_LOGI("lora","filling FIFO");
  for (int i = 0; i < size; i++)
  {
    ret = WriteReg(Registers::REG_FIFO, *buf++);
    if (ret != ESP_OK)
      return ret;
  }
  //ESP_LOGI("lora","FIFO is idle");
  ret = WriteReg(Registers::REG_PAYLOAD_LENGTH, size);
  if (ret != ESP_OK)
    return ret;

  /*
    * Start transmission and wait for conclusion.
    */
  //ESP_LOGI("lora","starting transmission");

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
  //ESP_LOGI("lora","Reg: %d",reg);
  //ESP_LOGI("lora","waiting for end of transmission");
  while ((reg & IrqMask::IRQ_TX_DONE_MASK) == 0)
  {
    ret = ReadReg(Registers::REG_IRQ_FLAGS, &reg);
    if (reg & IrqMask::IRQ_TX_DONE_MASK)
      break;
    //ESP_LOGI("lora","Reg: %d",reg);
    if (ret != ESP_OK)
      return ret;
    vTaskDelay(2);
  };
  //ESP_LOGI("lora","Sending done");
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
  //ret = lora_idle();
  //if (ret != ESP_OK)
    //return ret;
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

/**
 * Shutdown hardware.
 */
void SX1278_LoRa::Close(void)
{
  Sleep();
  if (mhSpi!=NULL)
  {
    spi_bus_remove_device(mhSpi);
    spi_bus_free(VSPI_HOST);
    mhSpi=NULL;
  }
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

esp_err_t SX1278_LoRa::SendLoraMsg(LoraCommand aCmd, uint8_t *aBuf, uint16_t aSize, uint32_t aTag)
{
  uint8_t MaxPayloadPerPaketSize = 255 - sizeof(LoraPacketHeader);
  uint8_t NumPackets = aSize / MaxPayloadPerPaketSize + 1;
  uint8_t LastPacketSize = aSize - (NumPackets - 1) * MaxPayloadPerPaketSize;
  LoraPacketHeader ph;
  ph.Address = mAddress;
  ph.Cmd = (uint8_t)aCmd;
  ph.NumPackets = NumPackets;
  ph.Tag=aTag;
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
    //ESP_LOGI(TAG, "Sending LoRa packet %d/%d, %d bytes", i + 1, NumPackets, sizeof(ph) + ph.PacketPayloadSize);
    ret = SendPacket(mLoraBuf, sizeof(ph) + ph.PacketPayloadSize);
    if (ret != ESP_OK)
      return ret;
    
    BytesWritten += ph.PacketPayloadSize;
  }
  return ESP_OK;
}


