#include <stdlib.h>
#include <string.h>


#include "lora.h"
#include <pigpio.h>
#include <cstdio>
#include <time.h>


SX1278_LoRa::SX1278_LoRa()
  : mInitialized(false)
  , mImplicit(false)
  , mSPIHandle(-1)
{
  
}

SX1278_LoRa::~SX1278_LoRa()
{
  Close();
}

/**
 * Shutdown hardware.
 */
void SX1278_LoRa::Close(void)
{
  lora_sleep();
  if (mSPIHandle>=0)
  {
    spiClose(mSPIHandle);
    mSPIHandle=-1;
  }
  gpioTerminate();
  mInitialized=false;
  fprintf(stdout,"SX1278_LoRa closed.\n");
}

/**
 * Perform hardware initialization.
 */
ReturnStatus SX1278_LoRa::Init(void)
{
  mInitialized=false;
  ReturnStatus ret;
  
  if (gpioInitialise() < 0)
  {
    fprintf(stderr,"gpioInitialise failed!");
    return RT_GPIO_SETUP_ERR;
  }
  // GPIO-Setup
  gpioSetMode(CONFIG_CS_GPIO, PI_OUTPUT);
  gpioSetMode(CONFIG_RST_GPIO, PI_OUTPUT);
  gpioSetMode(CONFIG_DIO0, PI_INPUT);
  
  // SPI-Setup
  mSPIHandle=spiOpen(0,500000,0);
  if (mSPIHandle<0)
  {
    fprintf(stderr,"gpioInitialise failed!");
    return RT_SPI_INIT_FAILED;
  }

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
    if (ret != RT_OK)
      return ret;
    if (version == 0x12)
      break;
    delay(2);
  }
  if (i == Misc::TIMEOUT_RESET)
    return RT_TIMEOUT;

  /*
    * Default configuration.
    */
  ret = lora_sleep();
  if (ret != RT_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_FIFO_RX_BASE_ADDR, 0);
  if (ret != RT_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_FIFO_TX_BASE_ADDR, 0);
  if (ret != RT_OK)
    return ret;
  ret = lora_read_reg(Registers::REG_LNA, &reg);
  if (ret != RT_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_LNA, reg | 0x03);
  if (ret != RT_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_MODEM_CONFIG_3, 0x04);
  if (ret != RT_OK)
    return ret;
  ret = lora_set_tx_power(10);
  if (ret != RT_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_IRQ_MASK_FLAGS, 0);
  if (ret != RT_OK)
    return ret;


  ret = lora_explicit_header_mode();
  if (ret != RT_OK)
  {
    fprintf(stderr,"lora_explicit_header_mode failed: %d", ret);
    return ret;
  }
  ret = lora_set_frequency(434.54e6);
  if (ret != RT_OK)
  {
    fprintf(stderr,"lora_set_frequency failed: %d", ret);
    return ret;
  }
  ret = lora_enable_crc();
  if (ret != RT_OK)
  {
    fprintf(stderr,"lora_enable_crc failed: %d", ret);
    return ret;
  }

  ret = lora_set_preamble_length(14);
  if (ret != RT_OK)
  {
    fprintf(stderr,"lora_set_preamble_length failed: %d", ret);
    return ret;
  }
  ret = lora_set_bandwidth(500E3);
  if (ret != RT_OK)
  {
    fprintf(stderr,"lora_set_bandwidth failed: %d", ret);
    return ret;
  }
  ret = lora_set_sync_word(0x3d);
  if (ret != RT_OK)
  {
    fprintf(stderr,"lora_set_sync_word failed: %d", ret);
    return ret;
  }

  mInitialized=true;
  return ret;
}


/**
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
ReturnStatus SX1278_LoRa::lora_write_reg(uint8_t reg, uint8_t val)
{
  uint8_t out[2] = {(uint8_t)(0x80 | reg), val};

  gpioWrite(CONFIG_CS_GPIO, 0);
  int ret=spiWrite(mSPIHandle, (char*)out, sizeof(out));
  gpioWrite(CONFIG_CS_GPIO, 1);

  return ret<0 ? RT_SPI_WRITE_FAILED : RT_OK;
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
ReturnStatus SX1278_LoRa::lora_read_reg(uint8_t reg, uint8_t *aInVal)
{
  uint8_t out[2] = {reg, 0xff};
  uint8_t in[2];

  gpioWrite(CONFIG_CS_GPIO, 0);
  int ret = spiXfer(mSPIHandle, (char*)out, (char*)in, sizeof(out));
  gpioWrite(CONFIG_CS_GPIO, 1);
  *aInVal = in[1];
  
  return ret<0 ? RT_SPI_READ_FAILED : RT_OK;
}

/**
 * Perform physical reset on the Lora chip
 */
void SX1278_LoRa::lora_reset(void)
{
  gpioWrite(CONFIG_RST_GPIO,0);
  delay(10);
  gpioWrite(CONFIG_RST_GPIO,1);
  delay(10);
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
ReturnStatus SX1278_LoRa::lora_explicit_header_mode(void)
{
  uint8_t reg;
  ReturnStatus ret = lora_read_reg(Registers::REG_MODEM_CONFIG_1, &reg);
  if (ret != RT_OK)
    return ret;
  mImplicit = false;
  return lora_write_reg(Registers::REG_MODEM_CONFIG_1, reg & 0xfe);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
ReturnStatus SX1278_LoRa::lora_implicit_header_mode(uint8_t size)
{
  uint8_t reg;
  mImplicit = true;
  ReturnStatus ret = lora_read_reg(Registers::REG_MODEM_CONFIG_1, &reg);
  if (ret != RT_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_MODEM_CONFIG_1, reg | 0x01);
  if (ret != RT_OK)
    return ret;
  return lora_write_reg(Registers::REG_PAYLOAD_LENGTH, size);
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
ReturnStatus SX1278_LoRa::lora_idle(void)
{
  return lora_write_reg(Registers::REG_OP_MODE, TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
ReturnStatus SX1278_LoRa::lora_sleep(void)
{
  return lora_write_reg(Registers::REG_OP_MODE, TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_SLEEP);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
ReturnStatus SX1278_LoRa::lora_receive(void)
{
  return lora_write_reg(Registers::REG_OP_MODE, TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_RX_CONTINUOUS);
  delay(1);
  // Überprüfen, ob der Chip wirklich in den Modus gewechselt hat
  uint8_t reg=0;
  ReturnStatus ret = lora_read_reg(Registers::REG_OP_MODE, &reg);
  if (ret != RT_OK)
    return ret;
  if (reg != (TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_RX_CONTINUOUS))
  {
    fprintf(stderr,"Umschalten in den RX-Modus hat nicht funktioniert! Reg: 0x%x\n", reg);
    return RT_SWITCH_LORA_FAILED;
  }
}

void SX1278_LoRa::RestartReceiver()
{
  lora_sleep();
  delay(1);
  lora_receive();
}

ReturnStatus SX1278_LoRa::IsReceiving(bool & aIsReceiving)
{
  aIsReceiving=false;
  uint8_t reg=0;
  ReturnStatus ret = lora_read_reg(Registers::REG_OP_MODE, &reg);
  if (ret != RT_OK)
    return ret;
  if ((reg & (TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_RX_CONTINUOUS)) == (TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_RX_CONTINUOUS))
    aIsReceiving=true;
  return RT_OK;
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
ReturnStatus SX1278_LoRa::lora_set_tx_power(uint8_t level)
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
ReturnStatus SX1278_LoRa::lora_set_frequency(long frequency)
{
  mFrequency = frequency;

  uint64_t frf = ((uint64_t)frequency << 19) / 32000000;
  ReturnStatus ret;
  ret = lora_write_reg(Registers::REG_FRF_MSB, (uint8_t)(frf >> 16));
  if (ret != RT_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_FRF_MID, (uint8_t)(frf >> 8));
  if (ret != RT_OK)
    return ret;
  return lora_write_reg(Registers::REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
ReturnStatus SX1278_LoRa::lora_set_spreading_factor(uint8_t sf)
{
  ReturnStatus ret;
  if (sf < 6)
    sf = 6;
  else if (sf > 12)
    sf = 12;

  if (sf == 6)
  {
    ret = lora_write_reg(Registers::REG_DETECTION_OPTIMIZE, 0xc5);
    if (ret != RT_OK)
      return ret;
    ret = lora_write_reg(Registers::REG_DETECTION_THRESHOLD, 0x0c);
    if (ret != RT_OK)
      return ret;
  }
  else
  {
    ret = lora_write_reg(Registers::REG_DETECTION_OPTIMIZE, 0xc3);
    if (ret != RT_OK)
      return ret;
    ret = lora_write_reg(Registers::REG_DETECTION_THRESHOLD, 0x0a);
    if (ret != RT_OK)
      return ret;
  }
  uint8_t reg;
  ret = lora_read_reg(Registers::REG_MODEM_CONFIG_2, &reg);
  if (ret != RT_OK)
    return ret;
  return lora_write_reg(Registers::REG_MODEM_CONFIG_2, (reg & 0x0f) | ((sf << 4) & 0xf0));
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Bandwidth in Hz (up to 500000)
 */
ReturnStatus SX1278_LoRa::lora_set_bandwidth(long sbw)
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

  ReturnStatus ret;
  uint8_t reg;
  ret = lora_read_reg(Registers::REG_MODEM_CONFIG_1, &reg);
  if (ret != RT_OK)
    return ret;
  return lora_write_reg(Registers::REG_MODEM_CONFIG_1, (reg & 0x0f) | (bw << 4));
}

/**
 * Set coding rate
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */
ReturnStatus SX1278_LoRa::lora_set_coding_rate(uint8_t denominator)
{
  if (denominator < 5)
    denominator = 5;
  else if (denominator > 8)
    denominator = 8;

  uint8_t cr = denominator - 4;

  ReturnStatus ret;
  uint8_t reg;
  ret = lora_read_reg(Registers::REG_MODEM_CONFIG_1, &reg);
  if (ret != RT_OK)
    return ret;
  return lora_write_reg(Registers::REG_MODEM_CONFIG_1, (reg & 0xf1) | (cr << 1));
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
ReturnStatus SX1278_LoRa::lora_set_preamble_length(uint16_t length)
{
  ReturnStatus ret;
  ret = lora_write_reg(Registers::REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
  if (ret != RT_OK)
    return ret;
  return lora_write_reg(Registers::REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
ReturnStatus SX1278_LoRa::lora_set_sync_word(uint8_t sw)
{
  return lora_write_reg(Registers::REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying packet CRC.
 */
ReturnStatus SX1278_LoRa::lora_enable_crc(void)
{
  ReturnStatus ret;
  uint8_t reg;
  ret = lora_read_reg(Registers::REG_MODEM_CONFIG_2, &reg);
  if (ret != RT_OK)
    return ret;
  return lora_write_reg(Registers::REG_MODEM_CONFIG_2, reg | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
ReturnStatus SX1278_LoRa::lora_disable_crc(void)
{
  ReturnStatus ret;
  uint8_t reg;
  ret = lora_read_reg(Registers::REG_MODEM_CONFIG_2, &reg);
  if (ret != RT_OK)
    return ret;
  return lora_write_reg(Registers::REG_MODEM_CONFIG_2, reg & 0xfb);
}



/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 * @return Error status.
 */
ReturnStatus SX1278_LoRa::lora_send_packet(uint8_t *buf, uint8_t size)
{
  uint8_t reg;
  ReturnStatus ret;
  /*
    * Transfer data to radio.
    */
  ret = lora_idle();
  if (ret != RT_OK)
    return ret;
  //ESP_LOGI("lora","lora is idle");
  ret = lora_write_reg(Registers::REG_FIFO_ADDR_PTR, 0);
  if (ret != RT_OK)
    return ret;
  //ESP_LOGI("lora","filling FIFO");
  for (int i = 0; i < size; i++)
  {
    ret = lora_write_reg(Registers::REG_FIFO, *buf++);
    if (ret != RT_OK)
      return ret;
  }
  //ESP_LOGI("lora","FIFO is idle");
  ret = lora_write_reg(Registers::REG_PAYLOAD_LENGTH, size);
  if (ret != RT_OK)
    return ret;

  /*
    * Start transmission and wait for conclusion.
    */
  //ESP_LOGI("lora","starting transmission");

  ret = lora_write_reg(Registers::REG_OP_MODE, TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_TX);
  if (ret != RT_OK)
    return ret;
  delay(1);
  // Überprüfen, ob der Chip wirklich in den Modus gewechselt hat
  ret = lora_read_reg(Registers::REG_OP_MODE, &reg);
  if (ret != RT_OK)
    return ret;
  if (reg != (TransceiverMode::MODE_LONG_RANGE_MODE | TransceiverMode::MODE_TX))
  {
    fprintf(stderr,"Umschalten in den TX-Modus hat nicht funktioniert! Reg: 0x%x\n", reg);
    return RT_SWITCH_LORA_FAILED;
  }
  //

  ret = lora_read_reg(Registers::REG_IRQ_FLAGS, &reg);
  if (ret != RT_OK)
    return ret;
  //ESP_LOGI("lora","Reg: %d",reg);
  //ESP_LOGI("lora","waiting for end of transmission");
  while ((reg & IrqMask::IRQ_TX_DONE_MASK) == 0)
  {
    ret = lora_read_reg(Registers::REG_IRQ_FLAGS, &reg);
    if (reg & IrqMask::IRQ_TX_DONE_MASK)
      break;
    //ESP_LOGI("lora","Reg: %d",reg);
    if (ret != RT_OK)
      return ret;
    delay(2);
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
ReturnStatus SX1278_LoRa::lora_receive_packet(uint8_t *buf, uint16_t size, uint16_t &BytesRead)
{
  ReturnStatus ret;
  uint8_t len = 0;
  uint8_t reg;

  /*
    * Check interrupts.
    */
  uint8_t irq;
  ret = lora_read_reg(Registers::REG_IRQ_FLAGS, &irq);
  if (ret != RT_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_IRQ_FLAGS, irq);
  if (ret != RT_OK)
    return ret;
  if ((irq & IrqMask::IRQ_RX_DONE_MASK) == 0) // nichts zu lesen
  {
    BytesRead = 0;
    return RT_OK;
  }
  if (irq & IrqMask::IRQ_PAYLOAD_CRC_ERROR_MASK)
    return RT_LORA_CRC_ERR;
    
  // Kein CRC im Header aktiviert? Das wird nicht akzeptiert
  uint8_t hop;
  ret = lora_read_reg(Registers::REG_HOP_CHANNEL, &hop);
  if (ret != RT_OK)
    return ret;
  if (!(hop & 0x40))
    return RT_HEADER_WITHOUT_CRC;

  /*
    * Find packet size.
    */
  ret = lora_read_reg(mImplicit ? Registers::REG_PAYLOAD_LENGTH : Registers::REG_RX_NB_BYTES, &len);
  if (ret != RT_OK)
    return ret;

  /*
    * Transfer data from radio.
    */
  //lora_idle();
  ret = lora_read_reg(Registers::REG_FIFO_RX_CURRENT_ADDR, &reg);
  if (ret != RT_OK)
    return ret;
  ret = lora_write_reg(Registers::REG_FIFO_ADDR_PTR, reg);
  if (ret != RT_OK)
    return ret;
  if (len > size)
    len = size;
  for (int i = 0; i < len; i++)
  {
    ret = lora_read_reg(Registers::REG_FIFO, &reg);
    if (ret != RT_OK)
      return ret;
    *buf++ = reg;
  }

  BytesRead = len;
  return ret;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
bool SX1278_LoRa::lora_received(bool aUseDI0Pin)
{
  if (aUseDI0Pin)
  {
    // GPIO-Pin vom Raspberry lesen, falls DI0-Pin vom SX1276/8 verbunden ist
    return gpioRead(CONFIG_DIO0);
  }
  else
  {
    // IRQ-Register lesen
    uint8_t reg;
    lora_read_reg(Registers::REG_IRQ_FLAGS, &reg);
    if (reg & IrqMask::IRQ_RX_DONE_MASK)
      return true;
    return false;
  }
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



ReturnStatus SX1278_LoRa::lora_dump_registers(void)
{
  ReturnStatus ret;
  uint8_t i, reg;
  fprintf(stdout,"00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
  for (i = 0; i < 0x40; i++)
  {
    ret = lora_read_reg(i, &reg);
    if (ret != RT_OK)
      return ret;
    fprintf(stdout,"%02X ", reg);
    if ((i & 0x0f) == 0x0f)
      fprintf(stdout,"\n");
  }
  fprintf(stdout,"\n");
  return RT_OK;
}

int SX1278_LoRa::delay(long milliseconds)
{
   struct timespec rem;
   struct timespec req= {
       (int)(milliseconds / 1000),     /* secs (Must be Non-Negative) */ 
       (milliseconds % 1000) * 1000000 /* nano (Must be in range of 0 to 999999999) */ 
   };

   return nanosleep(&req , &rem);
}

// CRC types 
#define CRC_TYPE_CCITT                        0 
#define CRC_TYPE_IBM                          1 
// Polynomial = X^16 + X^12 + X^5 + 1 
#define POLYNOMIAL_CCITT                      0x1021 
// Polynomial = X^16 + X^15 + X^2 + 1 
#define POLYNOMIAL_IBM                        0x8005 
// Seeds 
#define CRC_IBM_SEED                          0xFFFF 
#define CRC_CCITT_SEED                        0x1D0F
 
uint16_t SX1278_LoRa::RadioComputeCRC( uint8_t *buffer, uint8_t length, uint8_t crcType ) 
{ 
  uint8_t i = 0; 
  uint16_t crc = 0; 
  uint16_t polynomial = 0; 
  polynomial = ( crcType == CRC_TYPE_IBM ) ? POLYNOMIAL_IBM : POLYNOMIAL_CCITT;  
  crc = ( crcType == CRC_TYPE_IBM ) ? CRC_IBM_SEED : CRC_CCITT_SEED;  
  for( i = 0; i < length; i++ ) 
  { 
    crc = ComputeCrc( crc, buffer[i], polynomial ); 
  } 
  if( crcType == CRC_TYPE_IBM ) 
  { 
    return crc; 
  } 
  else 
  {     
    return( ( uint16_t ) ( ~crc )); 
  } 
} 


uint16_t SX1278_LoRa::ComputeCrc( uint16_t crc, uint8_t dataByte, uint16_t polynomial ) 
{ 
  uint8_t i; 
  for( i = 0; i < 8; i++ ) 
  { 
    if( ( ( ( crc & 0x8000 ) >> 8 ) ^ ( dataByte & 0x80 ) ) != 0 ) 
    { 
      crc <<= 1;              // shift left once 
      crc ^= polynomial;      // XOR with polynomial 
    } 
    else 
    { 
      crc <<= 1;              // shift left once 
    } 
    dataByte <<= 1;             // Next data bit 
  } 
  return crc; 
} 
