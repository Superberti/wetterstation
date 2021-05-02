
#ifndef __LORA_H__
#define __LORA_H__

void lora_reset(void);
void lora_explicit_header_mode(void);
void lora_implicit_header_mode(uint8_t size);
void lora_idle(void);
void lora_sleep(void); 
void lora_receive(void);
void lora_set_tx_power(uint8_t level);
void lora_set_frequency(long frequency);
void lora_set_spreading_factor(uint8_t sf);
void lora_set_bandwidth(long sbw);
void lora_set_coding_rate(uint8_t denominator);
void lora_set_preamble_length(uint16_t length);
void lora_set_sync_word(uint8_t sw);
void lora_enable_crc(void);
void lora_disable_crc(void);
int lora_init(void);
void lora_send_packet(uint8_t *buf, uint8_t size);
uint8_t lora_receive_packet(uint8_t *buf, uint8_t size);
uint8_t lora_received(void);
uint8_t lora_packet_rssi(void);
float lora_packet_snr(void);
void lora_close(void);
int lora_initialized(void);
void lora_dump_registers(void);

#endif
