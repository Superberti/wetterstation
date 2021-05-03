
#ifndef __LORA_H__
#define __LORA_H__

esp_err_t lora_reset(void);
esp_err_t lora_explicit_header_mode(void);
esp_err_t lora_implicit_header_mode(uint8_t size);
esp_err_t lora_idle(void);
esp_err_t lora_sleep(void);
esp_err_t lora_receive(void);
esp_err_t lora_set_tx_power(uint8_t level);
esp_err_t lora_set_frequency(long frequency);
esp_err_t lora_set_spreading_factor(uint8_t sf);
esp_err_t lora_set_bandwidth(long sbw);
esp_err_t lora_set_coding_rate(uint8_t denominator);
esp_err_t lora_set_preamble_length(uint16_t length);
esp_err_t lora_set_sync_word(uint8_t sw);
esp_err_t lora_enable_crc(void);
esp_err_t lora_disable_crc(void);
esp_err_t lora_init(void);
esp_err_t lora_send_packet(uint8_t *buf, uint8_t size);
esp_err_t lora_receive_packet(uint8_t *buf, uint8_t size, uint8_t *BytesRead);
uint8_t lora_received(void);
uint8_t lora_packet_rssi(void);
float lora_packet_snr(void);
void lora_close(void);
int lora_initialized(void);
esp_err_t lora_dump_registers(void);

#endif
