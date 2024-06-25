
#pragma once

#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Structure defining the configuration for the software I2C master bus
 */
typedef struct {
    gpio_num_t scl_pin;
    gpio_num_t sda_pin;
    uint8_t Address;
} soft_i2c_config_t;

/**
 * @brief Perform a write to the given device on the software I2C bus.
 *
 * @param bus Software I2C bus to perform the transfer on.
 * @param write_buffer Buffer containing the bytes to send on the buffer. Must not be NULL.
 * @param write_size Size of the write buffer. Must not be 0.
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_bb_master_write(soft_i2c_config_t * cfg, const uint8_t* write_buffer, size_t write_size);

/**
 * @brief Perform a read from the given device on the software I2C bus.
 *
 * @param bus Software I2C bus to perform the transfer on.
 * @param read_buffer Buffer that will contain the bytes received. Must not be NULL.
 * @param read_size Size of the read buffer. Must not be 0.
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_bb_master_read(soft_i2c_config_t * cfg, uint8_t* read_buffer, size_t read_size);

/**
 * @brief Perform a write followed by a read to the given device on the software I2C bus.
 *
 * @param bus Software I2C bus to perform the transfer on.
 * @param read_buffer Buffer that will contain the bytes received. Must not be NULL.
 *
 * @return ESP_OK on success
 */
esp_err_t i2c_bb_master_write_read(soft_i2c_config_t * cfg, const uint8_t* write_buffer, size_t write_size,
                                     uint8_t* read_buffer, size_t read_size);

#ifdef __cplusplus
}
#endif