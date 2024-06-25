#include <stdint.h>
#include "ulp_riscv.h"
#include "ulp_riscv_utils.h"
#include "ulp_riscv_gpio.h"
#include "ulp_i2c_bitbang.h"
#include "tools.h"

/* Forward declaration of static functions */
static esp_err_t emulate_i2c_transfer(soft_i2c_config_t *cfg, const uint8_t *write_buffer, uint32_t write_size,
                                      uint8_t *read_buffer, uint32_t read_size);

const char *__attribute__((used)) SOFT_I2C_MASTER_TAG = "soft_i2c_master";

esp_err_t i2c_bb_master_write(soft_i2c_config_t *cfg, const uint8_t *write_buffer, size_t write_size)
{
  esp_err_t ret;
  ret = emulate_i2c_transfer(cfg, write_buffer, write_size,
                             NULL, 0);
  return ret;
}

esp_err_t i2c_bb_master_read(soft_i2c_config_t *cfg, uint8_t *read_buffer, size_t read_size)
{
  esp_err_t ret;
  ret = emulate_i2c_transfer(cfg, NULL, 0, read_buffer, read_size);
  return ret;
}

esp_err_t i2c_bb_master_write_read(soft_i2c_config_t *cfg, const uint8_t *write_buffer, size_t write_size,
                                   uint8_t *read_buffer, size_t read_size)
{
  esp_err_t ret;
  ret = emulate_i2c_transfer(cfg, write_buffer, write_size,
                             read_buffer, read_size);
  return ret;
}

/***** Private implementation *****/

static inline void set_scl(soft_i2c_config_t *cfg, uint32_t value)
{
  ulp_riscv_gpio_output_level(cfg->scl_pin, value);
  // ulp_riscv_delay_cycles(3 * ULP_RISCV_CYCLES_PER_US); // 3 µs delay
  delay(3 * ULP_RISCV_CYCLES_PER_US);
}

static inline void set_sda(soft_i2c_config_t *cfg, uint32_t value)
{
  ulp_riscv_gpio_output_level(cfg->sda_pin, value);
  // ulp_riscv_delay_cycles(3 * ULP_RISCV_CYCLES_PER_US); // 3 µs delay
  delay(3 * ULP_RISCV_CYCLES_PER_US);
}

static inline uint32_t get_sda(soft_i2c_config_t *cfg)
{
  return ulp_riscv_gpio_get_level(cfg->sda_pin);
}

static inline void emulate_start(soft_i2c_config_t *cfg)
{
  /* A Start consists in pulling SDA low when SCL is high. */
  set_scl(cfg, 1);
  /* If SDA is low, pull it high first */
  set_sda(cfg, 1);
  set_sda(cfg, 0);
}

static inline void emulate_stop(soft_i2c_config_t *cfg)
{
  /* Stop consists in pulling SDA high while SCL is also high. First, pull both low, make sure
   * it is not detected as a "start" so let's do it one by one. */
  set_scl(cfg, 0);
  set_sda(cfg, 0);
  /* Pull SCL high */
  set_scl(cfg, 1);
  /* Pull SDA high */
  set_sda(cfg, 1);
}

static inline int emulate_write_byte(soft_i2c_config_t *cfg, uint_fast8_t byte)
{
  for (int i = 7; i >= 0; i--)
  {
    const uint_fast8_t bit = (byte >> i) & 1;
    /* Set SCL to low */
    set_scl(cfg, 0);
    /* Set SDA value now */
    set_sda(cfg, bit);
    /* Set SCL to high */
    set_scl(cfg, 1);
  }

  /* Send one more bit to get the ACK/NACK */
  set_scl(cfg, 0);
  set_sda(cfg, 1);
  set_scl(cfg, 1);

  /* Get the SDA bit */
  uint32_t ret = get_sda(cfg) == 0;

  /* Pull clock low, to tell the device to stop pulling SDA down */
  set_scl(cfg, 0);
  return ret;
}

static inline uint8_t emulate_read_byte(soft_i2c_config_t *cfg, int send_ack)
{
  uint8_t result = 0;
  uint32_t in = 0;

  /* Set SCL to low as we are going to put SDA in high-impedance */
  set_scl(cfg, 0);
  set_sda(cfg, 1);

  for (int i = 7; i >= 0; i--)
  {
    /* Set SCL to low */
    set_scl(cfg, 0);
    /* Get SDA value now and store it in the final result */
    in = get_sda(cfg);
    result = (result << 1) | in;
    /* Set SCL to high */
    set_scl(cfg, 1);
  }

  /* Send one more bit to set ACK/NACK */
  set_scl(cfg, 0);
  set_sda(cfg, !send_ack);
  set_scl(cfg, 1);

  return result;
}

static esp_err_t emulate_i2c_transfer(soft_i2c_config_t *cfg, const uint8_t *write_buffer, uint32_t write_size,
                                      uint8_t *read_buffer, uint32_t read_size)
{
  esp_err_t ret = ESP_OK;

  /* Set both pins to high to start */
  set_sda(cfg, 1);
  set_scl(cfg, 1);

  /* Perform a start/write on the bus */
  if (write_buffer != NULL && write_size != 0)
  {
    emulate_start(cfg);
    int ack = emulate_write_byte(cfg, cfg->Address << 1);
    if (!ack)
    {
      ret = ESP_ERR_NOT_FOUND;
    }

    for (int i = 0; i < write_size && ret == ESP_OK; i++)
    {
      /* Check the ACK returned by the device */
      ack = emulate_write_byte(cfg, write_buffer[i]);
      if (!ack)
      {
        ret = ESP_FAIL;
      }
    }
  }

  /* Perform a (re)start/read on the bus */
  if (ret == ESP_OK && read_buffer != NULL && read_size != 0)
  {
    emulate_start(cfg);
    int ack = emulate_write_byte(cfg, (cfg->Address << 1) | 1);
    if (!ack)
    {
      ret = ESP_ERR_NOT_FOUND;
    }
    else
    {
      for (int i = 0; i < read_size; i++)
      {
        /* We must send an ACK after each byte read, except the last one */
        const int send_ack = i != (read_size - 1);
        read_buffer[i] = emulate_read_byte(cfg, send_ack);
      }
    }
  }

  emulate_stop(cfg);

  return ret;
}