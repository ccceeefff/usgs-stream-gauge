#pragma once

#ifdef __cplusplus
  extern "C" {
#endif

#include "esp_err.h"
#include "driver/i2c.h"
#include "freertos/portmacro.h"

#define I2C_ADDRESS_BYTE(address, rw) (address<<1 | rw)

#define I2C_ACK   0
#define I2C_NACK  1

esp_err_t board_i2c_init(void);
esp_err_t board_i2c_deinit(void);

esp_err_t board_i2c_cmd_begin(i2c_cmd_handle_t cmd, portBASE_TYPE ticksToWait);

#ifdef __cplusplus
  }
#endif