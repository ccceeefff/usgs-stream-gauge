#include "board/i2c.h"

#include "soc/i2c_reg.h"

#include "board/config.h"

esp_err_t board_i2c_init(void){
  i2c_config_t config = {
    .mode = I2C_MODE_MASTER,
    .sda_io_num = I2C_SDA,
    .sda_pullup_en = GPIO_PULLUP_DISABLE,
    .scl_io_num = I2C_SCL,
    .scl_pullup_en = GPIO_PULLUP_DISABLE,
    .master.clk_speed = I2C_CLK_FREQ
  };
  esp_err_t err = i2c_param_config(I2C_BUS, &config);
  if(err != ESP_OK){
    return err;
  }
  i2c_set_timeout(I2C_BUS, I2C_TIMEOUT);
  return i2c_driver_install(I2C_BUS, config.mode, 0, 0, 0);
}

esp_err_t board_i2c_deinit(void){
  return i2c_driver_delete(I2C_BUS);
}

esp_err_t board_i2c_cmd_begin(i2c_cmd_handle_t cmd, portBASE_TYPE ticksToWait){
  return i2c_master_cmd_begin(I2C_BUS, cmd, ticksToWait);
}
