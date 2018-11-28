#include "peripherals/sx1502.h"
#include "peripherals/sx1502Regs.h"

#include "board/i2c.h"

esp_err_t sx1502_read_reg(uint8_t addr, uint8_t *data){
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // start transaction
  i2c_master_start(cmd);
  // send slave address with write bit
  i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_WRITE_BIT, true);
  // send register address
  i2c_master_write_byte(cmd, addr, true);
  // send repeated start condition
  i2c_master_start(cmd);
  // send slave address with read bit
  i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_READ_BIT, true);
  // read register and send nack
  i2c_master_read_byte(cmd, data, 1);
  // send stop
  i2c_master_stop(cmd);
  esp_err_t err = board_i2c_cmd_begin(cmd, portMAX_DELAY);
  i2c_cmd_link_delete(cmd);
  return err;
}

esp_err_t sx1502_write_reg(uint8_t addr, uint8_t data){
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  // send start
  i2c_master_start(cmd);
  // send slave address with write bit
  i2c_master_write_byte(cmd, (I2C_ADDRESS << 1) | I2C_WRITE_BIT, true);
  // send register address
  i2c_master_write_byte(cmd, addr, true);
  // send data
  i2c_master_write_byte(cmd, data, true);
  // send stop condition
  i2c_master_stop(cmd);
  esp_err_t err = board_i2c_cmd_begin(cmd, portMAX_DELAY);
  i2c_cmd_link_delete(cmd);
  return err;
}

void sx1502_set_level(gpio_ex_num_t pin, gpio_ex_level_t level){
  uint8_t data;
  sx1502_read_reg(REG_DATA, &data);
  if(level){
    data |= (1<<pin);  // set bit
  } else {
    data &= ~(1<<pin); // clear bit
  }
  sx1502_write_reg(REG_DATA, data);
}

gpio_ex_level_t sx1502_get_level(gpio_ex_num_t pin){
  uint8_t data;
  sx1502_read_reg(REG_DATA, &data);
  return (data >> pin) & 1; 
}

void sx1502_set_direction(gpio_ex_num_t pin, gpio_ex_direction_t direction){
  uint8_t data;
  sx1502_read_reg(REG_DIR, &data);
  if(direction){
    data |= (1<<pin);  // set bit
  } else {
    data &= ~(1<<pin); // clear bit
  }
  sx1502_write_reg(REG_DIR, data);
}

gpio_ex_direction_t sx1502_get_direction(gpio_ex_num_t pin){
  uint8_t direction;
  sx1502_read_reg(REG_DIR, &direction);
  return (direction >> pin) & 1; 
}