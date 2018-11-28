#pragma once

#include "esp_err.h"
#include "esp_types.h"

typedef enum {
  GPIO_EX_NUM_0 = 0,
  GPIO_EX_NUM_1 = 1,
  GPIO_EX_NUM_2 = 2,
  GPIO_EX_NUM_3 = 3,
  GPIO_EX_NUM_4 = 4,
  GPIO_EX_NUM_5 = 5,
  GPIO_EX_NUM_6 = 6,
  GPIO_EX_NUM_7 = 7,
  GPIO_EX_NUM_MAX = 8
} gpio_ex_num_t;

typedef enum {
  GPIO_EX_INPUT = 1,
  GPIO_EX_OUTPUT = 0
} gpio_ex_direction_t;

typedef enum {
  GPIO_EX_LEVEL_HIGH = 1,
  GPIO_EX_LEVEL_LOW = 0
} gpio_ex_level_t;

esp_err_t sx1502_read_reg(uint8_t addr, uint8_t *data);
esp_err_t sx1502_write_reg(uint8_t addr, uint8_t data);

void sx1502_set_level(gpio_ex_num_t pin, gpio_ex_level_t level);
gpio_ex_level_t sx1502_get_level(gpio_ex_num_t pin);

void sx1502_set_direction(gpio_ex_num_t pin, gpio_ex_direction_t direction);
gpio_ex_direction_t sx1502_get_direction(gpio_ex_num_t pin);