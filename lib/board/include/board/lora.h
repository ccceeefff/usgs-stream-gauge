#pragma once

#ifdef __cplusplus
  extern "C" {
#endif

#include "esp_types.h"

typedef enum {
  BOARD_LORA_ANT0 = 0,    // AE201
  BOARD_LORA_ANT1 = 1,    // AE202
  BOARD_LORA_ANT_MAX = 2
} board_lora_antenna_number_t;

void board_lora_init(void);

void board_lora_enable(bool enable);
bool board_lora_is_enabled(void);

void board_lora_antenna_set_active(board_lora_antenna_number_t antNumber);

board_lora_antenna_number_t board_lora_antenna_get_active(void);

#ifdef __cplusplus
  }
#endif