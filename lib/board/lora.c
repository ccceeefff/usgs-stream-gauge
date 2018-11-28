#include "board/lora.h"

#include "peripherals/sx1502.h"

#define LORA_ANT_SWITCH_GPIO_NUM GPIO_EX_NUM_0
#define LORA_ENABLE_PIN GPIO_EX_NUM_1

#define LORA_ENABLED_STATE GPIO_EX_LEVEL_HIGH

void board_lora_init(void){

  // set antenna switch gpio as output
  sx1502_set_direction(LORA_ANT_SWITCH_GPIO_NUM, GPIO_EX_OUTPUT);
}

void board_lora_enable(bool enable){
#if defined( TRAFFICSENSE )
  if(enable){
    sx1502_set_level(LORA_ENABLE_PIN, LORA_ENABLED_STATE);
    // when enabled, reset pin should be tri-stated, set as input
    sx1502_set_direction(LORA_ENABLE_PIN, GPIO_EX_INPUT);
  } else {
    // when disabling, reset pin should drive low, set as ouput
    sx1502_set_direction(LORA_ENABLE_PIN, GPIO_EX_OUTPUT);
    sx1502_set_level(LORA_ENABLE_PIN, !LORA_ENABLED_STATE);
  }
#elif defined( ENVISENSE )
  if(enabled){
    gpio_set_level(LORA_RADIO_RESET, LORA_ENABLED_STATE);
    gpio_set_direction(LORA_RADIO_RESET, GPIO_MODE_INPUT);
  } else {
    gpio_set_direction(LORA_RADIO_RESET, GPIO_MODE_OUTPUT);
    gpio_set_level(LORA_RADIO_RESET, !LORA_ENABLED_STATE);
  }
#endif
}

bool board_lora_is_enabled(void){
  // either the enable pin is tri-stated as an input
  // or the pin is being driven high
  return ((sx1502_get_direction(LORA_ENABLE_PIN) == GPIO_EX_INPUT) ||
        (sx1502_get_level(LORA_ENABLE_PIN) != !LORA_ENABLED_STATE));
}

void board_lora_antenna_set_active(board_lora_antenna_number_t antNumber){
  switch(antNumber){
    case BOARD_LORA_ANT1:
      sx1502_set_level(LORA_ANT_SWITCH_GPIO_NUM, GPIO_EX_LEVEL_HIGH);
      break;
    case BOARD_LORA_ANT0:
    default:
      sx1502_set_level(LORA_ANT_SWITCH_GPIO_NUM, GPIO_EX_LEVEL_LOW);
      break;
  }
}

board_lora_antenna_number_t board_lora_antenna_get_active(void){
  gpio_ex_level_t level = sx1502_get_level(LORA_ANT_SWITCH_GPIO_NUM);
  return (level == GPIO_EX_LEVEL_HIGH) ? BOARD_LORA_ANT1 : BOARD_LORA_ANT0;
}
