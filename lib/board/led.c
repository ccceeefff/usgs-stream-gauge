#include "board/led.h"

#include "assert.h"
#include "esp_log.h"
#include "driver/gpio.h"

/* Defines the GPIO number which the LED is connected to */
#define LED_GPIO            GPIO_NUM_14

/* Defines the logic level at which the LED is considered to be ON */
#define LED_LOGIC_LEVEL_ON  0

esp_err_t board_led_init(void){
  assert(GPIO_IS_VALID_OUTPUT_GPIO(LED_GPIO));
  gpio_config_t config = {
    .pin_bit_mask = (1 << LED_GPIO),
    .mode = GPIO_MODE_INPUT_OUTPUT,
    .pull_up_en = GPIO_PULLUP_DISABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  esp_err_t err = gpio_config(&config);
  return err;
}

void board_led_on(void){
  gpio_set_level(LED_GPIO, LED_LOGIC_LEVEL_ON);
}

void board_led_off(void){
  gpio_set_level(LED_GPIO, !LED_LOGIC_LEVEL_ON);
}

void board_led_set_state(int on){
  gpio_set_level(LED_GPIO, on ? LED_LOGIC_LEVEL_ON : !LED_LOGIC_LEVEL_ON);
}

int board_led_get_state(void){
  int level = gpio_get_level(LED_GPIO);
  return (level == LED_LOGIC_LEVEL_ON);
}

void board_led_toggle(void){
  bool state = board_led_get_state();
  board_led_set_state(!state);
}
