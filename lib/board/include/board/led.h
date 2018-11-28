#pragma once

#ifdef __cplusplus
  extern "C" {
#endif

#include "esp_err.h"

/*
 * Initializes the driver for the on-board LED
 */
esp_err_t board_led_init(void);

/*
 * Turn on the LED
 */
void board_led_on(void);

/*
 * Turn off the LED
 */
void board_led_off(void);

/*
 * Set the state of the LED
 */
void board_led_set_state(int on);

/*
 * Get the state of the LED
 */
int board_led_get_state(void);

/*
 * Toggle the LED state
 */
void board_led_toggle(void);

#ifdef __cplusplus
  }
#endif