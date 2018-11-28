#include "unity.h"
#include "board/led.h"
#include "rom/ets_sys.h"

TEST_CASE("toggle LED every second", "[board][led]")
{
  int delay_ms = 1000;
  int reps = 10;
  while(reps > 0){
    board_led_on();
    ets_delay_us(delay_ms * 1000);
    board_led_off();
    ets_delay_us(delay_ms * 1000);
    reps--;
  }
  
  TEST_PASS();
}

TEST_CASE("Can query LED state on", "[board][led]")
{
  // set to on and check state
  board_led_set_state(true);
  TEST_ASSERT_TRUE(board_led_get_state());  
}

TEST_CASE("Can query LED state off", "[board][led]")
{
  // set to off and check state
  board_led_set_state(false);
  TEST_ASSERT_FALSE(board_led_get_state());
}

TEST_CASE("Properly turns LED on", "[board][led]")
{
  board_led_set_state(false);
  
  board_led_on();
  TEST_ASSERT_TRUE(board_led_get_state());  
}

TEST_CASE("Properly turns LED off", "[board][led]")
{
  board_led_set_state(true);
  
  board_led_off();
  TEST_ASSERT_FALSE(board_led_get_state());
}

TEST_CASE("Toggling LED changes state", "[board][led]")
{
  board_led_set_state(false);
  board_led_toggle();
  TEST_ASSERT_TRUE_MESSAGE(board_led_get_state(), "LED should be on");
  board_led_toggle();
  TEST_ASSERT_FALSE_MESSAGE(board_led_get_state(), "LED should be off");
}