#include "unity.h"
#include "board/lora.h"

TEST_CASE("Can properly set LoRa Antenna to 0", "[board][lora][ant]")
{
  board_lora_antenna_set_active(BOARD_LORA_ANT0);
  TEST_ASSERT_EQUAL(board_lora_antenna_get_active(), BOARD_LORA_ANT0);
}

TEST_CASE("Can properly set LoRa Antenna to 1", "[board][lora][ant]")
{
  board_lora_antenna_set_active(BOARD_LORA_ANT1);
  TEST_ASSERT_EQUAL(board_lora_antenna_get_active(), BOARD_LORA_ANT1);
}