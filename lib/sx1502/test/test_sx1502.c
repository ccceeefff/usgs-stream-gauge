#include "unity.h"
#include "peripherals/sx1502.h"
#include "peripherals/sx1502Regs.h"

TEST_CASE("set and confirm pins 0,2,4,6", "[peripherals][sx1502]")
{
  uint8_t out = 0b01010101;
  uint8_t in = 0; 
  sx1502_write_reg(REG_DIR, out);
  sx1502_read_reg(REG_DIR, &in);
  TEST_ASSERT_EQUAL_HEX8(out, in);
}

TEST_CASE("set and confirm pins 1,3,5,7", "[peripherals][sx1502]")
{
  uint8_t out = 0b10101010;
  uint8_t in = 0; 
  sx1502_write_reg(REG_DIR, out);
  sx1502_read_reg(REG_DIR, &in);
  TEST_ASSERT_EQUAL_HEX8(out, in);
}

TEST_CASE("test output 6 pin high", "[peripherals][sx1502]")
{
  sx1502_set_direction(GPIO_EX_NUM_6, GPIO_EX_OUTPUT);
  sx1502_set_level(GPIO_EX_NUM_6, GPIO_EX_LEVEL_HIGH);
  TEST_PASS();
}

TEST_CASE("test output 7 pin high", "[peripherals][sx1502]")
{
  sx1502_set_direction(GPIO_EX_NUM_7, GPIO_EX_OUTPUT);
  sx1502_set_level(GPIO_EX_NUM_7, GPIO_EX_LEVEL_HIGH);
  TEST_PASS();
}

TEST_CASE("test output 6 pin low", "[peripherals][sx1502]")
{
  sx1502_set_direction(GPIO_EX_NUM_6, GPIO_EX_OUTPUT);
  sx1502_set_level(GPIO_EX_NUM_6, GPIO_EX_LEVEL_LOW);
  TEST_PASS();
}

TEST_CASE("test output 7 pin low", "[peripherals][sx1502]")
{
  sx1502_set_direction(GPIO_EX_NUM_7, GPIO_EX_OUTPUT);
  sx1502_set_level(GPIO_EX_NUM_7, GPIO_EX_LEVEL_LOW);
  TEST_PASS();
}