#include "unity.h"
#include "rom/ets_sys.h"
#include "radio.h"

TEST_CASE("Radio Read", "[lora][radio]")
{
  uint8_t val = Radio.Read(0x39);
  UnityPrintNumberHex(val, 4);
}