#pragma once

#define I2C_ADDRESS     0x20
#define I2C_WRITE_BIT   0x00
#define I2C_READ_BIT    0x01

#define REG_DATA        0x00
#define REG_DIR         0x01
#define REG_PULLUP      0x02
#define REG_PULLDOWN    0x03
// reserved
#define REG_INTR_MASK   0x05
#define REG_SENSE_HIGH  0x06
#define REG_SENSE_LOW   0x07
#define REG_INTR_SRC    0x08
#define REG_EVENT_STAT  0x09
#define REG_PLD_MODE    0x10
#define REG_PLD_TABLE0  0x11
#define REG_PLD_TABLE1  0x12
#define REG_PLD_TABLE2  0x13
#define REG_PLD_TABLE3  0x14
#define REG_PLD_TABLE4  0x15
#define REG_ADVANCED    0xAB