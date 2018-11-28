#ifndef __BOARD_CONFIG_H
#define __BOARD_CONFIG_H

#define ENVISENSE

/* I2C Configuration */
#define I2C_BUS I2C_NUM_0
#define I2C_SDA GPIO_NUM_12
#define I2C_SCL GPIO_NUM_13
#define I2C_CLK_FREQ 400000
#define I2C_TIMEOUT (I2C_SDA_SAMPLE_TIME_V << 5)

/* SPI Configuration */
#define SPI_MISO      GPIO_NUM_19
#define SPI_MOSI      GPIO_NUM_27
#define SPI_CLK       GPIO_NUM_5
#define SPI_QUAD_WP   -1
#define SPI_QUAD_HD   -1

/* LoRa Pins */
#define LORA_RADIO_RESET_ENABLED                         0
#define LORA_RADIO_RESET                                 -1
#define LORA_RADIO_NSS                                   GPIO_NUM_18
#define LORA_RADIO_DIO                                   GPIO_NUM_23
#define LORA_RADIO_DIO1                                  GPIO_NUM_23

#endif  //__BOARD_CONFIG_H
