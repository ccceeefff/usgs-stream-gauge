/*
 * This file is derived from the MicroPython project, http://micropython.org/
 *
 * Copyright (c) 2016, Pycom Limited and its licensors.
 *
 * This software is licensed under the GNU GPL version 3 or any later version,
 * with permitted additional terms. For more information see the Pycom Licence
 * v1.0 document supplied with this file, or available at:
 * https://www.pycom.io/opensource/licensing
 *
 * This file contains code under the following copyright and licensing notices.
 * The code has been changed but otherwise retained.
 */

/*
 / _____)             _              | |
( (____  _____ ____ _| |_ _____  ____| |__
 \____ \| ___ |    (_   _) ___ |/ ___)  _ \
 _____) ) ____| | | || |_| ____( (___| | | |
(______/|_____)_|_|_| \__)_____)\____)_| |_|
    (C)2013 Semtech

Description: Target board general functions implementation

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/

#ifndef LORA_BOARD_H_
#define LORA_BOARD_H_

#include <stdbool.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <stdint.h>

#include "gpio.h"
#include "timer.h"
#include "spi.h"
#include "delay.h"
#include "radio.h"
#include "timer-board.h"
#include "utilities.h"

#ifdef LORA_RADIO_SX1272
#include "sx1272.h"
#include "sx1272-board.h"
#elif LORA_RADIO_SX1276
#include "sx1276.h"
#include "sx1276-board.h"
#endif

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "driver/gpio.h"

#include "board/config.h"

#define USE_MODEM_LORA

/*!
 * Board MCU pins definitions
 */

#define RADIO_RESET                                 LORA_RADIO_RESET

#define RADIO_MOSI                                  SPI_MOSI
#define RADIO_MISO                                  SPI_MISO
#define RADIO_SCLK                                  SPI_CLK
#define RADIO_NSS                                   LORA_RADIO_NSS

#define RADIO_DIO                                   LORA_RADIO_DIO
#define RADIO_DIO1                                  LORA_RADIO_DIO1

void BoardInitPeriph( void );

void BoardInitMcu( void );

void BoardDeInitMcu( void );

uint32_t BoardGetRandomSeed( void );

void BoardGetUniqueId( uint8_t *id );

uint8_t BoardGetBatteryLevel( void );

#endif // LORA_BOARD_H_
