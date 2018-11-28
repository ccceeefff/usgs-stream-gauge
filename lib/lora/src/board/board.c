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

#include "board.h"
#include "driver/gpio.h"
#include "esp_system.h"

/*!
 * Flag to indicate if the MCU is Initialized
 */
static bool McuInitialized = false;

void BoardInitPeriph( void )
{

}

void BoardInitMcu( void )
{
    if( McuInitialized == false )
    {
      #ifdef LORA_RADIO_SX1272
        SpiInit( &SX1272.Spi, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, RADIO_NSS );
        SX1272IoInit( );
      #elif LORA_RADIO_SX1276
        SpiInit( &SX1276.Spi, RADIO_MOSI, RADIO_MISO, RADIO_SCLK, RADIO_NSS );
        SX1276IoInit( );
      #endif      
        
        // initialize gpio interrupt service
        gpio_install_isr_service(ESP_INTR_FLAG_LOWMED);

        TimerHwInit( );

        McuInitialized = true;
    }
}

void BoardDeInitMcu( void )
{
  #ifdef LORA_RADIO_SX1272
    SpiDeInit( &SX1272.Spi );
    SX1272IoDeInit( );
  #elif LORA_RADIO_SX1276
    SpiDeInit( &SX1276.Spi );
    SX1276IoDeInit( );
  #endif
  
    gpio_uninstall_isr_service();

    McuInitialized = false;
}

uint32_t BoardGetRandomSeed( void )
{
    return esp_random();
}

void BoardGetUniqueId( uint8_t *id )
{
    for (int i = 0; i < 8; i++) {
        id[i] = i;
    }
}

uint8_t BoardGetBatteryLevel( void )
{
    return 0;
}
