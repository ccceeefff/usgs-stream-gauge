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

Description: Bleeper STM32L151RD microcontroller pins definition

License: Revised BSD License, see LICENSE.TXT file include in the project

Maintainer: Miguel Luis and Gregory Cristian
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "esp_heap_alloc_caps.h"
#include "sdkconfig.h"
#include "esp_system.h"
#include "esp_spi_flash.h"
#include "nvs_flash.h"
#include "esp_event.h"
#include "esp_attr.h"
#include "driver/spi_master.h"
#include "esp_log.h"

#include "gpio.h"
#include "spi.h"

#include "board.h"
#include "soc/gpio_sig_map.h"
#include "soc/dport_reg.h"
#include "gpio-board.h"

#include "board/spi.h"

#define SPIDEV      SpiNum_SPI3

/*!
 * \brief Initializes the SPI object and MCU peripheral
 *
 * \remark When NSS pin is software controlled set the pin name to NC otherwise
 *         set the pin name to be used.
 *
 * \param [IN] obj  SPI object
 * \param [IN] mosi SPI MOSI pin name to be used
 * \param [IN] miso SPI MISO pin name to be used
 * \param [IN] sclk SPI SCLK pin name to be used
 * \param [IN] nss  SPI NSS pin name to be used
 */

// ESP32 notes:
//              SPI  => SPI_1
//              HSPI => SPI_2
//              VSPI => SPI_3

#define SPI_DMA_CHANNEL 2
#define SPI_DEVICE_CLOCK_SPEED 1000000   // 10 MHz   -- 1MHz for now.. the WROOM doesn't put out a good 10Mhz clock
#define SPI_DEVICE_QUEUE_SIZE  1
#define SPI_DEVICE_MODE        0

#define SPI_TX_MSB_FIRST        1
#define SPI_COMMAND_BITLEN      1
#define SPI_ADDRESS_BITLEN      7

void SpiInit( Spi_t *obj, PinNames mosi, PinNames miso, PinNames sclk, PinNames nss ) {
    // assign the pins
    obj->Miso.pin = miso;
    obj->Mosi.pin = mosi;
    obj->Sclk.pin = sclk;
    obj->Nss.pin = nss;
    
    spi_device_interface_config_t deviceConfig = {
      .command_bits=SPI_COMMAND_BITLEN,
      .address_bits=SPI_ADDRESS_BITLEN,
      .clock_speed_hz=SPI_DEVICE_CLOCK_SPEED, //Clock out at 10 MHz
      .mode=SPI_DEVICE_MODE,                  //SPI mode 0
      .spics_io_num=nss,                      //CS pin
      .queue_size=SPI_DEVICE_QUEUE_SIZE,      //We want to be able to queue 7 transactions at a time
      .flags=SPI_DEVICE_HALFDUPLEX,
    };
    
    board_spi_device_t *spiHandle = NULL;
    board_spi_install_device(&deviceConfig, &spiHandle);
    obj->Spi = (void *)spiHandle;
}

/*!
 * \brief De-initializes the SPI object and MCU peripheral
 *
 * \param [IN] obj SPI object
 */
void SpiDeInit( Spi_t *obj ) {
    board_spi_uninstall_device((board_spi_device_t **)&(obj->Spi));
}

/*!
 * \brief Configures the SPI peripheral
 *
 * \remark Slave mode isn't currently handled
 *
 * \param [IN] obj   SPI object
 * \param [IN] bits  Number of bits to be used. [8 or 16]
 * \param [IN] cpol  Clock polarity
 * \param [IN] cpha  Clock phase
 * \param [IN] slave When set the peripheral acts in slave mode
 */
void SpiFormat( Spi_t *obj, int8_t bits, int8_t cpol, int8_t cpha, int8_t slave ) {
    // configure the interface (only master mode supported)
}

/*!
 * \brief Sets the SPI speed
 *
 * \param [IN] obj SPI object
 * \param [IN] hz  SPI clock frequency in hz
 */
void SpiFrequency( Spi_t *obj, uint32_t hz ) {
    // configure the interface (only master mode supported)
}

/*!
 * \brief Sends outData and receives inData
 *
 * \param [IN] obj     SPI object
 * \param [IN] outData Byte to be sent
 * \retval inData      Received byte.
 */
IRAM_ATTR uint16_t SpiInOut(Spi_t *obj, uint16_t outData) {
    return 0;
}

IRAM_ATTR int SpiInOutBuffer(Spi_t *obj, uint16_t command, uint64_t address, void *txBuffer, uint32_t txBytes, void *rxBuffer, uint32_t rxBytes){
  return board_spi_transact((board_spi_device_t *)obj->Spi, command, address, txBuffer, txBytes, rxBuffer, rxBytes);
}

