#include "board/spi.h"

#include <string.h>
#include "assert.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#include "board/config.h"

#define SPI_BUS         HSPI_HOST
// spi dma currently has issues (missing last byte of rx data), when using half duplex mode
// it might work fine using full-duplex mode
// enable at your own risk...
#define SPI_DMA_CHANNEL 2


static SemaphoreHandle_t _lock;

#define MAX(A, B)     ((A > B) ? A : B)

esp_err_t board_spi_init(void){
  spi_bus_config_t busConfig = {
    .mosi_io_num = SPI_MOSI,
    .miso_io_num = SPI_MISO,
    .sclk_io_num = SPI_CLK,
    .quadwp_io_num = SPI_QUAD_WP,
    .quadhd_io_num = SPI_QUAD_HD
  };

  esp_err_t ret = spi_bus_initialize(SPI_BUS, &busConfig, SPI_DMA_CHANNEL);

  _lock = xSemaphoreCreateRecursiveMutex();

  return ret;
}

esp_err_t board_spi_deinit(void){
  vSemaphoreDelete(_lock);
  return spi_bus_free(SPI_BUS);
}

void board_spi_bus_lock(void){
  xSemaphoreTakeRecursive(_lock, portMAX_DELAY);
}

void board_spi_bus_unlock(void){
  xSemaphoreGiveRecursive(_lock);
}

esp_err_t board_spi_install_device(spi_device_interface_config_t *config, board_spi_device_t **device){
  assert(device != NULL);
  assert(config != NULL);

  board_spi_device_t *dev = malloc(sizeof(board_spi_device_t));
  if(dev == NULL){
    return ESP_ERR_NO_MEM;
  }

  esp_err_t ret = spi_bus_add_device(SPI_BUS, config, &(dev->handle));
  if(ret != ESP_OK){
    free(dev);
    return ret;
  }

  // copy over config
  dev->config = *config;

  *device = dev;

  return ESP_OK;
}

esp_err_t board_spi_uninstall_device(board_spi_device_t **device){
  esp_err_t ret = spi_bus_remove_device( (*device)->handle );
  if(ret != ESP_OK){
    return ret;
  }
  free(*device);
  *device = NULL;
  return ESP_OK;
}

static esp_err_t board_spi_transact_execute(
                              board_spi_device_t *device,
                              bool overrideBits,
                              uint8_t command_bits,
                              uint8_t address_bits,
                              uint8_t dummy_bits,
                              uint16_t command,
                              uint64_t address,
                              uint8_t *txBuffer,
                              uint32_t txLength,
                              uint8_t *rxBuffer,
                              uint32_t rxLength){

  spi_transaction_t trans = {
    .cmd = command,
    .addr = address,
    .length = txLength * 8,      // must be in number of bits
    .rxlength = rxLength * 8,    // must be in number of bits
  };

  if(overrideBits){
    trans.command_bits = command_bits;
    trans.address_bits = address_bits;
    trans.dummy_bits = dummy_bits;
    trans.flags |= SPI_TRANS_OVERRIDE_BIT_PHASES;
  }

  if(txBuffer == NULL || txLength > 4){
    // use dma transfer
    trans.tx_buffer = txBuffer;
  } else {
    // use tx_data for transfer
    trans.flags |= SPI_TRANS_USE_TXDATA;
    // copy data into tx_data
    memcpy(trans.tx_data, txBuffer, txLength);
  }

  if(rxBuffer == NULL || rxLength > 4){
    // use dma transfer
    trans.rx_buffer = rxBuffer;
  } else {
    // use tx_data for transfer
    ESP_LOGD("[board][spi]", "using rxdata");
    trans.flags |= SPI_TRANS_USE_RXDATA;
  }

  esp_err_t ret;

  board_spi_bus_lock();
  ret = spi_device_transmit(device->handle, &trans);
  board_spi_bus_unlock();

  if(trans.flags & SPI_TRANS_USE_RXDATA){
    // move rx_data into rxBuffer
    ESP_LOGD("[board][spi]", "rxdata: 0x[%x][%x][%x][%x]", trans.rx_data[0],trans.rx_data[1],trans.rx_data[2],trans.rx_data[3]);
    memcpy(rxBuffer, trans.rx_data, rxLength);
  }

  return ret;
}

esp_err_t board_spi_transact(board_spi_device_t *device,
                              uint16_t command,
                              uint64_t address,
                              uint8_t *txBuffer,
                              uint32_t txLength,
                              uint8_t *rxBuffer,
                              uint32_t rxLength){
    // // command and address msb adjustment is now done by driver
    // spi_transaction_t trans = {
    //   .cmd = command,
    //   .addr = address,
    //   .length = txLength * 8,      // must be in number of bits
    //   .rxlength = rxLength * 8,    // must be in number of bits
    //   .tx_buffer = txBuffer,
    //   .rx_buffer = rxBuffer,
    // };
    //
    // esp_err_t ret;
    // ret = spi_device_transmit(device->handle, &trans);
    return board_spi_transact_execute(device, false, 0,0,0, command, address, txBuffer, txLength, rxBuffer, rxLength);
}

esp_err_t board_spi_transact_override(
                              board_spi_device_t *device,
                              uint8_t command_bits,
                              uint8_t address_bits,
                              uint8_t dummy_bits,
                              uint16_t command,
                              uint64_t address,
                              uint8_t *txBuffer,
                              uint32_t txLength,
                              uint8_t *rxBuffer,
                              uint32_t rxLength){

  return board_spi_transact_execute(device, true, command_bits,address_bits,dummy_bits, command, address, txBuffer, txLength, rxBuffer, rxLength);

  // spi_transaction_t trans = {
  //   .cmd = command,
  //   .addr = address,
  //   .length = txLength * 8,      // must be in number of bits
  //   .rxlength = rxLength * 8,    // must be in number of bits
  //   .tx_buffer = txBuffer,
  //   .rx_buffer = rxBuffer,
  //   .command_bits = command_bits,
  //   .address_bits = address_bits,
  //   .dummy_bits = dummy_bits,
  //   .flags = SPI_TRANS_OVERRIDE_BIT_PHASES
  // };
  //
  // esp_err_t ret;
  // ret = spi_device_transmit(device->handle, &trans);
  // return ret;
}
