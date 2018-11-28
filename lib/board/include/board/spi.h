#pragma once

#ifdef __cplusplus
  extern "C" {
#endif

#include "esp_err.h"
#include "driver/spi_master.h"
#include "freertos/portmacro.h"

typedef struct {
  spi_device_interface_config_t config;
  spi_device_handle_t handle;
} board_spi_device_t;

esp_err_t board_spi_init(void);

esp_err_t board_spi_deinit(void);

esp_err_t board_spi_install_device(spi_device_interface_config_t *config, board_spi_device_t **device);

esp_err_t board_spi_uninstall_device(board_spi_device_t **device);

// locking tools for devices that do not hand over control
// of CS pin to spi bus driver
void board_spi_bus_lock(void);
void board_spi_bus_unlock(void);

esp_err_t board_spi_transact(board_spi_device_t *device,
                              uint16_t command,
                              uint64_t address,
                              uint8_t *txBuffer,
                              uint32_t txLength,
                              uint8_t *rxBuffer,
                              uint32_t rxLength);

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
                              uint32_t rxLength);

#ifdef __cplusplus
  }
#endif
