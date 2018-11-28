#ifndef __LORA_NETWORK_H
#define __LORA_NETWORK_H

#include "esp_types.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"

typedef struct {
  uint8_t acksRequired;
  uint8_t acksReceived;
  uint16_t txTrials;
} LoRaStats_t;

typedef struct {
  uint64_t txTime;
  uint64_t rxTime;
  uint64_t txTimeOnAir;
  uint64_t rxTimeOnAir;
  uint64_t rxDelay;
  uint8_t txDatarate;
  uint8_t rxDatarate;
  uint8_t rxSnr;
  uint8_t txAttempts;
  int16_t rxRssi;
} LoRaPacketStats_t;

typedef struct {
  uint8_t devEUI[8];
  uint8_t appEUI[8];
  uint8_t appKey[16];
} LoRaOTAASettings_t;

typedef struct {
  uint8_t  networkSessionKey[16];
  uint8_t  appSessionKey[16];
} LoRaABPSettings_t;

typedef struct {
  uint8_t otaa;
  uint8_t publicNetwork;
  uint8_t adrEnable;
  uint8_t defaultDataRate;
  uint8_t defaultTxPower;
  uint8_t maxTxAttempts;
  uint16_t fsbMask;   // each bit represents a sub-band, 1 means active, 0 means not active
  uint32_t networkID;
  uint32_t deviceID;
  union {
    LoRaOTAASettings_t ota;
    LoRaABPSettings_t abp;
  };
} LoRaNetworkSettings_t;

typedef void (*lora_receive_handler)(uint8_t port, uint8_t *buffer, uint8_t size);

void lora_network_init(LoRaNetworkSettings_t *settings, uint8_t priority);
void lora_network_shutdown(void);

bool lora_has_joined_network(void);

esp_err_t lora_send(bool confirmed, uint8_t port, uint8_t *buffer, uint8_t size, TickType_t ticksToWait);

void lora_register_receive_handler(uint8_t port, lora_receive_handler handler);

void lora_get_stats(LoRaStats_t *stats);

void lora_get_last_packet_stats(LoRaPacketStats_t *stats);

void lora_reset_stats(void);

#endif  //__LORA_NETWORK_H
