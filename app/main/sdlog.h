#ifndef __SDLOG_H
#define __SDLOG_H

#include "esp_types.h"
#include "sense.h"
#include "gps.h"
#include "network/lora.h"

bool sd_log_sensor_data(uint32_t sampleTime, bool sensorReadError, SenseData_t *data);
void sd_log_sensor_events(uint16_t bootCount,
                          uint16_t sensorFailureCount,
                          uint16_t loraFailureCount,
                          bool sensorError,
                          bool loraSendError,
                          bool sdWriteError,
                          uint16_t samplingInterval,
                          SenseData_t *data,
                          bool hasGpsData,
                          Position_t *pos,
                          LoRaStats_t *stats);

#endif //__SDLOG_H
