#ifndef __SENSE_H
#define __SENSE_H

#include "esp_types.h"

typedef struct __attribute__((packed)) {
  uint16_t reportCount;
  double waterLevel;
  double sensorTemperature;
  double pcbTemperature;
  double batteryVoltage;
} SenseData_t;

void sense_task(void *arg);

bool sense_gps_requested(void);

#endif //__SENSE_H
