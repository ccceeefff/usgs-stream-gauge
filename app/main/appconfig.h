#ifndef __APPCONFIG_H
#define __APPCONFIG_H

#include "esp_types.h"

void appconfig_init(char *appName);
void appconfig_close(void);

uint16_t appconfig_get_sampling_interval(uint16_t defaultInterval);
void appconfig_set_sampling_interval(uint16_t samplingInterval);

uint16_t appconfig_get_sensor_failure_count(uint16_t defaultValue);
void appconfig_set_sensor_failure_count(uint16_t failureCount);

uint16_t appconfig_get_hard_reset_count(uint16_t defaultValue);
void appconfig_set_hard_reset_count(uint16_t count);

uint16_t appconfig_get_lora_failed_count(uint16_t defaultValue);
void appconfig_set_lora_failed_count(uint16_t count);

#endif //__APPCONFIG_H
