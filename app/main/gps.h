#ifndef __GPS_H
#define __GPS_H

#include "esp_types.h"

typedef struct {
  double lat;
  double lng;
  double hdop;
} Position_t;

void gps_task(void *arg);

void gps_send_ubx_msg(uint8_t *buffer, uint16_t len);

void gps_configure_powermode(void);

void gps_set_enabled(bool enabled);

bool gps_has_location_fix(void);

void gps_get_location_fix(Position_t *pos);

#endif //__GPS_H
