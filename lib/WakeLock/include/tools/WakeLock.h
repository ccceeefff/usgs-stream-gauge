#ifndef __WAKE_LOCK_H
#define __WAKE_LOCK_H

#include "esp_types.h"

void wake_lock_init(void);
void wake_lock_acquire(void);
void wake_lock_release(void);
void wake_lock_deinit(void);

bool wake_lock_active(void);

#endif // __WAKE_LOCK_H
