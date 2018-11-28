#include "tools/WakeLock.h"

#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

#define MAX_LOCK_HOLDERS 10

static SemaphoreHandle_t _guard;
static uint8_t _locks = 0;

void wake_lock_init(void){
  _guard = xSemaphoreCreateMutex();
  _locks = 0;
}

void wake_lock_acquire(void){
  xSemaphoreTake(_guard, portMAX_DELAY);
  _locks++;
  xSemaphoreGive(_guard);
}

void wake_lock_release(void){
  xSemaphoreTake(_guard, portMAX_DELAY);
  // assert(_locks > 0);
  _locks--;
  xSemaphoreGive(_guard);
}

void wake_lock_deinit(void){
  vSemaphoreDelete(_guard);
  _locks = 0;
}

bool wake_lock_active(void){
  return (_locks > 0);
}
