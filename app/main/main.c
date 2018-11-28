#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_spi_flash.h"
#include "tools/WakeLock.h"
#include "board_init.h"
#include "rom/rtc.h"

#include "appconfig.h"
#include "sense.h"
#include "gps.h"
#include "config.h"

static RTC_DATA_ATTR struct {
  uint32_t lastWakeUpTime;
  uint32_t lastSleepTime;
} system_time;

static char *TAG = "[main]";

uint32_t system_get_sleep_time() {
  uint16_t sampleInterval = appconfig_get_sampling_interval(DEFAULT_SAMPLE_INTERVAL);

  struct timeval now;
  gettimeofday(&now, NULL);
  system_time.lastSleepTime = now.tv_sec;
  uint32_t sleepTime;

  if(system_time.lastSleepTime > system_time.lastWakeUpTime){
    // this is the normal case... unless a glitch in the timer happens..
    uint32_t wakeDuration =  (system_time.lastSleepTime - system_time.lastWakeUpTime);
    ESP_LOGI(TAG,"wake duration: %u", wakeDuration);
    if(wakeDuration > sampleInterval){
      // this means that we've taken too long to handle and we've missed the deadline...
      // just sleep for the next sample interval...
      ESP_LOGW(TAG,"wake duration longer than sampling time!! %u > %u", wakeDuration, sampleInterval);
      sleepTime = sampleInterval;
    } else {
      // if we're still in range
      ESP_LOGI(TAG,"Setting proper sampling interval...");
      sleepTime = sampleInterval - wakeDuration;
    }
  } else {
    // for some reason, our clocks have turned back in time...
    // just sleep for the regular sampling time to be sure...
    ESP_LOGW(TAG,"Time travel has occurred... wakeup(%u) sleeptime(%u)", system_time.lastWakeUpTime, system_time.lastSleepTime);
    sleepTime = sampleInterval;
  }
  if(sleepTime < MINIMUM_SAMPLE_INTERVAL){
    ESP_LOGW(TAG,"Sleep interval below minimum!");
    sleepTime = MINIMUM_SAMPLE_INTERVAL;
  } else if(sleepTime > MAXIMUM_SAMPLE_INTERVAL){
    ESP_LOGW(TAG,"Sleep interval above maximum!");
    sleepTime = MAXIMUM_SAMPLE_INTERVAL;
  }

  if(sense_gps_requested()){
    ESP_LOGI(TAG, "gps request running while in sleep");
    if(sleepTime > GPS_SLEEP_TIME_S){
      sleepTime = GPS_SLEEP_TIME_S;
    }
  }
  ESP_LOGI(TAG, "sleep time: %u\n", sleepTime);
  return sleepTime;
}

void increment_boot_count(void){
  uint16_t count = appconfig_get_hard_reset_count(0) + 1;
  appconfig_set_hard_reset_count(count);
}

void app_main()
{
    struct timeval now;
    gettimeofday(&now, NULL);
    system_time.lastWakeUpTime = now.tv_sec;
    time_t mtime = now.tv_sec;
    struct tm mtm;
    gmtime_r(&mtime, &mtm);
    const char* time_str = asctime(&mtm);
    printf("\n==============\nStart time: %s UTC\n=============\n", time_str);
    printf("wake up cause: %d reset reason(%d, %d)\n", esp_sleep_get_wakeup_cause(), rtc_get_reset_reason(0), rtc_get_reset_reason(1));
    board_init();
    wake_lock_acquire();

    switch (esp_sleep_get_wakeup_cause()) {
        case ESP_SLEEP_WAKEUP_TIMER: {
            int sleep_time = (now.tv_sec - system_time.lastSleepTime);
            ESP_LOGI(TAG, "Wake up from timer. Time spent in deep sleep: %d s", sleep_time);
            break;
        }
        case ESP_SLEEP_WAKEUP_UNDEFINED:
        default:
            ESP_LOGI(TAG, "Not a deep sleep reset");
            increment_boot_count();
            gps_set_enabled(true);
            break;
    }

    // spin up tasks here
    xTaskCreate(&sense_task, "SensorTask", ESP_TASK_MAIN_STACK, NULL, ESP_TASK_MAIN_PRIO, NULL);
    xTaskCreate(&gps_task, "gpstask", ESP_TASK_MAIN_STACK, NULL, ESP_TASK_MAIN_PRIO, NULL);
    // add some delay to allow tasks to boot up before releasing wake lock
    vTaskDelay(1);
    ESP_LOGW(TAG, "main end");
    wake_lock_release();
}
