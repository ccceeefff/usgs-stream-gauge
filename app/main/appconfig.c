#include "appconfig.h"

#include "esp_log.h"
#include "nvs.h"
#include "nvs_flash.h"
#include "string.h"
#include "config.h"

#if DEBUG
  #define NVS_KEY_SAMPLING_INTERVAL "samphz-dev"
  #define NVS_KEY_FAILURE_COUNT     "failcount-dev"
  #define NVS_KEY_BOOT_COUNT        "bootcount-dev"
  #define NVS_KEY_LORA_FAIL_COUNT   "loracount-dev"
#else
  #define NVS_KEY_SAMPLING_INTERVAL "samphz"
  #define NVS_KEY_FAILURE_COUNT     "failcount"
  #define NVS_KEY_BOOT_COUNT        "bootcount"
  #define NVS_KEY_LORA_FAIL_COUNT   "loracount"
#endif

static nvs_handle _nvs_ = NULL;

static const char *TAG = "[appconfig]";

void appconfig_init(char *appName){
  // Initialize NVS
  esp_err_t err = nvs_flash_init();
  if (err == ESP_ERR_NVS_NO_FREE_PAGES) {
      // NVS partition was truncated and needs to be erased
      // Retry nvs_flash_init
      ESP_ERROR_CHECK(nvs_flash_erase());
      err = nvs_flash_init();
  }
  ESP_ERROR_CHECK( err );

  if(_nvs_ == NULL){
    err = nvs_open(appName, NVS_READWRITE, &_nvs_);
    if (err != ESP_OK) {
      ESP_LOGE(TAG, "Error opening NVS handle (%d)", err);
    }
  }
}

void appconfig_close(void){
  nvs_commit(_nvs_);
  nvs_close(_nvs_);
  _nvs_ = NULL;
}

uint16_t appconfig_get_u16(char *key, uint16_t defaultValue){
  uint16_t value;
  esp_err_t err = nvs_get_u16(_nvs_, key, &value);
  switch (err) {
      case ESP_OK:
          ESP_LOGI(TAG, "found value: %s = %d", key, value);
          //value holds the right value
          break;
      default:
          ESP_LOGE(TAG, "Error retrieving value for %s: 0x%x", key, err);
          value = defaultValue;
          break;
  }
  return value;
}

void appconfig_set_u16(char *key, uint16_t value){
  esp_err_t err = nvs_set_u16(_nvs_, key, value);
  if(err != ESP_OK){
    ESP_LOGE(TAG, "Failed to write %s to nvs: 0x%x", key, err);
  }
  nvs_commit(_nvs_);
}

uint16_t appconfig_get_sampling_interval(uint16_t defaultInterval){
  return appconfig_get_u16(NVS_KEY_SAMPLING_INTERVAL, defaultInterval);
}

void appconfig_set_sampling_interval(uint16_t samplingInterval){
  appconfig_set_u16(NVS_KEY_SAMPLING_INTERVAL, samplingInterval);
}

uint16_t appconfig_get_sensor_failure_count(uint16_t defaultValue){
  return appconfig_get_u16(NVS_KEY_FAILURE_COUNT, defaultValue);
}

void appconfig_set_sensor_failure_count(uint16_t failureCount){
  appconfig_set_u16(NVS_KEY_FAILURE_COUNT, failureCount);
}

uint16_t appconfig_get_hard_reset_count(uint16_t defaultValue){
  return appconfig_get_u16(NVS_KEY_BOOT_COUNT, defaultValue);
}

void appconfig_set_hard_reset_count(uint16_t count){
  appconfig_set_u16(NVS_KEY_BOOT_COUNT, count);
}

uint16_t appconfig_get_lora_failed_count(uint16_t defaultValue){
  return appconfig_get_u16(NVS_KEY_LORA_FAIL_COUNT, defaultValue);
}

void appconfig_set_lora_failed_count(uint16_t count){
  appconfig_set_u16(NVS_KEY_LORA_FAIL_COUNT, count);
}
