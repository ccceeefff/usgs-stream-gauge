#include "sense.h"

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <stdio.h>
#include <stdbool.h>
#include <ctype.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "peripherals/sx1502.h"

#include "appconfig.h"
#include "driver/sdi12.h"
#include "tools/WakeLock.h"
#include "network/lora.h"
#include "sdlog.h"
#include "gps.h"
#include "config.h"

#define LORA_FAILURE_PORT           21
#define LORA_FAILURE_CONFIRMED_TX   true

#define LORA_SENSE_PORT           22
#define LORA_SENSE_CONFIRMED_TX   true

#define LORA_GPS_PORT           23
#define LORA_GPS_CONFIRMED_TX   true

#define LORA_MESSAGE_BUFLEN       11

#define DATA_FILENAME "/sdcard/test.csv"

static char *TAG = "[app]";

uint32_t sendTime = 0;
uint32_t recvTime = 0;
static RTC_DATA_ATTR uint16_t count = 0;
static RTC_DATA_ATTR uint8_t gpsRequested = false;
static RTC_DATA_ATTR uint8_t gpsAttempts = 0;

bool sense_gps_requested(void){
  return gpsRequested;
}

static void receive_handler(uint8_t port, uint8_t *buffer, uint8_t size){
  ESP_LOGI(TAG, "received data on port: %d", port);
  for(int i=0; i < size; i++){
    ESP_LOGD(TAG, "data[%d] = 0x%X", i, buffer[i]);
  }

  struct timeval now;
  gettimeofday(&now, NULL);
  recvTime = now.tv_sec;
  uint32_t networkDelay = recvTime - sendTime;

  // get current time in seconds
  uint32_t ts = buffer[0] | (buffer[1] << 8) | (buffer[2] << 16) | (buffer[3] << 24);
  uint16_t samplingInterval = (buffer[4] | buffer[5] << 8);
  uint16_t flags = ((buffer[6] | buffer[7] << 8));

  if(gpsRequested == false && (flags & 0x01)){
    // should we request for gps in the next cycle?
    gpsRequested = true;
    gpsAttempts = 0;
  }

  // apply network delay
  if(ts > 0){ // make sure we get a valid time value
    ts += networkDelay;
    ESP_LOGI(TAG, "previous time: %d | new time: %d | new sampling interval : %d", recvTime, ts, samplingInterval);
    now.tv_sec = ts;
    settimeofday(&now, NULL);
  } else {
    ESP_LOGE(TAG, "Invalid time packet received. keeping previous time.");
  }

  uint16_t oldSamplingInterval = appconfig_get_sampling_interval(15);
  if(oldSamplingInterval != samplingInterval){
    // only write if anything changed
    appconfig_set_sampling_interval(samplingInterval);
  }
}

void sense_data_serialize(SenseData_t *data, LoRaStats_t *stats, uint8_t *buffer){
  // write report count
  buffer[0] = (data->reportCount % 256);

  // write battery voltage as uint8_t * 10
  uint8_t batteryVoltage = (uint8_t)(data->batteryVoltage * 10);
  buffer[1] = batteryVoltage;

  // write water level data as int32_t * 1000 -> milliFt
  int32_t waterLevel = (int32_t)(data->waterLevel * 1000);
  buffer[2] = (waterLevel >> 0) & 0xFF;
  buffer[3] = (waterLevel >> 8) & 0xFF;
  buffer[4] = (waterLevel >> 16) & 0xFF;
  buffer[5] = (waterLevel >> 24) & 0xFF;

  // write sensor temperature as int16_t * 100
  int16_t sensorTemperature = (int16_t)(data->sensorTemperature * 100);
  buffer[6] = (sensorTemperature >> 0) & 0xFF;
  buffer[7] = (sensorTemperature >> 8) & 0xFF;

  /* Replace pcb temperature with LoRa Stats
  // write pcb temperature as int16_t * 100
  int16_t pcbTemperature = (int16_t)(data->pcbTemperature * 100);
  buffer[8] = (pcbTemperature >> 0) & 0xFF;
  buffer[9] = (pcbTemperature >> 8) & 0xFF;
  */
  buffer[8] = stats->acksRequired;
  buffer[9] = stats->acksReceived;
  buffer[10] = (stats->txTrials % 256);
}

uint16_t increment_send_failed_count(void){
  uint16_t loraFailureCount = appconfig_get_lora_failed_count(0) + 1;
  appconfig_set_lora_failed_count(loraFailureCount);
  return loraFailureCount;
}

bool sense_gps_report_send(Position_t *pos){
  uint8_t buffer[10];
  int32_t lat = (pos->lat) * 10000000;
  int32_t lng = (pos->lng) * 10000000;
  int16_t hdop = pos->hdop * 100;
  buffer[0] = (lat >> 0) & 0xFF;
  buffer[1] = (lat >> 8) & 0xFF;
  buffer[2] = (lat >> 16) & 0xFF;
  buffer[3] = (lat >> 24) & 0xFF;
  buffer[4] = (lng >> 0) & 0xFF;
  buffer[5] = (lng >> 8) & 0xFF;
  buffer[6] = (lng >> 16) & 0xFF;
  buffer[7] = (lng >> 24) & 0xFF;
  buffer[8] = (hdop >> 0) & 0xFF;
  buffer[9] = (hdop >> 9) & 0xFF;

  struct timeval now;
  gettimeofday(&now, NULL);
  sendTime = now.tv_sec;
  if(lora_send(LORA_GPS_CONFIRMED_TX, LORA_GPS_PORT, buffer, 10, portMAX_DELAY) == ESP_OK){
    // success!
    return false;
  } else {
    // failed to send
    return true;
  }
}

bool sense_report_send(SenseData_t *data, LoRaStats_t *stats){
  uint8_t buffer[LORA_MESSAGE_BUFLEN];
  sense_data_serialize(data, stats, buffer);

  struct timeval now;
  gettimeofday(&now, NULL);
  sendTime = now.tv_sec;
  if(lora_send(LORA_SENSE_CONFIRMED_TX, LORA_SENSE_PORT, buffer, LORA_MESSAGE_BUFLEN, portMAX_DELAY) == ESP_OK){
    // success!
    return false;
  } else {
    // failed to send
    return true;
  }
}

bool sense_report_failure(uint16_t bootCount, uint16_t failureCount, uint16_t loraFailureCount){
  uint8_t buffer[6];

  buffer[0] = (bootCount >> 0) & 0xFF;
  buffer[1] = (bootCount >> 8) & 0xFF;
  buffer[2] = (failureCount >> 0) & 0xFF;
  buffer[3] = (failureCount >> 8) & 0xFF;
  buffer[4] = (loraFailureCount >> 0) & 0xFF;
  buffer[5] = (loraFailureCount >> 8) & 0xFF;

  struct timeval now;
  gettimeofday(&now, NULL);
  sendTime = now.tv_sec;
  if(lora_send(LORA_FAILURE_CONFIRMED_TX, LORA_FAILURE_PORT, buffer, 6, portMAX_DELAY) == ESP_OK){
    // success!
    return false;
  } else {
    return true;
  }
}

bool read_sensor(SenseData_t *arg);

void sense_task(void *arg){
  wake_lock_acquire();

  SenseData_t data;
  LoRaStats_t stats;
  Position_t pos;
  bool hasPositionData = false;

  lora_register_receive_handler(LORA_SENSE_PORT, receive_handler);
  ESP_LOGI(TAG,"Sample begin");

  // get time before starting
  struct timeval now;
  gettimeofday(&now, NULL);
  uint32_t sampleTime = now.tv_sec;
  bool sensorReadError = read_sensor(&data);
  bool loraSendError = false;
  uint16_t bootCount = appconfig_get_hard_reset_count(0);
  uint16_t failureCount = appconfig_get_sensor_failure_count(0);
  uint16_t loraFailureCount = appconfig_get_lora_failed_count(0);
  lora_get_stats(&stats);
  if(sensorReadError){
    // failed to read sensor, send error message
    // update failure count
    failureCount += 1;
    appconfig_set_sensor_failure_count(failureCount);
    // send report fail
    loraSendError = sense_report_failure(bootCount, failureCount, loraFailureCount);
  } else {
    loraSendError = sense_report_send(&data, &stats);
  }
  if(loraSendError){
    loraFailureCount = increment_send_failed_count();
  }
  vTaskDelay(10);

  // did we request for gps?
  if(gpsRequested){
    // check if we have gps lock
    if(gps_has_location_fix()){
      gps_get_location_fix(&pos);
      hasPositionData = true;
      // if we do, report it
      if(sense_gps_report_send(&pos) == false){ // successfully sent gps!
        gpsRequested = false;
      }
    }
    gpsAttempts++;
  }

  if(gpsAttempts >= GPS_QUERY_ATTEMPTS){
    gpsRequested = false;
  }

  uint16_t samplingInterval = appconfig_get_sampling_interval(15);
  bool writeSuccess = sd_log_sensor_data(sampleTime, sensorReadError, &data);
  sd_log_sensor_events(bootCount,
                        failureCount,
                        loraFailureCount,
                        sensorReadError,
                        loraSendError,
                        !writeSuccess,
                        samplingInterval,
                        &data,
                        hasPositionData,
                        &pos,
                        &stats
                        );

  // are we requesting for a gps lock?
  // if so, leave gps enabled and go to sleep.
  // turn it off on next cycle
  if(gpsRequested){
    gps_set_enabled(true);
  } else {
    gps_set_enabled(false);
  }

  ESP_LOGI(TAG,"Sample end");

  wake_lock_release();
  vTaskDelete(NULL);
}

void resetSDI12(void){
  // set reset line low
  sx1502_set_direction(GPIO_EX_NUM_7, GPIO_EX_OUTPUT);
  sx1502_set_level(GPIO_EX_NUM_7, GPIO_EX_LEVEL_LOW);
  ESP_LOGW("sdi12", "reset low");
  vTaskDelay(1000);
  // set reset line high
  sx1502_set_level(GPIO_EX_NUM_7, GPIO_EX_LEVEL_HIGH);
  ESP_LOGW("sdi12", "reset high");
  vTaskDelay(1000);

  sx1502_set_direction(GPIO_EX_NUM_7, GPIO_EX_INPUT);
}

void read_water_level_and_temperature(uint8_t address, double *level, double *temperature){
  SDI12Measurement_t *measurement = NULL;
  esp_err_t err = sdi12_startAdditionalMeasurement(address, 7, false, false, &measurement);
  ESP_LOGI("main", "water_level: (%d)", err);
  double val = 0;
  if(measurement != NULL){
    ESP_LOGI("main", "water_level count: (%d)", measurement->count);
    for(int i=0; i < measurement->count; i++){
      ESP_LOGI("main", "water_level[%d]: %c%s", i, measurement->values[i].polarity, measurement->values[i].digits);
      sscanf(measurement->values[i].digits, "%lf", &val);
      if(measurement->values[i].polarity == '-'){
        val = -val;
      }
      ESP_LOGI("main", "value: %f", val);
      switch(i){
        case 0:
          *level = val;
          break;
        case 2:
          *temperature = val;
          break;
      }
    }
  }
  sdi12_cleanup_measurement(measurement);
}

void read_pcb_temperature(uint8_t address, double *temperature){
  SDI12Measurement_t *measurement = NULL;
  esp_err_t err = sdi12_startAdditionalMeasurement(address, 5, false, false, &measurement);
  ESP_LOGI("main", "pcb temp: (%d)", err);
  double val = 0;
  if(measurement != NULL){
    ESP_LOGI("main", "pcb temp count: (%d)", measurement->count);
    for(int i=0; i < measurement->count; i++){
      ESP_LOGI("main", "pcb temp[%d]: %c%s", i, measurement->values[i].polarity, measurement->values[i].digits);
      sscanf(measurement->values[i].digits, "%lf", &val);
      if(measurement->values[i].polarity == '-'){
        val = -val;
      }
      ESP_LOGI("main", "value: %f", val);
      if(i == 0){
          *temperature = val;
      }
    }
  }
}

void read_battery_voltage(uint8_t address, double *batteryVoltage){
  SDI12Measurement_t *measurement = NULL;
  esp_err_t err = sdi12_startAdditionalMeasurement(address, 6, false, false, &measurement);
  ESP_LOGI("main", "battery voltage: (%d)", err);
  double val = 0;
  if(measurement != NULL){
    ESP_LOGI("main", "battery voltage count: (%d)", measurement->count);
    for(int i=0; i < measurement->count; i++){
      ESP_LOGI("main", "battery voltage[%d]: %c%s", i, measurement->values[i].polarity, measurement->values[i].digits);
      sscanf(measurement->values[i].digits, "%lf", &val);
      if(measurement->values[i].polarity == '-'){
        val = -val;
      }
      ESP_LOGI("main", "value: %f", val);
      if(i == 0){
        *batteryVoltage = val;
      }
    }
  }
}

bool read_sensor(SenseData_t *data){
  uint8_t address = 0;
  esp_err_t err;
  uint8_t attempts = 0;
  do {
    resetSDI12();
    uint8_t queryAttempts = 0;
    do {
      err = sdi12_queryAddress(&address);
      ESP_LOGD("main", "query address (%d): %d", err, address);
      vTaskDelay(500);
      queryAttempts++;
    } while(err != ESP_OK && queryAttempts < SENSOR_QUERY_ATTEMPTS);
    attempts++;
  } while (err != ESP_OK && attempts < SENSOR_MAX_ATTEMPTS);

  if(attempts >= SENSOR_MAX_ATTEMPTS){
    return true;
  }

  err = sdi12_acknowledgeActive(address);
  ESP_LOGI("main", "acknowledge active: (%d)", err);

  SDI12Identification_t *id = NULL;
  err = sdi12_sendIdentification(address, &id);
  ESP_LOGI("main", "send identification: (%d)", err);
  if(id != NULL){
    ESP_LOGI("main", "[ID] SDI Version: %s", id->sdiVersion);
    ESP_LOGI("main", "[ID] Vendor ID: %s", id->vendorId);
    ESP_LOGI("main", "[ID] Model Number: %s", id->modelNumber);
    ESP_LOGI("main", "[ID] Sensor Version: %s", id->sensorVersion);
    ESP_LOGI("main", "[ID] Optional: %s", id->optional);
  }
  free(id);
  data->reportCount = count;
  read_water_level_and_temperature(address, &data->waterLevel, &data->sensorTemperature);
  read_pcb_temperature(address, &data->pcbTemperature);
  read_battery_voltage(address, &data->batteryVoltage);
  count++;
  return false;
}
