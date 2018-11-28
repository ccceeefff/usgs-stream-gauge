#include "sdlog.h"
#include "esp_system.h"
#include "rom/rtc.h"

#include "board.h"
#include "board/spi.h"
#include "driver/sdspi_host.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"
#include "esp_log.h"
#include "string.h"

#include <stdio.h>
#include <stdbool.h>
#include <ctype.h>
#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <sys/time.h>
#include "config.h"

#define DATA_WRITE_ATTEMPTS 5

static char *TAG = "[sdlog]";

void tm_to_utc_time(struct tm *t, char *buf){
  int year = t->tm_year + 1900;
  int month = t->tm_mon + 1;
  int day = t->tm_mday;
  int hour = t->tm_hour;
  int minute = t->tm_min;
  int second = t->tm_sec;
  sprintf(buf, "%d-%02d-%02dT%02d:%02d:%02d+00:00", year, month, day, hour, minute, second);
}

void sd_get_sensor_log_filename(struct tm *t, char *filename){
  int year = (t->tm_year + 1900)%100;
  int month = t->tm_mon + 1;
  int day = t->tm_mday;
  sprintf(filename, "/sdcard/d%02d%02d%02d.csv", year, month, day);
}

void sd_get_event_log_filename(struct tm *t, char *filename){
  int year = (t->tm_year + 1900)%100;
  int month = t->tm_mon + 1;
  int day = t->tm_mday;
  sprintf(filename, "/sdcard/l%02d%02d%02d.txt", year, month, day);
}

bool sd_log_sensor_data(uint32_t sampleTime, bool sensorReadError, SenseData_t *data){
  char buf[512];
  char timestr[30];
  char filename[80];

  time_t mtime = sampleTime;
  struct tm mtm;
  gmtime_r(&mtime, &mtm);
  tm_to_utc_time(&mtm, timestr);
  if(sensorReadError){
    sprintf(buf,"%s,,,ft,,C,,C,,V\n",
      timestr
    );
  } else {
    sprintf(buf,"%s,%d,%.3lf,ft,%.2lf,C,%.2lf,C,%.1lf,V\n",
      timestr,
      data->reportCount,
      data->waterLevel,
      data->sensorTemperature,
      data->pcbTemperature,
      data->batteryVoltage
    );
  }

  sd_get_sensor_log_filename(&mtm, filename);
  uint8_t writeAttempts = 0;
  bool writeSuccess = false;
  do {
    ESP_LOGI(TAG, "Log file write attempt: %d", writeAttempts);
    FILE* f = fopen(filename, "a+");
    if(f){
      if(fputs(buf, f) >= 0){
        writeSuccess = true;
      } else {
        ESP_LOGE(TAG, "Error writing file!");
      }
      fclose(f);
      ESP_LOGI(TAG, "log write complete");
    } else {
      ESP_LOGE(TAG, "failed to open file: %s for writing!!", filename);
    }
    writeAttempts++;
  } while(writeSuccess == false && writeAttempts < DATA_WRITE_ATTEMPTS);
  return writeSuccess;
}

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
                          LoRaStats_t *stats
                        ){
  char buf[256];
  char filename[80];
  char timestr[30];
  struct timeval now;
  gettimeofday(&now, NULL);
  time_t mtime = now.tv_sec;
  struct tm mtm;
  gmtime_r(&mtime, &mtm);
  tm_to_utc_time(&mtm, timestr);

  sd_get_event_log_filename(&mtm, filename);

  uint8_t deviceEUI[] = DEVICE_EUI;

  FILE* f = fopen(filename, "a+");
  if(f){
    tm_to_utc_time(&mtm, buf);
    fputs("========= ENTRY START ========\n", f);
    fprintf(f, "Device ID: %s (0x%02X.%02X)\n", DEVICE_NAME, deviceEUI[6], deviceEUI[7]);
    fprintf(f, "wake up cause: %d reset reason(%d, %d) \n", esp_sleep_get_wakeup_cause(), rtc_get_reset_reason(0), rtc_get_reset_reason(1));
    fprintf(f, "device time: %s\n", timestr);
    fprintf(f, "boot count: %d, sensor failure count(%d), lora failure count(%d)\n", bootCount, sensorFailureCount, loraFailureCount);
    fprintf(f, "sensor read error(%d), lora send error(%d), sd write error(%d)\n", sensorError, loraSendError, sdWriteError);
    fprintf(f, "lora stats: acksRequired(%d) acksReceived(%d) txTrials(%d)\n", stats->acksRequired, stats->acksReceived, stats->txTrials);
    fprintf(f, "current sampling interval: %d s\n", samplingInterval);
    if(hasGpsData){
      fprintf(f, "GPS Location: ( %.8f, %.8f ) HDOP(%.4f)\n", pos->lat, pos->lng, pos->hdop);
    }
    if(!sensorError){
        fprintf(f, "sensor data(%d): water level(%.3lf ft) sensor temp(%.2lf C) pcb temp(%.2lf C) battery(%.1lf V)\n",
          data->reportCount,
          data->waterLevel,
          data->sensorTemperature,
          data->pcbTemperature,
          data->batteryVoltage
        );
    } else {
      fputs("Failed to read sensor data\n", f);
    }
    fputs("========== ENTRY END =========\n\n", f);
    fclose(f);
    ESP_LOGI(TAG, "log write complete");
  } else {
    ESP_LOGE(TAG, "failed to open file: %s for writing!!", filename);
  }
}
