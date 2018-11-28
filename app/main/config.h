#ifndef __CONFIG_H
#define __CONFIG_H

#define LORA_JOIN_OTAA    1
#define LORA_JOIN_ABP     0

// set this field to select which join method to use
// #define LORA_JOIN_METHOD  LORA_JOIN_OTAA
#define LORA_JOIN_METHOD  LORA_JOIN_ABP

#define DEVICE_NAME       "DeviceName-1"

// LoRa OTAA Keys
#define DEVICE_EUI        {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define APP_EUI           {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define APP_KEY           {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

// LoRa ABP Keys
#define NETWORK_SESSION_KEY     {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}
#define APPLICATION_SESSION_KEY {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}

#define DEBUG             0

#define LORA_FSB          1

#define GPS_SLEEP_TIME_S  120

#define SENSOR_MAX_ATTEMPTS       10
#define SENSOR_QUERY_ATTEMPTS     5
#define GPS_QUERY_ATTEMPTS        5

#define LORA_MAX_TX_ATTEMPTS      8

#define DEFAULT_SAMPLE_INTERVAL 15 * 60ll           // 15 minutes
// technically the minimum is 1 minute, but since it also takes time to
// start up the sensor, query its values and send over lora
// we need to be able to set the sleep interval much lower to account for that
#define MINIMUM_SAMPLE_INTERVAL 15ll                // 15 seconds
#define MAXIMUM_SAMPLE_INTERVAL (60 * 60 * 12ll)    // half a day

#endif  //__CONFIG_H
