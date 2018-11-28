#ifndef __SDI12_H
#define __SDI12_H

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

typedef struct {
  char sdiVersion[4];    // two bytes indicating sdi12 version. e.g. 1.4 encoded as "14"
  char vendorId[9];      // 8 bytes indicating vendor identification
  char modelNumber[7];   // 6 bytes indicating sensor model number
  char sensorVersion[4]; // 3 bytes indicating sensor version
  char optional[14];     // upto 13 bytes of optional data (e.g. serial number)
} SDI12Identification_t;

typedef struct {
  char polarity;
  char digits[9]; // max of 7 digits + decimal point + null terminator
} SDI12MeasurementValue_t;

typedef struct {
  uint8_t count;                    // the number of measurement values taken
  SDI12MeasurementValue_t *values;  // array of values
} SDI12Measurement_t;

typedef struct {
  char *command;                    // excluding address and ! terminator
  size_t commandLen;                // strlen of command
  char *response;                   // buffer where response will be placed
  size_t bufferSize;                // size of provided buffer
  size_t responseLength;            // length of data returned
} SDI12ExtendedCommand_t;

typedef enum {
  SDI12_ACKNOWLEDGE_ACTIVE,
  SDI12_QUERY_ADDRESS,
  SDI12_SEND_IDENTIFICATION,
  SDI12_CHANGE_ADDRESS,
  SDI12_MEASUREMENT,
  // SDI12_CONTINUOUS_MEASUREMENT,
  SDI12_EXTENDED_COMMAND,
  SDI12_CUSTOM_COMMAND,
} sdi12_command_t;

#define SDI12_FLAGS_CONCURRENT  (1<<0)
#define SDI12_FLAGS_REQUEST_CRC (1<<1)

typedef struct {
  uint8_t address;
  uint8_t command;
  uint8_t arg;
  uint32_t flags;
  QueueHandle_t responseQueue;
  union {
    SDI12Identification_t *identification;
    SDI12Measurement_t *measurements;
    SDI12ExtendedCommand_t *extendedCommand;
    uint8_t deviceAddress;  // only for address query or change address
    // void *customCommandHook;  // function pointer to execute a custom comm sequence
  };
} SDI12Transaction_t;

// typedef void (*sdi12_custom_command_hook)(char *line, size_t size);

// typedef void (*sdi12_callback)(char *line, size_t size);
// typedef void (*sdi12_device_callback)(uint8_t address, char *response, size_t size);

// initialize sdi12 bus
esp_err_t sdi12_init(void);
esp_err_t sdi12_deinit(void);

esp_err_t sdi12_execute(SDI12Transaction_t *transaction);

esp_err_t sdi12_acknowledgeActive(uint8_t address);
esp_err_t sdi12_queryAddress(uint8_t *address); // only use this when a single sensor is attached
esp_err_t sdi12_sendIdentification(uint8_t address, SDI12Identification_t **identification);
esp_err_t sdi12_changeAddress(uint8_t currentAddress, uint8_t newAddress);

esp_err_t sdi12_startMeasurement(uint8_t address, bool concurrent, bool requestCrc, SDI12Measurement_t **measurement);
esp_err_t sdi12_startAdditionalMeasurement(uint8_t address, uint8_t additionalIndex, bool concurrent, bool requestCrc, SDI12Measurement_t **measurement);

// bool sdi12_continuousMeasurement(uint8_t address, uint8_t index, bool requestCrc, SDI12Measurement_t **measurement);

// send command excluding ! terminator
// provide a response buffer with enough storage for expected output
// size of response (excluding CRLF) will be returned
esp_err_t sdi12_extendedCommand(uint8_t address, char *command, size_t commandLength, char *response, size_t bufferSize, size_t *responseLength);

// esp_err_t sdi12_customCommand(uint8_t address, uint8_t arg, uint32_t flags )

// utils
SDI12Measurement_t *sdi12_create_measurement(uint16_t count);
void sdi12_cleanup_measurement(SDI12Measurement_t *measurement);

#endif //__SDI12_H
