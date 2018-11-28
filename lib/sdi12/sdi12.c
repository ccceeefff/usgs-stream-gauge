#include "driver/sdi12.h"

#include "driver/uart.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "string.h"

#define SDI12_UART_NUM              UART_NUM_1
#define SDI12_UART_BAUD             4800
#define SDI12_UART_BUF_SIZE         256
#define SDI12_RX_FLOW_CTRL_THRESH   122
#define SDI12_UART_TX_PIN           GPIO_NUM_25
#define SDI12_UART_RX_PIN           GPIO_NUM_14
#define SDI12_UART_RTS_PIN          -1
#define SDI12_UART_CTS_PIN          -1

#define SDI12_UART_TASK_PRIORITY    12
#define SDI12_UART_TASK_STACK_SIZE  4096
#define SDI12_UART_TASK_NAME        "SDI12_uart"

#define SDI12_QUEUE_SIZE            10

#define TAG   "SDI12"
#define CRLF  "\r\n"

typedef struct {
  QueueHandle_t transactionQueue;
  TaskHandle_t taskHandle;
  uart_port_t uartNum;
} sdi12_bus_config_t;

static sdi12_bus_config_t SDI12_BUS;

static int sdi12_send(sdi12_bus_config_t *bus, uint8_t *bytes, uint16_t len){
  uart_flush(bus->uartNum);
  return uart_write_bytes(bus->uartNum, (const char *)bytes, len);
}

static esp_err_t sdi12_readline(sdi12_bus_config_t *bus, uint8_t *buffer, size_t bufferSize, uint16_t timeout, size_t *bytesRead){
  // read a response up to <CR><LF>
  size_t len = 0;
  memset(buffer, '\0', bufferSize);
  while(len < bufferSize){
    // get one character at a time
    int s = uart_read_bytes(bus->uartNum, &buffer[len], 1, pdMS_TO_TICKS(timeout));
    if(s > 0){
      len++;
    } else {
      ESP_LOGE(TAG, "read timeout!");
      return ESP_ERR_TIMEOUT;
    }
    if(len >= 3){ // if at least 3 characters have been fetched, check if it contains <CR><LF>
      if(strstr((const char *)buffer, CRLF) != NULL){
        // break has been found, let's move on
        break;
      }
    }
  }
  // check that we did not overflow...
  if(len < bufferSize){
    buffer[len-2] = '\0';
    *bytesRead = len-2;
    ESP_LOGI(TAG, "read(%d): %s", *bytesRead, buffer);
    return ESP_OK;
  } else {
    ESP_LOGD(TAG, "buffer too small!");
    return ESP_ERR_INVALID_SIZE;
  }
}

static esp_err_t sdi12_execute_acknowledge_active(sdi12_bus_config_t *bus, SDI12Transaction_t *transaction){
  uint8_t buffer[8];
  size_t bytesRead;
  snprintf((char *)buffer, 8, "%d!\r\n", transaction->address);
  // send a!
  sdi12_send(bus, buffer, 4);
  // expect to receive a<CR><LF>
  esp_err_t err = sdi12_readline(bus, buffer, 8, 1000, &bytesRead);
  // check for correct response
  if(err == ESP_OK){
    if(bytesRead == 1 && (buffer[0] - '0') == transaction->address){
      // received acknowledgement
      return ESP_OK;
    } else {
      return ESP_ERR_INVALID_RESPONSE;
    }
  }
  return err;
}

static esp_err_t sdi12_execute_query_address(sdi12_bus_config_t *bus, SDI12Transaction_t *transaction){
  uint8_t buffer[8];
  size_t bytesRead;
  snprintf((char *)buffer, 8, "?!\r\n");
  // send ?!
  sdi12_send(bus, buffer, 4);
  // expect to receive a<CR><LF>
  esp_err_t err = sdi12_readline(bus, buffer, 8, 1000, &bytesRead);
  // check for correct response
  if(err == ESP_OK){
    if(bytesRead == 1){
      transaction->deviceAddress = (buffer[0] - '0');
      // received acknowledgement
      return ESP_OK;
    } else {
      return ESP_ERR_INVALID_RESPONSE;
    }
  }
  return err;
}

static esp_err_t sdi12_execute_send_identification(sdi12_bus_config_t *bus, SDI12Transaction_t *transaction){
  uint8_t buffer[40];
  size_t bytesRead;
  snprintf((char *)buffer, 40, "%dI!\r\n", transaction->address);
  // send aI!
  sdi12_send(bus, buffer, 5);
  // expect an identification string
  esp_err_t err = sdi12_readline(bus, buffer, 40, 1000, &bytesRead);
  // check for correct response
  transaction->identification = NULL;
  if(err == ESP_OK){
    if(bytesRead >= 20 && bytesRead < 34 && (buffer[0] - '0') == transaction->address){
      // valid response, allocate an identification block and parse
      transaction->identification = (SDI12Identification_t *)malloc(sizeof(SDI12Identification_t));

      snprintf(transaction->identification->sdiVersion, 4, "%c.%c", buffer[1], buffer[2]);
      snprintf(transaction->identification->vendorId, 9, "%s", &buffer[3]);
      snprintf(transaction->identification->modelNumber, 7, "%s", &buffer[11]);
      snprintf(transaction->identification->sensorVersion, 4, "%s", &buffer[17]);
      if(bytesRead > 20){
        // read optional bytes
        int i;
        for(i=20; i < bytesRead; i++){
          transaction->identification->optional[i-20] = buffer[i];
        }
        transaction->identification->optional[i-20] = '\0';
      }
      return ESP_OK;
    } else {
      return ESP_ERR_INVALID_RESPONSE;
    }
  }
  return err;
}

static esp_err_t sdi12_execute_change_address(sdi12_bus_config_t *bus, SDI12Transaction_t *transaction){
  uint8_t buffer[8];
  size_t bytesRead;
  snprintf((char *)buffer, 8, "%dA%d!\r\n", transaction->address, transaction->arg);
  // send aAb!
  sdi12_send(bus, buffer, 6);
  // expect to receive b<CR><LF>
  esp_err_t err = sdi12_readline(bus, buffer, 8, 1000, &bytesRead);
  // check for correct response
  if(err == ESP_OK){
    if(bytesRead == 1 && (buffer[0] - '0') == transaction->arg){
      // received acknowledgement
      return ESP_OK;
    } else {
      return ESP_ERR_INVALID_RESPONSE;
    }
  }
  return err;
}

static esp_err_t sdi12_execute_extended_command(sdi12_bus_config_t *bus, SDI12Transaction_t *transaction){
  uint8_t buffer[100];
  size_t bytesRead;
  snprintf((char *)buffer, 100, "%d%s!\r\n", transaction->address, transaction->extendedCommand->command);
  // send aAb!
  sdi12_send(bus, buffer, transaction->extendedCommand->commandLen + 4);  // plus address, terminator and CRLF
  esp_err_t err = sdi12_readline(bus, buffer, 100, 1000, &bytesRead);
  // check for correct response
  if(err == ESP_OK){
    if(bytesRead > 0 && (buffer[0] - '0') == transaction->address){
      // received acknowledgement
      snprintf(transaction->extendedCommand->response, transaction->extendedCommand->bufferSize, "%s", &buffer[1]);
      transaction->extendedCommand->responseLength = bytesRead-1;
      return ESP_OK;
    } else {
      return ESP_ERR_INVALID_RESPONSE;
    }
  }
  return err;
}

static esp_err_t sdi12_execute_measurement(sdi12_bus_config_t *bus, SDI12Transaction_t *transaction){
  uint8_t buffer[100];
  size_t bytesRead;

  ESP_LOGD("sdi12", "execute measurement: address(%d) arg(%d) flags(%d)", transaction->address, transaction->arg, transaction->flags);

  // send "a[MC][C][index]!<CR><LF>"
  int index=0;
  // set device address
  buffer[index] = transaction->address + '0';
  index++;

  // concurrent or single
  if(transaction->flags & SDI12_FLAGS_CONCURRENT){
    buffer[index] = 'C';
  } else {
    buffer[index] = 'M';
  }
  index++;

  // request for CRC?
  if(transaction->flags & SDI12_FLAGS_REQUEST_CRC){
    buffer[index]= 'C';
    index++;
  }

  // set index, if requested
  if(transaction->arg > 0){
    buffer[index] = transaction->arg + '0';
    index++;
  }

  // add terminator and CRLF
  buffer[index++] = '!';
  buffer[index++] = '\r';
  buffer[index++] = '\n';

  // send based on measurement type
  sdi12_send(bus, buffer, index);

  // receive a delay response [atttnn<CR><LF>]
  esp_err_t err = sdi12_readline(bus, buffer, 100, 1000, &bytesRead);
  if(err != ESP_OK){
    return err;
  }

  if(transaction->address != (buffer[0] - '0') || bytesRead < 5){
    return ESP_ERR_INVALID_RESPONSE;
  }

  uint16_t delay = (100 * (buffer[1] - '0')) +
                    (10 * (buffer[2] - '0')) +
                     (1 * (buffer[3] - '0'));
  uint16_t count = atoi((const char *)&buffer[4]);

  // pause for 'delay' amount or wait for service request
  err = sdi12_readline(bus, buffer, 100, delay*1000, &bytesRead);

  // create measurement storage
  transaction->measurements = sdi12_create_measurement(count);

  // fetch 'count' measurements
  uint8_t dataRequests = 0;
  for(int i=0; i < count; ){
    // send a data request
    snprintf((char *)buffer, 100, "%dD%d!\r\n", transaction->address, dataRequests);
    sdi12_send(bus, buffer, 6);

    // wait for a response
    err = sdi12_readline(bus, buffer, 100, 1000, &bytesRead);
    if(err != ESP_OK){
      return err;
    }

    if(transaction->address != (buffer[0] - '0') && bytesRead > 1){
      return ESP_ERR_INVALID_RESPONSE;
    }

    if(transaction->flags & SDI12_FLAGS_REQUEST_CRC){
      //TODO: ignore CRC for now
      bytesRead = bytesRead-3;
    }

    // parse values
    int digitIndex = 0;
    for(int bufferIndex=1; bufferIndex < bytesRead; bufferIndex++){
      if(buffer[bufferIndex] == '+' || buffer[bufferIndex] == '-'){
        // move to next index and capture polarity
        i++;
        digitIndex=0;
        transaction->measurements->values[i-1].polarity = buffer[bufferIndex];
        // clear out digit storage
        memset(transaction->measurements->values[i-1].digits, '\0', 9);
      } else {
        transaction->measurements->values[i-1].digits[digitIndex] = buffer[bufferIndex];
        digitIndex++;
      }
    }
  }
  return ESP_OK;
}

// static esp_err_t sdi12_execute_continuous_measurement(sdi12_bus_config_t bus, SDI12Transaction_t *transaction){
//
// }

static void sdi12_task(void *arg){
  sdi12_bus_config_t *bus = (sdi12_bus_config_t *)arg;
  SDI12Transaction_t *transaction;
  esp_err_t result;
  while(1){
    result = ESP_OK;
    // wait for incoming transactions
    if(xQueueReceive(bus->transactionQueue, &transaction, portMAX_DELAY)){
      // determine type of transaction to perform
      switch(transaction->command){
        case SDI12_ACKNOWLEDGE_ACTIVE:
          result = sdi12_execute_acknowledge_active(bus, transaction);
          break;
        case SDI12_QUERY_ADDRESS:
          result = sdi12_execute_query_address(bus, transaction);
          break;
        case SDI12_SEND_IDENTIFICATION:
          result = sdi12_execute_send_identification(bus, transaction);
          break;
        case SDI12_CHANGE_ADDRESS:
          result = sdi12_execute_change_address(bus, transaction);
          break;
        case SDI12_MEASUREMENT:
          result = sdi12_execute_measurement(bus, transaction);
          break;
        // case SDI12_CONTINUOUS_MEASUREMENT:
        //   break;
        case SDI12_EXTENDED_COMMAND:
          result = sdi12_execute_extended_command(bus, transaction);
          break;
        default:
          // unknown command!!
          result = ESP_ERR_NOT_SUPPORTED;
          break;
      }

      // send back response
      xQueueSendToBack(transaction->responseQueue, &result, portMAX_DELAY);
    }
  }
}

esp_err_t sdi12_execute(SDI12Transaction_t *transaction){
  esp_err_t err = ESP_OK;
  // create a response queue and set it
  transaction->responseQueue = xQueueCreate(1, sizeof(esp_err_t));
  // send it to the transaction queue
  xQueueSendToBack(SDI12_BUS.transactionQueue, &transaction, portMAX_DELAY);
  // wait for a response on the response queue
  xQueueReceive(transaction->responseQueue, &err, portMAX_DELAY);
  // delete the response queue
  vQueueDelete(transaction->responseQueue);
  // return the transaction result
  return err;
}

esp_err_t sdi12_acknowledgeActive(uint8_t address){
  SDI12Transaction_t trans;
  trans.command = SDI12_ACKNOWLEDGE_ACTIVE;
  trans.address = address;
  esp_err_t err = sdi12_execute(&trans);
  return err;
}

esp_err_t sdi12_queryAddress(uint8_t *address){
  SDI12Transaction_t trans;
  trans.command = SDI12_QUERY_ADDRESS;
  esp_err_t err = sdi12_execute(&trans);
  *address = trans.deviceAddress;
  return err;
}

esp_err_t sdi12_sendIdentification(uint8_t address, SDI12Identification_t **identification){
  SDI12Transaction_t trans;
  trans.command = SDI12_SEND_IDENTIFICATION;
  trans.address = address;
  esp_err_t err = sdi12_execute(&trans);
  *identification = trans.identification;
  return err;
}

esp_err_t sdi12_changeAddress(uint8_t currentAddress, uint8_t newAddress){
  SDI12Transaction_t trans;
  trans.command = SDI12_CHANGE_ADDRESS;
  trans.address = currentAddress;
  trans.arg = newAddress;
  esp_err_t err = sdi12_execute(&trans);
  return err;
}

esp_err_t sdi12_startMeasurement(uint8_t address, bool concurrent, bool requestCrc, SDI12Measurement_t **measurement){
  return sdi12_startAdditionalMeasurement(address, 0, concurrent, requestCrc, measurement);
}

esp_err_t sdi12_startAdditionalMeasurement(uint8_t address, uint8_t additionalIndex, bool concurrent, bool requestCrc, SDI12Measurement_t **measurement){
  SDI12Transaction_t trans;
  trans.command = SDI12_MEASUREMENT;
  trans.address = address;
  trans.arg = additionalIndex;
  trans.flags = 0;
  if(concurrent){
    trans.flags |= SDI12_FLAGS_CONCURRENT;
  }
  if(requestCrc){
    trans.flags |= SDI12_FLAGS_REQUEST_CRC;
  }
  ESP_LOGI("sdi12", "start measurement: address(%d) arg(%d) flags(%d)", trans.address, trans.arg, trans.flags);
  esp_err_t err = sdi12_execute(&trans);
  *measurement = trans.measurements;
  return err;
}

esp_err_t sdi12_extendedCommand(uint8_t address, char *command, size_t commandLength, char *response, size_t bufferSize, size_t *responseLength){
  SDI12ExtendedCommand_t extCommand;
  extCommand.command = command;
  extCommand.commandLen = commandLength;
  extCommand.response = response;
  extCommand.bufferSize = bufferSize;
  SDI12Transaction_t trans;
  trans.command = SDI12_EXTENDED_COMMAND;
  trans.address = address;
  trans.extendedCommand = &extCommand;
  esp_err_t err = sdi12_execute(&trans);
  *responseLength = extCommand.responseLength;
  return err;
}

esp_err_t sdi12_init(void){
  SDI12_BUS.uartNum = SDI12_UART_NUM;
  SDI12_BUS.transactionQueue = xQueueCreate(10, sizeof(SDI12Transaction_t *));

  uart_config_t config = {
    .baud_rate = SDI12_UART_BAUD,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = SDI12_RX_FLOW_CTRL_THRESH
  };
  uart_param_config(SDI12_BUS.uartNum, &config);
  uart_set_pin(SDI12_BUS.uartNum,
    SDI12_UART_TX_PIN,
    SDI12_UART_RX_PIN,
    SDI12_UART_RTS_PIN,
    SDI12_UART_CTS_PIN);
  uart_driver_install(SDI12_BUS.uartNum,
      SDI12_UART_BUF_SIZE*2,
      SDI12_UART_BUF_SIZE*2,
      0, NULL, 0);
  uart_flush(SDI12_BUS.uartNum);
  xTaskCreate(sdi12_task, SDI12_UART_TASK_NAME, SDI12_UART_TASK_STACK_SIZE, (void *)&SDI12_BUS, SDI12_UART_TASK_PRIORITY, &SDI12_BUS.taskHandle);
  return ESP_OK;
}

esp_err_t sdi12_deinit(void){
  uart_flush(SDI12_BUS.uartNum);
  esp_err_t err = uart_driver_delete(SDI12_BUS.uartNum);
  vTaskDelete(SDI12_BUS.taskHandle);
  vQueueDelete(SDI12_BUS.transactionQueue);
  return err;
}

SDI12Measurement_t *sdi12_create_measurement(uint16_t count){
  SDI12Measurement_t *m = (SDI12Measurement_t *)malloc(sizeof(SDI12Measurement_t));
  m->count = count;
  m->values = (SDI12MeasurementValue_t *)malloc(sizeof(SDI12MeasurementValue_t) * count);
  return m;
}

void sdi12_cleanup_measurement(SDI12Measurement_t *measurement){
  free(measurement->values);
  free(measurement);
}
