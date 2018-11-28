#include "gps.h"

#include "driver/uart.h"
#include "peripherals/sx1502.h"
#include "minmea/minmea.h"
#include <string.h>
#include "tools/WakeLock.h"
#include "esp_log.h"

#define GPS_UART_NUM              UART_NUM_2
#define GPS_UART_BAUD             9600
#define GPS_UART_BUF_SIZE         256
#define GPS_RX_FLOW_CTRL_THRESH   122
#define GPS_UART_TX_PIN           GPIO_NUM_26
#define GPS_UART_RX_PIN           GPIO_NUM_36
#define GPS_UART_RTS_PIN          -1
#define GPS_UART_CTS_PIN          -1

#define GPIO_GPS_EN       GPIO_EX_NUM_3
#define GPIO_GPS_NRESET   GPIO_EX_NUM_2
#define GPIO_GPS_EXTINT   GPIO_EX_NUM_1

#define GPS_FIX_BUFFER_COUNT 2

static uint8_t gpsIndex = GPS_FIX_BUFFER_COUNT;
static Position_t gpsFixes[GPS_FIX_BUFFER_COUNT];

static char *TAG = "[gps]";

void minmea_process_line(char *nmea, size_t size){
  struct minmea_sentence_gga frame;
  uint8_t index;
  nmea[size] = '\0';
  switch (minmea_sentence_id(nmea, false)) {
      case MINMEA_SENTENCE_GGA: {
        if (minmea_parse_gga(&frame, nmea)) {
          ESP_LOGV(TAG, "%s", nmea);
          if(frame.fix_quality > 0){
            index = (gpsIndex+1)%GPS_FIX_BUFFER_COUNT;
            gpsFixes[index].lat = minmea_tocoord(&frame.latitude);
            gpsFixes[index].lng = minmea_tocoord(&frame.longitude);
            gpsFixes[index].hdop = minmea_tofloat(&frame.hdop);
            gpsIndex = index;
            ESP_LOGI(TAG, "Got fix: %i", gpsIndex);
          }
        }
      } break;
      default: {
        // do nothing
      } break;
  }
}

bool gps_has_location_fix(void){
  return (gpsIndex != GPS_FIX_BUFFER_COUNT);
}

void gps_get_location_fix(Position_t *pos){
  if(gps_has_location_fix()){
    Position_t ref = gpsFixes[gpsIndex];
    pos->lat = ref.lat;
    pos->lng = ref.lng;
    pos->hdop = ref.hdop;
  }
}

void gps_task(void *arg){
  char data[GPS_UART_BUF_SIZE];
  char *ptr = data;
  uint32_t len = 0;
  int n = 0;

  uart_config_t config = {
    .baud_rate = GPS_UART_BAUD,
    .data_bits = UART_DATA_8_BITS,
    .parity = UART_PARITY_DISABLE,
    .stop_bits = UART_STOP_BITS_1,
    .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    .rx_flow_ctrl_thresh = GPS_RX_FLOW_CTRL_THRESH
  };
  uart_param_config(GPS_UART_NUM, &config);
  uart_set_pin(GPS_UART_NUM,
    GPS_UART_TX_PIN,
    GPS_UART_RX_PIN,
    GPS_UART_RTS_PIN,
    GPS_UART_CTS_PIN);
  uart_driver_install(GPS_UART_NUM,
      GPS_UART_BUF_SIZE*2,
      GPS_UART_BUF_SIZE*2,
      0, NULL, 0);
  uart_flush(GPS_UART_NUM);

  if(esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_UNDEFINED){
    wake_lock_acquire();
    // it takes a while before the gps is ready to receive messages
    // make sure to wait an appropriate amount of time before sending
    // the power configurations...?
    vTaskDelay(5000);
    gps_configure_powermode();
    wake_lock_release();
  }

  while(1){
    n = uart_read_bytes(GPS_UART_NUM, (uint8_t *)ptr, GPS_UART_BUF_SIZE-1, portMAX_DELAY);
    if(n > 0){
      ptr[n] = '\0'; // close buffer
      // printf("%s", data);
      len = strlen(data);
      // loop through data searching for each occurrence of '\n'
      char *next = NULL;
      ptr = data;
      while( (next = strchr(ptr, '\n')) != NULL ){
        minmea_process_line(ptr, (next-ptr));
        ptr = next+1;
      }
      if(ptr-data < len){
        // there are still characters in the buffer
        // place the remaining at the start of data
        memcpy(data, ptr, len-(ptr-data));
        ptr = data + (len-(ptr-data));
      } else {
        ptr = data;
      }
    }

  }
}

void gps_send_ubx_msg(uint8_t *buffer, uint16_t len){
  uint8_t preamble[2] = {0xB5,0x62};
  uint8_t chksum[2] = {0,0};

  // compute crc
  for(int i=0; i<len; i++){
    chksum[0] = chksum[0] + buffer[i];
    chksum[1] = chksum[1] + chksum[0];
  }

  uart_write_bytes(GPS_UART_NUM, (char *)preamble, 2);
  uart_write_bytes(GPS_UART_NUM, (char *)buffer, len);
  uart_write_bytes(GPS_UART_NUM, (char *)chksum, 2);
}

void gps_configure_powermode(void){
  const uint32_t extintSel = (0 << 4);      // bit 4
  const uint32_t extintWake = (1 << 5);     // bit 5 - force on when EXTINT high
  const uint32_t extintBackup = (1 << 6);   // bit 6 - force off when EXTINT low
  const uint32_t extintInactive = (1 << 7); // bit 7 - force back up in case EXTINT pin is inactive for longer than timeout
  const uint32_t limitPeakCurrent = (0b01 << 8);  // bit 8-9 - peak current is limited
  const uint32_t waitTimeFix = (0 << 10);   // wait for normal fix ok before starting on time
  const uint32_t updateRTC = (0 << 11);     // do not update rtc, only update during normal on time
  const uint32_t updateEPH = (0 << 12);     // do not udpate eph, only update during normal on time
  const uint32_t doNotEnterOff = (0 << 16); // disable this
  const uint32_t mode = (0b00 << 17);       // on/off operation

  const uint16_t payloadLength = 48;
  const uint32_t flags = (mode | doNotEnterOff | updateEPH | updateRTC | waitTimeFix | limitPeakCurrent | extintInactive | extintBackup | extintWake | extintSel);
  const uint8_t maxStartupState = 45;
  const uint32_t updatePeriod = 0;//10 * 1000;  // 10 seconds
  const uint32_t searchPeriod = 0;//10 * 1000;  // 10 seconds
  const uint32_t gridOffset = 0;
  const uint16_t onTime = 0;//30;   // 30 s
  const uint16_t minAcqTime = 0;//30; // 30 s
  const uint32_t extintInactivityTime = 5 * 60 * 1000; // force backup mode after 5 minutes
  const uint8_t cfgpm2[52] = {
    0x06, // CFG Class
    0x3B, // MSG ID
    (payloadLength >> 0) & 0xFF,
    (payloadLength >> 8) & 0xFF,
    0x02, // Message version
    0,
    maxStartupState, // max startup duration
    0,
    (flags >> 0) & 0xFF,
    (flags >> 8) & 0xFF,
    (flags >> 16) & 0xFF,
    (flags >> 24) & 0xFF,
    (updatePeriod >> 0) & 0xFF,
    (updatePeriod >> 8) & 0xFF,
    (updatePeriod >> 16) & 0xFF,
    (updatePeriod >> 24) & 0xFF,
    (searchPeriod >> 0) & 0xFF,
    (searchPeriod >> 8) & 0xFF,
    (searchPeriod >> 16) & 0xFF,
    (searchPeriod >> 24) & 0xFF,
    (gridOffset >> 0) & 0xFF,
    (gridOffset >> 8) & 0xFF,
    (gridOffset >> 16) & 0xFF,
    (gridOffset >> 24) & 0xFF,
    (onTime >> 0) & 0xFF,
    (onTime >> 8) & 0xFF,
    (minAcqTime >> 0) & 0xFF,
    (minAcqTime >> 8) & 0xFF,
    0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,
    (extintInactivityTime >> 0) & 0xFF,
    (extintInactivityTime >> 8) & 0xFF,
    (extintInactivityTime >> 16) & 0xFF,
    (extintInactivityTime >> 24) & 0xFF,
  };
  gps_send_ubx_msg((uint8_t *)cfgpm2, 52);
}

void gps_set_enabled(bool enabled){
  // control extint pin
  sx1502_set_direction(GPIO_GPS_EN, GPIO_EX_OUTPUT);
  sx1502_set_direction(GPIO_GPS_EXTINT, GPIO_EX_OUTPUT);
  if(enabled){
    ESP_LOGW(TAG, "setting gps extint high!!!");
    sx1502_set_level(GPIO_GPS_EXTINT, GPIO_EX_LEVEL_HIGH);
    sx1502_set_level(GPIO_GPS_EN, GPIO_EX_LEVEL_HIGH);
  } else {
    ESP_LOGW(TAG, "setting gps extint low!!!");
    sx1502_set_level(GPIO_GPS_EXTINT, GPIO_EX_LEVEL_LOW);
    sx1502_set_level(GPIO_GPS_EN, GPIO_EX_LEVEL_LOW);
  }
}
