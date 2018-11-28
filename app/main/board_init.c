#include "board_init.h"

#include "freertos/FreeRTOS.h"

#include "esp_freertos_hooks.h"
#include "esp_sleep.h"
#include "esp_log.h"

#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/sdspi_host.h"
#include "esp_intr_alloc.h"
#include "esp_intr.h"
#include "esp_vfs.h"
#include "esp_vfs_fat.h"

#include "board.h"
#include "board/i2c.h"
#include "board/spi.h"
#include "board/lora.h"

#include "peripherals/sx1502.h"

#include "driver/sdi12.h"
#include "tools/WakeLock.h"
#include "network/lora.h"
#include "appconfig.h"

#include "config.h"

#define GPIO_SDI12_EN     GPIO_EX_NUM_0
#define GPIO_SDI12_POWER  GPIO_EX_NUM_6
#define GPIO_SDI12_NRESET GPIO_EX_NUM_7

#define GPIO_GPS_EN       GPIO_EX_NUM_3
#define GPIO_GPS_NRESET   GPIO_EX_NUM_2
#define GPIO_GPS_EXTINT   GPIO_EX_NUM_1

#define GPIO_SDCARD_EN    GPIO_EX_NUM_4

bool system_idle_hook(void);

void board_enable_peripherals(void){
  // turn on sd card
  // need to make sure that the SD card CS line is held high before
  // powering the card on.
  gpio_set_level(GPIO_NUM_22,1);
  gpio_config_t config = {
    .pin_bit_mask = (1 << GPIO_NUM_22),
    .mode = GPIO_MODE_INPUT_OUTPUT,
    .pull_up_en = GPIO_PULLUP_ENABLE,
    .pull_down_en = GPIO_PULLDOWN_DISABLE,
    .intr_type = GPIO_INTR_DISABLE
  };
  esp_err_t err = gpio_config(&config);

  sx1502_set_direction(GPIO_SDCARD_EN, GPIO_EX_OUTPUT);
  sx1502_set_level(GPIO_SDCARD_EN, GPIO_EX_LEVEL_HIGH);

  sx1502_set_direction(GPIO_SDI12_EN, GPIO_EX_OUTPUT);
  sx1502_set_level(GPIO_SDI12_EN, GPIO_EX_LEVEL_HIGH);

  sx1502_set_direction(GPIO_SDI12_POWER, GPIO_EX_OUTPUT);
  sx1502_set_level(GPIO_SDI12_POWER, GPIO_EX_LEVEL_HIGH);

  sdmmc_host_t host = SDSPI_HOST_DEFAULT();
  sdspi_slot_config_t slot_config = SDSPI_SLOT_CONFIG_DEFAULT();
  slot_config.gpio_miso = GPIO_NUM_19;
  slot_config.gpio_mosi = GPIO_NUM_27;
  slot_config.gpio_sck  = GPIO_NUM_5;
  slot_config.gpio_cs   = GPIO_NUM_22;
  esp_vfs_fat_sdmmc_mount_config_t mount_config = {
      .format_if_mount_failed = true,
      .max_files = 5
  };
  err = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, NULL);
  if(err != ESP_OK){
    ESP_LOGE("[board]", "sd card mount error");
  }
}

void board_disable_peripherals(void){
  sx1502_set_direction((gpio_ex_num_t)GPIO_GPS_NRESET, GPIO_EX_INPUT);
  sx1502_set_direction((gpio_ex_num_t)GPIO_SDI12_NRESET, GPIO_EX_INPUT);

  sx1502_set_direction(GPIO_SDCARD_EN, GPIO_EX_OUTPUT);
  sx1502_set_level(GPIO_SDCARD_EN, GPIO_EX_LEVEL_LOW);

  sx1502_set_direction(GPIO_SDI12_EN, GPIO_EX_OUTPUT);
  sx1502_set_level(GPIO_SDI12_EN, GPIO_EX_LEVEL_LOW);

  sx1502_set_direction(GPIO_SDI12_POWER, GPIO_EX_OUTPUT);
  sx1502_set_level(GPIO_SDI12_POWER, GPIO_EX_LEVEL_LOW);
}

void board_init(void){
  wake_lock_init();

  board_i2c_init();

  board_enable_peripherals();

  // this will fail because the spi bus has already been initialized by the sdspi host
  // but run it anyway so we generate the locks
  board_spi_init();
  sdi12_init();

  BoardInitMcu( );
  BoardInitPeriph( );

  LoRaNetworkSettings_t settings = {
    .otaa = LORA_JOIN_METHOD,
    .publicNetwork = 1,
    .adrEnable = 1,
    .defaultDataRate = 3,   // not currently used
    .defaultTxPower = 20,   // not currently used
    .maxTxAttempts = LORA_MAX_TX_ATTEMPTS,
    .fsbMask = (1<<(LORA_FSB-1)),      // fsb1
    .networkID = 0,
    .deviceID = 0,
#if (LORA_JOIN_METHOD == LORA_OTAA)
      .ota = {
        .devEUI = DEVICE_EUI,
        .appEUI = APP_EUI,
        .appKey = APP_KEY
      },
#else
      .abp = {
        .networkSessionKey = NETWORK_SESSION_KEY,
        .appSessionKey = APPLICATION_SESSION_KEY
      },
#endif
  };

  lora_network_init(&settings, 12);

  appconfig_init("beale-afb");

  // register idle hook on cpu 0
  esp_register_freertos_idle_hook_for_cpu(&system_idle_hook, 0);
}

void board_deinit(void){
  sdi12_deinit();
  board_disable_peripherals();
  appconfig_close();
  wake_lock_deinit();
}

uint32_t system_get_sleep_time() __attribute__((weak));
uint32_t system_get_sleep_time() {
  return 15; // 15 seconds
}

bool system_idle_hook(void){
  if(wake_lock_active()){
    return true;
  }
  // compute next wake up time
  uint32_t sleepTimeS = system_get_sleep_time();
  ESP_LOGW("[system]", "Board deinitialized. Going to sleep for %u seconds...", sleepTimeS);
  // printf("Enabling timer wakeup: %ds\n", sleepTimeS);
  esp_sleep_enable_timer_wakeup(sleepTimeS * 1000000);

  // tasks have released wake lock, we can go into deep sleep
  board_deinit();

  // go to sleep
  // struct timeval now;
  // gettimeofday(&now, NULL);
  // system_time.lastSleepTime = now.tv_sec;
  esp_deep_sleep_start();
  // will never reach this point
  return false;
}
