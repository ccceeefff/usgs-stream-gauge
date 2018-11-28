#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"

#include <string.h>
#include <math.h>
#include "board.h"
#include "esp_log.h"

#include "LoRaMac.h"
#include "Commissioning.h"
#include "radio.h"

#include "peripherals/sx1502.h"
#include "board/i2c.h"
#include "board/spi.h"

#define LED_1                                       GPIO_NUM_13
#define LED_2                                       GPIO_NUM_22
#define LED_3                                       GPIO_NUM_21

Gpio_t Led1;
Gpio_t Led2;
Gpio_t Led3;

/*!
 * Defines the application data transmission duty cycle. 5s, value in [ms].
 */
#define APP_TX_DUTYCYCLE                            5000

/*!
 * Defines a random delay for application data transmission duty cycle. 1s,
 * value in [ms].
 */
#define APP_TX_DUTYCYCLE_RND                        1000

/*!
 * Default datarate
 */
#define LORAWAN_DEFAULT_DATARATE                    DR_0

/*!
 * LoRaWAN confirmed messages
 */
#define LORAWAN_CONFIRMED_MSG_ON                    true

/*!
 * LoRaWAN Adaptive Data Rate
 *
 * \remark Please note that when ADR is enabled the end-device should be static
 */
#define LORAWAN_ADR_ON                              1

#if defined( USE_BAND_868 )

#include "LoRaMacTest.h"

/*!
 * LoRaWAN ETSI duty cycle control enable/disable
 *
 * \remark Please note that ETSI mandates duty cycled transmissions. Use only for test purposes
 */
#define LORAWAN_DUTYCYCLE_ON                        true

#define USE_SEMTECH_DEFAULT_CHANNEL_LINEUP          1

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )

#define LC4                { 867100000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC5                { 867300000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC6                { 867500000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC7                { 867700000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC8                { 867900000, { ( ( DR_5 << 4 ) | DR_0 ) }, 0 }
#define LC9                { 868800000, { ( ( DR_7 << 4 ) | DR_7 ) }, 2 }
#define LC10               { 868300000, { ( ( DR_6 << 4 ) | DR_6 ) }, 1 }

#endif

#endif

#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )

// FSB 1
const uint16_t FSB_CHANNEL_MASK[] = {0x00FF, 0x0000, 0x0000, 0x0000, 0x0000, 0x0000};

#endif

/*!
 * LoRaWAN application port
 */
#define LORAWAN_APP_PORT                            2

/*!
 * User application data buffer size
 */
#if defined( USE_BAND_868 )

#define LORAWAN_APP_DATA_SIZE                       16

#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )

#define LORAWAN_APP_DATA_SIZE                       11

#endif

static uint8_t DevEui[] = LORAWAN_DEVICE_EUI;
static uint8_t AppEui[] = LORAWAN_APPLICATION_EUI;
static uint8_t AppKey[] = LORAWAN_APPLICATION_KEY;

#if( OVER_THE_AIR_ACTIVATION == 0 )

static uint8_t NwkSKey[] = LORAWAN_NWKSKEY;
static uint8_t AppSKey[] = LORAWAN_APPSKEY;

/*!
 * Device address
 */
static uint32_t DevAddr = LORAWAN_DEVICE_ADDRESS;

#endif

/*!
 * Application port
 */
static uint8_t AppPort = LORAWAN_APP_PORT;

/*!
 * User application data size
 */
static uint8_t AppDataSize = LORAWAN_APP_DATA_SIZE;

/*!
 * User application data buffer size
 */
#define LORAWAN_APP_DATA_MAX_SIZE                           64

/*!
 * User application data
 */
static uint8_t AppData[LORAWAN_APP_DATA_MAX_SIZE];

/*!
 * Indicates if the node is sending confirmed or unconfirmed messages
 */
static uint8_t IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;

/*!
 * Defines the application data transmission duty cycle
 */
static uint32_t TxDutyCycleTime;

/*!
 * Timer to handle the application data transmission duty cycle
 */
static TimerEvent_t TxNextPacketTimer;

/*!
 * Specifies the state of the application LED
 */
static bool AppLedStateOn = false;

/*!
 * Timer to handle the state of LED1
 */
static TimerEvent_t Led1Timer;

/*!
 * Timer to handle the state of LED2
 */
static TimerEvent_t Led2Timer;

/*!
 * Indicates if a new packet can be sent
 */
static bool NextTx = true;

/*!
 * Device states
 */
static enum eDeviceState
{
    DEVICE_STATE_INIT,
    DEVICE_STATE_JOIN,
    DEVICE_STATE_SEND,
    DEVICE_STATE_CYCLE,
    DEVICE_STATE_SLEEP
}DeviceState;

/*!
 * LoRaWAN compliance tests support data
 */
struct ComplianceTest_s
{
    bool Running;
    uint8_t State;
    bool IsTxConfirmed;
    uint8_t AppPort;
    uint8_t AppDataSize;
    uint8_t *AppDataBuffer;
    uint16_t DownLinkCounter;
    bool LinkCheck;
    uint8_t DemodMargin;
    uint8_t NbGateways;
}ComplianceTest;

/*!
 * \brief   Prepares the payload of the frame
 */
static void PrepareTxFrame( uint8_t port )
{
    switch( port )
    {
    case 2:
        {
#if defined( USE_BAND_868 )
            AppData[0] = 'h';
            AppData[1] = 'e';
            AppData[2] = 'l';
            AppData[3] = 'l';
            AppData[4] = 'o';
            AppData[5] = ' ';
            AppData[6] = 'w';
            AppData[7] = 'o';
            AppData[8] = 'r';
            AppData[9] = 'l';
            AppData[10] = 'd';
            AppData[11] = '!';
            AppData[12] = '\n';
            AppData[13] = '\0';
            AppData[14] = '\0';
            AppData[15] = '\0';
#elif defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
            AppData[0] = 'h';
            AppData[1] = 'e';
            AppData[2] = 'l';
            AppData[3] = 'l';
            AppData[4] = 'o';
            AppData[5] = ' ';
            AppData[6] = 'w';
            AppData[7] = 'o';
            AppData[8] = 'r';
            AppData[9] = 'l';
            AppData[10] = 'd';
#endif
        }
        break;
    case 224:
        if( ComplianceTest.LinkCheck == true )
        {
            ComplianceTest.LinkCheck = false;
            AppDataSize = 3;
            AppData[0] = 5;
            AppData[1] = ComplianceTest.DemodMargin;
            AppData[2] = ComplianceTest.NbGateways;
            ComplianceTest.State = 1;
        }
        else
        {
            switch( ComplianceTest.State )
            {
            case 4:
                ComplianceTest.State = 1;
                break;
            case 1:
                AppDataSize = 2;
                AppData[0] = ComplianceTest.DownLinkCounter >> 8;
                AppData[1] = ComplianceTest.DownLinkCounter;
                break;
            }
        }
        break;
    default:
        break;
    }
}

/*!
 * \brief   Prepares the payload of the frame
 *
 * \retval  [0: frame could be send, 1: error]
 */
static bool SendFrame( void )
{
    McpsReq_t mcpsReq;
    LoRaMacTxInfo_t txInfo;

    if( LoRaMacQueryTxPossible( AppDataSize, &txInfo ) != LORAMAC_STATUS_OK )
    {
        // Send empty frame in order to flush MAC commands
        mcpsReq.Type = MCPS_UNCONFIRMED;
        mcpsReq.Req.Unconfirmed.fBuffer = NULL;
        mcpsReq.Req.Unconfirmed.fBufferSize = 0;
        mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
    }
    else
    {
        if( IsTxConfirmed == false )
        {
            mcpsReq.Type = MCPS_UNCONFIRMED;
            mcpsReq.Req.Unconfirmed.fPort = AppPort;
            mcpsReq.Req.Unconfirmed.fBuffer = AppData;
            mcpsReq.Req.Unconfirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Unconfirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
        else
        {
            mcpsReq.Type = MCPS_CONFIRMED;
            mcpsReq.Req.Confirmed.fPort = AppPort;
            mcpsReq.Req.Confirmed.fBuffer = AppData;
            mcpsReq.Req.Confirmed.fBufferSize = AppDataSize;
            mcpsReq.Req.Confirmed.NbTrials = 8;
            mcpsReq.Req.Confirmed.Datarate = LORAWAN_DEFAULT_DATARATE;
        }
    }

    if( LoRaMacMcpsRequest( &mcpsReq ) == LORAMAC_STATUS_OK )
    {
        return false;
    }
    return true;
}

/*!
 * \brief Function executed on TxNextPacket Timeout event
 */
static void OnTxNextPacketTimerEvent( void )
{
    MibRequestConfirm_t mibReq;
    LoRaMacStatus_t status;

    TimerStop( &TxNextPacketTimer );

    mibReq.Type = MIB_NETWORK_JOINED;
    status = LoRaMacMibGetRequestConfirm( &mibReq );

    if( status == LORAMAC_STATUS_OK )
    {
        if( mibReq.Param.IsNetworkJoined == true )
        {
            DeviceState = DEVICE_STATE_SEND;
            NextTx = true;
        }
        else
        {
            DeviceState = DEVICE_STATE_JOIN;
        }
    }
}

/*!
 * \brief Function executed on Led 1 Timeout event
 */
static void OnLed1TimerEvent( void )
{
    TimerStop( &Led1Timer );
    // Switch LED 1 OFF
    // GpioWrite( &Led1, 1 );
}

/*!
 * \brief Function executed on Led 2 Timeout event
 */
static void OnLed2TimerEvent( void )
{
    TimerStop( &Led2Timer );
    // Switch LED 2 OFF
    // GpioWrite( &Led2, 1 );
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    ESP_LOGI("tx confirm", "status: %d", mcpsConfirm->Status);
    if( mcpsConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
    {
        switch( mcpsConfirm->McpsRequest )
        {
            case MCPS_UNCONFIRMED:
            {
                // Check Datarate
                ESP_LOGI("tx confirm", "Datarate: %d", mcpsConfirm->Datarate);
                // Check TxPower
                ESP_LOGI("tx confirm", "TxPower: %d", mcpsConfirm->TxPower);
                ESP_LOGI("tx confirm", "TxTimeOnAir: %d", mcpsConfirm->TxTimeOnAir);
                ESP_LOGI("tx confirm", "UpLinkCounter: %d", mcpsConfirm->UpLinkCounter);
                ESP_LOGI("tx confirm", "UpLinkFrequency: %d", mcpsConfirm->UpLinkFrequency);
                
                break;
            }
            case MCPS_CONFIRMED:
            {
                // Check Datarate
                ESP_LOGI("tx confirm", "Datarate: %d", mcpsConfirm->Datarate);
                // Check TxPower
                ESP_LOGI("tx confirm", "TxPower: %d", mcpsConfirm->TxPower);
                // Check AckReceived
                ESP_LOGI("tx confirm", "AckReceived: %d", mcpsConfirm->AckReceived);
                // Check NbTrials
                ESP_LOGI("tx confirm", "NbRetries: %d", mcpsConfirm->NbRetries);
                ESP_LOGI("tx confirm", "TxTimeOnAir: %d", mcpsConfirm->TxTimeOnAir);
                ESP_LOGI("tx confirm", "UpLinkCounter: %d", mcpsConfirm->UpLinkCounter);
                ESP_LOGI("tx confirm", "UpLinkFrequency: %d", mcpsConfirm->UpLinkFrequency);

                break;
            }
            case MCPS_PROPRIETARY:
            {
                break;
            }
            default:
                break;
        }

        // Switch LED 1 ON
        GpioWrite( &Led1, 0 );
        TimerStart( &Led1Timer );
    }
    NextTx = true;
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    if( mcpsIndication->Status != LORAMAC_EVENT_INFO_STATUS_OK )
    {
        return;
    }

    switch( mcpsIndication->McpsIndication )
    {
        case MCPS_UNCONFIRMED:
        {
            break;
        }
        case MCPS_CONFIRMED:
        {
            break;
        }
        case MCPS_PROPRIETARY:
        {
            break;
        }
        case MCPS_MULTICAST:
        {
            break;
        }
        default:
            break;
    }

    // Check Multicast
    // Check Port
    // Check Datarate
    // Check FramePending
    // Check Buffer
    // Check BufferSize
    // Check Rssi
    // Check Snr
    // Check RxSlot

    ESP_LOGI("rx confirm", "Multicast: %d", mcpsIndication->Multicast);
    ESP_LOGI("rx confirm", "Port: %d", mcpsIndication->Port);
    ESP_LOGI("rx confirm", "RxDatarate: %d", mcpsIndication->RxDatarate);
    ESP_LOGI("rx confirm", "Buffer: %p", mcpsIndication->Buffer);
    ESP_LOGI("rx confirm", "BufferSize: %d", mcpsIndication->BufferSize);
    if(mcpsIndication->Buffer != NULL && mcpsIndication->BufferSize > 0){
      printf("rxdata: 0x");
      for(int i=0; i < mcpsIndication->BufferSize; i++){
        printf("%02x", mcpsIndication->Buffer[i]);
      }
      printf("\n");
    }
    ESP_LOGI("rx confirm", "FramePending: %d", mcpsIndication->FramePending);
    ESP_LOGI("rx confirm", "RxData: %d", mcpsIndication->RxData);
    ESP_LOGI("rx confirm", "Rssi: %d", mcpsIndication->Rssi);
    ESP_LOGI("rx confirm", "Snr: %d", mcpsIndication->Snr);
    ESP_LOGI("rx confirm", "RxSlot: %d", mcpsIndication->RxSlot);
    ESP_LOGI("rx confirm", "AckReceived: %d", mcpsIndication->AckReceived);
    ESP_LOGI("rx confirm", "DownLinkCounter: %d", mcpsIndication->DownLinkCounter);

    if( ComplianceTest.Running == true )
    {
        ComplianceTest.DownLinkCounter++;
    }

    if( mcpsIndication->RxData == true )
    {
        switch( mcpsIndication->Port )
        {
        case 1: // The application LED can be controlled on port 1 or 2
        case 2:
            if( mcpsIndication->BufferSize == 1 )
            {
                AppLedStateOn = mcpsIndication->Buffer[0] & 0x01;
                // GpioWrite( &Led3, ( ( AppLedStateOn & 0x01 ) != 0 ) ? 0 : 1 );
            }
            break;
        case 224:
            if( ComplianceTest.Running == false )
            {
                // Check compliance test enable command (i)
                if( ( mcpsIndication->BufferSize == 4 ) &&
                    ( mcpsIndication->Buffer[0] == 0x01 ) &&
                    ( mcpsIndication->Buffer[1] == 0x01 ) &&
                    ( mcpsIndication->Buffer[2] == 0x01 ) &&
                    ( mcpsIndication->Buffer[3] == 0x01 ) )
                {
                    IsTxConfirmed = false;
                    AppPort = 224;
                    AppDataSize = 2;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.LinkCheck = false;
                    ComplianceTest.DemodMargin = 0;
                    ComplianceTest.NbGateways = 0;
                    ComplianceTest.Running = true;
                    ComplianceTest.State = 1;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = true;
                    LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( false );
#endif
                }
            }
            else
            {
                ComplianceTest.State = mcpsIndication->Buffer[0];
                switch( ComplianceTest.State )
                {
                case 0: // Check compliance test disable command (ii)
                    IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                    AppPort = LORAWAN_APP_PORT;
                    AppDataSize = LORAWAN_APP_DATA_SIZE;
                    ComplianceTest.DownLinkCounter = 0;
                    ComplianceTest.Running = false;

                    MibRequestConfirm_t mibReq;
                    mibReq.Type = MIB_ADR;
                    mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                    LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( USE_BAND_868 )
                    LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                    break;
                case 1: // (iii, iv)
                    AppDataSize = 2;
                    break;
                case 2: // Enable confirmed messages (v)
                    IsTxConfirmed = true;
                    ComplianceTest.State = 1;
                    break;
                case 3:  // Disable confirmed messages (vi)
                    IsTxConfirmed = false;
                    ComplianceTest.State = 1;
                    break;
                case 4: // (vii)
                    AppDataSize = mcpsIndication->BufferSize;

                    AppData[0] = 4;
                    for( uint8_t i = 1; i < AppDataSize; i++ )
                    {
                        AppData[i] = mcpsIndication->Buffer[i] + 1;
                    }
                    break;
                case 5: // (viii)
                    {
                        MlmeReq_t mlmeReq;
                        mlmeReq.Type = MLME_LINK_CHECK;
                        LoRaMacMlmeRequest( &mlmeReq );
                    }
                    break;
                case 6: // (ix)
                    {
                        MlmeReq_t mlmeReq;

                        // Disable TestMode and revert back to normal operation
                        IsTxConfirmed = LORAWAN_CONFIRMED_MSG_ON;
                        AppPort = LORAWAN_APP_PORT;
                        AppDataSize = LORAWAN_APP_DATA_SIZE;
                        ComplianceTest.DownLinkCounter = 0;
                        ComplianceTest.Running = false;

                        MibRequestConfirm_t mibReq;
                        mibReq.Type = MIB_ADR;
                        mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
                        LoRaMacMibSetRequestConfirm( &mibReq );
#if defined( USE_BAND_868 )
                        LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );
#endif
                        mlmeReq.Type = MLME_JOIN;

                        mlmeReq.Req.Join.DevEui = DevEui;
                        mlmeReq.Req.Join.AppEui = AppEui;
                        mlmeReq.Req.Join.AppKey = AppKey;
                        mlmeReq.Req.Join.NbTrials = 3;

                        LoRaMacMlmeRequest( &mlmeReq );
                        DeviceState = DEVICE_STATE_SLEEP;
                    }
                    break;
                case 7: // (x)
                    {
                        if( mcpsIndication->BufferSize == 3 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        else if( mcpsIndication->BufferSize == 7 )
                        {
                            MlmeReq_t mlmeReq;
                            mlmeReq.Type = MLME_TXCW_1;
                            mlmeReq.Req.TxCw.Timeout = ( uint16_t )( ( mcpsIndication->Buffer[1] << 8 ) | mcpsIndication->Buffer[2] );
                            mlmeReq.Req.TxCw.Frequency = ( uint32_t )( ( mcpsIndication->Buffer[3] << 16 ) | ( mcpsIndication->Buffer[4] << 8 ) | mcpsIndication->Buffer[5] ) * 100;
                            mlmeReq.Req.TxCw.Power = mcpsIndication->Buffer[6];
                            LoRaMacMlmeRequest( &mlmeReq );
                        }
                        ComplianceTest.State = 1;
                    }
                    break;
                default:
                    break;
                }
            }
            break;
        default:
            break;
        }
    }

    // Switch LED 2 ON for each received downlink
    GpioWrite( &Led2, 0 );
    TimerStart( &Led2Timer );
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
  ESP_LOGI("main", "confirm trigger");

    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Status is OK, node has joined the network
                DeviceState = DEVICE_STATE_SEND;
                ESP_LOGI("main", "node has joined network");
            }
            else
            {
                // Join was not successful. Try to join again
                DeviceState = DEVICE_STATE_JOIN;
                ESP_LOGI("main", "failed to join network. trying again...");
            }
            break;
        }
        case MLME_LINK_CHECK:
        {
            if( mlmeConfirm->Status == LORAMAC_EVENT_INFO_STATUS_OK )
            {
                // Check DemodMargin
                // Check NbGateways
                if( ComplianceTest.Running == true )
                {
                    ComplianceTest.LinkCheck = true;
                    ComplianceTest.DemodMargin = mlmeConfirm->DemodMargin;
                    ComplianceTest.NbGateways = mlmeConfirm->NbGateways;
                }
            }
            break;
        }
        default:
            break;
    }
    NextTx = true;
}

void app_main(void)
{
  // GpioInit( &Led1, LED_1, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
  // GpioInit( &Led2, LED_2, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
  // GpioInit( &Led3, LED_3, PIN_OUTPUT, PIN_PUSH_PULL, PIN_NO_PULL, 0 );
  
  board_i2c_init();
  board_spi_init();
  
  int gps_on_pin = 2;
  // query gpio expander
  int dir = sx1502_get_direction(gps_on_pin);
  ESP_LOGI("gpioex", "gps pin dir is %d", dir);
  sx1502_set_direction(gps_on_pin, 0);
  ESP_LOGI("gpioex", "setting gps pin dir to %d", 0);
  dir = sx1502_get_direction(gps_on_pin);
  ESP_LOGI("gpioex", "gps pin is %d", dir);
  
  // query pin level
  int level = sx1502_get_level(gps_on_pin);
  ESP_LOGI("gpioex", "gps pin level is %d", level);
  sx1502_set_level(gps_on_pin, 1);
  ESP_LOGI("gpioex", "setting gps pin level to %d", 1);
  level = sx1502_get_level(gps_on_pin);
  ESP_LOGI("gpioex", "gps pin level is %d", level);
  
  int ant_switch = 0;
  // query gpio expander
  dir = sx1502_get_direction(ant_switch);
  ESP_LOGI("ant_switch", "ant_switch pin dir is %d", dir);
  sx1502_set_direction(ant_switch, 0);
  ESP_LOGI("ant_switch", "setting ant_switch pin dir to %d", 0);
  dir = sx1502_get_direction(ant_switch);
  ESP_LOGI("ant_switch", "ant_switch pin is %d", dir);
  
  // query pin level
  level = sx1502_get_level(ant_switch);
  ESP_LOGI("ant_switch", "ant_switch pin level is %d", level);
  sx1502_set_level(ant_switch, 1);
  ESP_LOGI("ant_switch", "setting ant_switch pin level to %d", 1);
  level = sx1502_get_level(ant_switch);
  ESP_LOGI("ant_switch", "ant_switch pin level is %d", level);
  
  LoRaMacPrimitives_t LoRaMacPrimitives;
  LoRaMacCallback_t LoRaMacCallbacks;
  MibRequestConfirm_t mibReq;

  BoardInitMcu( );
  BoardInitPeriph( );

  DeviceState = DEVICE_STATE_INIT;

  while( 1 )
  {
      switch( DeviceState )
      {
          case DEVICE_STATE_INIT:
          {
              ESP_LOGI("main", "executing init");
              LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
              LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
              LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
              LoRaMacCallbacks.GetBatteryLevel = BoardGetBatteryLevel;
              LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks );

              TimerInit( &TxNextPacketTimer, OnTxNextPacketTimerEvent );
              
              TimerInit( &Led1Timer, OnLed1TimerEvent );
              TimerSetValue( &Led1Timer, 25 );
              
              TimerInit( &Led2Timer, OnLed2TimerEvent );
              TimerSetValue( &Led2Timer, 25 );

              mibReq.Type = MIB_ADR;
              mibReq.Param.AdrEnable = LORAWAN_ADR_ON;
              LoRaMacMibSetRequestConfirm( &mibReq );

              mibReq.Type = MIB_PUBLIC_NETWORK;
              mibReq.Param.EnablePublicNetwork = LORAWAN_PUBLIC_NETWORK;
              LoRaMacMibSetRequestConfirm( &mibReq );

#if defined( USE_BAND_868 )
              LoRaMacTestSetDutyCycleOn( LORAWAN_DUTYCYCLE_ON );

#if( USE_SEMTECH_DEFAULT_CHANNEL_LINEUP == 1 )
              LoRaMacChannelAdd( 3, ( ChannelParams_t )LC4 );
              LoRaMacChannelAdd( 4, ( ChannelParams_t )LC5 );
              LoRaMacChannelAdd( 5, ( ChannelParams_t )LC6 );
              LoRaMacChannelAdd( 6, ( ChannelParams_t )LC7 );
              LoRaMacChannelAdd( 7, ( ChannelParams_t )LC8 );
              LoRaMacChannelAdd( 8, ( ChannelParams_t )LC9 );
              LoRaMacChannelAdd( 9, ( ChannelParams_t )LC10 );

              mibReq.Type = MIB_RX2_DEFAULT_CHANNEL;
              mibReq.Param.Rx2DefaultChannel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
              LoRaMacMibSetRequestConfirm( &mibReq );

              mibReq.Type = MIB_RX2_CHANNEL;
              mibReq.Param.Rx2Channel = ( Rx2ChannelParams_t ){ 869525000, DR_3 };
              LoRaMacMibSetRequestConfirm( &mibReq );
#endif

#endif

#if defined( USE_BAND_915 ) || defined( USE_BAND_915_HYBRID )
              // set the proper channel mask
              // array of uint16_t representing masks for each valid channel
              // for BAND_915, these are divided into FSB 1-8, with 8 channels
              // each. For testing, we'll use the first 8 channels (FSB-1)
              mibReq.Type = MIB_CHANNELS_MASK;
              mibReq.Param.ChannelsMask = FSB_CHANNEL_MASK;
              if(LoRaMacMibSetRequestConfirm( &mibReq ) != LORAMAC_STATUS_OK){
                printf("Failed to set channel mask!");
                while(1);
              };
              
              mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
              mibReq.Param.ChannelsMask = FSB_CHANNEL_MASK;
              if(LoRaMacMibSetRequestConfirm( &mibReq ) != LORAMAC_STATUS_OK){
                printf("Failed to set default channel mask!");
                while(1);
              };
              
              /*
              mibReq.Type = MIB_CHANNELS_DATARATE;
              mibReq.Param.ChannelsDatarate = DR_1;
              if(LoRaMacMibSetRequestConfirm( &mibReq ) != LORAMAC_STATUS_OK){
                printf("Failed to set data rate!");
                while(1);
              };
              */
#endif

              DeviceState = DEVICE_STATE_JOIN;
              break;
          }
          case DEVICE_STATE_JOIN:
          {
              ESP_LOGI("main", "joining network...");
#if( OVER_THE_AIR_ACTIVATION != 0 )
              MlmeReq_t mlmeReq;

              // Initialize LoRaMac device unique ID
              // BoardGetUniqueId( DevEui );

              mlmeReq.Type = MLME_JOIN;

              mlmeReq.Req.Join.DevEui = DevEui;
              mlmeReq.Req.Join.AppEui = AppEui;
              mlmeReq.Req.Join.AppKey = AppKey;
              mlmeReq.Req.Join.NbTrials = 10;

              if( NextTx == true )
              {
                  LoRaMacMlmeRequest( &mlmeReq );
              }
              DeviceState = DEVICE_STATE_SLEEP;
#else
              // Choose a random device address if not already defined in Commissioning.h
              if( DevAddr == 0 )
              {
                  // Random seed initialization
                  srand1( BoardGetRandomSeed( ) );

                  // Choose a random device address
                  DevAddr = randr( 0, 0x01FFFFFF );
              }

              mibReq.Type = MIB_NET_ID;
              mibReq.Param.NetID = LORAWAN_NETWORK_ID;
              LoRaMacMibSetRequestConfirm( &mibReq );

              mibReq.Type = MIB_DEV_ADDR;
              mibReq.Param.DevAddr = DevAddr;
              LoRaMacMibSetRequestConfirm( &mibReq );

              mibReq.Type = MIB_NWK_SKEY;
              mibReq.Param.NwkSKey = NwkSKey;
              LoRaMacMibSetRequestConfirm( &mibReq );

              mibReq.Type = MIB_APP_SKEY;
              mibReq.Param.AppSKey = AppSKey;
              LoRaMacMibSetRequestConfirm( &mibReq );

              mibReq.Type = MIB_NETWORK_JOINED;
              mibReq.Param.IsNetworkJoined = true;
              LoRaMacMibSetRequestConfirm( &mibReq );

              DeviceState = DEVICE_STATE_SEND;
#endif
              break;
          }
          case DEVICE_STATE_SEND:
          {
              if( NextTx == true )
              {
                  PrepareTxFrame( AppPort );

                  NextTx = SendFrame( );
              }
              if( ComplianceTest.Running == true )
              {
                  // Schedule next packet transmission
                  TxDutyCycleTime = 5000; // 5000 ms
              }
              else
              {
                  // Schedule next packet transmission
                  TxDutyCycleTime = APP_TX_DUTYCYCLE + randr( -APP_TX_DUTYCYCLE_RND, APP_TX_DUTYCYCLE_RND );
              }
              DeviceState = DEVICE_STATE_CYCLE;
              break;
          }
          case DEVICE_STATE_CYCLE:
          {
              DeviceState = DEVICE_STATE_SLEEP;

              // Schedule next packet transmission
              TimerSetValue( &TxNextPacketTimer, TxDutyCycleTime );
              TimerStart( &TxNextPacketTimer );
              break;
          }
          case DEVICE_STATE_SLEEP:
          {
              // Wake up through events
              TimerLowPowerHandler( );
              break;
          }
          default:
          {
              DeviceState = DEVICE_STATE_INIT;
              break;
          }
      }
  }
}

