#include "network/lora.h"

#include "assert.h"
#include "esp_log.h"
#include "string.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

#include "board.h"
#include "LoRaMac.h"
#include "radio.h"

#define LORA_TASK_STACK_SIZE    configMINIMAL_STACK_SIZE*8
#define LORA_MAX_RECV_HANDLERS  256

// ideally we should do a linked list for this to save memory
// but, in the interest of time... nah.. we have memory
// TODO: encapsulate this in an API
static lora_receive_handler _handlers[LORA_MAX_RECV_HANDLERS];

static LoRaNetworkSettings_t *_settings = NULL;
static TaskHandle_t _loraTask = NULL;

static QueueHandle_t _opResponseQueue = NULL;
static QueueHandle_t _sendQueue = NULL;
static QueueHandle_t _sendResultQueue = NULL;

#define LORA_OP_QUEUE_SIZE        2
#define LORA_SEND_QUEUE_SIZE      5
#define LORA_RESULT_QUEUE_SIZE    1

#define LORA_MAX_PACKET_SIZE      256

static char *TAG = "[network][lora]";

static RTC_DATA_ATTR LoRaStats_t _loraStats;
static RTC_DATA_ATTR LoRaPacketStats_t _packetStats;

typedef enum {
  LoRaMachineStateInit,
  LoRaMachineStateJoin,
  LoRaMachineStateProcess
} LoRaMachineStates_t;

typedef enum {
  LoRaOpTypeJoin,
  LoRaOpTypeTx
} LoRaOpType_t;

typedef struct {
  LoRaOpType_t type;
  LoRaMacEventInfoStatus_t status;
} LoRaOpResponse_t;

typedef struct {
  uint8_t confirmed;
  uint8_t port;
  uint8_t size;
  uint8_t *buffer;
} LoRaTx_t;

typedef struct {
  uint8_t port;
  uint8_t size;
  uint8_t buffer[LORA_MAX_PACKET_SIZE];
} LoRaRx_t;

void lora_state_machine(void *arg);

void default_lora_receive_handler(uint8_t port, uint8_t *buffer, uint8_t size){
  ESP_LOGI(TAG, "default lora receive handler: Received Packet for port:[%d] with size:[%d]", port, size);
}

void lora_network_init(LoRaNetworkSettings_t *settings, uint8_t priority){
  assert(settings); // cannot function without settings
  if(_loraTask == NULL){
    // initialize receive handlers
    for(int i=0; i < LORA_MAX_RECV_HANDLERS; i++){
      _handlers[i] = default_lora_receive_handler;
    }

    // intialize queues
    _opResponseQueue = xQueueCreate(LORA_OP_QUEUE_SIZE, sizeof(LoRaOpResponse_t));
    _sendResultQueue = xQueueCreate(LORA_RESULT_QUEUE_SIZE, sizeof(esp_err_t));
    _sendQueue = xQueueCreate(LORA_SEND_QUEUE_SIZE, sizeof(LoRaTx_t));

    // allocate memory
    _settings = malloc(sizeof(LoRaNetworkSettings_t));
    memcpy(_settings, settings, sizeof(LoRaNetworkSettings_t));
    xTaskCreate(&lora_state_machine, "LoRaNetwork", LORA_TASK_STACK_SIZE, (void *)_settings, priority, &_loraTask);
  } else {
    ESP_LOGW(TAG, "LoRa network already started");
  }
}

void lora_network_shutdown(void){
  // remove the task and deallocate memory
  if(_loraTask){
    vTaskDelete(_loraTask);
    _loraTask = NULL;

    vQueueDelete(_opResponseQueue);
    vQueueDelete(_sendQueue);
    vQueueDelete(_sendResultQueue);

    free(_settings);
    _settings = NULL;
  } else {
    ESP_LOGW(TAG, "LoRa network not started");
  }
}

bool lora_has_joined_network(void){
  MibRequestConfirm_t mibReq;
  mibReq.Type = MIB_NETWORK_JOINED;
  mibReq.Param.IsNetworkJoined = false;
  if(LoRaMacMibGetRequestConfirm( &mibReq ) != LORAMAC_STATUS_OK){
    return false;
  }
  return mibReq.Param.IsNetworkJoined;
}

esp_err_t lora_send(bool confirmed, uint8_t port, uint8_t *buffer, uint8_t size, TickType_t ticksToWait){
  assert(ticksToWait > pdMS_TO_TICKS(10000)); // "LoRa needs minimum time to operate"
  esp_err_t err = ESP_OK;
  LoRaTx_t transaction;
  // allocate a response queue for this transaction
  transaction.confirmed = (confirmed ? 1 : 0);
  transaction.port = port;
  transaction.buffer = buffer;
  transaction.size = size;
  // send the transaction to the tx queue
  if(xQueueSendToBack(_sendQueue, &transaction, ticksToWait) != pdTRUE){
    return ESP_ERR_TIMEOUT;
  }
  // wait for a response on the transaction queue
  if(xQueueReceive(_sendResultQueue, &err, ticksToWait) != pdTRUE){
    return ESP_ERR_TIMEOUT;
  }
  // return transaction result
  return err;
}

void lora_register_receive_handler(uint8_t port, lora_receive_handler handler){
  // should add locking here
  _handlers[port] = handler;
}

void lora_get_last_packet_stats(LoRaPacketStats_t *stats){
  if(stats){
    memcpy(stats, &_packetStats, sizeof(LoRaPacketStats_t));
  }
}

/******************************************************************************/
/************************* STATE MACHINE INTERNALS ****************************/
/******************************************************************************/

static uint8_t LoRaGetBatteryLevel( void )
{
    return 0;
}

/*!
 * \brief   MCPS-Confirm event function
 *
 * \param   [IN] mcpsConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void McpsConfirm( McpsConfirm_t *mcpsConfirm )
{
    ESP_LOGI(TAG, "Tx Confirm: %d", mcpsConfirm->Status);
    LoRaOpResponse_t op;
    op.type = LoRaOpTypeTx;
    op.status = mcpsConfirm->Status;

    if(mcpsConfirm->AckReceived){
      _loraStats.acksReceived++;
    }
    _loraStats.txTrials += mcpsConfirm->NbRetries;

    _packetStats.txTime = mcpsConfirm->Timestamp;
    _packetStats.txTimeOnAir = mcpsConfirm->TxTimeOnAir;
    _packetStats.txDatarate = mcpsConfirm->Datarate;
    _packetStats.txAttempts = mcpsConfirm->NbRetries;

    /*
     * Log the number of retries and whether an acknowledgement was received
     */
    ESP_LOGI(TAG, "[Tx] Number of trials: %d", mcpsConfirm->NbRetries);
    ESP_LOGI(TAG, "[Tx] Acknowledgment Received: %d", mcpsConfirm->AckReceived);

    xQueueSendToBack(_opResponseQueue, &op, portMAX_DELAY);
}

/*!
 * \brief   MCPS-Indication event function
 *
 * \param   [IN] mcpsIndication - Pointer to the indication structure,
 *               containing indication attributes.
 */
static void McpsIndication( McpsIndication_t *mcpsIndication )
{
    ESP_LOGI(TAG, "Receive status: %d", mcpsIndication->Status);
    if(mcpsIndication->Status == LORAMAC_EVENT_INFO_STATUS_OK){

      _packetStats.rxTime = mcpsIndication->Timestamp;
      _packetStats.rxTimeOnAir = mcpsIndication->RxTimeOnAir;
      _packetStats.rxDatarate = mcpsIndication->RxDatarate;
      _packetStats.rxDelay = mcpsIndication->RxDelay;
      _packetStats.rxRssi = mcpsIndication->Rssi;
      _packetStats.rxSnr = mcpsIndication->Snr;

      /*
       * Log the radio details
       */
      ESP_LOGI(TAG, "[Rx] RSSI: %d | SNR: %d", mcpsIndication->Rssi, mcpsIndication->Snr);
      if(mcpsIndication->RxData == true){
        // get the response handler for port
        lora_receive_handler handler = _handlers[mcpsIndication->Port];
        if(handler != NULL){
          handler(mcpsIndication->Port, mcpsIndication->Buffer, mcpsIndication->BufferSize);
        }
      }
    }
}

/*!
 * \brief   MLME-Confirm event function
 *
 * \param   [IN] mlmeConfirm - Pointer to the confirm structure,
 *               containing confirm attributes.
 */
static void MlmeConfirm( MlmeConfirm_t *mlmeConfirm )
{
    switch( mlmeConfirm->MlmeRequest )
    {
        case MLME_JOIN:
        {
          ESP_LOGI(TAG, "Join status: %d", mlmeConfirm->Status);
          LoRaOpResponse_t op;
          op.type = LoRaOpTypeJoin;
          op.status = mlmeConfirm->Status;
          xQueueSendToBack(_opResponseQueue, &op, portMAX_DELAY);
        }
        default:
            break;
    }
}

void lora_radio_init(LoRaNetworkSettings_t *settings){
  MibRequestConfirm_t mibReq;

  ESP_LOGI(TAG, "Configuring ADR: %d", settings->adrEnable);
  mibReq.Type = MIB_ADR;
  mibReq.Param.AdrEnable = settings->adrEnable;
  LoRaMacMibSetRequestConfirm( &mibReq );

  ESP_LOGI(TAG, "Configuring Public Network: %d", settings->publicNetwork);
  mibReq.Type = MIB_PUBLIC_NETWORK;
  mibReq.Param.EnablePublicNetwork = settings->publicNetwork;
  LoRaMacMibSetRequestConfirm( &mibReq );

  uint8_t mask[12];
  for(int i=0; i < 12; i++){
    if((settings->fsbMask >> i) & 0x01){
      mask[i] = 0xFF;
    } else {
      mask[i] = 0x00;
    }
  }

  ESP_LOGI(TAG, "Configuring Channel Masks: [%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x][%x]",
    mask[0],mask[1],mask[2],mask[3],mask[4],mask[5],
    mask[6],mask[7],mask[8],mask[9],mask[10],mask[11]);

  mibReq.Type = MIB_CHANNELS_MASK;
  mibReq.Param.ChannelsMask = (uint16_t *)mask;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_CHANNELS_DEFAULT_MASK;
  mibReq.Param.ChannelsMask = (uint16_t *)mask;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_CHANNELS_DATARATE;
  mibReq.Param.ChannelsDatarate = settings->defaultDataRate;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_CHANNELS_DEFAULT_DATARATE;
  mibReq.Param.ChannelsDefaultDatarate = settings->defaultDataRate;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_CHANNELS_TX_POWER;
  mibReq.Param.ChannelsTxPower = TX_POWER_20_DBM;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_CHANNELS_DEFAULT_TX_POWER;
  mibReq.Param.ChannelsDefaultTxPower = TX_POWER_20_DBM;
  LoRaMacMibSetRequestConfirm( &mibReq );
}

bool lora_perform_join_otaa(LoRaNetworkSettings_t *settings){
  ESP_LOGI(TAG, "Joining via OTAA...");

  MlmeReq_t mlmeReq;

  mlmeReq.Type = MLME_JOIN;

  mlmeReq.Req.Join.DevEui = settings->ota.devEUI;
  mlmeReq.Req.Join.AppEui = settings->ota.appEUI;
  mlmeReq.Req.Join.AppKey = settings->ota.appKey;
  mlmeReq.Req.Join.NbTrials = settings->maxTxAttempts;

  LoRaMacMlmeRequest( &mlmeReq );

  // wait for join operation response
  LoRaOpResponse_t op;
  do {
    ESP_LOGI(TAG, "Waiting for join response...");
    xQueueReceive(_opResponseQueue, &op, portMAX_DELAY);
    ESP_LOGI(TAG, "Received join response... %d", op.status);
  } while(op.type != LoRaOpTypeJoin);

  return (op.status == LORAMAC_EVENT_INFO_STATUS_OK);
}

void lora_perform_join_abp(LoRaNetworkSettings_t *settings){
  ESP_LOGI(TAG, "Joining via ABP...");

  MibRequestConfirm_t mibReq;

  mibReq.Type = MIB_NET_ID;
  mibReq.Param.NetID = settings->networkID;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_DEV_ADDR;
  mibReq.Param.DevAddr = settings->deviceID;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_NWK_SKEY;
  mibReq.Param.NwkSKey = settings->abp.networkSessionKey;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_APP_SKEY;
  mibReq.Param.AppSKey = settings->abp.appSessionKey;
  LoRaMacMibSetRequestConfirm( &mibReq );

  mibReq.Type = MIB_NETWORK_JOINED;
  mibReq.Param.IsNetworkJoined = true;
  LoRaMacMibSetRequestConfirm( &mibReq );
}

void lora_handle_tx_confirm(QueueHandle_t queue){
  LoRaOpResponse_t op;
  esp_err_t txResult = ESP_OK;
  if(xQueueReceive(queue, &op, portMAX_DELAY)){
    ESP_LOGI(TAG, "processing tx confirm event");
    if(op.type == LoRaOpTypeTx){
      switch(op.status){
        case LORAMAC_EVENT_INFO_STATUS_OK:
          txResult = ESP_OK;
          break;
        case LORAMAC_EVENT_INFO_STATUS_TX_TIMEOUT:
          txResult = ESP_ERR_TIMEOUT;
          break;
        case LORAMAC_EVENT_INFO_STATUS_TX_DR_PAYLOAD_SIZE_ERROR:
          txResult = ESP_ERR_INVALID_SIZE;
          break;
        default:
          txResult = ESP_FAIL;
          break;
      }
      xQueueSendToBack(_sendResultQueue, &txResult, portMAX_DELAY);
    }
  }
}

esp_err_t lora_handle_tx_request(LoRaTx_t txPacket, LoRaNetworkSettings_t *settings){
  esp_err_t err = ESP_OK;
  McpsReq_t mcpsReq;
  LoRaMacTxInfo_t txInfo;

  if( LoRaMacQueryTxPossible( txPacket.size, &txInfo ) == LORAMAC_STATUS_OK )
  {
    mcpsReq.Type = txPacket.confirmed ? MCPS_CONFIRMED : MCPS_UNCONFIRMED;
    if(txPacket.confirmed){
      _loraStats.acksRequired++;
      mcpsReq.Req.Confirmed.fPort = txPacket.port;
      mcpsReq.Req.Confirmed.fBuffer = txPacket.buffer;
      mcpsReq.Req.Confirmed.fBufferSize = txPacket.size;
      mcpsReq.Req.Confirmed.NbTrials = settings->maxTxAttempts;
      mcpsReq.Req.Confirmed.Datarate = settings->defaultDataRate;
    } else {
      mcpsReq.Req.Unconfirmed.fPort = txPacket.port;
      mcpsReq.Req.Unconfirmed.fBuffer = txPacket.buffer;
      mcpsReq.Req.Unconfirmed.fBufferSize = txPacket.size;
      mcpsReq.Req.Unconfirmed.Datarate = settings->defaultDataRate;
    }
    if( LoRaMacMcpsRequest( &mcpsReq ) != LORAMAC_STATUS_OK )
    {
        err = ESP_FAIL;
    }
  } else {
    // cannot send this packet
    err = ESP_ERR_INVALID_ARG;
  }
  return err;
}

void lora_get_stats(LoRaStats_t *stats){
  stats->acksRequired = _loraStats.acksRequired;
  stats->acksReceived = _loraStats.acksReceived;
  stats->txTrials = _loraStats.txTrials;
}

void lora_reset_stats(void){
  _loraStats.acksRequired = 0;
  _loraStats.acksReceived = 0;
  _loraStats.txTrials = 0;
}

void lora_state_machine(void *arg)
{
  LoRaMacPrimitives_t LoRaMacPrimitives;
  LoRaMacCallback_t LoRaMacCallbacks;
  LoRaNetworkSettings_t *settings = (LoRaNetworkSettings_t *)arg;

  ESP_LOGI(TAG, "Starting up lora network...");
  LoRaMacPrimitives.MacMcpsConfirm = McpsConfirm;
  LoRaMacPrimitives.MacMcpsIndication = McpsIndication;
  LoRaMacPrimitives.MacMlmeConfirm = MlmeConfirm;
  LoRaMacCallbacks.GetBatteryLevel = LoRaGetBatteryLevel;
  LoRaMacInitialization( &LoRaMacPrimitives, &LoRaMacCallbacks );

  LoRaMachineStates_t state = LoRaMachineStateInit;

  while( 1 )
  {
      switch( state )
      {
          case LoRaMachineStateInit:
          {
              lora_radio_init(settings);
              if(lora_has_joined_network()){
                ESP_LOGI(TAG, "LoRa Network is up and processing!");
                state = LoRaMachineStateProcess;
              } else {
                ESP_LOGI(TAG, "Joining network...");
                state = LoRaMachineStateJoin;
              }
              break;
          }
          case LoRaMachineStateJoin:
          {
              if(settings->otaa){
                if(lora_perform_join_otaa(settings) == false){
                  ESP_LOGW(TAG, "Join failed... trying again!");
                  // need to delay a join retry for the lora state machine
                  // a chance to clear out any pending operations
                  vTaskDelay(2000);
                  continue;
                } else {
                  ESP_LOGI(TAG, "Node has joined network!");
                }
              } else {
                lora_perform_join_abp(settings);
              }

              if(lora_has_joined_network()){
                ESP_LOGI(TAG, "LoRa Network is up and processing!");
                lora_reset_stats();
                state = LoRaMachineStateProcess;
              }
              break;
          }
          case LoRaMachineStateProcess:
          {
              LoRaTx_t txPacket;
              if(xQueueReceive(_sendQueue, &txPacket, portMAX_DELAY)){
                // handle transmit request
                ESP_LOGI(TAG, "processing tx request event");
                esp_err_t err = lora_handle_tx_request(txPacket, settings);
                if(err != ESP_OK){
                  // send response error
                  ESP_LOGE(TAG, "Send error...");
                  xQueueSendToBack(_sendResultQueue, &err, portMAX_DELAY);
                } else {
                  lora_handle_tx_confirm(_opResponseQueue);
                }
              }
              break;
          }
      }
  }
}
