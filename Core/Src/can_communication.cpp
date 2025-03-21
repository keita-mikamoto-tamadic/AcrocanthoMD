#include "can_communication.h"

#include "main.h"
#include "ma735_enc.h"

// main.cppと同じインスタンスを使用
extern FDCAN_HandleTypeDef hfdcan1;

// ユーザーインスタンス
CanCom cancom(hfdcan1);
extern MA735Enc ma735enc;

CanCom::CanCom(FDCAN_HandleTypeDef& fdcanHandle)
  : hfdcan(fdcanHandle), canRxInterrupt(0), prevGenFuncRef(0), canTxFlag(0),
    data(std::make_unique<CanData>()) {}

void CanCom::initTxHeader(bool extendedId, bool fdFormat) {
  txHeader.Identifier = canDevID;
  txHeader.IdType = extendedId ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = fdFormat ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
  txHeader.FDFormat = fdFormat ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;
}

void CanCom::initFilter(void) {
  FDCAN_FilterTypeDef filter;

  filter.IdType = FDCAN_STANDARD_ID;
  filter.FilterIndex = 0;
  filter.FilterType = FDCAN_FILTER_MASK;
  filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
  filter.FilterID1 = canDevID;
  filter.FilterID2 = 0x00F;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

  // filter 2 global用
  filter.FilterIndex = 1;
  filter.FilterID1 = 0x000;
  filter.FilterID2 = 0x00F;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
  
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  
}

void CanCom::sendData(const uint8_t* data, size_t size) {
  if (size > 8) {
  // CANデータ長の上限は8バイト（クラシックCANの場合）
    Error_Handler();
    return;
  }

  uint8_t txData[8] = {0};
  for (size_t i = 0; i < size; ++i) {
    txData[i] = data[i];
  }

  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &txHeader, txData) != HAL_OK) {
    Error_Handler();
  }
}

void CanCom::rxFifo0Callback(uint32_t RxFifo0ITs) {
  if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
    if (HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
      Error_Handler();
    }

    if (HAL_FDCAN_ActivateNotification(&hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
      Error_Handler();
    }

    canRxInterrupt = true;

  }
}

void CanCom::rxTask() {
  rxMsglist(rxData);
  uint8_t currentGenFuncRef = data->genFuncRef;

  if (currentGenFuncRef == prevGenFuncRef) {
    data->genFuncCheck = false;   
    return;
  }
  data->genFuncCheck = true;
  prevGenFuncRef = currentGenFuncRef;
}

void CanCom::txTask(){
  if (canTxFlag) {
    ma735enc.prepareCanData(data->txBuff, sizeof(data->txBuff));
    sendData(data->txBuff, sizeof(data->txBuff));
    canTxFlag = false;
  }
}

void CanCom::rxMsglist(const uint8_t (&rx)[8]) {

  if (canRxInterrupt == true) {
    uint8_t funcbit = (rxHeader.Identifier >> 3);
    switch (funcbit) {
      // volt control
      case (0x200 >> 3):
        data->genFuncRef = rxData[0];
        data->drvMdRef = rxData[1];
        data->virAngFreq = static_cast<float>(rxData[2]);
        data->voltDRef = static_cast<float>(rxData[3]);
        data->voltQRef = static_cast<float>(rxData[4]);
        break;
      // current control
      case (0x300 >> 3):
        data->genFuncRef = rxData[0];
        data->drvMdRef = rxData[1];
        data->curDRef = static_cast<float>(rxData[2]);
        data->curQRef = static_cast<float>(static_cast<int8_t>(rxData[3]));
        break;
      // velocity control
      case (0x400 >> 3):
        data->genFuncRef = rxData[0];
        data->drvMdRef = rxData[1];
        data->velRef = static_cast<float>(static_cast<int8_t>(rxData[2]));
        break;
      // position control
      case (0x500 >> 3):
        data->genFuncRef = rxData[0];
        data->drvMdRef = rxData[1];
        data->posRef = static_cast<float>(static_cast<int8_t>(rxData[2]));
        //data->posRef = 3.0f;
        break;

    }
    canRxInterrupt = false;
    canTxFlag = true;
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
      cancom.rxFifo0Callback(RxFifo0ITs);
}
