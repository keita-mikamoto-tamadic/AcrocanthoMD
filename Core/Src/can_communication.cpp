#include "can_communication.h"

#include "main.h"
#include "ang.h"

// グローバルなインスタンスを使用
extern CanCom cancom;
extern Ang ang;

CanCom::CanCom(FDCAN_HandleTypeDef& fdcanHandle)
  : hfdcan(fdcanHandle), canRxInterrupt(0), prevGenFuncRef(0), canTxFlag(0),
    data(std::make_unique<canData>()) {}

void CanCom::initTxHeader(uint32_t canId, bool extendedId, bool fdFormat) {
  txHeader.Identifier = canId;
  txHeader.IdType = extendedId ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_8;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = fdFormat ? FDCAN_BRS_ON : FDCAN_BRS_OFF;
  txHeader.FDFormat = fdFormat ? FDCAN_FD_CAN : FDCAN_CLASSIC_CAN;
  txHeader.TxEventFifoControl = FDCAN_NO_TX_EVENTS;
  txHeader.MessageMarker = 0;
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

void CanCom::handleRxData() {
  if (canRxInterrupt == true) {
    data->genFuncRef = rxData[0];
    canRxInterrupt = false;
    canTxFlag = true;
  }
}

void CanCom::rxTask() {
  handleRxData();
 
  uint8_t currentGenFuncRef = data->genFuncRef;

  if (currentGenFuncRef == prevGenFuncRef) {
    return;
  }

  if (data->genFuncRef == 0) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
  } else if (data->genFuncRef == 1) {
    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
  }
  prevGenFuncRef = currentGenFuncRef;
}

void CanCom::txTask(){
  if (canTxFlag) {
    ang.prepareCanData(data->txBuff, sizeof(data->txBuff));
    sendData(data->txBuff, sizeof(data->txBuff));
    canTxFlag = false;
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
      cancom.rxFifo0Callback(RxFifo0ITs);
}
