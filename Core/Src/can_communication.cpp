/*
 *  can_communication.cpp
 *  =================================================
 *  can id を読み込んで各モード切替や返信要求先を決定する
 *  CANID(11bit)
 *  ---------------------------------------------------
 *  |         0x000000          |   00000   |
 *  | Cotrol Cmd & Return Level | Device ID |
 *  ---------------------------------------------------
 *  Control Cmd & Return Level : 5bit(0~31)
 *  Device ID : 6bit(0~63)
 *  優先度は アラーム返信 > ワーニング返信 > 受信 > データ返信
 *  となるように若い順に割り当てる.
 *  =================================================
 */

#include "can_communication.h"

#include "main.h"
#include "mode_control.h"
#include "bldc_ctrl.h"
#include "ma735_enc.h"
#include "foc.h"
#include "util.h"
#include "elecang_calib.h"

// main.cppと同じインスタンスを使用
extern FDCAN_HandleTypeDef hfdcan1;

// ユーザーインスタンス
CanCom cancom(hfdcan1);
extern MA735Enc ma735enc;
extern Foc foc;
extern ModeControl modecontrol;
extern BldcCtrl bldcctrl;
extern ElecangCalib elecangcalib;
extern Util util;

CanCom::CanCom(FDCAN_HandleTypeDef& fdcanHandle)
  : hfdcan(fdcanHandle), canRxInterrupt(0), prevGenFuncRef(0), canTxFlag(0),
    data(std::make_unique<CanData>()) {}

void CanCom::initTxHeader(bool extendedId, bool fdFormat) {
  txHeader.Identifier = 0x14 << 6 | canDevID;  // デバイスIDのみを設定
  txHeader.IdType = extendedId ? FDCAN_EXTENDED_ID : FDCAN_STANDARD_ID;
  txHeader.TxFrameType = FDCAN_DATA_FRAME;
  txHeader.DataLength = FDCAN_DLC_BYTES_32;
  txHeader.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
  txHeader.BitRateSwitch = FDCAN_BRS_ON;
  txHeader.FDFormat = FDCAN_FD_CAN;
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
  filter.FilterID2 = 0x01F;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);

  // filter 2 global用
  filter.FilterIndex = 1;
  filter.FilterID1 = 0x000;
  filter.FilterID2 = 0x01F;
  HAL_FDCAN_ConfigFilter(&hfdcan1, &filter);
  
  HAL_FDCAN_ConfigGlobalFilter(&hfdcan1, FDCAN_REJECT, FDCAN_REJECT, FDCAN_FILTER_REMOTE, FDCAN_FILTER_REMOTE);
  
}

void CanCom::floatTouint(float value, uint8_t (&result)[4]) {
  union {
      float f;
      uint32_t i;
  } converter;

  converter.f = value;
  result[0] = static_cast<uint8_t>(converter.i >> 24);
  result[1] = static_cast<uint8_t>(converter.i >> 16);
  result[2] = static_cast<uint8_t>(converter.i >> 8);
  result[3] = static_cast<uint8_t>(converter.i);
}

void CanCom::txMsgListFd(uint8_t (&tx_)[canTxSize]) {
  static uint32_t count = 0;
  static float testdata = 48.5f;
  
  MA735Enc::MA735Data* angdata = ma735enc.getData();
  Foc::FocData* focdata = foc.getData();
  ModeControl::ModeControlData* mdctrldata = modecontrol.getData();
  BldcCtrl::BldcCtrlData* bldcdata = bldcctrl.getData();
  Util::UtilData* utildata = util.getUtilData();
  ElecangCalib::ElecangCalibData* ecaldata = elecangcalib.getData();

  
  uint8_t bytes[4];

  // voltD Act
  floatTouint(mdctrldata->voltDRef, bytes);
  tx_[0] = bytes[0];
  tx_[1] = bytes[1];
  tx_[2] = bytes[2];
  tx_[3] = bytes[3];

  // voltQ Act
  floatTouint(mdctrldata->voltQRef, bytes);
  tx_[4] = bytes[0];
  tx_[5] = bytes[1];
  tx_[6] = bytes[2];
  tx_[7] = bytes[3];
  
  // curD Act
  floatTouint(focdata->id, bytes);
  tx_[8] = bytes[0];
  tx_[9] = bytes[1];
  tx_[10] = bytes[2];
  tx_[11] = bytes[3];

  // curQ Act
  floatTouint(focdata->iq, bytes);
  tx_[12] = bytes[0];
  tx_[13] = bytes[1];
  tx_[14] = bytes[2];
  tx_[15] = bytes[3];
  
  // vel Act
  floatTouint(static_cast<float>(angdata->rawAng), bytes);
  tx_[16] = bytes[0];
  tx_[17] = bytes[1];
  tx_[18] = bytes[2];
  tx_[19] = bytes[3];

  // 電気角オフセットキャリブ終了時のみキャリブ値を送信
  // サーボオフでフラグクリア
  if (utildata->endECalib == true) angdata->mechAng = ecaldata->elecAngOfs;

  // mechAng Act
  floatTouint(angdata->mechAng, bytes);
  tx_[20] = bytes[0];
  tx_[21] = bytes[1];
  tx_[22] = bytes[2];
  tx_[23] = bytes[3];

  // 32Byte固定
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &txHeader, tx_) != HAL_OK) {
    Error_Handler();
  }
  count++;

}

void CanCom::txMsgList(const uint8_t* data) {

}

void CanCom::rxFifo0Callback(uint32_t RxFifo0ITs) {
  FDCAN_RxHeaderTypeDef tempRxHeader;  // 一時的なヘッダー

  // FDCAN用
  //if (rxHeader.FDFormat == FDCAN_FD_CAN) {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
      if (HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &tempRxHeader, rxDataFd) != HAL_OK) {
        Error_Handler();
      }

      if (HAL_FDCAN_ActivateNotification(&hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
      }

      // 受信データのIDを保存
      rxHeader = tempRxHeader;
      canRxInterrupt = true;

   // }
  // クラシックCAN用
/*   } else {
    if ((RxFifo0ITs & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) != RESET) {
      if (HAL_FDCAN_GetRxMessage(&hfdcan, FDCAN_RX_FIFO0, &rxHeader, rxData) != HAL_OK) {
        Error_Handler();
      }

      if (HAL_FDCAN_ActivateNotification(&hfdcan, FDCAN_IT_RX_FIFO0_NEW_MESSAGE, 0) != HAL_OK) {
        Error_Handler();
      }

      canRxInterrupt = true;

    } */
  }
}

void CanCom::rxTask() {
  //if (txHeader.FDFormat == FDCAN_FD_CAN) {
    rxMsglistFd(rxDataFd);
/*   } else {
    rxMsglist(rxData);
  } */

  uint8_t currentGenFuncRef = data->genFuncRef;

  if (currentGenFuncRef == prevGenFuncRef) {
    data->genFuncCheck = false;   
    return;
  }
  data->genFuncCheck = true;
  prevGenFuncRef = currentGenFuncRef;
}

void CanCom::TEST_rxTask(){
  data->cmdRef = CTRLMODE_VOLT;
  switch (data->cmdRef) {
    // volt control
    case (CTRLMODE_VOLT):
      data->genFuncRef = 0x01;
      data->virAngFreq = 10.0f;
      data->voltDRef = 0.0f;
      data->voltQRef = 1.0f;
      break;
    // current control
    case (CTRLMODE_CUR):
      data->genFuncRef = 1;
      data->curDRef = 0.0f;
      data->curQRef = 0.5f;
      break;
    // velocity control
    case (CTRLMODE_VEL):
      data->genFuncRef = 1;
      data->velRef = 5.0f;
      break;
    // position control
    case (CTRLMODE_POS):
      data->genFuncRef = 1;
      data->posRef = 1.0f;
      break;
  }
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
//    if (txHeader.FDFormat == FDCAN_FD_CAN) {
      txMsgListFd(txDataFd);
//    }else{
//      txMsgList(txData);
//    }
    canTxFlag = false;
  }
}

void CanCom::txServoOffTask() {
      txMsgListFd(txDataFd);
}


float CanCom::uintTofloat(const uint8_t* bytes) {
    union {
        float f;
        uint32_t i;
    } converter;
    
    converter.i = bytes[0] << 24 | bytes[1] << 16 | bytes[2] << 8 | bytes[3];
    
    return converter.f;
}

void CanCom::rxMsglistFd(const uint8_t (&rx)[canRxSize]) {
  if (canRxInterrupt == true) {
    data->cmdRef = rxHeader.Identifier >> 6;
    switch (data->cmdRef) {
      case (CTRLMODE_NONE):
        data->genFuncRef = 0;
        data->virAngFreq = 0;
        data->voltDRef = 0;
        data->voltQRef = 0;
        data->curDRef = 0;
        data->curQRef = 0;
        data->velRef = 0;
        data->posRef = 0;
        break;
      // volt control
      case (CTRLMODE_VOLT):
        data->genFuncRef = rx[0] << 24 | rx[1] << 16 | rx[2] << 8 | rx[3];
        data->virAngFreq = uintTofloat(&rx[4]);
        data->voltDRef = uintTofloat(&rx[8]);
        data->voltQRef = uintTofloat(&rx[12]);
        break;
      // current control
      case (CTRLMODE_CUR):
        data->genFuncRef = rx[0] << 24 | rx[1] << 16 | rx[2] << 8 | rx[3];
        data->curDRef = uintTofloat(&rx[4]);
        data->curQRef = uintTofloat(&rx[8]);
        break;
      // velocity control
      case (CTRLMODE_VEL):
        data->genFuncRef = rx[0] << 24 | rx[1] << 16 | rx[2] << 8 | rx[3];
        data->velRef = uintTofloat(&rx[4]);
        break;
      // position control
      case (CTRLMODE_POS):
        data->genFuncRef = rx[0] << 24 | rx[1] << 16 | rx[2] << 8 | rx[3];
        data->posRef = uintTofloat(&rx[4]);
        break;
    }
    canRxInterrupt = false;
    cancom.canTxFlag = true;
  }
}

void CanCom::rxMsglist(const uint8_t (&rx)[8]) {
  if (canRxInterrupt == true) {
    data->cmdRef = rxHeader.Identifier >> 6;
    switch (data->cmdRef) {
      // volt control
      case (CTRLMODE_VOLT):
        data->genFuncRef = rx[0];
        data->virAngFreq = static_cast<float>(rx[1]);
        data->voltDRef = static_cast<float>(rx[2]);
        data->voltQRef = static_cast<float>(rx[3]);
        break;
      // current control
      case (CTRLMODE_CUR):
        data->genFuncRef = rx[0];
        data->curDRef = static_cast<float>(rx[1]);
        data->curQRef = static_cast<float>(static_cast<int8_t>(rx[2]));
        break;
      // velocity control
      case (CTRLMODE_VEL):
        data->genFuncRef = rx[0];
        data->velRef = static_cast<float>(static_cast<int8_t>(rx[1]));
        break;
      // position control
      case (CTRLMODE_POS):
        data->genFuncRef = rx[0];
        data->posRef = static_cast<float>(static_cast<int8_t>(rx[1]));
        break;

    }
    canRxInterrupt = false;
  }
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
      cancom.rxFifo0Callback(RxFifo0ITs);
}
