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
#include "byte_converter.h"

#include "main.h"
#include "mode_control.h"
#include "bldc_ctrl.h"
#include "ma735_enc.h"
#include "foc.h"
#include "util.h"
#include "elecang_calib.h"
#include "sens_cur.h"

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
extern SensCur senscur;

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

float CanCom::uintTofloat(const uint8_t* bytes) {
  return ByteConverter::readFloat(bytes, 0);
}

void CanCom::floatTouint(float value, uint8_t (&result)[4]) {
  ByteConverter::writeFloat(result, 0, value);
}

// 制御モード処理共通関数
void CanCom::processControlMode(uint16_t cmdRef, const uint8_t* rx) {
  switch (cmdRef) {
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
    case (CTRLMODE_VOLT):
      data->genFuncRef = ByteConverter::readUint32(rx, 0);
      data->virAngFreq = ByteConverter::readFloat(rx, 4);
      data->voltDRef = ByteConverter::readFloat(rx, 8);
      data->voltQRef = ByteConverter::readFloat(rx, 12);
      break;
    case (CTRLMODE_CUR):
      data->genFuncRef = ByteConverter::readUint32(rx, 0);
      data->curDRef = ByteConverter::readFloat(rx, 4);
      data->curQRef = ByteConverter::readFloat(rx, 8);
      break;
    case (CTRLMODE_VEL):
      data->genFuncRef = ByteConverter::readUint32(rx, 0);
      data->velRef = ByteConverter::readFloat(rx, 4);
      break;
    case (CTRLMODE_POS):
      data->genFuncRef = ByteConverter::readUint32(rx, 0);
      data->posRef = ByteConverter::readFloat(rx, 4);
      break;
  }
}

// genFuncRef状態更新共通関数
void CanCom::updateGenFuncStatus() {
  uint8_t currentGenFuncRef = data->genFuncRef;

  if (currentGenFuncRef == prevGenFuncRef) {
    data->genFuncCheck = false;   
    return;
  }
  data->genFuncCheck = true;
  prevGenFuncRef = currentGenFuncRef;
}

void CanCom::rxFifo0Callback(uint32_t RxFifo0ITs) {
  FDCAN_RxHeaderTypeDef tempRxHeader;

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
  }
}

void CanCom::rxTask() {
  rxMsglistFd(rxDataFd);
  updateGenFuncStatus();
}

void CanCom::TEST_rxTask(){
  // テスト用固定値設定
  data->cmdRef = CTRLMODE_VOLT;
  data->genFuncRef = 0x01;
  data->virAngFreq = 10.0f;
  data->voltDRef = 0.0f;
  data->voltQRef = 3.0f;
  
  updateGenFuncStatus();
}

void CanCom::txTask() {
  if (canTxFlag) {
    txMsgListFd(txDataFd);
    canTxFlag = false;
  }
}

void CanCom::txServoOffTask() {
  txMsgListFd(txDataFd);
}

void CanCom::txMsgListFd(uint8_t (&tx_)[canTxSize]) {
  static uint32_t count = 0;
  
  // データポインタキャッシュ（静的取得）
  static MA735Enc::MA735Data* angdata = ma735enc.getData();
  static Foc::FocData* focdata = foc.getData();
  static ModeControl::ModeControlData* mdctrldata = modecontrol.getData();
  static Util::UtilData* utildata = util.getUtilData();
  static ElecangCalib::ElecangCalibData* ecaldata = elecangcalib.getData();

  // ByteConverterクラスを使用した型安全なデータ変換
  ByteConverter::writeFloat(tx_, 0, mdctrldata->voltDRef);   // voltD Act
  ByteConverter::writeFloat(tx_, 4, mdctrldata->voltQRef);   // voltQ Act
  ByteConverter::writeFloat(tx_, 8, focdata->id);            // curD Act
  ByteConverter::writeFloat(tx_, 12, focdata->iq);           // curQ Act
  ByteConverter::writeFloat(tx_, 16, angdata->mechAngVelLPF); // vel Act

  // 電気角オフセットキャリブ終了時のみキャリブ値を送信
  if (utildata->endECalib == true) angdata->mechAng = ecaldata->elecAngOfs;
  ByteConverter::writeFloat(tx_, 20, angdata->mechAng);       // mechAng Act

  // 32Byte固定
  if (HAL_FDCAN_AddMessageToTxFifoQ(&hfdcan, &txHeader, tx_) != HAL_OK) {
    Error_Handler();
  }
  count++;
}
void CanCom::rxMsglistFd(const uint8_t (&rx)[canRxSize]) {
  if (canRxInterrupt == true) {
    data->cmdRef = rxHeader.Identifier >> 6;
    processControlMode(data->cmdRef, rx);
    canRxInterrupt = false;
    cancom.canTxFlag = true;
  }
}


// 未使用の関数（CANFD環境では不要）

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef* hfdcan, uint32_t RxFifo0ITs) {
      cancom.rxFifo0Callback(RxFifo0ITs);
}
