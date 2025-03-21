#include "ma735_enc.h"
#include <cstring>

#include "main.h"
#include "param.h"
#include "can_communication.h"

// main.cppと同じインスタンスを使用
extern SPI_HandleTypeDef hspi2;

MA735Enc ma735enc(hspi2);
extern CanCom cancom;

MA735Enc::MA735Enc(SPI_HandleTypeDef& spiHandle)
  : hspi2(spiHandle), readStart(false), actAngle(0.0f), spi_rx_complete(false), spi_tx_complete(false),
    diffRaw(0), data(std::make_unique<MA735Data>()) {}

bool MA735Enc::ma735Init(){
  static uint8_t count = 0;
  getAngle();
  zeroPosOffset();
  count++;
  if (count > 10) return true;
  return false;
};

void MA735Enc::ma735angle(){
  getAngle();
  getVel();
  elecAngleIn();
  mechAngleIn();
};

void MA735Enc::read() {
  if (!readStart) {
    uint16_t command = 0x0000;
    
    // CSアクティブLow
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
    HAL_SPI_TransmitReceive_DMA(&hspi2, (uint8_t*)&command, (uint8_t*)rawEnc, 2);
    readStart = true;
  }
}

void MA735Enc::getAngle() {
  read();

  if (spi_rx_complete) {
    // CS非アクティブ
    HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);
    
    data->rawAngPast = data->rawAng;
    // 16ビットデータを受信し、後ろ4ビットを削除して12ビットに変換
    data->rawAng = (static_cast<uint16_t>(rawEnc[1] << 8) | rawEnc[0]) >> 4;

    // test
    data->rawAngtest = data->rawAng;
    // test
    
    readStart = false;
    spi_rx_complete = false;
    // 速度時間割りのためのカウンタ保存
    compTime = comp;
    comp = 0;

  } else ++comp;
  data->eleccomp = comp;
}

void MA735Enc::getVel() {
  if (comp) {
    // 更新なしのため速度更新不要
  } else {
    // 速度計算
    diff = static_cast<int16_t>(data->rawAng - data->rawAngPast);

    // 0またぎ判定処理
    if (diff > ANG_RESL_12BIT / 2) {
      diff -= ANG_RESL_12BIT;
      zeroPointTh = MULT_TURN_NEG;
    } else if (diff < -ANG_RESL_12BIT / 2) {
      diff += ANG_RESL_12BIT;
      zeroPointTh = MULT_TURN_POS;
    }
    data->mechAngVel = raw2rads(diff);
    mechAngleVelLPF();
  }
  data->testdiff = diff;
}

void MA735Enc::mechAngleVelLPF(){
  float timeConst, alpha;
  
  timeConst = 1.0f / (user2pi * lpfFreq);
  alpha = (TASK_TIME * static_cast<float>(compTime)) / timeConst;
  data->mechAngVelLPF = (alpha * data->mechAngVel + (1.0f - alpha) * data->mechAngVelLPF) * GR_RATIO;
}

float MA735Enc::elecAng(float _eofs) {
  // comp = 0のときサンプル値更新あり
  if (comp == 0) rawElecComp = data->rawAng;
  else rawElecComp = data->rawAng + (diff);
  
  // 電気角反転
  static uint16_t elecAngtemp_ = 0;
  if (elecAngDir > 0) {
    elecAngtemp_ = rawElecComp;
  }else elecAngtemp_ = ANG_RESL_12BIT - rawElecComp;

  // CWとCCWを切替
  static float ofs_ = 0.0f;
  if (rotDir > 0) {
    ofs_ = _eofs + user2pi;
  }else ofs_ = _eofs; // 極性反転不要

  static uint16_t offset_ = 0;
  static uint16_t elecAngtemp2_ = 0;
  
  offset_ = static_cast<uint16_t>(ofs_ * static_cast<float>(ANG_RESL_12BIT) / user2pi);
  elecAngtemp2_ = (elecAngtemp_ * POLE_PAIRS + offset_) % ANG_RESL_12BIT;

  return static_cast<float>(elecAngtemp2_) / static_cast<float>(ANG_RESL_12BIT) * user2pi;
}

float MA735Enc::elecAngVirtual(float _virFreqRef) {
  // 仮想電気角
  static float theta_ = 0.0f;
  // タスク周期で分割して足しこむ
  float _deltatheta = _virFreqRef * TASK_TIME;
  theta_ += _deltatheta;

  // 0~1の範囲に収める
  if (theta_ > 1.0f) theta_ -= 1.0f;
  if (theta_ < 0.0f) theta_ += 1.0f;

  // ラジアンで返却
  return theta_ * user2pi;
}

void MA735Enc::elecAngleIn(){
  CanCom::CanData* candata = cancom.getData();
  
  data->elecAngTest = elecAng(EOFS);
  if (candata->virAngFreq > 0.0f) {
    data->elecAng = elecAngVirtual(candata->virAngFreq);
  } else {
    data->elecAng = elecAng(EOFS);
  }
}

void MA735Enc::mechAngleIn() {
  // シングルターン値をラジアンに変換（0-2πの範囲）
  float tempMtAng_ = raw2rad(data->rawAng);
  
  // 一回転の判定とカウント更新
  if (zeroPointTh == MULT_TURN_NEG) {
    // 負の方向に回転
    mtCount--;
  } else if (zeroPointTh == MULT_TURN_POS) {
    // 正の方向に回転
    mtCount++;
  }
  
  // マルチターン分を加算（2πの倍数を加算）
  data->mechAng = (tempMtAng_ - data->zeroPosOffs + (static_cast<float>(mtCount) * user2pi)) * GR_RATIO;
  
  // マルチターンフラグリセット
  zeroPointTh = MULT_TURN_NONE;
}

void MA735Enc::zeroPosOffset() {
  data->zeroPosOffs = raw2rad(data->rawAng);
}

void MA735Enc::prepareCanData(uint8_t* buffer, size_t bufferSize) const {
  memcpy(buffer, &(data->elecAng), sizeof(data->elecAng));
}

void MA735Enc::spiTxRxCallback() {
  spi_rx_complete = true;
}

// SPIコールバック関数
void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
  if (hspi->Instance == SPI2) {
    ma735enc.spiTxRxCallback();
  }
}
