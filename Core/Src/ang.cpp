#include "ang.h"
#include <cstring>

#include "main.h"
#include "param.h"
#include "can_communication.h"

// main.cppと同じインスタンスを使用
extern I2C_HandleTypeDef hi2c1;

Ang ang(hi2c1);
extern CanCom cancom;

Ang::Ang(I2C_HandleTypeDef& i2cHandle)
  : hi2c1(i2cHandle), readStart(false), actAngle(0.0f), i2c_rx_complete(false), i2c_tx_complete(false),
    rawAng(0), rawAngPast(0), diffRaw(0), data(std::make_unique<AngData>()) {}

void Ang::read() {
  if (!readStart) {
    uint8_t readRegAddr = 0x0C;
    HAL_I2C_Master_Transmit_DMA(&hi2c1, AS5600ADDR, &readRegAddr, 1);
    readStart = true;
  }
}

void Ang::receive() {
  if (i2c_tx_complete) {
    HAL_I2C_Master_Receive_DMA(&hi2c1, AS5600ADDR, rawEnc, 2);
    i2c_tx_complete = false;
  }
}

void Ang::getAngle() {
  read();
  receive();

  if (i2c_rx_complete) {
    rawAngPast = rawAng;
    data->rawAngPasttest = rawAng;
    rawAng = static_cast<uint16_t>(rawEnc[0] << 8) | rawEnc[1];

    // test
    data->rawAngtest = rawAng;
    // test
    
    readStart = false;
    i2c_rx_complete = false;
    // 速度時間割りのためのカウンタ保存
    compTime = comp;
    comp = 0;

  } else ++comp;
  data->eleccomp = comp;

}

void Ang::getVel() {
  if (comp) {
    // 更新なしのため速度更新不要
  } else {
    // 速度計算
    if (rawAng != rawAngPast) {
      diff = static_cast<int16_t>(rawAng - rawAngPast);
    } else {
      diff = 0;
    }

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

void Ang::mechAngleVelLPF(){
  float timeConst, alpha;
  
  timeConst = 1.0f / (user2pi * lpfFreq);
  alpha = (TASK_TIME * static_cast<float>(compTime)) / timeConst;
  data->mechAngVelLPF = (alpha * data->mechAngVel + (1.0f - alpha) * data->mechAngVelLPF) * GR_RATIO;

}

float Ang::elecAng(float _eofs) {
  // comp = 0のときサンプル値更新あり
  // 次の更新まで2周期を補間するので3で割った値を足す
  if (comp == 0) rawElecComp = rawAng;
  else rawElecComp = rawAng + (diff / 3);
  
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

float Ang::elecAngVirtual(float _virFreqRef) {
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

void Ang::elecAngleIn(){
  CanCom::CanData* candata = cancom.getData();
  
  data->elecAngTest = elecAng(EOFS);
  if (candata->virAngFreq > 0.0f) {
    data->elecAng = elecAngVirtual(candata->virAngFreq);
  } else {
    data->elecAng = elecAng(EOFS);
  }
}

// 位置生値から機械角（減速比込み）を算出
// 電気角から算出したほうが初期応答が良くなりそうだが、
// エンコーダ値の更新が毎周期ではなく一定値補間が入るので、
// 位置制御の速度の収束が遅くなりそう。よって機械角から計算する。
void Ang::mechAngleIn() {
  // シングルターン値をラジアンに変換（0-2πの範囲）
  float tempMtAng_ = raw2rad(rawAng);
  
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

// ゼロ点設定（userTaskの初期化処理時に数回実行）
void Ang::zeroPosOffset() {
  data->zeroPosOffs = raw2rad(rawAng);
}

void Ang::prepareCanData(uint8_t* buffer, size_t bufferSize) const {
  memcpy(buffer, &(data->elecAng), sizeof(data->elecAng));
}

void Ang::i2cMasterTxCallback() {
  i2c_tx_complete = true;    
}

void Ang::i2cMasterRxCallback() {
  i2c_rx_complete = true;
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
  ang.i2cMasterTxCallback();
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
  ang.i2cMasterRxCallback();
}
