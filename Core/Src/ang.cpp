#include "ang.h"
#include <cstring>

#include "main.h"
#include "param.h"
#include "can_communication.h"

extern Ang ang;
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
    rawAng = static_cast<uint16_t>(rawEnc[0] << 8) | rawEnc[1];
    mechAngPast = data->mechAng;
    data->mechAng = raw2rad(rawAng);
    
    readStart = false;
    i2c_rx_complete = false;
    // 速度時間割りのためのカウンタ保存
    compTime = comp;
    comp = 0;

  } else ++comp;

}

void Ang::getVel() {
  if (comp) {
    // 更新なしのため速度更新不要
  } else {
    // 速度計算
    diff = static_cast<int16_t>(rawAng - rawAngPast);

    if (diff > ANG_RESL_12BIT / 2) {
      diff -= ANG_RESL_12BIT;
    } else if (diff < -ANG_RESL_12BIT / 2) {
      diff += ANG_RESL_12BIT;
    }
    
    data->actVel = raw2rads(diff);
    mechAngleVelLPF();
  }
  data->veltemp = diff;
}

void Ang::mechAngleVelLPF(){
  float timeConst, alpha;
  
  timeConst = 1.0f / (user2pi * lpfFreq);
  alpha = (TASK_TIME * static_cast<float>(compTime)) / timeConst;
  data->actVelLPF = alpha * data->actVel + (1.0f - alpha) * data->actVelLPF;

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
  elecAngtemp2_ = (elecAngtemp_ * polePairs + offset_) % ANG_RESL_12BIT;

  return static_cast<float>(elecAngtemp2_) / static_cast<float>(ANG_RESL_12BIT) * user2pi;

}

float Ang::elecAngVirtual(float _virFreqRef) {
  // 仮想電気角
  static float _theta = 0.0f;
  // タスク周期で分割して足しこむ
  float _deltatheta = _virFreqRef * TASK_TIME;
  _theta += _deltatheta;

  // 0~1の範囲に収める
  if (_theta > 1.0f) _theta -= 1.0f;
  if (_theta < 0.0f) _theta += 1.0f;

  // ラジアンで返却
  return _theta * user2pi;
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

int16_t Ang::compAng() {
  return 0;
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
