#include "ang.h"
#include <cstring>

#include "main.h"
#include "param.h"
#include "user_task.h"

extern Ang ang;

Ang::Ang(I2C_HandleTypeDef& i2cHandle)
  : hi2c1(i2cHandle), readStart(false), actAngle(0.0f), i2c_rx_complete(false), i2c_tx_complete(false),
    rawAng(0), rawAngPast(0), diffRaw(0), data(std::make_unique<angData>()) {}

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

    //HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_8);
    if (diff > 2048) {
      diff -= 4096;
    } else if (diff < -2048) {
      diff += 4096;
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

void Ang::elecAng() {
  // comp = 0のときサンプル値更新あり
  // 次の更新まで2周期を補間するので3で割った値を足す
  if (comp == 0) rawElecComp = rawAng;
  else rawElecComp = rawAng + (diff / 3);
  
  // 電気角反転
  static uint16_t elecAngtemp_ = 0;
  if (elecAngDir > 0) {
    elecAngtemp_ = rawElecComp;
  }else elecAngtemp_ = 4096 - rawElecComp;

  // CWとCCWを切替
  static float ofs_ = 0.0f;
  if (rotDir > 0) {
    ofs_ = elecAngOfs + user2pi;
  }else ofs_ = elecAngOfs; // 極性反転不要

  static uint16_t offset_ = 0;
  static uint16_t elecAngtemp2_ = 0;
  
  offset_ = static_cast<uint16_t>(ofs_ * 4096.0f / user2pi);
  elecAngtemp2_ = (elecAngtemp_ * polePairs + offset_) % 4096;
  data->elecAng = static_cast<float>(elecAngtemp2_) / 4096.0f * user2pi;

}

void Ang::elecAngVirtual() {
  // 仮想電気角
  static float elecAngtemp = 0.0f;
  elecAngtemp = data->mechAng * polePairs;
  data->elecAng = elecAngtemp;
}

void Ang::elecAngleIn(){
  elecAng();

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
