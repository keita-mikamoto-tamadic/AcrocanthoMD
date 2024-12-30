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
    floatdiff = data->mechAng - mechAngPast;

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
  alpha = (TaskTime * static_cast<float>(compTime)) / timeConst;
  data->actVelLPF = alpha * data->actVel + (1.0f - alpha) * data->actVelLPF;

}

int16_t Ang::compAng() {
  return 0;
}


void Ang::prepareCanData(uint8_t* buffer, size_t bufferSize) const {

  memcpy(buffer, &(data->mechAng), sizeof(data->mechAng));
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
