#include "ang.h"
#include <cstring>

#include "main.h"
#include "param.h"

extern Ang ang;

Ang::Ang(I2C_HandleTypeDef& i2cHandle)
  : hi2c1(i2cHandle), readStart(false), actAngle(0.0f), i2c_rx_complete(false), i2c_tx_complete(false),
    data(std::make_unique<angData>()) {}

void Ang::read() {
  if (!readStart) {
    uint8_t readRegAddr = 0x0C;
    HAL_I2C_Master_Transmit_DMA(&hi2c1, AS5600ADDR, &readRegAddr, 1);
    readStart = true;
  }
}

void Ang::receive() {
  if (i2c_tx_complete) {
    HAL_I2C_Master_Receive_DMA(&hi2c1, AS5600ADDR, rawAng, 2);
    i2c_tx_complete = false;
  }
  
  if (i2c_rx_complete) {
    uint16_t temp_ = static_cast<uint16_t>(rawAng[0] << 8) | rawAng[1];
    data->mechAng = static_cast<float>(temp_) / 4096.9f;
    data->actAng = data->mechAng;
    
    readStart = false;
    i2c_rx_complete = false;
  }
}

void Ang::prepareCanData(uint8_t* buffer, size_t bufferSize) const {

  memcpy(buffer, &(data->actAng), sizeof(data->actAng));
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
