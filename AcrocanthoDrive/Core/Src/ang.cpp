#include "ang.h"

volatile uint8_t i2c_rx_complete;
volatile uint8_t i2c_tx_complete;

Ang::Ang(I2C_HandleTypeDef& i2cHandle)
  : i2cHandle_(i2cHandle), readStart_(false), txComplete_(false), rxComplete_(false), actAngle_(0.0f) {}

void Ang::read() {
  if (!readStart_) {
    uint8_t readRegAddr = 0x0C;
    HAL_I2C_Master_Transmit_DMA(&i2cHandle_, AS5600ADDR, &readRegAddr, 1);
    readStart_ = true;
  }
}

void Ang::receive() {
  if (i2c_tx_complete) {
    HAL_I2C_Master_Receive_DMA(&i2cHandle_, AS5600ADDR, rawAng_, 2);
    i2c_tx_complete = false;
  }
  if (i2c_rx_complete) {
    uint16_t temp = static_cast<uint16_t>(rawAng_[0] << 8) | rawAng_[1];
    actAngle_ = static_cast<float>(temp) / 4095.0f;
    
    readStart_ = false;
    i2c_rx_complete = false;
  }
  
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    i2c_tx_complete = 1;    
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    i2c_rx_complete = 1;
}