#pragma once

#include "main.h"

#include <memory>

struct angData{
  float elecAng = 0.0f;
  float mechAng = 0.0f;
  float actAng = 0.0f;
  uint8_t ldata = 0;
  uint8_t hdata = 0;
};

class Ang {
private:
  I2C_HandleTypeDef& hi2c1;
  bool readStart;
  float actAngle;
  uint8_t rawAng[2];
  volatile int8_t i2c_tx_complete;
  volatile int8_t i2c_rx_complete;
  
  std::unique_ptr<angData> data;


public:
  Ang(I2C_HandleTypeDef& hi2c1);
  
  void read();
  void receive();
  float getAngle() const;
  void i2cMasterTxCallback();
  void i2cMasterRxCallback();
  void prepareCanData(uint8_t* buffer, size_t bufferSize) const;

  
};
