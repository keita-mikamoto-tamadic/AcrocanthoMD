#pragma once

#include "main.h"

#include <memory>
#include "user_math.h"
#include "user_task.h"

struct angData{
  float elecAng = 0.0f;
  float mechAng = 0.0f;
  float actAng = 0.0f;
  float actVel = 0.0f;
  float actVelLPF = 0.0f;
  int16_t veltemp = 0;
};

class Ang {
private:
  const float lpfFreq = 50.0f;

  I2C_HandleTypeDef& hi2c1;
  bool readStart;
  uint8_t comp = 0;
  uint8_t compTime  = 0;
  float actAngle;
  uint8_t rawEnc[2];
  uint16_t rawAng;
  uint16_t rawAngPast;
  float mechAngPast;
  int16_t diff;
  float floatdiff;
  int16_t diffRaw;
  volatile int8_t i2c_tx_complete;
  volatile int8_t i2c_rx_complete;
  
  int16_t compAng();
  void mechAngleVelLPF();
  
  float raw2rad(uint16_t raw){
    return static_cast<float>(raw) * user2pi / 4096.0f;
  }
  
  float raw2rads(int16_t raw){
    return static_cast<float>(raw) * user2pi / 4096.0f / (TaskTime * static_cast<float>(compTime));
  }
  
  // ユニークポインタでデータ保持
  std::unique_ptr<angData> data;


public:
  Ang(I2C_HandleTypeDef& hi2c1);
  
  void read();
  void receive();
  void getAngle();
  void getVel();
  void i2cMasterTxCallback();
  void i2cMasterRxCallback();
  void prepareCanData(uint8_t* buffer, size_t bufferSize) const;
  
};