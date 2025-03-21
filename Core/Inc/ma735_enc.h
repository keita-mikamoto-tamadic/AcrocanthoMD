#pragma once

#include "main.h"

#include <memory>
#include "user_math.h"
#include "user_task.h"
#include "param.h"

#define ANG_RESL_12BIT (4095)
#define MULT_TURN_NONE (0)
#define MULT_TURN_POS (1)
#define MULT_TURN_NEG (2)

constexpr uint8_t rotDir = 0;
constexpr uint8_t elecAngDir = 1;

class MA735Enc {
public:
  struct MA735Data{
    float elecAng = 0.0f;
    float elecVel = 0.0f;
    float mechAng = 0.0f;
    float actAng = 0.0f;
    float mechAngVel = 0.0f;
    float mechAngVelLPF = 0.0f;
    int16_t veltemp = 0;
    float elecAngTest = 0.0f;
    uint32_t eleccomp = 0;
    int16_t testdiff = 0.0f;
    uint16_t rawAngtest = 0;
    uint16_t rawAngPasttest = 0;
    float zeroPosOffs = 0.0f;
    uint16_t rawAng;
    uint16_t rawAngPast;
  };

private:
  // ユニークポインタでデータ保持
  std::unique_ptr<MA735Data> data;

  const float lpfFreq = 150.0f;

  SPI_HandleTypeDef& hspi2;
  bool readStart;
  uint8_t comp = 0;
  uint8_t compTime = 0;
  float actAngle;
  uint8_t rawEnc[2];
  float mtAng = 0.0f;
  float mechAngPast;
  int16_t diff;
  float floatdiff;
  int16_t diffRaw;
  volatile int8_t spi_tx_complete;
  volatile int8_t spi_rx_complete;
  uint8_t zeroPointTh = 0;
  int32_t mtCount = 0;
  
  void read();

  float elecAng(float _eofs);
  uint16_t rawElecComp = 0;
  
  float elecAngVirtual(float virfreq);
  void elecAngVel();

  void mechAngleVelLPF();
  
  float raw2rad(uint16_t raw){
    return static_cast<float>(raw) * user2pi / 4095.0f;
  }
  
  float raw2rads(int16_t raw){
    return static_cast<float>(raw) * user2pi / 4095.0f / (TASK_TIME * static_cast<float>(compTime));
  }

public:
  MA735Enc(SPI_HandleTypeDef& hspi2);
  
  bool ma735Init();
  void ma735angle();
  void getAngle();
  void getVel();
  void elecAngleIn();
  void mechAngleIn();
  void zeroPosOffset();
  void spiTxRxCallback();
  void prepareCanData(uint8_t* buffer, size_t bufferSize) const;
  
  MA735Data* getData() const { return data.get(); }
};
