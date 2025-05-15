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

#define CMD_MA735_READ ((uint16_t)0x00)
#define CMD_MA735_MAGTH ((uint16_t)0x5B00)
#define CMD_MA735_MGLHT ((uint16_t)0x4600)

constexpr uint8_t rotDir = 0;
constexpr uint8_t elecAngDir = 0;
constexpr uint8_t mechAngDir = 0;

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
  const float timeConst = 1.0f / (user2pi * lpfFreq);
  const float kalpha = TASK_TIME / timeConst;
  
  const float invResol = 1.0f / ANG_RESL_12BIT;
  const float invTaskTime = 1.0f / TASK_TIME;

  SPI_HandleTypeDef& hspi1;
  bool readStart;
  uint8_t comp = 0;
  uint8_t compTime = 0;
  float actAngle;
  uint8_t rawEnc[2];
  float mtAng = 0.0f;
  float mechAngPast;
  int16_t diffRaw; // 電気角に使用
  int16_t diff;
  float floatdiff;
  volatile int8_t spi_tx_complete;
  volatile int8_t spi_rx_complete;
  uint8_t zeroPointTh = 0;
  int32_t mtCount = 0;
  
  void read(uint16_t reg);

  float elecAng(float _eofs);
  uint16_t rawElecComp = 0;
  
  float elecAngVirtual(float virfreq);
  void elecAngVel();

  void mechAngleVelLPF();
  
  float raw2rad(uint16_t raw){
    return static_cast<float>(raw) * user2pi * invResol;
  }
  
  float raw2rads(int16_t raw){
//    return static_cast<float>(raw) * user2pi / 4095.0f / (TASK_TIME * static_cast<float>(compTime));
    return static_cast<float>(raw) * user2pi * invResol * invTaskTime;
  }

public:
  MA735Enc(SPI_HandleTypeDef& hspi1);
  
  bool ma735Init();
  void magFieldTh();
  void ma735angle();
  void getAngle();
  void getVel();
  void elecAngleIn();
  void mechAngleIn();
  void zeroPosOffset();
  void spiRxCallback();
  void spiTxCallback();
  
  MA735Data* getData() const { return data.get(); }
};
