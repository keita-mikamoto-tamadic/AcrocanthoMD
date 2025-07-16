#pragma once

#include "main.h"

#include "user_math.h"
#include "user_task.h"
#include "param.h"

// MA735エンコーダ定数
static constexpr uint16_t ANG_RESL_12BIT = 4095;
static constexpr uint8_t MULT_TURN_NONE = 0;
static constexpr uint8_t MULT_TURN_POS = 1;
static constexpr uint8_t MULT_TURN_NEG = 2;
static constexpr uint16_t ANG_RESL_HALF = ANG_RESL_12BIT / 2;

// MA735コマンド定数
static constexpr uint16_t CMD_MA735_READ = 0x00;
static constexpr uint16_t CMD_MA735_MAGTH = 0x5B00;
static constexpr uint16_t CMD_MA735_MGLHT = 0x4600;

static constexpr uint8_t rotDir = 0;
static constexpr uint8_t elecAngDir = 0;
static constexpr uint8_t mechAngDir = 0;

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
  // データ保持
  MA735Data data;

  static constexpr float lpfFreq = 150.0f;
  static constexpr float timeConst = 1.0f / (user2pi * lpfFreq);
  static constexpr float kalpha = TASK_TIME / timeConst;
  static constexpr float invResol = 1.0f / ANG_RESL_12BIT;
  static constexpr float invTaskTime = 1.0f / TASK_TIME;

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
  
  MA735Data* getData() { return &data; }
};
