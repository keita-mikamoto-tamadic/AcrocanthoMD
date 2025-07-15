#pragma once

#include <cstdint>
#include <memory>

#include "param.h"
#include "user_math.h"

class BldcCtrl {
public:
  struct BldcCtrlData {
    float testerrQ = 0.0f;
    float testerrD = 0.0f;
    float testvelErr = 0.0f;
    float testvelErrSum = 0.0f;
    float testposErr = 0.0f;
    float testposErrSum = 0.0f;
  };

private:
  // PID制御共通データ構造
  struct PidData {
    float pidRaw = 0.0f;
    float errSum = 0.0f;
    float errLPF = 0.0f;
    float errLPFPast = 0.0f;
  };

  // PID制御パラメータ構造
  struct PidParams {
    float kp;
    float ki;
    float kd;
    float outMin;
    float outMax;
  };
  // クリアしたいデータは構造体に入れておく
  struct VoltCtrlData {
    float voltD = 0.0f;
    float voltQ = 0.0f;
  };

  VoltCtrlData voltData;
  PidData curDData;
  PidData curQData;
  PidData velData;
  PidData posData;

  // モーター設定構造体
  struct MotorConfig {
    float volMin;
    float volMax;
    float curQMin;
    float curQMax;
    float velMin;
    float velMax;
    float curKp;
    float curKi;
    float curKd;
    float velKp;
    float velKi;
    float velKd;
    float posKp;
    float posKi;
    float posKd;
  };

  // constexpr定数
  static constexpr float cutOffFreq = 50.0f;
  static constexpr float timeConst = 1.0f / (user2pi * cutOffFreq);
  static constexpr float lpfCoef = TASK_TIME / timeConst;

  // モーター固有設定
  #ifdef GIM6010_8
  static constexpr MotorConfig motorConfig = {
    -24.0f, 24.0f,    // volMin, volMax
    -10.0f, 10.0f,    // curQMin, curQMax
    -9.0f, 9.0f,      // velMin, velMax
    0.2f, 100.0f, 0.0f, // curKp, curKi, curKd
    5.0f, 30.0f, 0.0f,  // velKp, velKi, velKd
    8.0f, 0.5f, 0.0f    // posKp, posKi, posKd
  };
  #endif

  #ifdef GIM8108_8
  static constexpr MotorConfig motorConfig = {
    -24.0f, 24.0f,    // volMin, volMax
    -10.0f, 10.0f,    // curQMin, curQMax
    -5.0f, 5.0f,      // velMin, velMax
    0.2f, 20.0f, 0.0f,  // curKp, curKi, curKd
    10.0f, 0.5f, 0.0f,  // velKp, velKi, velKd
    17.0f, 0.4f, 0.0f   // posKp, posKi, posKd
  };
  #endif

  // PID制御共通関数
  float pidControl(float reference, float feedback, PidData& pidData, 
                   const PidParams& params, float* testErr = nullptr, 
                   float* testErrSum = nullptr);

  float curDPidCtrl(float _curDRef);
  float curQPidCtrl(float _curQRef);
  
  std::unique_ptr<BldcCtrlData> data;
  
public:
  BldcCtrl();

  float voltDCtrl(float _curD);
  float voltQCtrl(float _curQ);
  float velPidCtrl(float _velRef);
  float posPidCtrl(float _posRef);

  // サーボオフ時にデータをリセット
  void resetData() {
    voltData = VoltCtrlData();
    curDData = PidData();
    curQData = PidData();
    velData = PidData();
    posData = PidData();
  }
  
  BldcCtrl::BldcCtrlData* getData() { return data.get(); }

};
