#pragma once

#include <cstdint>
#include <memory>

#include "param.h"
#include "user_math.h"

class BldcCtrl {
private:
  // クリアしたいデータは構造体に入れておく
  struct VoltCtrlData {
    float voltD = 0.0f;
    float voltQ = 0.0f;
  };

  struct CurPidData {
    float curDPidRaw = 0.0f;
    float curDErrSum = 0.0f;
    float curDErrLPF = 0.0f;
    float curDErrLPFPast = 0.0f;

    float curQPidRaw = 0.0f;
    float curQErrSum = 0.0f;
    float curQErrLPF = 0.0f;
    float curQErrLPFPast = 0.0f;
  };

  struct VelPidData {
    float velPidRaw = 0.0f;
    float velErrLPF = 0.0f;
    float velErrLPFPast = 0.0f;
    float velErrSum = 0.0f;
  };
  
  struct PosPidData {
    float Vel = 0.0f;
  };

  VoltCtrlData voltData;
  CurPidData curData;
  VelPidData velData;
  PosPidData posData;

  // hw param
  const float volMin = -12.0f;
  const float volMax = 12.0f;
  const float curMin = -1.0f;
  const float curMax = 1.0f;

  // cur param
  const float cutOffFreq = 100.0f;
  const float TimeConst = 1.0f / (user2pi * cutOffFreq);
  const float lpfcoef = TASK_TIME / TimeConst;
  const float curKp = 0.5f;
  const float curKi = 1.0f;
  const float curKd = 0.0f;

  // vel param

  float curDPidCtrl(float _curDRef);
  float curQPidCtrl(float _curQRef);
  float velPidCtrl(float _velRef);
  
public:
  BldcCtrl();

  float voltDCtrl(float _curD);
  float voltQCtrl(float _curQ);

  // サーボオフ時にデータをリセット
  void resetData() {
    voltData = VoltCtrlData();
    curData = CurPidData();
    velData = VelPidData();
    posData = PosPidData();
  }

};
