#pragma once

#include <cstdint>
#include <memory>

#include "param.h"
#include "user_math.h"

class BldcCtrl {
private:
  struct VoltCtrlData {
    float voltD = 0.0f;
    float voltQ = 0.0f;
  };

  struct CurPidData {
    float CurDPid = 0.0f;
    float curQPid = 0.0f;
  };

  struct VelPidData {
    float CurD = 0.0f;
    float CurQ = 0.0f;
  };
  
  struct PosPidData {
    float Vel = 0.0f;
  };

  VoltCtrlData voltData;
  CurPidData curData;
  VelPidData velData;
  PosPidData posData;

  // parameter
  const float volMin = -12.0f;
  const float volMax = 12.0f;
  const float curMin = -1.0f;
  const float curMax = 1.0f;

  const float cutOffFreq = 100.0f;
  const float TimeConst = 1.0f / (user2pi * cutOffFreq);
  const float lpfcoef = TASK_TIME / TimeConst;
  const float curKp = 0.5f;
  const float curKi = 1.0f;
  const float curKd = 0.0f;
  // parameter
  
  float curDPidCtrl(float _curDRef);
  float curDCtrlOut = 0.0f;
  float curDErrSum = 0.0f;
  float curDErrLPF = 0.0f;
  float curDErrLPFPast = 0.0f;

  float curQPidCtrl(float _curQRef);
  float curQCtrlOut = 0.0f;
  float curQErrSum = 0.0f;
  float curQErrLPF = 0.0f;
  float curQErrLPFPast = 0.0f;
  
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

    curDErrSum = 0.0f;
    curQErrSum = 0.0f;
    curDErrLPF = 0.0f;
    curQErrLPF = 0.0f;
    curDErrLPFPast = 0.0f;
    curQErrLPFPast = 0.0f;
  }

};
