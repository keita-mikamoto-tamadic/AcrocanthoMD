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
    float posPidRaw = 0.0f;
    float posErrSum = 0.0f;
    float posErrLPF = 0.0f;
    float posErrLPFPast = 0.0f;
  };

  VoltCtrlData voltData;
  CurPidData curData;
  VelPidData velData;
  PosPidData posData;

  #ifdef GIM6010_8
  // hw param
  const float volMin = -24.0f;
  const float volMax = 24.0f;
  const float curQMin = -10.0f;
  const float curQMax = 10.0f;
  const float velMin = -9.0f;
  const float velMax = 9.0f;

  // cur param
  const float cutOffFreq = 50.0f;
  const float TimeConst = 1.0f / (user2pi * cutOffFreq);
  const float lpfcoef = TASK_TIME / TimeConst;
  const float curKp = 0.6f;
  const float curKi = 200.0f;
  const float curKd = 0.0f;

  // vel param
  const float velKp = 1.3f;
  const float velKi = 30.0f;
  const float velKd = 0.0f;

  // pos param
  const float posKp = 8.0f;
  const float posKi = 0.5f;
  const float posKd = 0.0f;
  #endif

  #ifdef GIM8108_8
  // hw param
  const float volMin = -24.0f;
  const float volMax = 24.0f;
  const float curQMin = -15.0f;
  const float curQMax = 15.0f;
  const float velMin = -9.0f;
  const float velMax = 9.0f;

  // cur param
  const float cutOffFreq = 50.0f;
  const float TimeConst = 1.0f / (user2pi * cutOffFreq);
  const float lpfcoef = TASK_TIME / TimeConst;
  const float curKp = 0.1f;
  const float curKi = 1.0f;
  const float curKd = 0.0f;

  // vel param
  const float velKp = 0.1f;
  const float velKi = 1.0f;
  const float velKd = 0.0f;

  // pos param
  const float posKp = 1.0f;
  const float posKi = 0.5f;
  const float posKd = 0.0f;
  #endif

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
    curData = CurPidData();
    velData = VelPidData();
    posData = PosPidData();
  }
  
  BldcCtrl::BldcCtrlData* getData() { return data.get(); }

};
