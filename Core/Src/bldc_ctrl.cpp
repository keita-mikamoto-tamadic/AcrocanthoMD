#include "bldc_ctrl.h"

#include <memory>

#include "ang.h"
#include "param.h"
#include "foc.h"

BldcCtrl bldcctrl;
extern Foc foc;
extern Ang ang;

BldcCtrl::BldcCtrl()
  : data(std::make_unique<BldcCtrlData>()){}
  
float BldcCtrl::voltDCtrl(float _curD) {
  voltData.voltD = curDPidCtrl(_curD);
  return voltData.voltD;
}

float BldcCtrl::voltQCtrl(float _curQ) {
  voltData.voltQ = curQPidCtrl(_curQ);
  return voltData.voltQ;
}

float BldcCtrl::curDPidCtrl(float _curDRef) {
  Foc::FocData* focdata = foc.getData();
  
  float curDCtrlOut = 0.0f;

  // ==== PControl ====
  float curDErr_ = _curDRef - focdata->id;
  data->testerrD = curDErr_;

  // ==== IControl ====
  // アンチワインドアップ
  if ((volMin < curData.curDPidRaw) &&
      (curData.curDPidRaw < volMax)) {
    curData.curDErrSum += (curDErr_ * TASK_TIME);
  } else {
    // Do nothing
  }

  //==== DControl ====
  curData.curDErrLPF = (1.0f - lpfcoef) * curData.curDErrLPF + lpfcoef * curDErr_;
  float curDErrDiff_ = (curDErr_ - curData.curDErrLPFPast) / TASK_TIME;
  // 微分用前回値
  curData.curDErrLPFPast = curData.curDErrLPF;
  
  // ==== PID Control ====
  curData.curDPidRaw = 
    curKp * curDErr_ + curKi * curData.curDErrSum + curKd * curDErrDiff_;
  
  // 出力飽和
  if (curData.curDPidRaw > volMax) {
    curDCtrlOut = volMax;
  } else if (curData.curDPidRaw < volMin) {
    curDCtrlOut = volMin;
  } else {
    curDCtrlOut = curData.curDPidRaw;
  }

  return curDCtrlOut;
}

float BldcCtrl::curQPidCtrl(float _curQRef) {
  Foc::FocData* focdata = foc.getData();
  float curQCtrlOut = 0.0f;
  
  // ==== PControl ====
  float curQErr_ = _curQRef - focdata->iq;
  data->testerrQ = curQErr_;
  
  // ==== IControl ====
  // アンチワインドアップ
  if ((volMin < curData.curQPidRaw) &&
      (curData.curQPidRaw < volMax)) {
    curData.curQErrSum += (curQErr_ * TASK_TIME);
  } else {
    // Do nothing
  }

  // ==== DControl ====
  curData.curQErrLPF = (1.0f - lpfcoef) * curData.curQErrLPF + lpfcoef * curQErr_;
  float curQErrDiff_ = (curQErr_ - curData.curQErrLPFPast) / TASK_TIME;
  // 微分用前回値
  curData.curQErrLPFPast = curData.curQErrLPF;

  // ==== PID Control ====
  curData.curQPidRaw = 
    curKp * curQErr_ + curKi * curData.curQErrSum + curKd * curQErrDiff_;
  
  if (curData.curQPidRaw > volMax) {
    curQCtrlOut = volMax;
  } else if (curData.curQPidRaw < volMin) {
    curQCtrlOut = volMin;
  } else {
    curQCtrlOut = curData.curQPidRaw;
  }
  
  return curQCtrlOut;
}

float BldcCtrl::velPidCtrl(float _velRef) {
  Ang::AngData* angdata = ang.getData();

  float velCtrlOut = 0.0f;
  
  // ==== PControl ====
  float velErr_ = _velRef - angdata->actVelLPF;
  
  // ==== IControl ====
  // アンチワインドアップ - 出力が制限値内の場合のみ積分を実行
  if (curQMin < velData.velPidRaw && velData.velPidRaw < curQMax) {
    velData.velErrSum += (velErr_ * TASK_TIME);
  } else {
    // 積分項をリセット
    velData.velErrSum = 0.0f;
  }
  // ==== DControl ====
  velData.velErrLPF = (1.0f - lpfcoef) * velData.velErrLPF + lpfcoef * velErr_;
  float velErrDiff_ = (velErr_ - velData.velErrLPFPast) / TASK_TIME;
  // 微分用前回値
  velData.velErrLPFPast = velData.velErrLPF;
  
  // ==== PID Control ====
  velData.velPidRaw = 
    velKp * velErr_ + velKi * velData.velErrSum + velKd * velErrDiff_;
  
  // 出力飽和
  if (velData.velPidRaw > curQMax) {
    velCtrlOut = curQMax;
  } else if (velData.velPidRaw < curQMin) {
    velCtrlOut = curQMin;
  } else {
    velCtrlOut = velData.velPidRaw;
  }

  return velCtrlOut;
}
