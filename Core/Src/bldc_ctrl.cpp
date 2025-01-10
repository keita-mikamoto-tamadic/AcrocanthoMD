#include "bldc_ctrl.h"

#include <memory>

#include "param.h"
#include "foc.h"

BldcCtrl bldcctrl;
extern Foc foc;

BldcCtrl::BldcCtrl(){}
  
float BldcCtrl::voltDCtrl(float _curD) {
  curDPidCtrl(_curD);
  voltData.voltD = curData.CurDPid;
  return voltData.voltD;
}

float BldcCtrl::voltQCtrl(float _curQ) {
  curQPidCtrl(_curQ);
  voltData.voltQ = curData.curQPid;
  return voltData.voltQ;
}

float BldcCtrl::curDPidCtrl(float _curDRef) {
  Foc::FocData* focdata = foc.getData();

  // PControl
  float curDErr_ = _curDRef - focdata->id;

  // IControl
  // アンチワインドアップ
  if ((volMin < curData.CurDPid) &&
      (curData.CurDPid < volMax)) {
    curDErrSum += (curDErr_ * TASK_TIME);
  } else {
    // Do nothing
  }
  // DControl
  curDErrLPF = (1.0f - lpfcoef) * curDErrLPF + lpfcoef * curDErr_;
  float curDErrDiff_ = (curDErr_ - curDErrLPFPast) / TASK_TIME;
  // 微分用前回値
  curDErrLPFPast = curDErrLPF;
  
  curData.CurDPid = 
    curKp * curDErr_ + curKi * curDErrSum + curKd * curDErrDiff_;

  return curData.CurDPid;
}

float BldcCtrl::curQPidCtrl(float _curQRef) {
  Foc::FocData* focdata = foc.getData();
  
  // PControl
  float curQErr_ = _curQRef - focdata->iq;
  
  // IControl
  // アンチワインドアップ
  if ((volMin < curData.curQPid) &&
      (curData.curQPid < volMax)) {
    curQErrSum += (curQErr_ * TASK_TIME);
  } else {
    // Do nothing
  }
  // DControl
  curQErrLPF = (1.0f - lpfcoef) * curQErrLPF + lpfcoef * curQErr_;
  float curQErrDiff_ = (curQErr_ - curQErrLPFPast) / TASK_TIME;
  // 微分用前回値
  curQErrLPFPast = curQErrLPF;

  curData.curQPid = 
    curKp * curQErr_ + curKi * curQErrSum + curKd * curQErrDiff_;
  
  return curData.curQPid;
}
