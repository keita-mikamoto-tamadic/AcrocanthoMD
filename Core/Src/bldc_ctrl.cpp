#include "bldc_ctrl.h"

#include <memory>

#include "param.h"
#include "foc.h"

BldcCtrl bldcctrl;
extern Foc foc;

BldcCtrl::BldcCtrl(){}
  
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

  // ==== PControl ====
  float curDErr_ = _curDRef - focdata->id;

  // ==== IControl ====
  // アンチワインドアップ
  if ((volMin < curData.CurDPid) &&
      (curData.CurDPid < volMax)) {
    curDErrSum += (curDErr_ * TASK_TIME);
  } else {
    // Do nothing
  }

  //==== DControl ====
  curDErrLPF = (1.0f - lpfcoef) * curDErrLPF + lpfcoef * curDErr_;
  float curDErrDiff_ = (curDErr_ - curDErrLPFPast) / TASK_TIME;
  // 微分用前回値
  curDErrLPFPast = curDErrLPF;
  
  // ==== PID Control ====
  curData.CurDPid = 
    curKp * curDErr_ + curKi * curDErrSum + curKd * curDErrDiff_;
  
  // 出力飽和
  if (curData.CurDPid > volMax) {
    curDCtrlOut = volMax;
  } else if (curData.CurDPid < volMin) {
    curDCtrlOut = volMin;
  } else {
    curDCtrlOut = curData.CurDPid;
  }

  return curDCtrlOut;
}

float BldcCtrl::curQPidCtrl(float _curQRef) {
  Foc::FocData* focdata = foc.getData();
  
  // ==== PControl ====
  float curQErr_ = _curQRef - focdata->iq;
  
  // ==== IControl ====
  // アンチワインドアップ
  if ((volMin < curData.curQPid) &&
      (curData.curQPid < volMax)) {
    curQErrSum += (curQErr_ * TASK_TIME);
  } else {
    // Do nothing
  }

  // ==== DControl ====
  curQErrLPF = (1.0f - lpfcoef) * curQErrLPF + lpfcoef * curQErr_;
  float curQErrDiff_ = (curQErr_ - curQErrLPFPast) / TASK_TIME;
  // 微分用前回値
  curQErrLPFPast = curQErrLPF;

  // ==== PID Control ====
  curData.curQPid = 
    curKp * curQErr_ + curKi * curQErrSum + curKd * curQErrDiff_;
  
  if (curData.curQPid > volMax) {
    curQCtrlOut = volMax;
  } else if (curData.curQPid < volMin) {
    curQCtrlOut = volMin;
  } else {
    curQCtrlOut = curData.curQPid;
  }
  
  return curQCtrlOut;
}
