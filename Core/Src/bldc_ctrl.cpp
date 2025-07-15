#include "bldc_ctrl.h"

#include <memory>

#include "ma735_enc.h"
#include "param.h"
#include "foc.h"

BldcCtrl bldcctrl;
extern Foc foc;
extern MA735Enc ma735enc;

BldcCtrl::BldcCtrl()
  : data(std::make_unique<BldcCtrlData>()){}

// PID制御共通関数
float BldcCtrl::pidControl(float reference, float feedback, PidData& pidData, 
                           const PidParams& params, float* testErr, float* testErrSum) {
  float ctrlOut = 0.0f;

  // ==== PControl ====
  float err = reference - feedback;
  if (testErr) *testErr = err;

  // ==== IControl ====
  // アンチワインドアップ
  if ((params.outMin < pidData.pidRaw) && (pidData.pidRaw < params.outMax)) {
    pidData.errSum += (err * TASK_TIME);
  }
  if (testErrSum) *testErrSum = pidData.errSum;

  // ==== DControl ====
  pidData.errLPF = (1.0f - lpfCoef) * pidData.errLPF + lpfCoef * err;
  float errDiff = (err - pidData.errLPFPast) / TASK_TIME;
  pidData.errLPFPast = pidData.errLPF;

  // ==== PID Control ====
  pidData.pidRaw = params.kp * err + params.ki * pidData.errSum + params.kd * errDiff;

  // 出力飽和
  if (pidData.pidRaw > params.outMax) {
    ctrlOut = params.outMax;
  } else if (pidData.pidRaw < params.outMin) {
    ctrlOut = params.outMin;
  } else {
    ctrlOut = pidData.pidRaw;
  }

  return ctrlOut;
}
  
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
  
  const PidParams curDParams = {
    motorConfig.curKp, motorConfig.curKi, motorConfig.curKd,
    motorConfig.volMin, motorConfig.volMax
  };
  
  return pidControl(_curDRef, focdata->id, curDData, curDParams, &data->testerrD);
}

float BldcCtrl::curQPidCtrl(float _curQRef) {
  Foc::FocData* focdata = foc.getData();
  
  const PidParams curQParams = {
    motorConfig.curKp, motorConfig.curKi, motorConfig.curKd,
    motorConfig.volMin, motorConfig.volMax
  };
  
  return pidControl(_curQRef, focdata->iq, curQData, curQParams, &data->testerrQ);
}

float BldcCtrl::velPidCtrl(float _velRef) {
  MA735Enc::MA735Data* angdata = ma735enc.getData();
  
  const PidParams velParams = {
    motorConfig.velKp, motorConfig.velKi, motorConfig.velKd,
    motorConfig.curQMin, motorConfig.curQMax
  };
  
  return pidControl(_velRef, angdata->mechAngVelLPF, velData, velParams, 
                   &data->testvelErr, &data->testvelErrSum);
}

float BldcCtrl::posPidCtrl(float _posRef) {
  MA735Enc::MA735Data* angdata = ma735enc.getData();
  
  const PidParams posParams = {
    motorConfig.posKp, motorConfig.posKi, motorConfig.posKd,
    motorConfig.velMin, motorConfig.velMax
  };
  
  return pidControl(_posRef, angdata->mechAng, posData, posParams, 
                   &data->testposErr, &data->testposErrSum);
}
