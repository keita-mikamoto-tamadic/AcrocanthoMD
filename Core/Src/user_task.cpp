#include "user_task.h"

#include "main.h"
#include "ang.h"
#include "out_pwm.h"
#include "can_communication.h"
#include "mode_control.h"
#include "sens_cur.h"
#include "util.h"
#include "elecang_calib.h"
#include "foc.h"
#include "bldc_ctrl.h"

UserTask usertask;

extern Ang ang;
extern SensCur senscur;
extern OutPwm outpwm;
extern CanCom cancom;
extern ModeControl modecontrol;
extern Util util;
extern ElecangCalib elecangcalib;
extern Foc foc;
extern BldcCtrl bldcctrl;

UserTask::UserTask()
  : count(0){}


void UserTask::cyclicTask() {
  ElecangCalib::ElecangCalibData* ecaldata = elecangcalib.getData();
  Ang::AngData* angdata = ang.getData();
  static SeqID_t seqID = INIT;

   switch (seqID) {
    case LOOP:

      // 強制停止
      if (!servoCheck()) {
        outpwm.Poff();
        resetAll();

        seqID = STEP00;
        break;
      }

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      senscur.sensCurIN();
      ang.getAngle();
      ang.getVel();
      ang.elecAngleIn();
      
      // elecAng offset
      elecangcalib.elecCalSeq();
      test = ecaldata->elecAngOfs;
      eofsm = ecaldata->elecAngOfsMinus;
      eofsp = ecaldata->elecAngOfsPlus;
      // 
      
      motorControl();

      break;
    case INIT:
      // 初期化のためにエンコーダ値の初回読み取り
      resetAll();
      if (count < 10) {
        ang.getAngle();
        count++;
      } else {
        // 電流オフセット補正
        ang.getAngle();
        ang.getVel();
        if (senscur.sensCurInit()) {
          senscur.sensCurIN();
          count = 0;

          seqID = STEP00;
        }
        
      }
      break;
    case STEP00:
      senscur.sensCurIN();
      ang.getAngle();
      ang.getVel();
      ang.elecAngleIn();
      if (servoCheck()){
        outpwm.Pon();

        seqID = LOOP;
        break;
      }
      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);

      outpwm.Poff();
      break;

    default:
      seqID = INIT;
      break;
    }
}

void UserTask::idleTask() {

  cancom.rxTask();
  util.genFuncCtrl();

  cancom.initTxHeader(false, false);
  cancom.txTask();
}

// PON後のモータ制御
void UserTask::motorControl() {
  using namespace Acrocantho;
  ModeControl::ModeControlData* mdctrldata = modecontrol.getData();
  Ang::AngData* angdata = ang.getData();
  Foc::FocData* focdata = foc.getData();
  CanCom::CanData* candata = cancom.getData();
  BldcCtrl::BldcCtrlData* bldcdata = bldcctrl.getData();
  SensCur::SensCurData* senscurdata = senscur.getData();
  Cordic cordic;

  // SinCos演算
  SinCos result = cordic.radians(angdata->elecAng);

  // dq変換
  foc.forwardCtrl(result);
  
  testid = focdata->id;
  testiq = focdata->iq;
  
  // drvMdとgenfuncによる指令値切替
  modecontrol.modeCtrl();
  
  testvd = mdctrldata->voltDRef;
  testvq = mdctrldata->voltQRef;
  testvel = angdata->actVelLPF;
  testdiff = angdata->testdiff;
//  testvelact = angdata->actVel;
  testpos = angdata->mechAng;
  //testelec = angdata->elecAng;
  //testerrD = candata->curDRef;
  //testerrQ = candata->curQRef;
  //testCurU = senscurdata->testU;
  //testCurW = senscurdata->testW;
  //testcomp = angdata->eleccomp;
  testrawAng = angdata->rawAngtest;
  testrawAngPast = angdata->rawAngPasttest;
  testvelerr = bldcdata->testvelErr;
  testvelerrsum = bldcdata->testvelErrSum;


  // dq逆変換
  foc.inverseCtrl(result, mdctrldata->voltDRef, mdctrldata->voltQRef);
  // PWM出力
  outpwm.setReg(focdata->vu, focdata->vv, focdata->vw);
}

bool UserTask::servoCheck() {
  CanCom::CanData* candata = cancom.getData();
  return (candata->genFuncRef & 0x01) != 0 ? true : false;
}

void UserTask::resetAll() {
  bldcctrl.resetData();
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
  usertask.cyclicTask();
}
