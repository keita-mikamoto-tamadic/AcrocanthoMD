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

UserTask usertask;

extern Ang ang;
extern SensCur senscur;
extern OutPwm outpwm;
extern CanCom cancom;
extern ModeControl modecontrol;
extern Util util;
extern ElecangCalib elecangcalib;
extern Foc foc;

UserTask::UserTask()
  : count(0){}


void UserTask::cyclicTask() {
  Ang::AngData* angdata = ang.getAngData();
  ElecangCalib::ElecangCalibData* ecaldata = elecangcalib.getData();
  static SeqID_t seqID = INIT;
  static bool curcal = false;

   switch (seqID) {
    case LOOP:

      // 強制停止
      if (!servoCheck()) {
        outpwm.Poff();
        seqID = STEP00;
        break;
      }

      HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
      senscur.sensCurIN();
      ang.getAngle();
      ang.getVel();
      ang.elecAngleIn();
      
      elecangcalib.elecCalSeq();
      test = ecaldata->elecAngOfs;
      motorControl();

      break;
    case INIT:
      // 初期化のためにエンコーダ値の初回読み取り
      if (count < 10) {
        ang.getAngle();
        count++;
      } else {
        // 電流オフセット補正
        ang.getAngle();
        ang.getVel();
        if (senscur.sensCurInit()) {
          senscur.sensCurIN();
          seqID = STEP00;
          count = 0;
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

  cancom.initTxHeader(0x01, false, false);
  cancom.txTask();
}

// PON後のモータ制御
void UserTask::motorControl() {
  using namespace Acrocantho;
  ModeControl::ModeControlData* mdctrldata = modecontrol.getData();
  Ang::AngData* angdata = ang.getAngData();
  Foc::FocData* focdata = foc.getData();
  Cordic cordic;

  // SinCos演算
  SinCos result = cordic.radians(angdata->elecAng);

  // dq変換
  foc.forwardCtrl(result);
  
  testid = focdata->id;
  testiq = focdata->iq;
  
  // drvMdとgenfuncによる指令値切替
  modecontrol.modeCtrl();

  // dq逆変換
  foc.inverseCtrl(result, mdctrldata->voltDRef, mdctrldata->voltQRef);
  // PWM出力
  outpwm.setReg(focdata->vu, focdata->vv, focdata->vw);
}

bool UserTask::servoCheck() {
  CanCom::CanData* candata = cancom.getData();
  return (candata->genFuncRef & 0x01) != 0 ? true : false;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
  usertask.cyclicTask();
}
