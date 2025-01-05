#include "user_task.h"

#include "main.h"
#include "ang.h"
#include "out_pwm.h"
#include "can_communication.h"
#include "mode_control.h"
#include "sens_cur.h"
#include "util.h"
#include "elecang_calib.h"

UserTask usertask;

extern Ang ang;
extern SensCur senscur;
extern OutPwm outpwm;
extern CanCom cancom;
extern ModeControl modecontrol;
extern Util util;
extern ElecangCalib elecangcalib;

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
      ang.getAngle();
      ang.getVel();
      ang.elecAngleIn();
      
      elecangcalib.elecCalSeq();
      //test = angdata->actVelLPF;
      test_ofs = ecaldata->elecAngOfsPlus;
      test_ang = angdata->mechAng;
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
      ang.getAngle();
      ang.getVel();
      ang.elecAngleIn();
      if (servoCheck()){
        outpwm.Pon();
        seqID = LOOP;
        break;
      }
      senscur.sensCurIN();
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
  ModeControl::ModeControlData* mdctrldata = modecontrol.getData();
  Ang::AngData* angdata = ang.getAngData();
  Acrocantho::Cordic cordic;

  modecontrol.modeCtrl();

  senscur.sensCurIN();
  
  // SinCos演算
  Acrocantho::SinCos result = cordic.radians(angdata->elecAng);
  float s = result.s;
  float c = result.c;
  
  // dq逆変換
  Acrocantho::TrigonTransform tt(result, mdctrldata->voltDRef, mdctrldata->voltQRef);
  Acrocantho::InverseDqTransform idt(tt._trigon1, tt._trigon2);
  
  outpwm.setReg(idt.u_ini, idt.v_ini, idt.w_ini);
}

bool UserTask::servoCheck() {
  CanCom::CanData* candata = cancom.getData();
  return (candata->genFuncRef & 0x01) != 0 ? true : false;
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
  usertask.cyclicTask();
}
