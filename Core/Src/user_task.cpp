#include "user_task.h"

#include "main.h"
#include "ang.h"
#include "out_pwm.h"
#include "can_communication.h"
#include "mode_control.h"
#include "sens_cur.h"

UserTask usertask;

extern Ang ang;
extern SensCur senscur;
extern OutPwm outpwm;
extern CanCom cancom;
extern ModeControl modecontrol;

UserTask::UserTask()
  : count(0), data(std::make_unique<UserTaskData>()){}


void UserTask::cyclicTask() {
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
  setRef();

  cancom.initTxHeader(0x01, false, false);
  cancom.txTask();
  servocheck = servoCheck();
}

void UserTask::motorControl() {
  Ang::AngData* angdata = ang.getAngData();
  Acrocantho::Cordic cordic;

  // drvMd0のとき電圧を0にする
  modecontrol.modeCtrl(data->drvMdRef);
  
  senscur.sensCurIN();
  
  // SinCos演算
  Acrocantho::SinCos result = cordic.radians(angdata->elecAng);
  float s = result.s;
  float c = result.c;
  
  // dq逆変換
  Acrocantho::TrigonTransform tt(result, data->voltDRef, data->voltQRef);
  Acrocantho::InverseDqTransform idt(tt._trigon1, tt._trigon2);
  
  outpwm.setReg(idt.u_ini, idt.v_ini, idt.w_ini);
}

// Canで受け取った指令値のセット
void UserTask::setRef() {
  CanCom::CanData* candata = cancom.getData();
  
  data->genFuncRef = candata->genFuncRef;
  data->drvMdRef = candata->drvMdRef;
  data->voltDRef = static_cast<float>(candata->voltDRef);
  data->voltQRef = static_cast<float>(candata->voltQRef);
  data->virAngFreq = static_cast<float>(candata->virAngFreq);
  
}

// Poffのみfalseを返す
bool UserTask::servoCheck() {
  switch (data->genFuncRef) {
    case 0x00:
      return false;
    case 0x01:
      return true;
    default:
      return false;
  }
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
  usertask.cyclicTask();
}
