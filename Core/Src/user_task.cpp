#include "user_task.h"

#include "main.h"
#include "ang.h"
#include "out_pwm.h"
#include "can_communication.h"

UserTask usertask;

extern Ang ang;
extern OutPwm outpwm;
extern CanCom cancom;

UserTask::UserTask()
  : count(0){}


void UserTask::cyclicTask() {
  static SeqID_t seqID = STEP00;

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
        ang.getAngle();
        count = 0;
        seqID = STEP00;
      }
      break;
    case STEP00:
      ang.getAngle();
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
  cancom.initTxHeader(0x01, false, false);
  cancom.rxTask();
  cancom.txTask();
  servocheck = servoCheck();
}

void UserTask::motorControl() {
}

// Poffのみfalseを返す
bool UserTask::servoCheck() {
  CanCom::canData* candata = cancom.getData();
  
  switch (candata->genFuncRef) {
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
