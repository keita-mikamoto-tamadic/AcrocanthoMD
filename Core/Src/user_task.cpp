#include "user_task.h"

#include "main.h"
#include "ang.h"

// main.cpp で定義されたインスタンスを使用
userTask usertask;

extern Ang ang;

userTask::userTask()
  : count(0){}


void userTask::cyclic_task() {
   switch (seqID) {
    case LOOP:
      ang.getAngle();
      ang.getVel();
      break;
    case STEP00:
      // 初期化のためにエンコーダ値の初回読み取り
      if (count < 10) {
        ang.getAngle();
        count++;
      } else {
        count = 0;
        seqID = LOOP;
      }

      break;
    default:
      seqID = STEP00;
      break;
    }
}

void userTask::idle_task() {
  
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
  HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_9);
  usertask.cyclic_task();
}
