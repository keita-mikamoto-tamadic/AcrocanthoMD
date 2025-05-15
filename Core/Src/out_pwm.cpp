#include "out_pwm.h"

#include "main.h"
#include "user_math.h"
#include "param.h"
#include "util.h"

constexpr float DutyGuard = 0.99f;
constexpr float VoltGuard = VOLT_PBM;

OutPwm outpwm;

OutPwm::OutPwm(){}

void OutPwm::Pon(){
  setReg(DUTY_BASE, DUTY_BASE, DUTY_BASE);
}

void OutPwm::Poff(){
  TIM8->CCR1 = 0; 
  TIM8->CCR2 = 0; 
  TIM8->CCR3 = 0; 

}

void OutPwm::setReg(float u, float v, float w){
  midVol(u, v, w);

  TIM8->CCR1 = (uint16_t)((1.0f - dutyGuard(u)) * (float)CCR_MAX);
  TIM8->CCR2 = (uint16_t)((1.0f - dutyGuard(v)) * (float)CCR_MAX);
  TIM8->CCR3 = (uint16_t)((1.0f - dutyGuard(w)) * (float)CCR_MAX);
}

void OutPwm::TEST_setReg(float u, float v, float w){

  TIM8->CCR1 = (uint16_t)((1.0f - u) * (float)CCR_MAX);
  TIM8->CCR2 = (uint16_t)((1.0f - v) * (float)CCR_MAX);
  TIM8->CCR3 = (uint16_t)((1.0f - w) * (float)CCR_MAX);
}

void OutPwm::TESTSINGLE_setReg(uint8_t phase){
  switch (phase) {
    case 0:
      TIM8->CCR1 = (uint16_t)((1.0f - DUTY_BASE) * (float)CCR_MAX);
      TIM8->CCR2 = 0;
      TIM8->CCR3 = 0;
      break;
    case 1:
      TIM8->CCR1 = 0;
      TIM8->CCR2 = (uint16_t)((1.0f - DUTY_BASE) * (float)CCR_MAX);
      TIM8->CCR3 = 0;
      break;
    case 2:
      TIM8->CCR1 = 0;
      TIM8->CCR2 = 0;
      TIM8->CCR3 = (uint16_t)((1.0f - DUTY_BASE) * (float)CCR_MAX);
      break;
    default:
      break;
  }

}

float OutPwm::dutyGuard(float _rawDuty){
  float result_ = 0.0f;
  float limp = 0.0f;
  float limm = 0.0f;
  float powerSwitch_ = 0.0f;
  
  if (VOLT_PBM >= VoltGuard) {
    powerSwitch_ = VOLT_PBM;
  } else {
    powerSwitch_ = VoltGuard;
  }
  
  // 中間電位差し引く
  limp = ((_rawDuty - midvol) / powerSwitch_) + DUTY_BASE;
  limm = 1.0f - DutyGuard;
  
  if (limp > DutyGuard){
    result_ = DutyGuard;
  } else if (limp < limm){
    result_ = limm;
  } else {
    result_ = limp;
  }
  
  return result_;
}

void OutPwm::midVol(float u_, float v_, float w_){
  // 3相の中で最大電圧を算出
  maxvol = u_;
  if (v_ > maxvol) maxvol = v_;
  if (w_ > maxvol) maxvol = w_;
  
  // 3相の中で最小電圧を算出
  minvol = u_;
  if (v_ < minvol) minvol = v_;
  if (w_ < minvol) minvol = w_;
  
  // 中間電位を算出
  midvol = (maxvol + minvol) / 2.0f;
}