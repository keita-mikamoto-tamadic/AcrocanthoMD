#include "out_pwm.h"

#include "main.h"
#include "user_math.h"
#include "param.h"
#include "util.h"

constexpr float DutyGuard = 0.99f;
constexpr float VoltGuard = 24.0f;

OutPwm outpwm;

OutPwm::OutPwm(){}

void OutPwm::Pon(){
  setReg(DUTY_BASE, DUTY_BASE, DUTY_BASE);
}

void OutPwm::Poff(){
#if defined(PWMTIM_NO_1)
  //TIM1->CCR1 = 0; 
  //TIM1->CCR2 = 0; 
  //TIM1->CCR3 = 0; 

#elif defined(PWMTIM_NO_8)
  TIM8->CCR1 = 0; 
  TIM8->CCR2 = 0; 
  TIM8->CCR3 = 0; 
#endif

}

void OutPwm::setReg(float u, float v, float w){
  midVol(u, v, w);

#if defined(PWMTIM_NO_1)
  TIM1->CCR1 = (uint16_t)((1.0f - dutyGuard(u)) * (float)CCR_MAX);
  TIM1->CCR2 = (uint16_t)((1.0f - dutyGuard(v)) * (float)CCR_MAX);
  TIM1->CCR3 = (uint16_t)((1.0f - dutyGuard(w)) * (float)CCR_MAX);

#elif defined(PWMTIM_NO_8)
  TIM8->CCR1 = (uint16_t)((1.0f - dutyGuard(u)) * (float)CCR_MAX);
  TIM8->CCR2 = (uint16_t)((1.0f - dutyGuard(v)) * (float)CCR_MAX);
  TIM8->CCR3 = (uint16_t)((1.0f - dutyGuard(w)) * (float)CCR_MAX);
#endif
}

void OutPwm::TEST_setReg(float u, float v, float w){

#if defined(PWMTIM_NO_1)
  TIM1->CCR1 = (uint16_t)((1.0f - u) * (float)CCR_MAX);
  TIM1->CCR2 = (uint16_t)((1.0f - v) * (float)CCR_MAX);
  TIM1->CCR3 = (uint16_t)((1.0f - w) * (float)CCR_MAX);

#elif defined(PWMTIM_NO_8)
  TIM8->CCR1 = (uint16_t)((1.0f - u) * (float)CCR_MAX);
  TIM8->CCR2 = (uint16_t)((1.0f - v) * (float)CCR_MAX);
  TIM8->CCR3 = (uint16_t)((1.0f - w) * (float)CCR_MAX);
#endif
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