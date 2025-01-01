#include "out_pwm.h"

#include "main.h"
#include "user_math.h"
#include "param.h"
#include "can_communication.h"

OutPwm outpwm;

OutPwm::OutPwm()
    : data(std::make_unique<outPwmData>()) {}

void OutPwm::Pon(){
  setReg(DUTY_BASE, DUTY_BASE, DUTY_BASE);
}

void OutPwm::Poff(){
  TIM1->CCR1 = 0; 
  TIM1->CCR2 = 0; 
  TIM1->CCR3 = 0; 
}

void OutPwm::setReg(float u, float v, float w){

  TIM1->CCR1 = (uint16_t)((1.0f - dutyGuard(u)) * (float)CCR_MAX);
  TIM1->CCR2 = (uint16_t)((1.0f - dutyGuard(v)) * (float)CCR_MAX);
  TIM1->CCR3 = (uint16_t)((1.0f - dutyGuard(w)) * (float)CCR_MAX);
}


float OutPwm::dutyGuard(float _rawDuty){
    float result = 0.0f;
    float sum = 0.0f;
    static float limp = 0.95f;
    static float limm = 0.01f;
    
    sum = (_rawDuty / VOLT_PBM) + DUTY_BASE;
    
    if (sum > limp){
        result = limp;
    }else if(sum < limm){
        result = limm;
    }else{
        result = sum;
    }
    return result;
}
