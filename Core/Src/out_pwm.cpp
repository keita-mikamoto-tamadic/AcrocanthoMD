#include "out_pwm.h"

#include "main.h"
#include "user_math.h"
#include "param.h"
#include "util.h"

OutPwm outpwm;

void OutPwm::Pon(){
  setReg(DUTY_BASE, DUTY_BASE, DUTY_BASE);
}

void OutPwm::Poff(){
  TIM8->CCR1 = 0; 
  TIM8->CCR2 = 0; 
  TIM8->CCR3 = 0; 

}

void OutPwm::setReg(float u, float v, float w){
  calculateMidVoltage(u, v, w);
  setCCR(dutyGuard(u), dutyGuard(v), dutyGuard(w));
}

void OutPwm::TEST_setReg(float u, float v, float w){
  setCCR(u, v, w);
}

void OutPwm::TESTSINGLE_setReg(uint8_t phase){
  constexpr float DUTY_VAL = DUTY_BASE;
  
  switch (phase) {
    case 0:
      setCCR(DUTY_VAL, 0.0f, 0.0f);
      break;
    case 1:
      setCCR(0.0f, DUTY_VAL, 0.0f);
      break;
    case 2:
      setCCR(0.0f, 0.0f, DUTY_VAL);
      break;
    default:
      setCCR(0.0f, 0.0f, 0.0f);
      break;
  }
}

float OutPwm::dutyGuard(float rawDuty){
  const float powerSwitch = std::max(VOLT_PBM, VOLT_GUARD);
  
  // 中間電位差し引く
  const float normalizedDuty = ((rawDuty - midvol) / powerSwitch) + DUTY_BASE;
  const float lowerLimit = 1.0f - DUTY_GUARD;
  
  // クランプ処理（C++14対応）
  if (normalizedDuty < lowerLimit) {
    return lowerLimit;
  }
  else if (normalizedDuty > DUTY_GUARD) {
    return DUTY_GUARD;
  }
  else {
    return normalizedDuty;
  }
}

void OutPwm::calculateMidVoltage(float u, float v, float w){
  // 3相の最大値と最小値を効率的に算出
  const float maxvol = std::max({u, v, w});
  const float minvol = std::min({u, v, w});
  
  // 中間電位を算出
  midvol = (maxvol + minvol) * 0.5f;
}

// CCR設定の共通化
void OutPwm::setCCR(float u, float v, float w){
  TIM8->CCR1 = static_cast<uint16_t>((1.0f - u) * static_cast<float>(CCR_MAX));
  TIM8->CCR2 = static_cast<uint16_t>((1.0f - v) * static_cast<float>(CCR_MAX));
  TIM8->CCR3 = static_cast<uint16_t>((1.0f - w) * static_cast<float>(CCR_MAX));
}