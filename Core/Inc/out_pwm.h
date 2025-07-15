#pragma once

#include "main.h"
#include "user_math.h"
#include "param.h"
#include <cstdint>
#include <algorithm>

class OutPwm{

private:
  // PWM設定定数
  static constexpr float DUTY_GUARD = 0.99f;
  static constexpr float VOLT_GUARD = VOLT_PBM;
  
  // 中間電位計算用変数
  float midvol = 0.0f;
  
  // 内部メソッド
  float dutyGuard(float rawDuty);
  void calculateMidVoltage(float u, float v, float w);
  void setCCR(float u, float v, float w);
  
public:
  OutPwm() = default;
  void Pon();
  void Poff();
  void setReg(float u, float v, float w);
  void TEST_setReg(float u, float v, float w);
  void TESTSINGLE_setReg(uint8_t phase);
  
};
