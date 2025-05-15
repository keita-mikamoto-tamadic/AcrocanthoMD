#pragma once

#include <memory>
#include "main.h"
#include "user_math.h"

class OutPwm{

private:

  float dutyGuard(float _rawDuty);
  void midVol(float u, float v, float w);
  float maxvol = 0.0f;
  float midvol = 0.0f;
  float minvol = 0.0f;

public:
  OutPwm();
  void Pon();
  void Poff();
  void setReg(float u, float v, float w);
  void TEST_setReg(float u, float v, float w);
  void TESTSINGLE_setReg(uint8_t phase);
  
};
