#pragma once

#include <memory>
#include "main.h"
#include "user_math.h"

class OutPwm{

private:

  float dutyGuard(float _rawDuty);

public:
  OutPwm();
  void Pon();
  void Poff();
  void setReg(float u, float v, float w);
  
};
