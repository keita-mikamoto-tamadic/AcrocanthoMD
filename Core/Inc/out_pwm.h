#pragma once

#include <memory>
#include "main.h"
#include "user_math.h"


struct outPwmData {
  float dutyU = 0.0f;
  float dutyV = 0.0f;
  float dutyW = 0.0f;
};

class OutPwm{
private:
  
  float pwmDutyU = 0.0f;
  float pwmDutyV = 0.0f;
  float pwmDutyW = 0.0f;

  float dutyGuard(float _rawDuty);
  float limp = 0.01f;
  float limm = 0.95f;
  
  void setReg(float u, float v, float w);
  

  std::unique_ptr<outPwmData> data;

public:
  OutPwm();
  void Pon();
  void Poff();
  void outPwmSeq();

};
