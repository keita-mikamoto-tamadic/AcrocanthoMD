#pragma once

#include <cstdint>

typedef enum {
    STEP00 = 0U,
    STEP01,
    STEP02,
    STEP03,
    STEP04,
    STEP05,
    STEP06,
    STEP07,
    STEP08,
    STEP09,
    STEP10,
    LOOP,
    RUN,
    INIT,
    IDLE,
    FAIL,
    END
} SeqID_t;

class UserTask {
public:

private:
  float test = 0.0f;
  float testid = 0.0f;
  float testiq = 0.0f;

  uint8_t count = 0;
  bool servocheck = false;
  SeqID_t seqID = STEP00;

  void setRef();
  void motorControl();
  void resetAll();
  
public:
  UserTask();
  void cyclicTask();
  void idleTask();
  bool servoCheck();

};
