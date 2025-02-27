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
    END,
    TEST
} SeqID_t;

class UserTask {
public:

private:
  // test
  float test = 0.0f;
  float eofsp = 0.0f;
  float eofsm = 0.0f;
  float testid = 0.0f;
  float testiq = 0.0f;
  float testvd = 0.0f;
  float testvq = 0.0f;
  float testpos = 0.0f;
  float testvel = 0.0f;
  float testelec = 0.0f;
  float testvelact = 0.0f;
  float testerrQ = 0.0f;
  float testerrD = 0.0f;
  float testCurU = 0.0f;
  float testCurW = 0.0f;
  uint32_t testcomp = 0;
  uint16_t testrawAng = 0;
  uint16_t testrawAngPast = 0;
  int16_t testdiff = 0;
  float testvelerr = 0.0f;
  float testvelerrsum = 0.0f;
  float testposerr = 0.0f;
  float testposerrsum = 0.0f;
  float testposout = 0.0f;
  // test

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
