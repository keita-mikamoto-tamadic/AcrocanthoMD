#pragma once

#include <cstdint>
#include "main.h"

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
    TEST,
    TESTCONST,
    TESTSINGLE
} SeqID_t;

class UserTask {
public:

private:
  // テストデータ構造体（統一管理）
  struct TestData {
    bool initEnd = false;
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
    float testmechpos = 0.0f;
  };

  // 状態管理構造体
  struct StateData {
    bool toggleState = false;
    GPIO_PinState prevB1State = GPIO_PIN_RESET;
    uint8_t onewatect = 0;
    bool canExecute = true;
    uint8_t stableCount = 0;
    bool needTurnOff = false;
    uint8_t execCount = 0;
    static constexpr uint8_t stableThreshold = 10;
    static constexpr uint8_t requiredExecs = 100;
  };

  TestData testData;
  StateData stateData;
  uint8_t count = 0;
  bool servocheck = false;
  SeqID_t seqID = STEP00;

  // センサ読み取り統一関数
  void readSensors();
  // テストデータ収集（デバッグ時のみ）
  void collectTestData();
  // ボタン処理共通関数
  GPIO_PinState handleButtonInput();

  void setRef();
  void motorControl();
  void resetAll();
  
public:
  UserTask();
  void cyclicTask();
  void idleTask();
  bool servoCheck();
  void canDataPrepare();

};
