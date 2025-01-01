#pragma once

#include <cstdint>
#include <memory>

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
  UserTask();
  
  struct UserTaskData {
    uint8_t genFuncRef = 0;
    uint8_t drvMdRef = 0;
    float voltDRef = 0;
    float voltQRef = 0;
    float virAngFreq = 0;
  };

  void cyclicTask();
  void idleTask();
  void motorControl();

  // getter
  UserTaskData* getData() { return data.get(); }

private:
  std::unique_ptr<UserTaskData> data;

  uint8_t count = 0;
  bool servocheck = false;
  SeqID_t seqID = STEP00;

  bool servoCheck();
  void setRef();
};
