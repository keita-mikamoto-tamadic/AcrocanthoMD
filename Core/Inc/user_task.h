#pragma once

#include <cstdint>

constexpr float TaskTime = 0.0001f;

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
    STEP11,
    STEP12,
    STEP13,
    STEP14,
    STEP15,
    STEP16,
    STEP17,
    STEP18,
    STEP19,
    STEP20,
    STEP21,
    STEP22,
    STEP23,
    STEP24,
    STEP25,
    STEP26,
    STEP27,
    STEP28,
    STEP29,
    STEP30,
    LOOP,
    RUN,
    INIT,
    IDLE,
    FAIL,
    END
} SeqID_t;

class userTask {
public:
  userTask();

  void cyclic_task();
  void idle_task();

private:
  uint8_t count = 0;
  SeqID_t seqID = STEP00;

};
