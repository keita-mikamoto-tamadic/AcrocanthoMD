#pragma once

#include <cstdint>

// 制御モード定数
static constexpr uint8_t CTRLMODE_NONE = 0U;
static constexpr uint8_t CTRLMODE_VOLT = 6U;
static constexpr uint8_t CTRLMODE_CUR = 7U;
static constexpr uint8_t CTRLMODE_VEL = 8U;
static constexpr uint8_t CTRLMODE_POS = 9U;

class ModeControl {
public:
  struct ModeControlData {
    uint8_t genFuncRef = 0;
    uint8_t drvMdRef = 0;
    float voltDRef = 0.0f;
    float voltQRef = 0.0f;
    float virAngFreq = 0.0f;
    float vel = 0.0f;
    float posout = 0.0f;
  };
  
private:
  ModeControlData data;
  float s_voltQRef = 0.0f;
  float s_voltDRef = 0.0f;
  float s_virAngFreq = 0.0f;
  float s_curDRef = 0.0f;
  float s_curQRef  = 0.0f;
  float s_velRef = 0.0f;
  float s_posRef = 0.0f;
  void refCtrl();

public:
  ModeControl();
  void modeCtrl();
  void modeCtrlReset();
  
  ModeControlData* getData() { return &data; }

private:
  void limitVoltage(float& voltDRef, float& voltQRef);
};