#pragma once

#include <cstdint>
#include <memory>

typedef enum {
  CTRLMODE_NONE = 0U,
  CTRLMODE_VOLT,
  CTRLMODE_CUR,
  CTRLMODE_VEL,
  CTRLMODE_POS
} st_mode;

class ModeControl {
public:
  struct ModeControlData {
    uint8_t genFuncRef = 0;
    uint8_t drvMdRef = 0;
    float voltDRef = 0;
    float voltQRef = 0;
    float virAngFreq = 0;
  };
  
private:
  std::unique_ptr<ModeControlData> data;
  uint8_t s_drvMdRef = 0;
  float s_voltQRef = 0.0f;
  float s_voltDRef = 0.0f;
  float s_virAngFreq = 0.0f;
  st_mode mode = CTRLMODE_NONE;
  void refCtrl();


public:
  ModeControl();
  void modeCtrl();
  void modeCtrlReset();
  
  ModeControlData* getData() { return data.get(); }

};