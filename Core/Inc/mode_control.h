#pragma once

#include <cstdint>
#include <memory>


#define CTRLMODE_NONE 0U
#define CTRLMODE_VOLT 6U
#define CTRLMODE_CUR 7U
#define CTRLMODE_VEL 8U
#define CTRLMODE_POS 9U

class ModeControl {
public:
  struct ModeControlData {
    uint8_t genFuncRef = 0;
    uint8_t drvMdRef = 0;
    float voltDRef = 0;
    float voltQRef = 0;
    float virAngFreq = 0;
    float vel = 0.0f;
    float posout = 0.0f;
  };
  
private:
  std::unique_ptr<ModeControlData> data;
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
  
  ModeControlData* getData() { return data.get(); }

};