#pragma once

#include <cstdint>
#include <memory>
#include "user_task.h"

#define CALIB_NUM       (60)                           /* 分割数 [-] */

class ElecangCalib
{
public:
  struct ElecangCalibData
  {
    float elecAngOfsPlus = 0.0f;
    float elecAngOfsMinus = 0.0f;
    uint8_t drvMd = 0;
    float voltQRef = 0.0f;
    float elecAngOfs = 0.0f;

  };

private:
  std::unique_ptr<ElecangCalibData> data;

  SeqID_t seqIDSub = STEP00;

  float elecAngOfsRP = 0.0f;
  float elecAngOfsRM = 0.0f;
  float elecAngOfsFP = 0.0f;
  float elecAngOfsFM = 0.0f;
  float tuneDiff = 0.0f;
  int32_t idleCount = 0;
  

  bool calibSub(float _voltDRef, float _elecAngOfsCur, float *_elecAngOfsMax, float _calDelta);

  int32_t count = 0;
  float elecAngOfsVal = 0.0f;
  uint32_t indexnum = 0.0f;
  float velOutAxLast = 0.0f;
  float velOut[CALIB_NUM] = {0.0f};
  float elecAngOfs[CALIB_NUM] = {0.0f};

public:
  ElecangCalib();
  
  void elecCalSeq();

  ElecangCalibData* getData() const { return data.get(); }
};