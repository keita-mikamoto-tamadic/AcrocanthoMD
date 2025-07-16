#pragma once

#include "main.h"
#include <cstdint>

// 機能制御用ビットマスク定数
constexpr uint8_t GENFUNC_ECALIB_MASK = 0x10;
constexpr uint8_t GENFUNC_SIXPT_MASK = 0x20;


class Util {
public:
  struct UtilData {
    bool eCalib = false;
    bool endECalib = false;
    bool sixPtRotation = false;
  };

private:
  UtilData data;

public:
  Util();
  
  void genFuncCtrl();

  UtilData* getUtilData() { return &data; }
};
