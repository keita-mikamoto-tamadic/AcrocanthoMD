#pragma once

#include "main.h"
#include <cstdint>
#include <memory>

// コンパイルスイッチ用マクロ定義
#define PWMTIM_NO_8

//#define TEST_MODE

class Util {
public:
  struct UtilData {
    bool eCalib = false;
  };

private:
  std::unique_ptr<UtilData> data;

public:
  Util();
  
  void genFuncCtrl();

  UtilData* getUtilData() const { return data.get(); }
};
