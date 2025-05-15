#pragma once

#include "main.h"
#include <cstdint>
#include <memory>

// コンパイルスイッチ用マクロ定義


class Util {
public:
  struct UtilData {
    bool eCalib = false;
    bool endECalib = false;
    bool sixPtRotation = false;
  };

private:
  std::unique_ptr<UtilData> data;

public:
  Util();
  
  void genFuncCtrl();

  UtilData* getUtilData() const { return data.get(); }
};
