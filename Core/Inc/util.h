#pragma once

#include "main.h"
#include <cstdint>
#include <memory>

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
