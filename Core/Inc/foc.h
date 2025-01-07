#pragma once

#include <cstdint>
#include <memory>
#include "user_math.h"


class Foc {
public:
  struct FocData {
    float id = 0.0f;
    float iq = 0.0f;
    float vu = 0.0f;
    float vv = 0.0f;
    float vw = 0.0f;
  };

private:
  std::unique_ptr<FocData> data;

public:
  Foc();
  void forwardCtrl(const Acrocantho::SinCos _sc);
  void inverseCtrl(const Acrocantho::SinCos _sc, float _vd, float _vq);
  FocData* getData() const { return data.get(); }
};
