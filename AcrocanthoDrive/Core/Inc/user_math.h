#pragma once

#include <cstdint>
#include <main.h>

#include "stm32g4xx_ll_cordic.h"

namespace Acrocantho {

// constexpr
constexpr float user2pi = 6.283185307179586f;
constexpr float userpi = 3.141592653589793f;

struct SinCos {
  float s;
  float c;
};

int32_t floatToQ31(float) __attribute__((always_inline));

inline int32_t floatToQ31(float x) {
  const float scaled = x / user2pi;
  const int32_t i = static_cast<int>(scaled);
  float mod = scaled - i;
  if (mod < 0) { mod += 1.0f; }

  return static_cast<int32_t>(((mod > 0.5f) ? (mod - 1.0f) : mod) * 4294967296.0f);
}

class Cordic {
  public:
  Cordic() {
    __HAL_RCC_CORDIC_CLK_ENABLE();
    LL_CORDIC_Config(
        CORDIC,
        LL_CORDIC_FUNCTION_COSINE,
        LL_CORDIC_PRECISION_5CYCLES,
        LL_CORDIC_SCALE_0,
        LL_CORDIC_NBWRITE_1,
        LL_CORDIC_NBREAD_2,
        LL_CORDIC_INSIZE_32BITS,
        LL_CORDIC_OUTSIZE_32BITS);
  }

  SinCos operator()(int32_t theta_q31) const {
    LL_CORDIC_WriteData(CORDIC, theta_q31);
    SinCos result;
    result.c = from_q31(LL_CORDIC_ReadData(CORDIC));
    result.s = from_q31(LL_CORDIC_ReadData(CORDIC));
    return result;
  };
  
  SinCos radians(float theta) const {
    return (*this)(floatToQ31(theta));
  }

  static float from_q31(uint32_t val) {
    return static_cast<float>(static_cast<int32_t>(val)) * (1.0f / 2147483648.0f);
  }
};

} // Acrocantho
