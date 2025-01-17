#pragma once

#include <cstdint>
#include <main.h>

#include "stm32g4xx_ll_cordic.h"

constexpr float user2pi = 6.283185307179586f;
constexpr float userpi = 3.141592653589793f;
constexpr float absqrt1 = 1.224745f;    // route(2/3) * 3/2
constexpr float absqrt2 = 0.7071068f;   // route(2/3) * route(3) / 2
constexpr float absqrt3 = 1.4142136f;   // route(2/3) * route(3)
constexpr float invsqrt1 = 0.81649658f; // route(2/3) = 0.81649658f
constexpr float invsqrt2 = 0.70710679f; // route(2/3) * route(3) / 2 = 0.70710679f
constexpr float invsqrt3 = 0.40824829f; // route(2/3) / 2 = 0.40824829f

namespace Acrocantho {

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

// 3相→αβ(Clark)変換
struct ClarkeTransform {
  ClarkeTransform(float curU, float curV, float curW)
    : alpha(absqrt1 * curU),
      beta((absqrt2 * curU) + (absqrt3 * curV)) {}
  
  const float alpha;
  const float beta;
};

// αβ→dq(Park)変換
struct ParkTransform {
  ParkTransform(const SinCos& sc, float alpha, float beta)
    : id(sc.c * alpha + sc.s * beta),
      iq(-sc.s * alpha + sc.c * beta) {}
  
  const float id;
  const float iq;
};

// 逆変換準備(逆Park変換)
struct TrigonTransform {
  /* f4t_trigon1 : V_d*cos - V_q*sin */
  /* f4t_trigon2 : V_d*sin + V_q*cos */
  TrigonTransform(const SinCos& sc, float vd, float vq)
    : _trigon1(sc.c * vd - sc.s * vq),
      _trigon2(sc.s * vd + sc.c * vq) {}
  
  const float _trigon1;
  const float _trigon2;
};

// 逆Clarke変換
struct InverseClarkeTransform {
  InverseClarkeTransform(float _tri1, float _tri2)
      : u_ini(_tri1 * invsqrt1),
        v_ini(-(_tri1 * invsqrt3) + _tri2 * invsqrt2),
        w_ini(-(_tri1 * invsqrt3) - _tri2 * invsqrt2) {}

  const float u_ini;
  const float v_ini;
  const float w_ini;
};

} // Acrocantho
