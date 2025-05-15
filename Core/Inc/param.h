#pragma once

#include <cstdint>
#include "user_math.h"

#define AS5600ADDR static_cast<int16_t>(0x36 << 1)

#define VOLT_PBM            (24.0f)       /* Power */
//#define VOLT_PBM            (38.0f)       /* Power */
#define CCLK_Hz             (170.0e6)
#define CCR_MAX             ((uint16_t)16998)
#define CCR_END             ((CCR_MAX * 2)-1)
#define TASK_TIME           ((1/CCLK_Hz)*CCR_END) // 三角波なのでカウント数の2倍が1周期分
#define DUTY_BASE           (0.5f)

// モーター定義
//#define GIM6010_8
#define GIM8108_8

// 極対数
#ifdef GIM6010_8
#define POLE_PAIRS          (14)
#endif

#ifdef GIM8108_8
#define POLE_PAIRS          (21)
#endif

// 電気角オフセット
#ifdef GIM6010_8
#define EOFS               (3.3160305f)
#endif

#ifdef GIM8108_8
#define EOFS           (5.655f)
#endif

// ギア比
#define GR_RATIO            (0.125f)
