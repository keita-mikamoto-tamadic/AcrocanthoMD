#pragma once

#include <cstdint>
#include "user_math.h"

#define AS5600ADDR static_cast<int16_t>(0x36 << 1)

#define VOLT_PBM            (24.0f)       /* Power */
//#define VOLT_PBM            (48.0f)       /* Power */
#define CCLK_Hz             (170.0e6)
#define CCR_MAX             ((uint16_t)8500)
#define CCR_END             ((CCR_MAX * 2)-1)
#define TASK_TIME           ((1/CCLK_Hz)*CCR_END) // 三角波なのでカウント数の2倍が1周期分
#define DUTY_BASE           (0.5f)

// 電流センサ定義
//#define TMCS1107A1B
#define AD8418A

// CANID
constexpr uint32_t canDevID = 2;

// モーター定義
#define GIM6010_8
//#define GIM8108_8

// 極対数
#ifdef GIM6010_8
#define POLE_PAIRS          (14)
#endif

#ifdef GIM8108_8
#define POLE_PAIRS          (21)
#endif

// 電気角オフセット
#ifdef GIM6010_8
// ID 2
#define EOFS                (0.221829f)
// ID 4
//#define EOFS               (0.42152f)
#endif

#ifdef GIM8108_8
// ID 1
//#define EOFS           (0.339161634f)
// ID 3
#define EOFS           (2.7897923f)
#endif

// ギア比
#define GR_RATIO            (0.125f)
