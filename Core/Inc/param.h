#pragma once

#include <cstdint>
#include "user_math.h"

#define AS5600ADDR static_cast<int16_t>(0x36 << 1)

// キャリア周波数
// キャリア周波数の変更時は、USER_CARIER_COUNTの値を変更すること
#define CARIER_COUNT_20kHz (4249)
#define CARIER_COUNT_10kHz (8499)
#define CARIER_COUNT_5kHz  (16999)

/* ****************ユーザー修正範囲**************** */

// ユーザー修正可能なキャリア周波数、↑に定義したマクロを使用
#define USER_CARIER_COUNT  (CARIER_COUNT_20kHz)

// ユーザー修正可能なデッドタイム(0~255カウント)
#define USER_DEADTIME       (180)

// ユーザー修正可能な電圧レンジの設定。通常は電源電圧を設定
#define VOLT_PBM            (48.0f)       /* Power */

/* ********************************************** */

// 以下ユーザー修正不可
#define CCLK_Hz             (170.0e6)
#define CCR_MAX             ((uint16_t)USER_CARIER_COUNT)
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
