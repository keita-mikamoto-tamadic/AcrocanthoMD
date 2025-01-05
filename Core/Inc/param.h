#pragma once

#include <cstdint>

#define AS5600ADDR static_cast<int16_t>(0x36 << 1)

#define VOLT_PBM            (24.0f)       /* Power */
#define CCLK_Hz             (170.0e6)
#define CCR_MAX             (8499)
#define CCR_END             ((CCR_MAX * 2)-1)
// 三角波なのでカウント数の2倍が1周期分
#define TASK_TIME           ((1/CCLK_Hz)*CCR_END)
#define DUTY_BASE           (0.5F)
