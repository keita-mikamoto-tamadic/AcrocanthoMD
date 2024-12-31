#pragma once

#include <cstdint>

#define AS5600ADDR static_cast<int16_t>(0x36 << 1)

#define VOLT_PBM            (24.0f)       /* Power */
#define CONST_VOLT2PWM_f4_1 (0.81649658F) /* route(2/3) = 0.81649658f */
#define CONST_VOLT2PWM_f4_2 (0.70710679F) /* route(2/3) * route(3) / 2 = 0.70710679f */
#define CONST_VOLT2PWM_f4_3 (0.40824829F) /* route(2/3) / 2 = 0.40824829f */
#define CCLK_Hz             (170.0e6)
// CCR_MAX 三角波なのでカウント数の2倍が1周期分
#define CCR_MAX             (8499)
#define TASK_TIME           ((1/CCLK_Hz)*CCR_MAX)
#define DUTY_BASE           (0.5F)