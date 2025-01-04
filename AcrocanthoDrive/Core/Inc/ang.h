#pragma once

#include <cstdint>

#include "stm32g4xx_hal.h"

// I2Cアドレスのおまじない。アドレスを16bitにキャストしなおして1bit左シフト
constexpr int16_t AS5600ADDR = static_cast<int16_t>(0x36 << 1);

class Ang {
  public:
    Ang(I2C_HandleTypeDef& hi2c1);

    void read();
    void receive();
    float getAngle() const;
    
  private:
    I2C_HandleTypeDef& i2cHandle_;
    bool readStart_;
    float actAngle_;
    uint8_t rawAng_[2];

    void resetFlags();

};

// globalScope rxtxflag
extern volatile uint8_t i2c_tx_complete;
extern volatile uint8_t i2c_rx_complete;
