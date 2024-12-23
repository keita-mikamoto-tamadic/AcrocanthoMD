#pragma once

#include <cstdint>
#include "main.h"

class CanCom {
private:
  FDCAN_HandleTypeDef& hfdcan;
  FDCAN_TxHeaderTypeDef txHeader;
  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t txData[8];
  uint8_t rxData[8];
  uint8_t prevGenFuncRef;

public:
  volatile int8_t canRxInterrupt;
  struct CanData {
    uint8_t genFuncRef;
  } canData;

  // Constracta
  CanCom(FDCAN_HandleTypeDef& fdcanHandle);

  void initTxHeader(uint32_t canId, bool extendedId = false, bool fdFormat = false);
  void sendData(const uint8_t* data, size_t size);
  void rxFifo0Callback(uint32_t RxFifo0ITs);
  void handleRxData();
  void rxTask();
};

