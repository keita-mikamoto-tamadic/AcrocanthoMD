#pragma once

#include <cstdint>
#include <memory>
#include "main.h"

struct canData {
  uint8_t genFuncRef = 0;
  uint8_t txBuff[8] = {0};
  uint8_t rxBuff[8] = {0};
};

class CanCom {
private:
  FDCAN_HandleTypeDef& hfdcan;
  FDCAN_TxHeaderTypeDef txHeader;
  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t txData[8];
  uint8_t rxData[8];
  uint8_t prevGenFuncRef;
  volatile bool canRxInterrupt;

  std::unique_ptr<canData> data;

public:
  volatile bool canTxFlag;

  // Constracta
  CanCom(FDCAN_HandleTypeDef& fdcanHandle);

  void initTxHeader(uint32_t canId, bool extendedId = false, bool fdFormat = false);
  void sendData(const uint8_t* data, size_t size);
  void rxFifo0Callback(uint32_t RxFifo0ITs);
  void handleRxData();
  void rxTask();
  void txTask();
};

