#pragma once

#include <cstdint>
#include <memory>
#include "main.h"

class CanCom {
public:
  struct CanData {
    uint8_t genFuncRef = 0;
    uint8_t drvMdRef = 0;
    float voltDRef = 0;
    float voltQRef = 0;
    float virAngFreq = 0;

    bool genFuncCheck = false;
    uint8_t txBuff[8] = {0};
    uint8_t rxBuff[8] = {0};
  };

private:
  std::unique_ptr<CanData> data;

  FDCAN_HandleTypeDef& hfdcan;
  FDCAN_TxHeaderTypeDef txHeader;
  FDCAN_RxHeaderTypeDef rxHeader;
  uint8_t txData[8];
  uint8_t rxData[8];
  uint8_t prevGenFuncRef;
  volatile bool canRxInterrupt;

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

  CanData* getData() { return data.get(); }
};

