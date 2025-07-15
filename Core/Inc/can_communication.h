#pragma once

#include <cstdint>
#include <memory>
#include "main.h"
#include "byte_converter.h"

constexpr uint32_t canDevID = 1;
constexpr uint8_t canRxSize = 32;
constexpr uint8_t canTxSize = 32;

class CanCom {
public:
  struct CanData {
    // Ref
    int32_t genFuncRef = 0;
    float voltDRef = 0;
    float voltQRef = 0;
    float virAngFreq = 0;
    float curQRef = 0;
    float curDRef = 0;
    float velRef = 0.0f;
    float posRef = 0.0f;
    
    uint16_t cmdRef = 0;
    uint32_t txMsgRef = 0;
    
    // Act
    float actPos = 0.0f;

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
  uint8_t txDataFd[canTxSize];
  uint8_t rxData[8];
  uint8_t rxDataFd[canRxSize];
  uint8_t prevGenFuncRef;
  volatile bool canRxInterrupt;
  
  // 制御処理統一関数
  void processControlMode(uint16_t cmdRef, const uint8_t* rx);
  void updateGenFuncStatus();

  void rxMsglistFd(const uint8_t (&rx)[canRxSize]);
  void txMsgListFd(uint8_t (&tx_)[canTxSize]);
  
  // レガシー関数（内部使用）
  float uintTofloat(const uint8_t* bytes);

public:
  volatile bool canTxFlag;

  // Constracta
  CanCom(FDCAN_HandleTypeDef& fdcanHandle);

  void initTxHeader(bool extendedId = false, bool fdFormat = false);
  void initFilter(void);
  void rxFifo0Callback(uint32_t RxFifo0ITs);
  void rxTask();
  void txTask();
  void txServoOffTask();
  
  void TEST_rxTask(void);

  // レガシー関数（後方互換性のため残存）
  static void floatTouint(float value, uint8_t (&result)[4]);

  CanData* getData() { return data.get(); }
};
