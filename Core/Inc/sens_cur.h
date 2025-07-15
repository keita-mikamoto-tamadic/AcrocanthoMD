#include <cstdint>

#include "main.h"

class SensCur {
public:
  struct SensCurData {
    float curU = 0.0f;
    float curV = 0.0f;
    float curW = 0.0f;
    
    uint16_t testU = 0;
    uint16_t testW = 0;
  };

private:
  SensCurData data;
  
  // チャンネル別処理用構造体
  struct ChannelData {
    uint32_t offset = 0;
    float rawCurrent = 0.0f;
    uint16_t adcRaw = 0;
    volatile uint32_t* adcRegister;
  };
  
  ChannelData channels[2];  // U相(0), W相(1)
  
  void getRawCur();
  float lpfCur(float _curRaw, float _curPast, float _cutOffFreq);

public:
  SensCur();
  
  bool sensCurInit();
  void sensCurIN();

  SensCurData* getData() { return &data; }
  const SensCurData* getData() const { return &data; }
};