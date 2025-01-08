#include <cstdint>
#include <memory>

#include "main.h"

class SensCur {
public:
  struct SensCurData {
    float curU = 0.0f;
    float curV = 0.0f;
    float curW = 0.0f;
    
  };

private:
  std::unique_ptr<SensCurData> data;
  
  uint32_t curOffsU = 0;
  uint32_t curOffsW = 0;

  float curURaw = 0.0f;
  float curVRaw = 0.0f;
  float curWRaw = 0.0f;
  
  void getRawCur();
  uint16_t adcRawU = 0;
  uint16_t adcRawW = 0;

  bool adjustCur();
  float lpfCur(float _curRaw, float _cutOffFreq);

public:
  SensCur();
  
  bool sensCurInit();
  void sensCurIN();

  SensCurData* getData() { return data.get(); }
  
};