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
  uint16_t rawCurU = 0;
  uint16_t rawCurW = 0;
  
  void getRawCur();
  bool adjustCur();

public:
  SensCur();
  
  bool sensCurInit();
  void sensCurIN();

  SensCurData* getData() { return data.get(); }
  
};