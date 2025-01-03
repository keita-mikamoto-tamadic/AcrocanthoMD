#include <cstdint>
#include <memory>

#include "main.h"

#define CALCOUNT (100) /* キャリブレーション回数 */
#define ADVOLT           (3.3f) /* AD入力電圧範囲は3.3[V] */
#define AD_RESL (4095.0f) /* 12bit分解能 */
#define GAIN_AMP                  (20.0f)  /* AD8418A */
#define GAIN_SHUNT                (0.010f) /* 10mohm */
#define AMPGAIN                   (1.0f / (GAIN_AMP * GAIN_SHUNT))
#define ADGAIN                    ((1.0f / AD_RESL) * ADVOLT * AMPGAIN)
#define ADC_TO_CUR(U2_V, U2_OFFS) (((float)((int16_t)U2_V - (int16_t)U2_OFFS)) * ADGAIN) /* アンプからモータへの電流流し込みでプラス */


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