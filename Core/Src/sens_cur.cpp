#include "sens_cur.h"

#include "main.h"

#include <cstdint>

SensCur senscur;

SensCur::SensCur()
  : data(std::make_unique<SensCurData>()) {}
  
void SensCur::getRawCur() {
  rawCurU = ADC1 -> JDR1;
  rawCurW = ADC2 -> JDR1;
}

void SensCur::sensCurIN() {
  getRawCur();
  
  data->curU = ADC_TO_CUR(rawCurU, 0);
  data->curW = ADC_TO_CUR(rawCurW, 0);
  
  // V相電流はIu + Iv + Iw = 0より計算
  data->curV = -data->curU - data->curW;
}