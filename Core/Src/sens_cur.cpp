#include "sens_cur.h"

#include "main.h"
#include "out_pwm.h"
#include "user_task.h"
#include "param.h"

#include <cstdint>

#define CALCOUNT (100) /* キャリブレーション回数 */
#define ADVOLT           (3.3f) /* AD入力電圧範囲は3.3[V] */
#define AD_RESL (4095.0f) /* 12bit分解能 */
#define GAIN_AMP                  (20.0f)  /* AD8418A */
#define GAIN_SHUNT                (0.010f) /* 10mohm */
#define AMPGAIN                   (1.0f / (GAIN_AMP * GAIN_SHUNT))
#define ADGAIN                    ((1.0f / AD_RESL) * ADVOLT * AMPGAIN)
// キャリア波比較でのPWM生成をmode1で設定しており、update event　= キャリア波谷検出時 = PWM High時に
// ADCをスキャンしているため、DUTYBASE基準で電圧正のときに電流は電圧とは逆の符号で流れる。
#define ADC_TO_CUR(U2_V, U2_OFFS) (((float)((int16_t)U2_V - (int16_t)U2_OFFS)) * ADGAIN * -1) /* アンプからモータへの電流流し込みでプラス */

SensCur senscur;
extern OutPwm outpwm;

SensCur::SensCur()
  : data(std::make_unique<SensCurData>()) {}
  
void SensCur::getRawCur() {
  adcRawU = ADC1 -> JDR1;
  adcRawW = ADC2 -> JDR1;
  data->testU = adcRawU;
  data->testW = adcRawW;
}

void SensCur::sensCurIN() {
  getRawCur();
  
  curURaw = ADC_TO_CUR(adcRawU, curOffsU);
  curWRaw = ADC_TO_CUR(adcRawW, curOffsW);

  // V相電流はIu + Iv + Iw = 0より計算
  curVRaw = -curURaw - curWRaw;
  
  data->curU = curURaw;
  data->curV = curVRaw;
  data->curW = curWRaw;
  //data->curU = lpfCur(curURaw, data->curU, 5000.0f);
  //data->curV = lpfCur(curVRaw, data->curV, 5000.0f);
  //data->curW = lpfCur(curWRaw, data->curW, 5000.0f);
}

float SensCur::lpfCur(float _curRaw, float _curPast, float _cutOffFreq) {
  float timeConst, alpha;
  
  timeConst = 1.0f / (user2pi * _cutOffFreq);
  alpha = TASK_TIME / timeConst;
  float curLPF_ = (alpha * _curRaw + (1.0f - alpha) * _curPast);

  return curLPF_;
}

bool SensCur::sensCurInit() {
  static SeqID_t seqID = STEP00;
  static bool _getReady = false;
  static uint16_t _calcount = 0;
  
  switch (seqID) {
    case STEP00:
      // ServoON
      outpwm.Pon();
      seqID = STEP01;
      break;
    case STEP01:
      // キャリブレーション
      if (_calcount < CALCOUNT) {
        getRawCur();
        curOffsU += adcRawU;
        curOffsW += adcRawW;
        _calcount++;
      } else {
        curOffsU /= _calcount;
        curOffsW /= _calcount;
        seqID = STEP02;
      }
      break;
    case STEP02:
      // キャリブ完了
      outpwm.Poff();
      _getReady = true;
    default:
      seqID = STEP00;
      break;
  }
  return _getReady;
}

