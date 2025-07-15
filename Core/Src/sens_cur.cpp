#include "sens_cur.h"

#include "main.h"
#include "out_pwm.h"
#include "user_task.h"
#include "param.h"

#include <cstdint>

constexpr uint16_t CALCOUNT = 100;  // キャリブレーション回数
constexpr float ADVOLT = 3.3f;      // AD入力電圧範囲は3.3[V]
constexpr float AD_RESL = 4095.0f;  // 12bit分解能

#ifdef AD8418A
constexpr float GAIN_AMP = 20.0f;   // AD8418A
constexpr float GAIN_SHUNT = 0.005f; // 10mohm
constexpr float AMPGAIN = 1.0f / (GAIN_AMP * GAIN_SHUNT);
constexpr float ADGAIN = (1.0f / AD_RESL) * ADVOLT * AMPGAIN;
// キャリア波比較でのPWM生成をmode1で設定しており、update event　= キャリア波谷検出時 = PWM High時に
// ADCをスキャンしているため、DUTYBASE基準で電圧正のときに電流は電圧とは逆の符号で流れる。
constexpr float ADC_POLARITY = -1.0f;  // アンプからモータへの電流流し込みでプラス
#endif

#ifdef TMCS1107A1B
constexpr float GAIN_AMP = 0.050f;  // TMCS1107A1B
constexpr float ADGAIN = ADVOLT / (AD_RESL * GAIN_AMP);
constexpr float ADC_POLARITY = 1.0f;
#endif

SensCur senscur;
extern OutPwm outpwm;

SensCur::SensCur() {
  // ADCレジスタの設定
  channels[0].adcRegister = &(ADC1->JDR1);  // U相
  channels[1].adcRegister = &(ADC2->JDR1);  // W相
}
  
void SensCur::getRawCur() {
  // 統一化されたADC読み取り
  for (int i = 0; i < 2; i++) {
    channels[i].adcRaw = static_cast<uint16_t>(*channels[i].adcRegister);
  }
  
  // テストデータの設定
  data.testU = channels[0].adcRaw;
  data.testW = channels[1].adcRaw;
}

void SensCur::sensCurIN() {
  getRawCur();
  
  // 統一化されたADC→電流変換
  for (int i = 0; i < 2; i++) {
    const float adcValue = static_cast<float>(static_cast<int16_t>(channels[i].adcRaw - channels[i].offset));
    channels[i].rawCurrent = adcValue * ADGAIN * ADC_POLARITY;
  }

  // V相電流はIu + Iv + Iw = 0より計算
  const float curVRaw = -channels[0].rawCurrent - channels[1].rawCurrent;
  
  data.curU = channels[0].rawCurrent;
  data.curV = curVRaw;
  data.curW = channels[1].rawCurrent;
  
  // LPF適用（必要に応じてコメントアウト解除）
  //data.curU = lpfCur(channels[0].rawCurrent, data.curU, 5000.0f);
  //data.curV = lpfCur(curVRaw, data.curV, 5000.0f);
  //data.curW = lpfCur(channels[1].rawCurrent, data.curW, 5000.0f);
}

float SensCur::lpfCur(float _curRaw, float _curPast, float _cutOffFreq) {
  const float timeConst = 1.0f / (user2pi * _cutOffFreq);
  const float alpha = TASK_TIME / timeConst;
  return (alpha * _curRaw + (1.0f - alpha) * _curPast);
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
      // キャリブレーション - 統一化処理
      if (_calcount < CALCOUNT) {
        getRawCur();
        // 両チャンネルのオフセット値を蓄積
        for (int i = 0; i < 2; i++) {
          channels[i].offset += channels[i].adcRaw;
        }
        _calcount++;
      } else {
        // 平均値計算
        for (int i = 0; i < 2; i++) {
          channels[i].offset /= _calcount;
        }
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

