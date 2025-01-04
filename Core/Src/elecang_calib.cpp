#include "elecang_calib.h"

#include "main.h"
#include "user_task.h"
#include "user_math.h"
#include "ang.h"
#include "util.h"
#include "can_communication.h"

#define CALIB_PERIOD    (0.5f)                         /* 計測時間 [s] */
#define FINE_WIDTH      (0.6f)                         /* FINE_TUNE測定範囲 [rad] */
#define STIMUL_VOLTDREF (1.0f)                         /* 電気角オフセットキャリブレーション印加電圧 [V] */
#define STANDBY_PERIOD  (0.5f)                         /* 助走時間 */
#define CUTOFF_FREQ     (2)                            /* LPFのカットオフ周波数 [Hz] */
#define TIME_CONST      (1 / (user2pi * CUTOFF_FREQ)) /* 時定数 [s] */
#define LPF_COEFF       (TASK_TIME / TIME_CONST)     /* 離散化した1次遅れLPFの係数 */
#define STANDBY_COUNT   ((int32_t)(STANDBY_PERIOD / TASK_TIME))
#define CALIB_COUNT     ((int32_t)(CALIB_PERIOD / TASK_TIME))
#define CALIB_ROUGH     (user2pi / CALIB_NUM)
#define CALIB_FINE      (FINE_WIDTH / CALIB_NUM)

ElecangCalib elecangcalib;

extern Ang ang;
extern UserTask usertask;
extern Util util;
extern CanCom cancom;

ElecangCalib::ElecangCalib()
  : data(std::make_unique<ElecangCalibData>()){}

void ElecangCalib::elecCalSeq(){
  Util::UtilData* utildata = util.getUtilData();
  CanCom::CanData* candata = cancom.getData();
  static SeqID_t seqID = INIT;

  float elecAngOfsCur = 0.0f;
  float ecalVoltDRef = STIMUL_VOLTDREF;
  
  if (candata->voltQRef > STIMUL_VOLTDREF) ecalVoltDRef = candata->voltQRef;
  
  switch (seqID) {
    case INIT:
      if (utildata->eCalib) {
        seqID = STEP00;
      }
      break;
    case STEP00:
      if (calibSub(ecalVoltDRef, elecAngOfsCur, &elecAngOfsRP, CALIB_ROUGH)){
        seqID = (seqIDSub == FAIL) ? END : STEP01;
      }
      break;
    case STEP01:
      elecAngOfsCur = elecAngOfsRP - (FINE_WIDTH / 2);
      if (elecAngOfsCur < 0.0f) elecAngOfsCur += user2pi;
      if (calibSub(ecalVoltDRef, elecAngOfsCur, &elecAngOfsFP, CALIB_FINE)){
        seqID = (seqIDSub == FAIL) ? END : STEP02;
      }
      break;
      
    case STEP02:
      if (calibSub(-ecalVoltDRef, elecAngOfsCur, &elecAngOfsRM, CALIB_ROUGH)){
        seqID = (seqIDSub == FAIL) ? END : STEP03;
      }
      break;
    case STEP03:
      elecAngOfsCur = elecAngOfsRM + (FINE_WIDTH / 2);
      if (elecAngOfsCur < 0.0f) elecAngOfsCur += user2pi;
      if (calibSub(-ecalVoltDRef, elecAngOfsCur, &elecAngOfsFM, CALIB_FINE)){
        seqID = (seqIDSub == FAIL) ? END : STEP04;
      }
      break;
      
    case STEP04:
      // 最終オフセット値算出
      seqID = END;
      break;
      
    case END:
      data->drvMd = 0;
      data->voltQRef = 0.0f;
      utildata->eCalib = false;
      seqID = INIT;
      break;
    default:
      seqID = INIT;
      break;
  }

}

bool ElecangCalib::calibSub(float _voltDRef, float _elecAngOfsCur, float *_elecAngOfsMax, float _calDelta) {
  Ang::AngData* angdata = ang.getAngData();
  Util::UtilData* utildata = util.getUtilData();
  static bool returnVal = false;
  
  // forloop用
  float velOutMax_ = 0.0f;
  
  switch (seqIDSub) {
    case STEP00:
      // Standby
      data->drvMd = 1;
      data->voltQRef = _voltDRef;
      elecAngOfsVal = _elecAngOfsCur;
      angdata->elecAng += elecAngOfsVal;

      // 中断処理
      if (!(utildata->eCalib)) { seqIDSub = FAIL; break; }

      if (count++ < STANDBY_COUNT) {
        velOutAxLast = (1 - LPF_COEFF) * velOutAxLast + LPF_COEFF * angdata->actVel;
      } else {
        count = 0;
        seqIDSub = STEP01;
      }
      break;
    case STEP01:
      // Run
      data->drvMd = 1;
      data->voltQRef = _voltDRef;
      angdata->elecAng += elecAngOfsVal;

      // 中断処理
      if (!(utildata->eCalib)) { seqIDSub = FAIL; break; }

      if (count++ < CALIB_COUNT) {
        if (user2pi < angdata->elecAng) angdata->elecAng -= user2pi;
        velOutAxLast = (1 - LPF_COEFF) * velOutAxLast + LPF_COEFF * angdata->actVel;
      } else {
        // 電気角オフセットの更新
        if (indexnum < CALIB_NUM) {
          count = 0;
          velOut[indexnum] = velOutAxLast;
          elecAngOfs[indexnum] = elecAngOfsVal;
          elecAngOfsVal += _calDelta;
          indexnum++;
        } else {
          indexnum = 0;
          seqIDSub = STEP02;
        }
      }
      break;
    case STEP02:
      // Mesuring
      
      // 中断処理
      if (!(utildata->eCalib)) { seqIDSub = FAIL; break; }

      for (indexnum = 0; indexnum < CALIB_NUM; indexnum++) {
        if ((_voltDRef > 0.0f) && (velOut[indexnum] >= velOutMax_)) {
          velOutMax_ = velOut[indexnum];
          *_elecAngOfsMax = elecAngOfs[indexnum];
        }
        else if ((_voltDRef < 0.0f) && (velOut[indexnum] <= velOutMax_)) {
          velOutMax_ = velOut[indexnum];
          *_elecAngOfsMax = elecAngOfs[indexnum];
        }
      }
      indexnum = 0;
      seqIDSub = END;

      break;
      
    case END:
      // End
      count = 0;
      indexnum = 0;
      velOutAxLast = 0.0f;
      elecAngOfsVal = 0.0f;
      returnVal = true;
      seqIDSub = STEP00;
      break;
    case FAIL:
      // キャンセル処理
      count = 0;
      indexnum = 0;
      velOutAxLast = 0.0f;
      elecAngOfsVal = 0.0f;
      returnVal = true;
      
      break;
    default:
    seqIDSub = STEP00;
    break;
  } 
  return returnVal;
}