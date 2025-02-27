#include "elecang_calib.h"

#include "main.h"
#include "user_task.h"
#include "user_math.h"
#include "ang.h"
#include "util.h"
#include "can_communication.h"

#define CALIB_PERIOD    (0.5f)                         /* 計測時間 [s] */
#define FINE_WIDTH      (0.6f)                         /* FINE_TUNE測定範囲 [rad] */
#define STIMUL_VOLTDREF (1.5f)                         /* 電気角オフセットキャリブレーション印加電圧 [V] */
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
  static SeqID_t seqID_prev = STEP00;
  float elecAngOfsCur = 0.0f;
  float ecalVoltDRef = STIMUL_VOLTDREF;
  
  if (candata->voltQRef > STIMUL_VOLTDREF) ecalVoltDRef = candata->voltQRef;
  
  switch (seqID) {
    case INIT:
      if (utildata->eCalib) {
        seqID = STEP00;
      }
      break;
    // 正の電圧印可 電気角オフセット遷移は0~2piの範囲
    // elecAngOfsRPに正の回転方向でのオフセット値を記録
    case STEP00:
      if (calibSub(ecalVoltDRef, elecAngOfsCur, &elecAngOfsRP, CALIB_ROUGH)){
        seqID_prev = STEP00;
        seqID = (seqIDSub == FAIL) ? END : IDLE;
      }
      break;
    // 正の電圧印可 電気角オフセット遷移はelecAngOfsRPからFINE_WIDTH/2戻った地点から
    // FINE_WIDTHの範囲で分割
    // elecAngOfsFPに正の回転方向での詳細オフセット値を記録
    case STEP01:
      elecAngOfsCur = elecAngOfsRP - (FINE_WIDTH / 2);
      if (elecAngOfsCur < 0.0f) elecAngOfsCur += user2pi;
      if (calibSub(ecalVoltDRef, elecAngOfsCur, &elecAngOfsFP, CALIB_FINE)){
        seqID_prev = STEP01;
        seqID = (seqIDSub == FAIL) ? END : IDLE;
        data->elecAngOfsPlus = elecAngOfsFP;
      }
      break;
      
    // 負の電圧印可 電気角オフセット遷移は0~2piの範囲
    // elecAngOfsRMに負の回転方向でのオフセット値を記録
    case STEP02:
      if (calibSub(-ecalVoltDRef, elecAngOfsCur, &elecAngOfsRM, CALIB_ROUGH)){
        seqID_prev = STEP02;
        seqID = (seqIDSub == FAIL) ? END : IDLE;
      }
      break;
    // 負の電圧印可 電気角オフセット遷移はelecAngOfsRMからFINE_WIDTH/2戻った地点から
    // FINE_WIDTHの範囲で分割
    // elecAngOfsFMに正の回転方向での詳細オフセット値を記録
    case STEP03:
      elecAngOfsCur = elecAngOfsRM - (FINE_WIDTH / 2);
      if (elecAngOfsCur < 0.0f) elecAngOfsCur += user2pi;
      if (calibSub(-ecalVoltDRef, elecAngOfsCur, &elecAngOfsFM, CALIB_FINE)){
        seqID_prev = STEP03;
        seqID = (seqIDSub == FAIL) ? END : IDLE;
        data->elecAngOfsMinus = elecAngOfsFM;
      }
      break;
      
    case STEP04:
      // 最終オフセット値算出
      if (!utildata->eCalib) { seqID = END; break; }

     
      tuneDiff = elecAngOfsFM - elecAngOfsFP;
      data->elecAngOfsPlus = elecAngOfsFP;
      data->elecAngOfsMinus = elecAngOfsFM;
      if (((elecAngOfsFP + userpi) < elecAngOfsFM) || (elecAngOfsFM < (elecAngOfsFP - userpi))) {
        data->elecAngOfs = ((elecAngOfsFP + elecAngOfsFM) / 2) - userpi;
      }else{
        data->elecAngOfs = (elecAngOfsFP + elecAngOfsFM) / 2;
      }
      
      if (data->elecAngOfs <= 0.0f) data->elecAngOfs += user2pi;

      seqID = END;
      seqID_prev = IDLE;
      break;
      
    case END:
      data->drvMd = 0;
      data->voltQRef = 0.0f;
      utildata->eCalib = false;
      seqID = INIT;
      break;
      
    case IDLE:
      if (idleCount++ < 1000) {
        data->drvMd = 0;
        data->voltQRef = 0.0f;
        break;

      } else {
        idleCount = 0;
        switch (seqID_prev)
        {
        case STEP00:
          seqID = STEP01;
          break;
        case STEP01:
          seqID = STEP02;
          break;
        case STEP02:
          seqID = STEP03;
          break;
        case STEP03:
          seqID = STEP04;
          break;
        case IDLE:
          break;  

        default:
          break;
        }
      }
      break;
      
    default:
      seqID = INIT;
      break;
  }

}

bool ElecangCalib::calibSub(float _voltDRef, float _elecAngOfsCur, float *_elecAngOfsMax, float _calDelta) {
  Ang::AngData* angdata = ang.getData();
  Util::UtilData* utildata = util.getUtilData();
  bool returnVal = false;
  
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
        velOutAxLast = (1 - LPF_COEFF) * velOutAxLast + LPF_COEFF * angdata->mechAngVel;
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
          velOutAxLast = (1 - LPF_COEFF) * velOutAxLast + LPF_COEFF * angdata->mechAngVel;
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