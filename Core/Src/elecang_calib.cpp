#include "elecang_calib.h"

#include "main.h"
#include "ma735_enc.h"
#include "user_task.h"
#include "user_math.h"
#include "util.h"
#include "can_communication.h"

// 電気角キャリブレーション定数
static constexpr float CALIB_PERIOD = 0.5f;         // 計測時間 [s]
static constexpr float FINE_WIDTH = 0.6f;           // FINE_TUNE測定範囲 [rad]
static constexpr float STIMUL_VOLTDREF = 2.0f;      // 電気角オフセットキャリブレーション印加電圧 [V]
static constexpr float STANDBY_PERIOD = 0.5f;       // 助走時間 [s]
static constexpr float CUTOFF_FREQ = 2.0f;           // LPFのカットオフ周波数 [Hz]
static constexpr float TIME_CONST = 1.0f / (user2pi * CUTOFF_FREQ); // 時定数 [s]
static constexpr float LPF_COEFF = TASK_TIME / TIME_CONST;           // 離散化した1次遅れLPFの係数
static constexpr int32_t STANDBY_COUNT = static_cast<int32_t>(STANDBY_PERIOD / TASK_TIME);
static constexpr int32_t CALIB_COUNT = static_cast<int32_t>(CALIB_PERIOD / TASK_TIME);
static constexpr float CALIB_ROUGH = user2pi / CALIB_NUM;
static constexpr float CALIB_FINE = FINE_WIDTH / CALIB_NUM;
static constexpr float FINE_WIDTH_HALF = FINE_WIDTH / 2.0f;

ElecangCalib elecangcalib;

extern MA735Enc ma735enc;
extern UserTask usertask;
extern Util util;
extern CanCom cancom;

ElecangCalib::ElecangCalib(){}

void ElecangCalib::elecCalSeq(){
  Util::UtilData* utildata = util.getData();
  CanCom::CanData* candata = cancom.getData();

  static SeqID_t seqID = INIT;
  static SeqID_t seqID_prev = STEP00;
  float elecAngOfsCur = 0.0f;
  float ecalVoltDRef = STIMUL_VOLTDREF;
  
  if (candata->voltQRef > STIMUL_VOLTDREF) {
    ecalVoltDRef = candata->voltQRef;
  }
  
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
      elecAngOfsCur = elecAngOfsRP - FINE_WIDTH_HALF;
      if (elecAngOfsCur < 0.0f) {
        elecAngOfsCur += user2pi;
      }
      if (calibSub(ecalVoltDRef, elecAngOfsCur, &elecAngOfsFP, CALIB_FINE)){
        seqID_prev = STEP01;
        seqID = (seqIDSub == FAIL) ? END : IDLE;
        data.elecAngOfsPlus = elecAngOfsFP;
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
      elecAngOfsCur = elecAngOfsRM - FINE_WIDTH_HALF;
      if (elecAngOfsCur < 0.0f) {
        elecAngOfsCur += user2pi;
      }
      if (calibSub(-ecalVoltDRef, elecAngOfsCur, &elecAngOfsFM, CALIB_FINE)){
        seqID_prev = STEP03;
        seqID = (seqIDSub == FAIL) ? END : IDLE;
        data.elecAngOfsMinus = elecAngOfsFM;
      }
      break;
      
    case STEP04:
      {
        // 最終オフセット値算出
        if (!utildata->eCalib) {
          seqID = END;
          break;
        }

        tuneDiff = elecAngOfsFM - elecAngOfsFP;
        data.elecAngOfsPlus = elecAngOfsFP;
        data.elecAngOfsMinus = elecAngOfsFM;
        
        const float avgOffset = (elecAngOfsFP + elecAngOfsFM) / 2.0f;
        if (((elecAngOfsFP + userpi) < elecAngOfsFM) || (elecAngOfsFM < (elecAngOfsFP - userpi))) {
          data.elecAngOfs = avgOffset - userpi;
        } else {
          data.elecAngOfs = avgOffset;
        }
        
        if (data.elecAngOfs <= 0.0f) {
          data.elecAngOfs += user2pi;
        }

        seqID = END;
        seqID_prev = IDLE;
      }
      break;
      
    case END:
      data.drvMd = 0;
      data.voltQRef = 0.0f;
      utildata->eCalib = false;
      utildata->endECalib = true;
      seqID = INIT;
      break;
      
    case IDLE:
      if (idleCount++ < 1000) {
        data.drvMd = 0;
        data.voltQRef = 0.0f;
        break;
      }
      
      idleCount = 0;
      switch (seqID_prev) {
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
      break;
      
    default:
      seqID = INIT;
      break;
  }

}

bool ElecangCalib::calibSub(float _voltDRef, float _elecAngOfsCur, float *_elecAngOfsMax, float _calDelta) {
  MA735Enc::MA735Data* ma735data = ma735enc.getData();
  Util::UtilData* utildata = util.getData();
  bool returnVal = false;
  
  // forloop用
  float velOutMax_ = 0.0f;
  
  switch (seqIDSub) {
    case STEP00:
      // Standby
      data.drvMd = 1;
      data.voltQRef = _voltDRef;
      elecAngOfsVal = _elecAngOfsCur;
      ma735data->elecAng += elecAngOfsVal;

      // 中断処理
      if (!(utildata->eCalib)) {
        seqIDSub = FAIL;
        break;
      }

      if (count++ < STANDBY_COUNT) {
        velOutAxLast = (1.0f - LPF_COEFF) * velOutAxLast + LPF_COEFF * ma735data->mechAngVel;
      } else {
        count = 0;
        seqIDSub = STEP01;
      }
      break;
    case STEP01:
      // Run
      data.drvMd = 1;
      data.voltQRef = _voltDRef;
      ma735data->elecAng += elecAngOfsVal;

      // 中断処理
      if (!(utildata->eCalib)) {
        seqIDSub = FAIL;
        break;
      }

      if (count++ < CALIB_COUNT) {
        if (user2pi < ma735data->elecAng) {
          ma735data->elecAng -= user2pi;
        }
        velOutAxLast = (1.0f - LPF_COEFF) * velOutAxLast + LPF_COEFF * ma735data->mechAngVel;
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
      // Measuring
      
      // 中断処理
      if (!(utildata->eCalib)) {
        seqIDSub = FAIL;
        break;
      }

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