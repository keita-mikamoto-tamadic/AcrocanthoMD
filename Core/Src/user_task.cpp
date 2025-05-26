#include "user_task.h"

#include "main.h"
#include "out_pwm.h"
#include "can_communication.h"
#include "mode_control.h"
#include "sens_cur.h"
#include "util.h"
#include "elecang_calib.h"
#include "foc.h"
#include "bldc_ctrl.h"
#include "ma735_enc.h"

UserTask usertask;

extern MA735Enc ma735enc;
extern SensCur senscur;
extern OutPwm outpwm;
extern CanCom cancom;
extern ModeControl modecontrol;
extern Util util;
extern ElecangCalib elecangcalib;
extern Foc foc;
extern BldcCtrl bldcctrl;

volatile bool adcflag = true;

#define TEST_MODE

UserTask::UserTask()
  : count(0){}


void UserTask::cyclicTask() {
  ElecangCalib::ElecangCalibData* ecaldata = elecangcalib.getData();
  MA735Enc::MA735Data* angdata = ma735enc.getData();
  Util::UtilData* utildata = util.getUtilData();

  static bool toggleState = false;  // 出力状態
  static GPIO_PinState prevB1State = GPIO_PIN_RESET;  // 前回のボタン状態
  static uint8_t onewatect = 0;
  static bool canExecute = true;     // 実行可能フラグ
  static uint8_t stableCount = 0;    // 安定度カウンタ
  static const uint8_t stableThreshold = 10;  // 安定判定のしきい値
  static bool needTurnOff = false;   // 次周期でオフにするフラグ
  static uint8_t execCount = 0;       // 実行回数カウンタ
  static const uint8_t requiredExecs = 100;
      
  // 現在のボタン状態を取得
  GPIO_PinState currentB1State = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

  // 初期値でtest状態へ遷移するかどうか決まる
  static SeqID_t seqID = TEST;
  //static SeqID_t seqID = TESTCONST;
  //static SeqID_t seqID = TESTSINGLE;
  //static SeqID_t seqID = INIT;

   switch (seqID) {
    case LOOP:

      // 強制停止
       if (!servoCheck()) {
        outpwm.Poff();
        resetAll();

        seqID = STEP00;
        break;
      }

      //senscur.sensCurIN();
      ma735enc.ma735angle();
      
      // elecAng offset
      elecangcalib.elecCalSeq();
      test = ecaldata->elecAngOfs;
      eofsm = ecaldata->elecAngOfsMinus;
      eofsp = ecaldata->elecAngOfsPlus;
      
      motorControl();

      break;
    case INIT:
      // 初期化のためにエンコーダ値の初回読み取り
      resetAll();
      initEnd = false;
      if (true != ma735enc.ma735Init()) {
      } else {
        // 電流オフセット補正
        ma735enc.ma735angle();
        if (senscur.sensCurInit()) {
          senscur.sensCurIN();
          count = 0;

          initEnd = true;
          seqID = STEP00;
        }
      }
      break;
    case STEP00:
      utildata->endECalib = false;
      senscur.sensCurIN();
      ma735enc.ma735angle();
      // ma735enc.magFieldTh();

      if (onewatect == 1){
        if (servoCheck()){
          outpwm.Pon();
          seqID = LOOP;
          break;
        }
      }

      outpwm.Poff();
      onewatect = 1;
      break;
    
     case TESTCONST:
      // テストモード 50%Duty出力
      // ボタンの立ち上がりエッジを検出
      if (currentB1State == GPIO_PIN_SET && prevB1State == GPIO_PIN_RESET) {
        toggleState = !toggleState;  // 状態を反転
      }
      
      // 状態に応じて出力を切り替え
      if (toggleState) {
        outpwm.TEST_setReg(0.5f, 0.5f, 0.5f);
      } else {
        outpwm.Poff();
      }
      
      // ボタンの状態を保存
      prevB1State = currentB1State;
      break;
      
    case TEST:
      if (currentB1State == GPIO_PIN_SET && prevB1State == GPIO_PIN_RESET) {
        toggleState = !toggleState;  // 状態を反転
      }
      // 状態に応じて出力を切り替え
      if (toggleState) {
        seqID = INIT;
      } else {
        // 何もしない
      }
      // ボタンの状態を保存
      prevB1State = currentB1State;
      break;

    case TESTSINGLE:
      
    // ボタンの状態が変化したらカウンタリセット
    if (currentB1State != prevB1State) {
        stableCount = 0;
    }
    // 同じ状態が続いている場合、カウンタ増加
    else if (stableCount < stableThreshold) {
        stableCount++;
    }
    
    // 状態が安定している場合の処理
    if (stableCount >= stableThreshold) {
        // 安定してONで、かつ実行可能な場合
        if (currentB1State == GPIO_PIN_SET && canExecute) {
            if (execCount < requiredExecs) {
                outpwm.TESTSINGLE_setReg(0);  // U相:0, V相:1, W相:2
                execCount++;                   // 実行回数をカウントアップ
            }
            if (execCount >= requiredExecs) {
                outpwm.Poff();                // 2回実行完了後にPoff
                canExecute = false;           // 次の実行をブロック
            }
        }
        // 安定してOFFの場合
        else if (currentB1State == GPIO_PIN_RESET) {
            canExecute = true;     // 次の実行を許可
            execCount = 0;         // 実行回数カウンタをリセット
        }
    }
    
    prevB1State = currentB1State;
    break;


    default:

      break;
    }
}


void UserTask::idleTask() {
  if (initEnd == true) {
  #ifdef TEST_MODE
    cancom.TEST_rxTask();
  #else
    cancom.rxTask();
    cancom.initTxHeader(false, true);
    cancom.txTask();
  #endif

  util.genFuncCtrl();
  }
}

// PON後のモータ制御
void UserTask::motorControl() {
  using namespace Acrocantho;
  ModeControl::ModeControlData* mdctrldata = modecontrol.getData();
  MA735Enc::MA735Data* angdata = ma735enc.getData();
  Foc::FocData* focdata = foc.getData();
  CanCom::CanData* candata = cancom.getData();
  BldcCtrl::BldcCtrlData* bldcdata = bldcctrl.getData();
  SensCur::SensCurData* senscurdata = senscur.getData();
  Cordic cordic;

  // SinCos演算
  SinCos result = cordic.radians(angdata->elecAng);

  // dq変換
  foc.forwardCtrl(result);
  
  testid = focdata->id;
  testiq = focdata->iq;
  
  // drvMdとgenfuncによる指令値切替
  modecontrol.modeCtrl();
  
  testvd = mdctrldata->voltDRef;
  testvq = mdctrldata->voltQRef;
  testvel = angdata->mechAngVelLPF;
  testdiff = angdata->testdiff;
  testvelact = angdata->mechAngVel;
  testpos = angdata->mechAng;
  testelec = angdata->elecAng;
  testerrD = bldcdata->testerrD;
  testerrQ = bldcdata->testerrQ;
  testCurU = senscurdata->testU;
  testCurW = senscurdata->testW;
  testcomp = angdata->eleccomp;
  testrawAng = angdata->rawAngtest;
  testrawAngPast = angdata->rawAngPasttest;
  testvelerr = bldcdata->testvelErr;
  testvelerrsum = bldcdata->testvelErrSum;
  testmechpos = angdata->mechAng;
  testposerr = bldcdata->testposErr;
  testposerrsum = bldcdata->testposErrSum;
  testposout = mdctrldata->posout;


  // dq逆変換
  foc.inverseCtrl(result, mdctrldata->voltDRef, mdctrldata->voltQRef);
  // PWM出力
  outpwm.setReg(focdata->vu, focdata->vv, focdata->vw);
}

bool UserTask::servoCheck() {
  CanCom::CanData* candata = cancom.getData();
  return (candata->genFuncRef & 0x01) != 0 ? true : false;
}

void UserTask::canDataPrepare() {
  CanCom::CanData* candata = cancom.getData();
  ModeControl::ModeControlData* mdctrldata = modecontrol.getData();
  MA735Enc::MA735Data* angdata = ma735enc.getData();
  Foc::FocData* focdata = foc.getData();
  BldcCtrl::BldcCtrlData* bldcdata = bldcctrl.getData();
  SensCur::SensCurData* senscurdata = senscur.getData();

  // CANデータの準備
  candata->actPos = angdata->mechAng;
}

void UserTask::resetAll() {
  bldcctrl.resetData();
}

void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef *hadc){
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  usertask.cyclicTask();
//  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}
