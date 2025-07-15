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

//#define TEST_MODE

UserTask::UserTask()
  : count(0){}

// センサ読み取り統一関数
void UserTask::readSensors() {
  senscur.sensCurIN();
  ma735enc.ma735angle();
}

// テストデータ収集（デバッグ時のみ）
void UserTask::collectTestData() {
  ElecangCalib::ElecangCalibData* ecaldata = elecangcalib.getData();
  MA735Enc::MA735Data* angdata = ma735enc.getData();
  ModeControl::ModeControlData* mdctrldata = modecontrol.getData();
  Foc::FocData* focdata = foc.getData();
  BldcCtrl::BldcCtrlData* bldcdata = bldcctrl.getData();
  SensCur::SensCurData* senscurdata = senscur.getData();

  testData.test = ecaldata->elecAngOfs;
  testData.eofsm = ecaldata->elecAngOfsMinus;
  testData.eofsp = ecaldata->elecAngOfsPlus;
  testData.testid = focdata->id;
  testData.testiq = focdata->iq;
  testData.testvd = mdctrldata->voltDRef;
  testData.testvq = mdctrldata->voltQRef;
  testData.testvel = angdata->mechAngVelLPF;
  testData.testdiff = angdata->testdiff;
  testData.testvelact = angdata->mechAngVel;
  testData.testpos = angdata->mechAng;
  testData.testelec = angdata->elecAng;
  testData.testerrD = bldcdata->testerrD;
  testData.testerrQ = bldcdata->testerrQ;
  testData.testCurU = senscurdata->testU;
  testData.testCurW = senscurdata->testW;
  testData.testcomp = angdata->eleccomp;
  testData.testrawAng = angdata->rawAngtest;
  testData.testrawAngPast = angdata->rawAngPasttest;
  testData.testvelerr = bldcdata->testvelErr;
  testData.testvelerrsum = bldcdata->testvelErrSum;
  testData.testmechpos = angdata->mechAng;
  testData.testposerr = bldcdata->testposErr;
  testData.testposerrsum = bldcdata->testposErrSum;
  testData.testposout = mdctrldata->posout;
}

// ボタン処理共通関数
GPIO_PinState UserTask::handleButtonInput() {
  GPIO_PinState currentB1State = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);
  
  // ボタンの状態が変化したらカウンタリセット
  if (currentB1State != stateData.prevB1State) {
    stateData.stableCount = 0;
  }
  // 同じ状態が続いている場合、カウンタ増加
  else if (stateData.stableCount < stateData.stableThreshold) {
    stateData.stableCount++;
  }
  
  stateData.prevB1State = currentB1State;
  return currentB1State;
}

void UserTask::cyclicTask() {
  Util::UtilData* utildata = util.getUtilData();
  static SeqID_t seqID = TESTCONST;

   switch (seqID) {
    case LOOP:
      // 強制停止
      if (!servoCheck()) {
        outpwm.Poff();
        resetAll();
        seqID = STEP00;
        break;
      }

      readSensors();
      
      // elecAng offset
      elecangcalib.elecCalSeq();
      collectTestData();  // テストデータ収集
      
      motorControl();
      break;
    case INIT:
      // 初期化のためにエンコーダ値の初回読み取り
      resetAll();
      testData.initEnd = false;
      if (true != ma735enc.ma735Init()) {
      } else {
        // 電流オフセット補正とセンサ初期化
        if (senscur.sensCurInit()) {
          readSensors();
          count = 0;
          testData.initEnd = true;
          seqID = STEP00;
        }
      }
      break;
    case STEP00:
      utildata->endECalib = false;
      readSensors();

      if (stateData.onewatect == 1){
        if (servoCheck()){
          outpwm.Pon();
          seqID = LOOP;
          break;
        }
      }

      outpwm.Poff();
      stateData.onewatect = 1;
      break;
    
     case TESTCONST: {
      // テストモード 50%Duty出力
      GPIO_PinState prevState = stateData.prevB1State;
      GPIO_PinState currentB1State = handleButtonInput();
      
      // ボタンの立ち上がりエッジを検出
      if (currentB1State == GPIO_PIN_SET && prevState == GPIO_PIN_RESET) {
        stateData.toggleState = !stateData.toggleState;
      }
      
      // 状態に応じて出力を切り替え
      if (stateData.toggleState) {
        outpwm.TEST_setReg(0.5f, 0.5f, 0.5f);
      } else {
        outpwm.Poff();
      }
      break;
    }
      
    case TEST: {
      ma735enc.ma735angle();
      GPIO_PinState prevState = stateData.prevB1State;
      GPIO_PinState currentB1State = handleButtonInput();
      
      if (currentB1State == GPIO_PIN_SET && prevState == GPIO_PIN_RESET) {
        stateData.toggleState = !stateData.toggleState;
      }
      
      if (stateData.toggleState) {
        seqID = INIT;
      }
      break;
    }

    case TESTSINGLE: {
      GPIO_PinState currentB1State = handleButtonInput();
      
      // 状態が安定している場合の処理
      if (stateData.stableCount >= stateData.stableThreshold) {
        // 安定してONで、かつ実行可能な場合
        if (currentB1State == GPIO_PIN_SET && stateData.canExecute) {
          if (stateData.execCount < stateData.requiredExecs) {
            outpwm.TESTSINGLE_setReg(0);  // U相:0, V相:1, W相:2
            stateData.execCount++;
          }
          if (stateData.execCount >= stateData.requiredExecs) {
            outpwm.Poff();
            stateData.canExecute = false;
          }
        }
        // 安定してOFFの場合
        else if (currentB1State == GPIO_PIN_RESET) {
          stateData.canExecute = true;
          stateData.execCount = 0;
        }
      }
      break;
    }


    default:

      break;
    }
}


void UserTask::idleTask() {
  if (testData.initEnd == true) {
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
  
  // データポインタキャッシュ（1回取得のみ）
  static ModeControl::ModeControlData* mdctrldata = modecontrol.getData();
  static MA735Enc::MA735Data* angdata = ma735enc.getData();
  static Foc::FocData* focdata = foc.getData();
  
  Cordic cordic;

  // SinCos演算
  SinCos result = cordic.radians(angdata->elecAng);

  // dq変換
  foc.forwardCtrl(result);
  
  // drvMdとgenfuncによる指令値切替
  modecontrol.modeCtrl();

  // dq逆変換
  foc.inverseCtrl(result, mdctrldata->voltDRef, mdctrldata->voltQRef);
  // PWM出力
  outpwm.setReg(focdata->vu, focdata->vv, focdata->vw);
}

bool UserTask::servoCheck() {
  static CanCom::CanData* candata = cancom.getData();
  return (candata->genFuncRef & 0x01) != 0 ? true : false;
}

void UserTask::canDataPrepare() {
  static CanCom::CanData* candata = cancom.getData();
  static MA735Enc::MA735Data* angdata = ma735enc.getData();

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
