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

//#define TEST_MODE

UserTask::UserTask()
  : count(0){}


void UserTask::cyclicTask() {
  ElecangCalib::ElecangCalibData* ecaldata = elecangcalib.getData();
  MA735Enc::MA735Data* angdata = ma735enc.getData();

  static bool toggleState = false;  // 出力状態
  static GPIO_PinState prevB1State = GPIO_PIN_RESET;  // 前回のボタン状態
      
  // 現在のボタン状態を取得
  GPIO_PinState currentB1State = HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin);

  static SeqID_t seqID = INIT;

   switch (seqID) {
    case LOOP:

      // 強制停止
      if (!servoCheck()) {
        outpwm.Poff();
        resetAll();

        seqID = STEP00;
        break;
      }

      senscur.sensCurIN();
      ma735enc.ma735angle();
      
      // elecAng offset
      elecangcalib.elecCalSeq();
      test = ecaldata->elecAngOfs;
      eofsm = ecaldata->elecAngOfsMinus;
      eofsp = ecaldata->elecAngOfsPlus;
      // 
      
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
      senscur.sensCurIN();
      ma735enc.ma735angle();

      // test
      testpos = angdata->mechAng;
      //testelec = angdata->mechAngVelLPF;
      

      if (servoCheck()){
        outpwm.Pon();
        seqID = LOOP;
        break;
      }

      outpwm.Poff();
      break;
    
/*     case TEST:
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
      break; */
      

    default:
      seqID = INIT;
      break;
    }
}


void UserTask::idleTask() {
  if (initEnd == true) {
  #ifdef TEST_MODE
    cancom.TEST_rxTask();
    util.genFuncCtrl();
  #else
    cancom.rxTask();
    util.genFuncCtrl();
    cancom.initTxHeader(false, true);
    cancom.txTask();
  #endif
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
  //testCurU = senscurdata->testU;
  //testCurW = senscurdata->testW;
  testcomp = angdata->eleccomp;
  testrawAng = angdata->rawAngtest;
  testrawAngPast = angdata->rawAngPasttest;
  //testvelerr = bldcdata->testvelErr;
  //testvelerrsum = bldcdata->testvelErrSum;
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
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_SET);
  usertask.cyclicTask();
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8, GPIO_PIN_RESET);
}
