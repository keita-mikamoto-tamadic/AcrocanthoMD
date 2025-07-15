#include "mode_control.h"
#include "user_task.h"
#include "can_communication.h"
#include "util.h"
#include "elecAng_calib.h"
#include "foc.h"
#include "bldc_ctrl.h"
#include "param.h"
#include "math.h" // sqrtf

ModeControl modecontrol;
extern UserTask usertask;
extern CanCom cancom;
extern Util util;
extern ElecangCalib elecangcalib;
extern BldcCtrl bldcctrl;

// 電圧制限定数
static constexpr float PBM_SCALEFACTOR = 1.15f;
static constexpr float VOLTAGE_COEFF = 3.0f / 8.0f;
static constexpr float VOLTAGE_SQRT_COEFF = 0.8660254f; // sqrt(3/4)
static constexpr float PBM_SCALE_SQUARED = PBM_SCALEFACTOR * PBM_SCALEFACTOR;

ModeControl::ModeControl() = default;

void ModeControl::modeCtrl(){
  CanCom::CanData* candata = cancom.getData();

  float voltQRef = 0.0f;
  float voltDRef = 0.0f;
  float velPIDOut = 0.0f;
  
  refCtrl();
  
  switch (candata->cmdRef) {
    case CTRLMODE_NONE:
      break;
    case CTRLMODE_VOLT:
      voltDRef = s_voltDRef;
      voltQRef = s_voltQRef;
      break;
    case CTRLMODE_CUR:
      voltDRef = bldcctrl.voltDCtrl(s_curDRef);
      voltQRef = bldcctrl.voltQCtrl(s_curQRef);
      break;
    case CTRLMODE_VEL:
      velPIDOut = bldcctrl.velPidCtrl(s_velRef);
      data.vel = velPIDOut;
      voltDRef = bldcctrl.voltDCtrl(0.0f);
      voltQRef = bldcctrl.voltQCtrl(velPIDOut);
      break;
    case CTRLMODE_POS:
      {
        const float posPIDout = bldcctrl.posPidCtrl(s_posRef);
        velPIDOut = bldcctrl.velPidCtrl(posPIDout);
        data.posout = posPIDout;
        data.vel = velPIDOut;
        voltDRef = bldcctrl.voltDCtrl(0.0f);
        voltQRef = bldcctrl.voltQCtrl(velPIDOut);
      }
      break;
    default:
      // 不正なコマンドを安全にリセット
      candata->cmdRef = CTRLMODE_NONE;
      break;
  }
  
  limitVoltage(voltDRef, voltQRef);
  
  data.voltDRef = voltDRef;
  data.voltQRef = voltQRef;
}

void ModeControl::refCtrl(){
  Util::UtilData* utildata = util.getUtilData();
  CanCom::CanData* candata = cancom.getData();
  ElecangCalib::ElecangCalibData* elecangcalibdata = elecangcalib.getData();

  if (usertask.servoCheck()) {
    // 電気角キャリブ
    if (utildata->eCalib) {
      s_voltQRef = elecangcalibdata->voltQRef;
      s_voltDRef = 0.0f;
      s_curDRef = 0.0f;
      s_curQRef = 0.0f;
      s_velRef = 0.0f;
    }
    // どの特殊モードにも当てはまらない場合、上位指令をセット
    else
    {
      s_voltDRef = candata->voltDRef;
      s_voltQRef = candata->voltQRef;
      s_curDRef = candata->curDRef;
      s_curQRef = candata->curQRef;
      s_velRef = candata->velRef;
      s_posRef = candata->posRef;
    }
  }
}

// 電圧制限処理（過変調防止）
void ModeControl::limitVoltage(float& voltDRef, float& voltQRef) {
  // 電圧ノルムによる出力制限、3倍高調波の重畳による効果を加味して、
  // MOSが駆動できる最大の電圧は1.155倍になる
  // 相電圧のピーク値 = sqrt(2/3) * sqrt(voltD^2 + voltQ^2)
  // 過変調にならない領域は、相電圧のピーク値 < K * PBM/2 
  // Vq < sqrt(3/8 * PBM^2 * K^2 - Vd^2)
  
  const float voltageLimit = VOLTAGE_COEFF * VOLT_PBM * VOLT_PBM * PBM_SCALE_SQUARED;
  const float voltQMax = voltageLimit - (voltDRef * voltDRef);
  
  if (voltQMax >= 0.0f) {
    const float voltQLimit = sqrtf(voltQMax);
    if (voltQRef > voltQLimit) voltQRef = voltQLimit;
    else if (voltQRef < -voltQLimit) voltQRef = -voltQLimit;
  }
  else {
    voltQRef = 0.0f;
    const float voltDLimit = VOLTAGE_SQRT_COEFF * VOLT_PBM * PBM_SCALEFACTOR;
    if (voltDRef > voltDLimit) {
      voltDRef = voltDLimit;
    }
    else if (voltDRef < -voltDLimit) {
      voltDRef = -voltDLimit;
    }
  }
}

void ModeControl::modeCtrlReset() {
  s_voltQRef = 0.0f;
  s_voltDRef = 0.0f;
  s_virAngFreq = 0.0f;
  s_curDRef = 0.0f;
  s_curQRef = 0.0f;
  s_velRef = 0.0f;
  s_posRef = 0.0f;
  
  data.genFuncRef = 0;
  data.drvMdRef = 0;
  data.voltDRef = 0.0f;
  data.voltQRef = 0.0f;
  data.virAngFreq = 0.0f;
  data.vel = 0.0f;
  data.posout = 0.0f;
}
