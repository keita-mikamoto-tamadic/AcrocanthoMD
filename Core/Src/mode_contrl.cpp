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

#define PBM_SCALEFACTOR (1.155f)

ModeControl::ModeControl()
  : data(std::make_unique<ModeControlData>()){}


void ModeControl::modeCtrl(){


  float voltQRef_ = 0.0f;
  float voltDRef_ = 0.0f;
  
  refCtrl();
  
  switch (s_drvMdRef) {
    case CTRLMODE_NONE:
      voltQRef_ = 0.0f;
      voltDRef_ = 0.0f;
      break;
    case CTRLMODE_VOLT:
      voltDRef_ = s_voltDRef;
      voltQRef_ = s_voltQRef;
      break;
    case CTRLMODE_CUR:
      voltDRef_ = bldcctrl.voltDCtrl(s_curDRef);
      voltQRef_ = bldcctrl.voltQCtrl(s_curQRef);
      break;
    case CTRLMODE_VEL:
      break;
    case CTRLMODE_POS:
      break;
    default:
      mode = CTRLMODE_NONE;
      break;
  }
  
  // 電圧ノルムによる出力制限、3倍高調波の重畳による効果を加味して、
  // MOSが駆動できる最大の電圧は1.155倍になる
  // 相電圧のピーク値 = sqrt(2/3) * sqrt(voltD^2 + voltQ^2)
  // 過変調にならない領域は、相電圧のピーク値 < K * PBM/2 
  // Vq < sqrt(3/8 * PBM^2 * K^2 - Vd^2)
  
  float voltQMax = (3.0f / 8.0f * VOLT_PBM * VOLT_PBM * PBM_SCALEFACTOR * PBM_SCALEFACTOR) - (voltDRef_ * voltDRef_);
  float voltDMax = 0.0f;
  if ( voltQMax >= 0.0f ) {
    voltQMax = sqrtf(voltQMax);
    if (voltQMax < voltQRef_) voltQRef_ = voltQMax;
    else if (voltQRef_ < -voltQMax) voltQRef_ = -voltQMax;
  }
  else {
    voltQRef_ = 0.0f;
    voltDMax = sqrtf(3.0f / 8.0f) * VOLT_PBM * PBM_SCALEFACTOR;
    if (voltQMax < voltDRef_) voltDRef_ = voltDMax;
    else if (voltDRef_ < -voltDMax) voltDRef_ = -voltDMax;
  }
  
  data->voltDRef = voltDRef_;
  data->voltQRef = voltQRef_;
  
}

void ModeControl::refCtrl(){
  Util::UtilData* utildata = util.getUtilData();
  CanCom::CanData* candata = cancom.getData();
  ElecangCalib::ElecangCalibData* elecangcalibdata = elecangcalib.getData();

  if (usertask.servoCheck()) {
    // 電気角キャリブ
    if (utildata->eCalib) {
      s_drvMdRef = elecangcalibdata->drvMd;
      s_voltQRef = elecangcalibdata->voltQRef;
      s_voltDRef = 0.0f;
      
    }
    // どの特殊モードにも当てはまらない場合、上位指令をセット
    else
    {
      s_drvMdRef = candata->drvMdRef;
      s_voltDRef = candata->voltDRef;
      s_voltQRef = candata->voltQRef;
      s_curDRef = candata->curDRef;
      s_curQRef = candata->curQRef;
    }
  }
}
