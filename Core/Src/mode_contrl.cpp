#include "mode_control.h"
#include "user_task.h"
#include "can_communication.h"
#include "util.h"
#include "elecAng_calib.h"
#include "foc.h"
#include "bldc_ctrl.h"

ModeControl modecontrol;
extern UserTask usertask;
extern CanCom cancom;
extern Util util;
extern ElecangCalib elecangcalib;
extern BldcCtrl bldcctrl;

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