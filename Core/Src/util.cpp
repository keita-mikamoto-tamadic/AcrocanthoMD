#include "util.h"

#include "user_task.h"
#include "can_communication.h"

Util util;
extern CanCom cancom;

Util::Util() {
}

// PON以外の機能制御
void Util::genFuncCtrl() {
  const CanCom::CanData* candata = cancom.getData();

  if (candata->genFuncCheck) {
    // 電気角キャリブレーション制御
    if ((candata->genFuncRef & GENFUNC_ECALIB_MASK) != 0) {
      data.eCalib = true;
    } else {
      data.eCalib = false;
    }
    
    // 6点回転制御
    if ((candata->genFuncRef & GENFUNC_SIXPT_MASK) != 0) {
      data.sixPtRotation = true;
    } else {
      data.sixPtRotation = false;
    }
  }
}