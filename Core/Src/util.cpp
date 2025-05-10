#include "util.h"

#include "user_task.h"
#include "can_communication.h"

Util util;
extern CanCom cancom;

Util::Util()
  : data(std::make_unique<UtilData>()) {}

// PON以外の機能制御
void Util::genFuncCtrl() {
  CanCom::CanData* candata = cancom.getData();

  if (candata->genFuncCheck){
    // genfuncRef 0b00010000
    data->eCalib = (candata->genFuncRef & 0x10) != 0 ? true : false;
    // genfuncRef 0b00100000
    data->sixPtRotation = (candata->genFuncRef & 0x20) != 0 ? true : false;
  }
  
}