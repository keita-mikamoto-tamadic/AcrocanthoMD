#include "mode_control.h"


ModeControl::ModeControl()
  : mode(CTRLMODE_NONE){}


void ModeControl::modeCtrl(){
  switch (mode) {
    case CTRLMODE_NONE:
      break;
    case CTRLMODE_VOLT:
      break;
    case CTRLMODE_CUR:
      break;
    case CTRLMODE_VEL:
      break;
    case CTRLMODE_POS:
      break;
    default:
      break;
  }
}
