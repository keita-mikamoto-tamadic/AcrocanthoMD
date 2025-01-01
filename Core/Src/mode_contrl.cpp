#include "mode_control.h"
#include "user_task.h"

ModeControl modecontrol;
extern UserTask usertask;

ModeControl::ModeControl()
  : mode(CTRLMODE_NONE){}


void ModeControl::modeCtrl(uint8_t mode){
  UserTask::UserTaskData* usertaskdata = usertask.getData();
  
  switch (mode) {
    case CTRLMODE_NONE:
      usertaskdata->voltDRef = 0.0f;
      usertaskdata->voltQRef = 0.0f;
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
      mode = CTRLMODE_NONE;
      break;
  }
}
