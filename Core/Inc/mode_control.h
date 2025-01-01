#include <cstdint>

typedef enum {
  CTRLMODE_NONE = 0U,
  CTRLMODE_VOLT,
  CTRLMODE_CUR,
  CTRLMODE_VEL,
  CTRLMODE_POS
} st_mode;

class ModeControl {
private:  
  st_mode mode;
  void genFuncCtrl();


public:
  ModeControl();
  void modeCtrl(uint8_t mode);
  void modeCtrlReset();

};