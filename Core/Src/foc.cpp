#include "foc.h"

#include "main.h"
#include "user_math.h"
#include "ma735_enc.h"
#include "sens_cur.h"

Foc foc;

extern MA735Enc ang;
extern SensCur senscur;

using namespace Acrocantho;

Foc::Foc()
    : data(std::make_unique<FocData>()) {}

void Foc::forwardCtrl(const SinCos _sc){
  SensCur::SensCurData* senscurdata = senscur.getData();
  Cordic cordic;
  
  // Clarke transform
  ClarkeTransform ct(senscurdata->curU, senscurdata->curV, senscurdata->curW);
  
  // Park transform
  ParkTransform pt(_sc, ct.alpha, ct.beta);
  data->id = pt.id;
  data->iq = pt.iq;
  
}

void Foc::inverseCtrl(const SinCos _sc, float _vd, float _vq){
  TrigonTransform tt(_sc, _vd, _vq);
  
  // Inverse Clarke transform
  InverseClarkeTransform ict(tt._trigon1, tt._trigon2);
  
  data->vu = ict.u_ini;
  data->vv = ict.v_ini;
  data->vw = ict.w_ini;
}
