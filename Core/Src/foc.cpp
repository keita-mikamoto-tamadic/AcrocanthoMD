#include "foc.h"

#include "main.h"
#include "user_math.h"
#include "ma735_enc.h"
#include "sens_cur.h"

Foc foc;

extern MA735Enc ang;
extern SensCur senscur;

using namespace Acrocantho;

void Foc::forwardCtrl(const SinCos _sc){
  static SensCur::SensCurData* senscurdata = senscur.getData();
  
  // Clarke transform - direct calculation
  const float alpha = absqrt1 * senscurdata->curU;
  const float beta = (absqrt2 * senscurdata->curU) + (absqrt3 * senscurdata->curV);
  
  // Park transform - direct calculation
  data.id = _sc.c * alpha + _sc.s * beta;
  data.iq = -_sc.s * alpha + _sc.c * beta;
}

void Foc::inverseCtrl(const SinCos _sc, float _vd, float _vq){
  // Inverse Park transform - direct calculation
  const float trigon1 = _sc.c * _vd - _sc.s * _vq;
  const float trigon2 = _sc.s * _vd + _sc.c * _vq;
  
  // Inverse Clarke transform - direct calculation
  data.vu = trigon1 * invsqrt1;
  data.vv = -(trigon1 * invsqrt3) + trigon2 * invsqrt2;
  data.vw = -(trigon1 * invsqrt3) - trigon2 * invsqrt2;
}
