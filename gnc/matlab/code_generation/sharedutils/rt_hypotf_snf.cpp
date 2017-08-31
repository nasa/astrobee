//
// File: rt_hypotf_snf.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed May 17 14:42:31 2017
//
#include "rtwtypes.h"
#include <math.h>
#include "rt_nonfinite.h"
#include "rt_hypotf_snf.h"

real32_T rt_hypotf_snf(real32_T u0, real32_T u1)
{
  real32_T y;
  real32_T a;
  a = (real32_T)fabs((real_T)u0);
  y = (real32_T)fabs((real_T)u1);
  if (a < y) {
    a /= y;
    y *= (real32_T)sqrt((real_T)(a * a + 1.0F));
  } else if (a > y) {
    y /= a;
    y = (real32_T)sqrt((real_T)(y * y + 1.0F)) * a;
  } else {
    if (!rtIsNaNF(y)) {
      y = a * 1.41421354F;
    }
  }

  return y;
}

//
// File trailer for generated code.
//
// [EOF]
//
