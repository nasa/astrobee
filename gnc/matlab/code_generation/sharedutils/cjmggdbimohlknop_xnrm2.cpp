//
// File: cjmggdbimohlknop_xnrm2.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:06:07 2018
//
#include "rtwtypes.h"
#include <math.h>
#include "cjmggdbimohlknop_xnrm2.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
real32_T cjmggdbimohlknop_xnrm2(const real32_T x[3], int32_T ix0)
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  int32_T k;
  y = 0.0F;
  scale = 1.17549435E-38F;
  for (k = ix0; k <= (int32_T)(ix0 + 1); k++) {
    absxk = (real32_T)fabs((real_T)x[(int32_T)(k - 1)]);
    if (absxk > scale) {
      t = scale / absxk;
      y = y * t * t + 1.0F;
      scale = absxk;
    } else {
      t = absxk / scale;
      y += t * t;
    }
  }

  return scale * (real32_T)sqrt((real_T)y);
}

//
// File trailer for generated code.
//
// [EOF]
//
