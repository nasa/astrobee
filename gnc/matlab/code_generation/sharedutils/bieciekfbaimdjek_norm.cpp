//
// File: bieciekfbaimdjek_norm.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1090
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Thu Mar  2 13:46:15 2017
//
#include "rtwtypes.h"
#include <math.h>
#include "bieciekfbaimdjek_norm.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
real32_T bieciekfbaimdjek_norm(const real32_T x_data[], const int32_T x_sizes)
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  int32_T k;
  if (x_sizes == 0) {
    y = 0.0F;
  } else {
    y = 0.0F;
    if (x_sizes == 1) {
      y = (real32_T)fabs((real_T)x_data[0]);
    } else {
      scale = 1.17549435E-38F;
      for (k = 1; k <= x_sizes; k++) {
        absxk = (real32_T)fabs((real_T)x_data[k - 1]);
        if (absxk > scale) {
          t = scale / absxk;
          y = y * t * t + 1.0F;
          scale = absxk;
        } else {
          t = absxk / scale;
          y += t * t;
        }
      }

      y = scale * (real32_T)sqrt((real_T)y);
    }
  }

  return y;
}

//
// File trailer for generated code.
//
// [EOF]
//
