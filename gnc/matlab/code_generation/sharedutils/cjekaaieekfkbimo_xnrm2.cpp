//
// File: cjekaaieekfkbimo_xnrm2.cpp
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed May 17 14:43:17 2017
//
#include "rtwtypes.h"
#include <math.h>
#include "cjekaaieekfkbimo_xnrm2.h"

// Function for MATLAB Function: '<S9>/MATLAB Function'
real32_T cjekaaieekfkbimo_xnrm2(int32_T n, const real32_T x[36], int32_T ix0)
{
  real32_T y;
  real32_T scale;
  int32_T kend;
  real32_T absxk;
  real32_T t;
  int32_T k;
  y = 0.0F;
  if (!(n < 1)) {
    if (n == 1) {
      y = (real32_T)fabs((real_T)x[(int32_T)(ix0 - 1)]);
    } else {
      scale = 1.17549435E-38F;
      kend = (int32_T)((int32_T)(ix0 + n) - 1);
      for (k = ix0; k <= kend; k++) {
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
