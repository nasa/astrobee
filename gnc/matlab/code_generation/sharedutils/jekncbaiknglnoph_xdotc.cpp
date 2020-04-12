//
// File: jekncbaiknglnoph_xdotc.cpp
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:07:06 2018
//
#include "rtwtypes.h"
#include "jekncbaiknglnoph_xdotc.h"

// Function for MATLAB Function: '<S12>/MATLAB Function'
real32_T jekncbaiknglnoph_xdotc(int32_T n, const real32_T x[36], int32_T ix0,
  const real32_T y[36], int32_T iy0)
{
  real32_T d;
  int32_T ix;
  int32_T iy;
  int32_T k;
  d = 0.0F;
  if (!(n < 1)) {
    ix = ix0;
    iy = iy0;
    for (k = 1; k <= n; k++) {
      d += x[(int32_T)(ix - 1)] * y[(int32_T)(iy - 1)];
      ix++;
      iy++;
    }
  }

  return d;
}

//
// File trailer for generated code.
//
// [EOF]
//
