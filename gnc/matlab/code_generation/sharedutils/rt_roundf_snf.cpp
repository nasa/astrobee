//
// File: rt_roundf_snf.cpp
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 11:59:30 2018
//
#include "rtwtypes.h"
#include <math.h>
#include "rt_roundf_snf.h"

real32_T rt_roundf_snf(real32_T u)
{
  real32_T y;
  if ((real32_T)fabs((real_T)u) < 8.388608E+6F) {
    if (u >= 0.5F) {
      y = (real32_T)floor((real_T)(u + 0.5F));
    } else if (u > -0.5F) {
      y = u * 0.0F;
    } else {
      y = (real32_T)ceil((real_T)(u - 0.5F));
    }
  } else {
    y = u;
  }

  return y;
}

//
// File trailer for generated code.
//
// [EOF]
//
