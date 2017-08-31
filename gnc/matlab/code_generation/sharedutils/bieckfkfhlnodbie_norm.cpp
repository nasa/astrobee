//
// File: bieckfkfhlnodbie_norm.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1090
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Mon Mar  6 17:33:02 2017
//
#include "rtwtypes.h"
#include <math.h>
#include "bieckfkfhlnodbie_norm.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
real32_T bieckfkfhlnodbie_norm(const real32_T x[2])
{
  real32_T y;
  real32_T scale;
  real32_T absxk;
  real32_T t;
  scale = 1.17549435E-38F;
  absxk = (real32_T)fabs((real_T)x[0]);
  if (absxk > 1.17549435E-38F) {
    y = 1.0F;
    scale = absxk;
  } else {
    t = absxk / 1.17549435E-38F;
    y = t * t;
  }

  absxk = (real32_T)fabs((real_T)x[1]);
  if (absxk > scale) {
    t = scale / absxk;
    y = y * t * t + 1.0F;
    scale = absxk;
  } else {
    t = absxk / scale;
    y += t * t;
  }

  return scale * (real32_T)sqrt((real_T)y);
}

//
// File trailer for generated code.
//
// [EOF]
//
