//
// File: lngdpphlmgdjekno_skip_to_last_equal_value.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:14:10 2017
//
#include "rtwtypes.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include <math.h>
#include "lngdpphlmgdjekno_skip_to_last_equal_value.h"

// Function for MATLAB Function: '<S47>/generate_output'
real32_T lngdpphlmgdjekno_skip_to_last_equal_value(int32_T *k, const real32_T x
  [50])
{
  real32_T xk;
  real32_T absxk;
  int32_T exponent;
  boolean_T exitg1;
  xk = x[*k - 1];
  exitg1 = false;
  while ((!exitg1) && (*k < 50)) {
    absxk = (real32_T)fabs((real_T)(xk / 2.0F));
    if ((!rtIsInfF(absxk)) && (!rtIsNaNF(absxk))) {
      if (absxk <= 1.17549435E-38F) {
        absxk = 1.4013E-45F;
      } else {
        frexp((real_T)absxk, &exponent);
        absxk = (real32_T)ldexp((real_T)1.0F, exponent - 24);
      }
    } else {
      absxk = (rtNaNF);
    }

    if (((real32_T)fabs((real_T)(xk - x[*k])) < absxk) || (rtIsInfF(x[*k]) &&
         rtIsInfF(xk) && ((x[*k] > 0.0F) == (xk > 0.0F)))) {
      (*k)++;
    } else {
      exitg1 = true;
    }
  }

  return xk;
}

//
// File trailer for generated code.
//
// [EOF]
//
