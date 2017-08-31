//
// File: ohdjbaimcjmojecj_rows_differ.cpp
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
#include "ohdjbaimcjmojecj_rows_differ.h"

// Function for MATLAB Function: '<S47>/generate_output'
boolean_T ohdjbaimcjmojecj_rows_differ(const real32_T b_data[], const int32_T
  b_sizes[2], int32_T k0, int32_T k)
{
  boolean_T p;
  int32_T j;
  real32_T absxk;
  int32_T exponent;
  boolean_T exitg1;
  p = false;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 2)) {
    absxk = (real32_T)fabs((real_T)(b_data[(b_sizes[0] * j + k) - 1] / 2.0F));
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

    if (!(((real32_T)fabs((real_T)(b_data[(b_sizes[0] * j + k) - 1] - b_data
            [(b_sizes[0] * j + k0) - 1])) < absxk) || (rtIsInfF(b_data[(b_sizes
            [0] * j + k0) - 1]) && rtIsInfF(b_data[(b_sizes[0] * j + k) - 1]) &&
          ((b_data[(b_sizes[0] * j + k0) - 1] > 0.0F) == (b_data[(b_sizes[0] * j
             + k) - 1] > 0.0F))))) {
      p = true;
      exitg1 = true;
    } else {
      j++;
    }
  }

  return p;
}

//
// File trailer for generated code.
//
// [EOF]
//
