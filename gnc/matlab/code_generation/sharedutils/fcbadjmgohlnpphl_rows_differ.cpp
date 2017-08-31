//
// File: fcbadjmgohlnpphl_rows_differ.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed May 17 14:43:55 2017
//
#include "rtwtypes.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include <math.h>
#include "fcbadjmgohlnpphl_rows_differ.h"

// Function for MATLAB Function: '<S47>/generate_output'
boolean_T fcbadjmgohlnpphl_rows_differ(const real32_T b_data[], const int32_T
  b_sizes[2], int32_T k0, int32_T k)
{
  boolean_T p;
  int32_T j;
  boolean_T b_p;
  real32_T absxk;
  int32_T exponent;
  boolean_T exitg1;
  p = false;
  j = 0;
  exitg1 = false;
  while ((!exitg1) && (j < 2)) {
    absxk = (real32_T)fabs((real_T)(b_data[(int32_T)((int32_T)((int32_T)
      (b_sizes[0] * j) + k) - 1)] / 2.0F));
    if ((!rtIsInfF(absxk)) && (!rtIsNaNF(absxk))) {
      if (absxk <= 1.17549435E-38F) {
        absxk = 1.4013E-45F;
      } else {
        frexp((real_T)absxk, &exponent);
        absxk = (real32_T)ldexp((real_T)1.0F, (int32_T)(exponent - 24));
      }
    } else {
      absxk = (rtNaNF);
    }

    if (((real32_T)fabs((real_T)(b_data[(int32_T)((int32_T)((int32_T)(b_sizes[0]
              * j) + k) - 1)] - b_data[(int32_T)((int32_T)((int32_T)(b_sizes[0] *
              j) + k0) - 1)])) < absxk) || (rtIsInfF(b_data[(int32_T)((int32_T)
           ((int32_T)(b_sizes[0] * j) + k0) - 1)]) && rtIsInfF(b_data[(int32_T)
          ((int32_T)((int32_T)(b_sizes[0] * j) + k) - 1)]) && ((b_data[(int32_T)
           ((int32_T)((int32_T)(b_sizes[0] * j) + k0) - 1)] > 0.0F) == (b_data
           [(int32_T)((int32_T)((int32_T)(b_sizes[0] * j) + k) - 1)] > 0.0F))))
    {
      b_p = true;
    } else {
      b_p = false;
    }

    if (!b_p) {
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
