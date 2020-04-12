//
// File: fkngdjekgdjepphl_sum.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:06:07 2018
//
#include "rtwtypes.h"
#include "fkngdjekgdjepphl_sum.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void fkngdjekgdjepphl_sum(const uint8_T x[800], real_T y[50])
{
  real_T s;
  int32_T j;
  int32_T k;
  for (j = 0; j < 50; j++) {
    s = (real_T)x[j];
    for (k = 0; k < 15; k++) {
      s += (real_T)x[(int32_T)((int32_T)((int32_T)(k + 1) * 50) + j)];
    }

    y[j] = s;
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
