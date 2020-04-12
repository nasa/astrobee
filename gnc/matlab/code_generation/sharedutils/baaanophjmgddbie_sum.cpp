//
// File: baaanophjmgddbie_sum.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:06:07 2018
//
#include "rtwtypes.h"
#include "baaanophjmgddbie_sum.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
real_T baaanophjmgddbie_sum(const int32_T x[16])
{
  real_T y;
  int32_T k;
  y = (real_T)x[0];
  for (k = 0; k < 15; k++) {
    y += (real_T)x[(int32_T)(k + 1)];
  }

  return y;
}

//
// File trailer for generated code.
//
// [EOF]
//
