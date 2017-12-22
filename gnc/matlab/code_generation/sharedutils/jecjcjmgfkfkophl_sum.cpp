//
// File: jecjcjmgfkfkophl_sum.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Dec  4 08:34:45 2017
//
#include "rtwtypes.h"
#include "jecjcjmgfkfkophl_sum.h"

// Function for MATLAB Function: '<S83>/generate_output'
real_T jecjcjmgfkfkophl_sum(const boolean_T x[50])
{
  real_T y;
  int32_T k;
  y = (real_T)x[0];
  for (k = 0; k < 49; k++) {
    y += (real_T)x[(int32_T)(k + 1)];
  }

  return y;
}

//
// File trailer for generated code.
//
// [EOF]
//
