//
// File: moppbaaafkfkimgd_diag.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:06:07 2018
//
#include "rtwtypes.h"
#include "moppbaaafkfkimgd_diag.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void moppbaaafkfkimgd_diag(const real_T v_data[], const int32_T v_sizes, real_T
  d_data[], int32_T d_sizes[2])
{
  int32_T i;
  int32_T loop_ub;
  int8_T b_idx_0;
  int8_T b_idx_1;
  b_idx_0 = (int8_T)v_sizes;
  b_idx_1 = (int8_T)v_sizes;
  d_sizes[0] = (int32_T)b_idx_0;
  d_sizes[1] = (int32_T)b_idx_1;
  loop_ub = (int32_T)((int32_T)b_idx_0 * (int32_T)b_idx_1);
  for (i = 0; i <= (int32_T)(loop_ub - 1); i++) {
    d_data[i] = 0.0;
  }

  for (loop_ub = 0; (int32_T)(loop_ub + 1) <= v_sizes; loop_ub++) {
    d_data[(int32_T)(loop_ub + (int32_T)((int32_T)b_idx_0 * loop_ub))] =
      v_data[loop_ub];
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
