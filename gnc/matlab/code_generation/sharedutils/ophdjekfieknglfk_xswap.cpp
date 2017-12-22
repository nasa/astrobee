//
// File: ophdjekfieknglfk_xswap.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Dec  4 08:33:06 2017
//
#include "rtwtypes.h"
#include "ophdjekfieknglfk_xswap.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void ophdjekfieknglfk_xswap(int32_T n, real32_T x_data[], int32_T ix0, int32_T
  iy0)
{
  int32_T ix;
  int32_T iy;
  real32_T temp;
  int32_T k;
  ix = (int32_T)(ix0 - 1);
  iy = (int32_T)(iy0 - 1);
  for (k = 1; k <= n; k++) {
    temp = x_data[ix];
    x_data[ix] = x_data[iy];
    x_data[iy] = temp;
    ix++;
    iy++;
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
