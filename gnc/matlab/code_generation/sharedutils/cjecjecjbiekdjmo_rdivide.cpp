//
// File: cjecjecjbiekdjmo_rdivide.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include "cjecjecjbiekdjmo_rdivide.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void cjecjecjbiekdjmo_rdivide(const real32_T x_data[], const int32_T x_sizes[2],
  const real32_T y_data[], real32_T z_data[], int32_T z_sizes[2])
{
  int32_T i;
  int32_T loop_ub;
  z_sizes[0] = 2;
  z_sizes[1] = x_sizes[1];
  loop_ub = x_sizes[0] * x_sizes[1];
  for (i = 0; i < loop_ub; i++) {
    z_data[i] = x_data[i] / y_data[i];
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
