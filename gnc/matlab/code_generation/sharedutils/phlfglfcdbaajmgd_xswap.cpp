//
// File: phlfglfcdbaajmgd_xswap.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:06:07 2018
//
#include "rtwtypes.h"
#include "phlfglfcdbaajmgd_xswap.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void phlfglfcdbaajmgd_xswap(real32_T x_data[])
{
  real32_T temp;
  temp = x_data[0];
  x_data[0] = x_data[3];
  x_data[3] = temp;
  temp = x_data[1];
  x_data[1] = x_data[4];
  x_data[4] = temp;
  temp = x_data[2];
  x_data[2] = x_data[5];
  x_data[5] = temp;
}

//
// File trailer for generated code.
//
// [EOF]
//
