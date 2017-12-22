//
// File: cbimeknolnglpphd_xswap.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Dec  4 08:33:06 2017
//
#include "rtwtypes.h"
#include "cbimeknolnglpphd_xswap.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void cbimeknolnglpphd_xswap(real32_T x[4])
{
  real32_T temp;
  temp = x[0];
  x[0] = x[2];
  x[2] = temp;
  temp = x[1];
  x[1] = x[3];
  x[3] = temp;
}

//
// File trailer for generated code.
//
// [EOF]
//
