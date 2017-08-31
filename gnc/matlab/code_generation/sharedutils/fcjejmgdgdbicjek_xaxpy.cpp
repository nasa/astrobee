//
// File: fcjejmgdgdbicjek_xaxpy.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed May 17 14:42:31 2017
//
#include "rtwtypes.h"
#include "fcjejmgdgdbicjek_xaxpy.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void fcjejmgdgdbicjek_xaxpy(real32_T a, real32_T y[4])
{
  if (!(a == 0.0F)) {
    y[2] += a * y[0];
    y[3] += a * y[1];
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
