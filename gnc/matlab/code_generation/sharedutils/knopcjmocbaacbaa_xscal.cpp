//
// File: knopcjmocbaacbaa_xscal.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:06:07 2018
//
#include "rtwtypes.h"
#include "knopcjmocbaacbaa_xscal.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void knopcjmocbaacbaa_xscal(real32_T a, real32_T x[4], int32_T ix0)
{
  int32_T k;
  for (k = ix0; k <= (int32_T)(ix0 + 1); k++) {
    x[(int32_T)(k - 1)] *= a;
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
