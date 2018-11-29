//
// File: glfkdbaibaaakfcb_xscal.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 11:58:17 2018
//
#include "rtwtypes.h"
#include "glfkdbaibaaakfcb_xscal.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void glfkdbaibaaakfcb_xscal(real32_T a, real32_T x[16], int32_T ix0)
{
  int32_T k;
  for (k = ix0; k <= (int32_T)(ix0 + 3); k++) {
    x[(int32_T)(k - 1)] *= a;
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
