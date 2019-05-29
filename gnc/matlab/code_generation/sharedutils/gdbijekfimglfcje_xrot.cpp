//
// File: gdbijekfimglfcje_xrot.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:06:07 2018
//
#include "rtwtypes.h"
#include "gdbijekfimglfcje_xrot.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void gdbijekfimglfcje_xrot(real32_T x[4], int32_T ix0, int32_T iy0, real32_T c,
  real32_T s)
{
  int32_T ix;
  int32_T iy;
  real32_T temp;
  ix = (int32_T)(ix0 - 1);
  iy = (int32_T)(iy0 - 1);
  temp = c * x[ix] + s * x[iy];
  x[iy] = c * x[iy] - s * x[ix];
  x[ix] = temp;
  iy++;
  ix++;
  temp = c * x[ix] + s * x[iy];
  x[iy] = c * x[iy] - s * x[ix];
  x[ix] = temp;
}

//
// File trailer for generated code.
//
// [EOF]
//
