//
// File: gdjmmgdjekfccjmg_xrot.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed May 17 14:42:31 2017
//
#include "rtwtypes.h"
#include "gdjmmgdjekfccjmg_xrot.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void gdjmmgdjekfccjmg_xrot(real32_T x[9], int32_T ix0, int32_T iy0, real32_T c,
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
