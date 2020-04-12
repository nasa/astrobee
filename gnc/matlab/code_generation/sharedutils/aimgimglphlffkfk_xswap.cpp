//
// File: aimgimglphlffkfk_xswap.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:06:07 2018
//
#include "rtwtypes.h"
#include "aimgimglphlffkfk_xswap.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void aimgimglphlffkfk_xswap(real32_T x[16], int32_T ix0, int32_T iy0)
{
  int32_T ix;
  int32_T iy;
  real32_T temp;
  ix = (int32_T)(ix0 - 1);
  iy = (int32_T)(iy0 - 1);
  temp = x[ix];
  x[ix] = x[iy];
  x[iy] = temp;
  ix++;
  iy++;
  temp = x[ix];
  x[ix] = x[iy];
  x[iy] = temp;
  ix++;
  iy++;
  temp = x[ix];
  x[ix] = x[iy];
  x[iy] = temp;
  ix++;
  iy++;
  temp = x[ix];
  x[ix] = x[iy];
  x[iy] = temp;
}

//
// File trailer for generated code.
//
// [EOF]
//
