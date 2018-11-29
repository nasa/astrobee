//
// File: jmohnoppbieciecj_xrot.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 11:58:17 2018
//
#include "rtwtypes.h"
#include "jmohnoppbieciecj_xrot.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void jmohnoppbieciecj_xrot(real32_T x_data[], int32_T ix0, int32_T iy0, real32_T
  c, real32_T s)
{
  int32_T ix;
  int32_T iy;
  real32_T temp;
  ix = (int32_T)(ix0 - 1);
  iy = (int32_T)(iy0 - 1);
  temp = c * x_data[ix] + s * x_data[iy];
  x_data[iy] = c * x_data[iy] - s * x_data[ix];
  x_data[ix] = temp;
  iy++;
  ix++;
  temp = c * x_data[ix] + s * x_data[iy];
  x_data[iy] = c * x_data[iy] - s * x_data[ix];
  x_data[ix] = temp;
  iy++;
  ix++;
  temp = c * x_data[ix] + s * x_data[iy];
  x_data[iy] = c * x_data[iy] - s * x_data[ix];
  x_data[ix] = temp;
}

//
// File trailer for generated code.
//
// [EOF]
//
