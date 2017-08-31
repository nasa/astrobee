//
// File: jecjhdbaohdbdbim_cat.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:14:10 2017
//
#include "rtwtypes.h"
#include "jecjhdbaohdbdbim_cat.h"

// Function for MATLAB Function: '<S47>/generate_output'
void jecjhdbaohdbdbim_cat(const real32_T varargin_1_data[], const int32_T
  varargin_1_sizes[2], const int32_T varargin_2_sizes[3], real32_T y_data[],
  int32_T y_sizes[3])
{
  int32_T iy;
  int32_T b;
  int32_T j;
  y_sizes[0] = (int8_T)varargin_1_sizes[0];
  y_sizes[1] = (int8_T)varargin_1_sizes[1];
  y_sizes[2] = 16;
  iy = -1;
  b = varargin_1_sizes[0] << 1;
  for (j = 1; j <= b; j++) {
    iy++;
    y_data[iy] = varargin_1_data[j - 1];
  }

  b = (varargin_2_sizes[0] << 1) * 15;
  for (j = 1; j <= b; j++) {
    iy++;
    y_data[iy] = 0.0F;
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
