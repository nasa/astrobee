//
// File: fkfkbiecjecjbaie_cat.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed May 17 14:43:55 2017
//
#include "rtwtypes.h"
#include "fkfkbiecjecjbaie_cat.h"

// Function for MATLAB Function: '<S47>/generate_output'
void fkfkbiecjecjbaie_cat(const int32_T varargin_1_sizes, const int32_T
  varargin_2_sizes[2], uint8_T y_data[], int32_T y_sizes[2])
{
  int32_T iy;
  int32_T j;
  int32_T b_j;
  y_sizes[0] = (int32_T)(int8_T)varargin_1_sizes;
  y_sizes[1] = 16;
  iy = -1;
  for (j = 1; j <= varargin_1_sizes; j++) {
    iy++;
    y_data[iy] = 1U;
  }

  j = (int32_T)(varargin_2_sizes[0] * 15);
  for (b_j = 1; b_j <= j; b_j++) {
    iy++;
    y_data[iy] = 0U;
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
