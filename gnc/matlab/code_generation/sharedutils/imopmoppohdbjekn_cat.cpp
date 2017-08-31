//
// File: imopmoppohdbjekn_cat.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:14:10 2017
//
#include "rtwtypes.h"
#include "imopmoppohdbjekn_cat.h"

// Function for MATLAB Function: '<S47>/generate_output'
void imopmoppohdbjekn_cat(const uint8_T varargin_2_data[], const int32_T
  varargin_2_sizes[2], const uint8_T varargin_3_data[], const int32_T
  varargin_3_sizes[2], uint8_T y_data[], int32_T y_sizes[2])
{
  int32_T iy;
  int32_T j;
  int32_T b_j;
  y_sizes[0] = 50;
  y_sizes[1] = (int8_T)((int8_T)(1 + varargin_2_sizes[1]) + varargin_3_sizes[1]);
  iy = -1;
  for (j = 0; j < 50; j++) {
    iy++;
    y_data[iy] = 0U;
  }

  j = 50 * varargin_2_sizes[1];
  for (b_j = 1; b_j <= j; b_j++) {
    iy++;
    y_data[iy] = varargin_2_data[b_j - 1];
  }

  j = 50 * varargin_3_sizes[1];
  for (b_j = 1; b_j <= j; b_j++) {
    iy++;
    y_data[iy] = varargin_3_data[b_j - 1];
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
