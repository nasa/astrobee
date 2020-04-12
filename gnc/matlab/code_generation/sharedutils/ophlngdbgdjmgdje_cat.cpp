//
// File: ophlngdbgdjmgdje_cat.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:08:00 2018
//
#include "rtwtypes.h"
#include "ophlngdbgdjmgdje_cat.h"

// Function for MATLAB Function: '<S83>/generate_output'
void ophlngdbgdjmgdje_cat(const real32_T varargin_2_data[], const int32_T
  varargin_2_sizes[3], const real32_T varargin_3_data[], const int32_T
  varargin_3_sizes[3], real32_T y_data[], int32_T y_sizes[3])
{
  int32_T iy;
  int32_T j;
  int32_T b_j;
  y_sizes[0] = 50;
  y_sizes[1] = 2;
  y_sizes[2] = 16;
  iy = -1;
  for (j = 0; j < 100; j++) {
    iy++;
    y_data[iy] = 0.0F;
  }

  j = (int32_T)(100 * varargin_2_sizes[2]);
  for (b_j = 1; b_j <= j; b_j++) {
    iy++;
    y_data[iy] = varargin_2_data[(int32_T)(b_j - 1)];
  }

  j = (int32_T)(100 * varargin_3_sizes[2]);
  for (b_j = 1; b_j <= j; b_j++) {
    iy++;
    y_data[iy] = varargin_3_data[(int32_T)(b_j - 1)];
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
