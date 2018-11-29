//
// File: mglncbaaiekflfcb_cat.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 12:00:33 2018
//
#include "rtwtypes.h"
#include "mglncbaaiekflfcb_cat.h"

// Function for MATLAB Function: '<S83>/generate_output'
void mglncbaaiekflfcb_cat(const real32_T varargin_1_data[], const int32_T
  varargin_1_sizes[2], const int32_T varargin_2_sizes[3], real32_T y_data[],
  int32_T y_sizes[3])
{
  int32_T iy;
  int32_T b;
  int32_T j;
  y_sizes[0] = (int32_T)(int8_T)varargin_1_sizes[0];
  y_sizes[1] = 2;
  y_sizes[2] = 16;
  iy = -1;
  b = (int32_T)(varargin_1_sizes[0] << 1);
  for (j = 1; j <= b; j++) {
    iy++;
    y_data[iy] = varargin_1_data[(int32_T)(j - 1)];
  }

  b = (int32_T)((int32_T)(varargin_2_sizes[0] << 1) * 15);
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
