//
// File: cbiephdblnopophl_cat.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:08:00 2018
//
#include "rtwtypes.h"
#include "cbiephdblnopophl_cat.h"

// Function for MATLAB Function: '<S83>/generate_output'
void cbiephdblnopophl_cat(const real32_T varargin_2[1500], real32_T y[1600])
{
  int32_T iy;
  int32_T j;
  iy = -1;
  for (j = 0; j < 100; j++) {
    iy++;
    y[iy] = 0.0F;
  }

  for (j = 0; j < 1500; j++) {
    iy++;
    y[iy] = varargin_2[j];
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
