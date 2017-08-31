//
// File: bimoiekfmgdbjmop_rdivide.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include "bimoiekfmgdbjmop_rdivide.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void bimoiekfmgdbjmop_rdivide(const real32_T x[2], real32_T y, real32_T z[2])
{
  z[0] = x[0] / y;
  z[1] = x[1] / y;
}

//
// File trailer for generated code.
//
// [EOF]
//
