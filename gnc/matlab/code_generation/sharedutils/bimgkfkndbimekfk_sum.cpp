//
// File: bimgkfkndbimekfk_sum.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include "bimgkfkndbimekfk_sum.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void bimgkfkndbimekfk_sum(const uint8_T x[800], real_T y[50])
{
  real_T s;
  int32_T ix;
  int32_T iy;
  int32_T ixstart;
  int32_T j;
  int32_T k;
  iy = -1;
  ixstart = -1;
  for (j = 0; j < 50; j++) {
    ixstart++;
    ix = ixstart;
    s = x[ixstart];
    for (k = 0; k < 15; k++) {
      ix += 50;
      s += (real_T)x[ix];
    }

    iy++;
    y[iy] = s;
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
