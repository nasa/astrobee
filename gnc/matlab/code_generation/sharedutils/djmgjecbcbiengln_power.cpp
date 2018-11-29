//
// File: djmgjecbcbiengln_power.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 11:58:17 2018
//
#include "rtwtypes.h"
#include "djmgjecbcbiengln_power.h"

// Function for MATLAB Function: '<S16>/Compute Residual and H'
void djmgjecbcbiengln_power(const real32_T a_data[], const int32_T a_sizes,
  real32_T y_data[], int32_T *y_sizes)
{
  int32_T k;
  *y_sizes = (int32_T)(uint8_T)a_sizes;
  for (k = 0; (int32_T)(k + 1) <= a_sizes; k++) {
    y_data[k] = a_data[k] * a_data[k];
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
