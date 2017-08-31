//
// File: cbiecbiehlngfkno_power.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include "cbiecbiehlngfkno_power.h"

// Function for MATLAB Function: '<S16>/Compute Residual and H'
void cbiecbiehlngfkno_power(const real32_T a_data[], const int32_T a_sizes,
  real32_T y_data[], int32_T *y_sizes)
{
  int32_T k;
  *y_sizes = (uint8_T)a_sizes;
  for (k = 0; k + 1 <= a_sizes; k++) {
    y_data[k] = a_data[k] * a_data[k];
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
