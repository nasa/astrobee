//
// File: mgdbbiekfknonglf_nullAssignment.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:06:07 2018
//
#include "rtwtypes.h"
#include "mgdbbiekfknonglf_nullAssignment.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void mgdbbiekfknonglf_nullAssignment(real32_T x_data[], int32_T *x_sizes, const
  boolean_T idx_data[], const int32_T idx_sizes)
{
  int32_T nxin;
  int32_T nxout;
  int32_T k0;
  int32_T k;
  real32_T x_data_0[100];
  nxin = *x_sizes;
  nxout = 0;
  for (k0 = 1; k0 <= idx_sizes; k0++) {
    nxout += (int32_T)idx_data[(int32_T)(k0 - 1)];
  }

  nxout = (int32_T)(*x_sizes - nxout);
  k0 = -1;
  for (k = 1; k <= nxin; k++) {
    if ((k > idx_sizes) || (!idx_data[(int32_T)(k - 1)])) {
      k0++;
      x_data[k0] = x_data[(int32_T)(k - 1)];
    }
  }

  if (1 > nxout) {
    nxout = 0;
  }

  for (k0 = 0; k0 <= (int32_T)(nxout - 1); k0++) {
    x_data_0[k0] = x_data[k0];
  }

  *x_sizes = nxout;
  for (k0 = 0; k0 <= (int32_T)(nxout - 1); k0++) {
    x_data[k0] = x_data_0[k0];
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
