//
// File: biekmglfbimoknoh_eml_null_assignment.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include "biekmglfbimoknoh_eml_null_assignment.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void biekmglfbimoknoh_eml_null_assignment(real32_T x_data[], int32_T *x_sizes,
  const boolean_T idx_data[], const int32_T idx_sizes)
{
  int32_T nxin;
  int32_T nxout;
  int32_T k0;
  int32_T k;
  real32_T x_data_0[100];
  nxin = *x_sizes;
  nxout = 0;
  for (k0 = 1; k0 <= idx_sizes; k0++) {
    nxout += idx_data[k0 - 1];
  }

  nxout = *x_sizes - nxout;
  k0 = -1;
  for (k = 1; k <= nxin; k++) {
    if ((k > idx_sizes) || (!idx_data[k - 1])) {
      k0++;
      x_data[k0] = x_data[k - 1];
    }
  }

  if (*x_sizes != 1) {
    if (1 > nxout) {
      nxout = 0;
    }

    for (k0 = 0; k0 < nxout; k0++) {
      x_data_0[k0] = x_data[k0];
    }

    *x_sizes = nxout;
    for (k0 = 0; k0 < nxout; k0++) {
      x_data[k0] = x_data_0[k0];
    }
  } else if (1 > nxout) {
    *x_sizes = 0;
  } else {
    *x_sizes = nxout;
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
