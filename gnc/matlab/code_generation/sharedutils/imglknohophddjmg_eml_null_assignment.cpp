//
// File: imglknohophddjmg_eml_null_assignment.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include "imglknohophddjmg_eml_null_assignment.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void imglknohophddjmg_eml_null_assignment(real32_T x_data[], int32_T x_sizes[2],
  const boolean_T idx_data[], const int32_T idx_sizes)
{
  int32_T nrowx;
  int32_T nrows;
  int32_T i;
  int32_T k;
  int32_T j;
  real32_T x_data_0[600];
  nrowx = x_sizes[0];
  nrows = 0;
  for (i = 1; i <= idx_sizes; i++) {
    nrows += idx_data[i - 1];
  }

  nrows = x_sizes[0] - nrows;
  i = 0;
  for (k = 1; k <= nrowx; k++) {
    if ((k > idx_sizes) || (!idx_data[k - 1])) {
      for (j = 0; j < 6; j++) {
        x_data[i + x_sizes[0] * j] = x_data[(x_sizes[0] * j + k) - 1];
      }

      i++;
    }
  }

  if (1 > nrows) {
    nrows = 0;
  }

  for (nrowx = 0; nrowx < 6; nrowx++) {
    for (i = 0; i < nrows; i++) {
      x_data_0[i + nrows * nrowx] = x_data[x_sizes[0] * nrowx + i];
    }
  }

  x_sizes[0] = nrows;
  x_sizes[1] = 6;
  for (nrowx = 0; nrowx < 6; nrowx++) {
    for (i = 0; i < nrows; i++) {
      x_data[i + x_sizes[0] * nrowx] = x_data_0[nrows * nrowx + i];
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
