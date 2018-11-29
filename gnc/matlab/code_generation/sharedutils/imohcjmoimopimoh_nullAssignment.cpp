//
// File: imohcjmoimopimoh_nullAssignment.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 11:58:17 2018
//
#include "rtwtypes.h"
#include "imohcjmoimopimoh_nullAssignment.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void imohcjmoimopimoh_nullAssignment(real32_T x_data[], int32_T x_sizes[2],
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
    nrows += (int32_T)idx_data[(int32_T)(i - 1)];
  }

  nrows = (int32_T)(x_sizes[0] - nrows);
  i = 0;
  for (k = 1; k <= nrowx; k++) {
    if ((k > idx_sizes) || (!idx_data[(int32_T)(k - 1)])) {
      for (j = 0; j < 6; j++) {
        x_data[(int32_T)(i + (int32_T)(x_sizes[0] * j))] = x_data[(int32_T)
          ((int32_T)((int32_T)(x_sizes[0] * j) + k) - 1)];
      }

      i++;
    }
  }

  if (1 > nrows) {
    nrows = 0;
  }

  for (nrowx = 0; nrowx < 6; nrowx++) {
    for (i = 0; i <= (int32_T)(nrows - 1); i++) {
      x_data_0[(int32_T)(i + (int32_T)(nrows * nrowx))] = x_data[(int32_T)
        ((int32_T)(x_sizes[0] * nrowx) + i)];
    }
  }

  x_sizes[0] = nrows;
  x_sizes[1] = 6;
  for (nrowx = 0; nrowx < 6; nrowx++) {
    for (i = 0; i <= (int32_T)(nrows - 1); i++) {
      x_data[(int32_T)(i + (int32_T)(x_sizes[0] * nrowx))] = x_data_0[(int32_T)
        ((int32_T)(nrows * nrowx) + i)];
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
