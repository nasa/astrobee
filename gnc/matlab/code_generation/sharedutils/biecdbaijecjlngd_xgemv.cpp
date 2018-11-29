//
// File: biecdbaijecjlngd_xgemv.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 11:58:17 2018
//
#include "rtwtypes.h"
#include "biecdbaijecjlngd_xgemv.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void biecdbaijecjlngd_xgemv(int32_T m, int32_T n, const real32_T A_data[],
  int32_T ia0, int32_T lda, const real32_T x_data[], int32_T ix0, real32_T
  y_data[])
{
  int32_T ix;
  real32_T c;
  int32_T iy;
  int32_T b;
  int32_T iac;
  int32_T d;
  int32_T ia;
  if (n != 0) {
    for (iy = 1; iy <= n; iy++) {
      y_data[(int32_T)(iy - 1)] = 0.0F;
    }

    iy = 0;
    b = (int32_T)((int32_T)((int32_T)(n - 1) * lda) + ia0);
    iac = ia0;
    while ((lda > 0) && (iac <= b)) {
      ix = ix0;
      c = 0.0F;
      d = (int32_T)((int32_T)(iac + m) - 1);
      for (ia = iac; ia <= d; ia++) {
        c += A_data[(int32_T)(ia - 1)] * x_data[(int32_T)(ix - 1)];
        ix++;
      }

      y_data[iy] += c;
      iy++;
      iac += lda;
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
