//
// File: lfcjpphlnoppphdb_xgerc.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 11:58:17 2018
//
#include "rtwtypes.h"
#include "lfcjpphlnoppphdb_xgerc.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void lfcjpphlnoppphdb_xgerc(int32_T m, int32_T n, real32_T alpha1, int32_T ix0,
  const real32_T y_data[], real32_T A_data[], int32_T ia0, int32_T lda)
{
  int32_T jA;
  int32_T jy;
  real32_T temp;
  int32_T ix;
  int32_T j;
  int32_T b;
  int32_T ijA;
  if (!(alpha1 == 0.0F)) {
    jA = (int32_T)(ia0 - 1);
    jy = 0;
    for (j = 1; j <= n; j++) {
      if (y_data[jy] != 0.0F) {
        temp = y_data[jy] * alpha1;
        ix = ix0;
        b = (int32_T)(m + jA);
        for (ijA = jA; (int32_T)(ijA + 1) <= b; ijA++) {
          A_data[ijA] += A_data[(int32_T)(ix - 1)] * temp;
          ix++;
        }
      }

      jy++;
      jA += lda;
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
