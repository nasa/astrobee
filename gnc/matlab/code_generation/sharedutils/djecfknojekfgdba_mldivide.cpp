//
// File: djecfknojekfgdba_mldivide.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Dec  4 08:33:06 2017
//
#include "rtwtypes.h"
#include <math.h>
#include <string.h>
#include "djecfknojekfgdba_mldivide.h"

// Function for MATLAB Function: '<S94>/MATLAB Function'
void djecfknojekfgdba_mldivide(const real32_T A[225], real32_T B[225])
{
  real32_T temp;
  real32_T b_A[225];
  int8_T ipiv[15];
  int32_T j;
  real32_T s;
  int32_T iy;
  int32_T c_ix;
  int32_T d;
  int32_T ijA;
  int32_T b_jBcol;
  int32_T b_kAcol;
  int32_T c_i;
  memcpy(&b_A[0], &A[0], (uint32_T)(225U * sizeof(real32_T)));
  for (j = 0; j < 15; j++) {
    ipiv[j] = (int8_T)(int32_T)(1 + j);
  }

  for (j = 0; j < 14; j++) {
    b_jBcol = (int32_T)(j << 4);
    iy = 0;
    b_kAcol = b_jBcol;
    temp = (real32_T)fabs((real_T)b_A[b_jBcol]);
    for (c_i = 2; c_i <= (int32_T)(15 - j); c_i++) {
      b_kAcol++;
      s = (real32_T)fabs((real_T)b_A[b_kAcol]);
      if (s > temp) {
        iy = (int32_T)(c_i - 1);
        temp = s;
      }
    }

    if (b_A[(int32_T)(b_jBcol + iy)] != 0.0F) {
      if (iy != 0) {
        ipiv[j] = (int8_T)(int32_T)((int32_T)(j + iy) + 1);
        b_kAcol = j;
        iy += j;
        for (c_i = 0; c_i < 15; c_i++) {
          temp = b_A[b_kAcol];
          b_A[b_kAcol] = b_A[iy];
          b_A[iy] = temp;
          b_kAcol += 15;
          iy += 15;
        }
      }

      iy = (int32_T)((int32_T)(b_jBcol - j) + 15);
      for (b_kAcol = (int32_T)(b_jBcol + 1); (int32_T)(b_kAcol + 1) <= iy;
           b_kAcol++) {
        b_A[b_kAcol] /= b_A[b_jBcol];
      }
    }

    iy = b_jBcol;
    b_kAcol = (int32_T)(b_jBcol + 15);
    for (c_i = 1; c_i <= (int32_T)(14 - j); c_i++) {
      temp = b_A[b_kAcol];
      if (b_A[b_kAcol] != 0.0F) {
        c_ix = (int32_T)(b_jBcol + 1);
        d = (int32_T)((int32_T)(iy - j) + 30);
        for (ijA = (int32_T)(16 + iy); (int32_T)(ijA + 1) <= d; ijA++) {
          b_A[ijA] += b_A[c_ix] * -temp;
          c_ix++;
        }
      }

      b_kAcol += 15;
      iy += 15;
    }
  }

  for (j = 0; j < 14; j++) {
    if ((int32_T)(j + 1) != (int32_T)ipiv[j]) {
      b_jBcol = (int32_T)((int32_T)ipiv[j] - 1);
      for (iy = 0; iy < 15; iy++) {
        temp = B[(int32_T)((int32_T)(15 * iy) + j)];
        B[(int32_T)(j + (int32_T)(15 * iy))] = B[(int32_T)((int32_T)(15 * iy) +
          b_jBcol)];
        B[(int32_T)(b_jBcol + (int32_T)(15 * iy))] = temp;
      }
    }
  }

  for (j = 0; j < 15; j++) {
    b_jBcol = (int32_T)(15 * j);
    for (iy = 0; iy < 15; iy++) {
      b_kAcol = (int32_T)(15 * iy);
      if (B[(int32_T)(iy + b_jBcol)] != 0.0F) {
        for (c_i = (int32_T)(iy + 1); (int32_T)(c_i + 1) < 16; c_i++) {
          B[(int32_T)(c_i + b_jBcol)] -= B[(int32_T)(iy + b_jBcol)] * b_A
            [(int32_T)(c_i + b_kAcol)];
        }
      }
    }
  }

  for (j = 0; j < 15; j++) {
    b_jBcol = (int32_T)(15 * j);
    for (iy = 14; iy >= 0; iy += -1) {
      b_kAcol = (int32_T)(15 * iy);
      if (B[(int32_T)(iy + b_jBcol)] != 0.0F) {
        B[(int32_T)(iy + b_jBcol)] /= b_A[(int32_T)(iy + b_kAcol)];
        for (c_i = 0; (int32_T)(c_i + 1) <= iy; c_i++) {
          B[(int32_T)(c_i + b_jBcol)] -= B[(int32_T)(iy + b_jBcol)] * b_A
            [(int32_T)(c_i + b_kAcol)];
        }
      }
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
