//
// File: kfkfcbiekfcjmohl_mldivide.cpp
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
#include "kfkfcbiekfcjmohl_mldivide.h"

// Function for MATLAB Function: '<S112>/MATLAB Function'
void kfkfcbiekfcjmohl_mldivide(const real32_T A[16], real32_T B[16])
{
  real32_T temp;
  real32_T b_A[16];
  int8_T ipiv[4];
  int32_T j;
  real32_T s;
  int32_T iy;
  int32_T c_ix;
  int32_T d;
  int32_T ijA;
  int32_T b_jBcol;
  int32_T b_kAcol;
  int32_T c_i;
  memcpy(&b_A[0], &A[0], (uint32_T)(sizeof(real32_T) << 4U));
  ipiv[0] = 1;
  ipiv[1] = 2;
  ipiv[2] = 3;
  ipiv[3] = 4;
  for (j = 0; j < 3; j++) {
    b_jBcol = (int32_T)(j * 5);
    iy = 0;
    b_kAcol = b_jBcol;
    temp = (real32_T)fabs((real_T)b_A[b_jBcol]);
    for (c_i = 2; c_i <= (int32_T)(4 - j); c_i++) {
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
        iy += j;
        temp = b_A[j];
        b_A[j] = b_A[iy];
        b_A[iy] = temp;
        b_kAcol = (int32_T)(j + 4);
        iy += 4;
        temp = b_A[b_kAcol];
        b_A[b_kAcol] = b_A[iy];
        b_A[iy] = temp;
        b_kAcol += 4;
        iy += 4;
        temp = b_A[b_kAcol];
        b_A[b_kAcol] = b_A[iy];
        b_A[iy] = temp;
        b_kAcol += 4;
        iy += 4;
        temp = b_A[b_kAcol];
        b_A[b_kAcol] = b_A[iy];
        b_A[iy] = temp;
      }

      iy = (int32_T)((int32_T)(b_jBcol - j) + 4);
      for (b_kAcol = (int32_T)(b_jBcol + 1); (int32_T)(b_kAcol + 1) <= iy;
           b_kAcol++) {
        b_A[b_kAcol] /= b_A[b_jBcol];
      }
    }

    iy = b_jBcol;
    b_kAcol = (int32_T)(b_jBcol + 4);
    for (c_i = 1; c_i <= (int32_T)(3 - j); c_i++) {
      temp = b_A[b_kAcol];
      if (b_A[b_kAcol] != 0.0F) {
        c_ix = (int32_T)(b_jBcol + 1);
        d = (int32_T)((int32_T)(iy - j) + 8);
        for (ijA = (int32_T)(5 + iy); (int32_T)(ijA + 1) <= d; ijA++) {
          b_A[ijA] += b_A[c_ix] * -temp;
          c_ix++;
        }
      }

      b_kAcol += 4;
      iy += 4;
    }
  }

  for (j = 0; j < 3; j++) {
    if ((int32_T)(j + 1) != (int32_T)ipiv[j]) {
      b_jBcol = (int32_T)((int32_T)ipiv[j] - 1);
      temp = B[j];
      B[j] = B[b_jBcol];
      B[b_jBcol] = temp;
      temp = B[(int32_T)(j + 4)];
      B[(int32_T)(j + 4)] = B[(int32_T)(b_jBcol + 4)];
      B[(int32_T)(b_jBcol + 4)] = temp;
      temp = B[(int32_T)(j + 8)];
      B[(int32_T)(j + 8)] = B[(int32_T)(b_jBcol + 8)];
      B[(int32_T)(b_jBcol + 8)] = temp;
      temp = B[(int32_T)(j + 12)];
      B[(int32_T)(j + 12)] = B[(int32_T)(b_jBcol + 12)];
      B[(int32_T)(b_jBcol + 12)] = temp;
    }
  }

  for (j = 0; j < 4; j++) {
    b_jBcol = (int32_T)(j << 2);
    if (B[b_jBcol] != 0.0F) {
      for (c_i = 1; (int32_T)(c_i + 1) < 5; c_i++) {
        B[(int32_T)(c_i + b_jBcol)] -= B[b_jBcol] * b_A[c_i];
      }
    }

    if (B[(int32_T)(1 + b_jBcol)] != 0.0F) {
      for (c_i = 2; (int32_T)(c_i + 1) < 5; c_i++) {
        B[(int32_T)(c_i + b_jBcol)] -= B[(int32_T)(1 + b_jBcol)] * b_A[(int32_T)
          (c_i + 4)];
      }
    }

    if (B[(int32_T)(2 + b_jBcol)] != 0.0F) {
      for (c_i = 3; (int32_T)(c_i + 1) < 5; c_i++) {
        B[(int32_T)(c_i + b_jBcol)] -= B[(int32_T)(2 + b_jBcol)] * b_A[(int32_T)
          (c_i + 8)];
      }
    }
  }

  for (j = 0; j < 4; j++) {
    b_jBcol = (int32_T)(j << 2);
    if (B[(int32_T)(3 + b_jBcol)] != 0.0F) {
      B[(int32_T)(3 + b_jBcol)] /= b_A[15];
      for (c_i = 0; (int32_T)(c_i + 1) < 4; c_i++) {
        B[(int32_T)(c_i + b_jBcol)] -= B[(int32_T)(3 + b_jBcol)] * b_A[(int32_T)
          (c_i + 12)];
      }
    }

    if (B[(int32_T)(2 + b_jBcol)] != 0.0F) {
      B[(int32_T)(2 + b_jBcol)] /= b_A[10];
      for (c_i = 0; (int32_T)(c_i + 1) < 3; c_i++) {
        B[(int32_T)(c_i + b_jBcol)] -= B[(int32_T)(2 + b_jBcol)] * b_A[(int32_T)
          (c_i + 8)];
      }
    }

    if (B[(int32_T)(1 + b_jBcol)] != 0.0F) {
      B[(int32_T)(1 + b_jBcol)] /= b_A[5];
      for (c_i = 0; (int32_T)(c_i + 1) < 2; c_i++) {
        B[(int32_T)(c_i + b_jBcol)] -= B[(int32_T)(1 + b_jBcol)] * b_A[(int32_T)
          (c_i + 4)];
      }
    }

    if (B[b_jBcol] != 0.0F) {
      B[b_jBcol] /= b_A[0];
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
