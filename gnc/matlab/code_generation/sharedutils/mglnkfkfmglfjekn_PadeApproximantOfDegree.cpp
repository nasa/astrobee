//
// File: mglnkfkfmglfjekn_PadeApproximantOfDegree.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:06:07 2018
//
#include "rtwtypes.h"
#include <string.h>
#include "kfkfcbiekfcjmohl_mldivide.h"
#include "mglnkfkfmglfjekn_PadeApproximantOfDegree.h"

// Function for MATLAB Function: '<S112>/MATLAB Function'
void mglnkfkfmglfjekn_PadeApproximantOfDegree(const real32_T A[16], uint8_T m,
  real32_T F[16])
{
  real32_T A2[16];
  int32_T d;
  real32_T A3[16];
  real32_T A4[16];
  real32_T A_0[16];
  int32_T i;
  for (d = 0; d < 4; d++) {
    for (i = 0; i < 4; i++) {
      A2[(int32_T)(i + (int32_T)(d << 2))] = 0.0F;
      A2[(int32_T)(i + (int32_T)(d << 2))] += A[(int32_T)(d << 2)] * A[i];
      A2[(int32_T)(i + (int32_T)(d << 2))] += A[(int32_T)((int32_T)(d << 2) + 1)]
        * A[(int32_T)(i + 4)];
      A2[(int32_T)(i + (int32_T)(d << 2))] += A[(int32_T)((int32_T)(d << 2) + 2)]
        * A[(int32_T)(i + 8)];
      A2[(int32_T)(i + (int32_T)(d << 2))] += A[(int32_T)((int32_T)(d << 2) + 3)]
        * A[(int32_T)(i + 12)];
    }
  }

  if ((int32_T)m == 3) {
    memcpy(&F[0], &A2[0], (uint32_T)(sizeof(real32_T) << 4U));
    F[0] += 60.0F;
    F[5] += 60.0F;
    F[10] += 60.0F;
    F[15] += 60.0F;
    for (d = 0; d < 4; d++) {
      for (i = 0; i < 4; i++) {
        A_0[(int32_T)(i + (int32_T)(d << 2))] = 0.0F;
        A_0[(int32_T)(i + (int32_T)(d << 2))] += F[(int32_T)(d << 2)] * A[i];
        A_0[(int32_T)(i + (int32_T)(d << 2))] += F[(int32_T)((int32_T)(d << 2) +
          1)] * A[(int32_T)(i + 4)];
        A_0[(int32_T)(i + (int32_T)(d << 2))] += F[(int32_T)((int32_T)(d << 2) +
          2)] * A[(int32_T)(i + 8)];
        A_0[(int32_T)(i + (int32_T)(d << 2))] += F[(int32_T)((int32_T)(d << 2) +
          3)] * A[(int32_T)(i + 12)];
      }
    }

    for (d = 0; d < 4; d++) {
      F[(int32_T)(d << 2)] = A_0[(int32_T)(d << 2)];
      F[(int32_T)(1 + (int32_T)(d << 2))] = A_0[(int32_T)((int32_T)(d << 2) + 1)];
      F[(int32_T)(2 + (int32_T)(d << 2))] = A_0[(int32_T)((int32_T)(d << 2) + 2)];
      F[(int32_T)(3 + (int32_T)(d << 2))] = A_0[(int32_T)((int32_T)(d << 2) + 3)];
    }

    for (d = 0; d < 16; d++) {
      A4[d] = 12.0F * A2[d];
    }

    d = 120;
  } else {
    for (d = 0; d < 4; d++) {
      for (i = 0; i < 4; i++) {
        A3[(int32_T)(i + (int32_T)(d << 2))] = 0.0F;
        A3[(int32_T)(i + (int32_T)(d << 2))] += A2[(int32_T)(d << 2)] * A2[i];
        A3[(int32_T)(i + (int32_T)(d << 2))] += A2[(int32_T)((int32_T)(d << 2) +
          1)] * A2[(int32_T)(i + 4)];
        A3[(int32_T)(i + (int32_T)(d << 2))] += A2[(int32_T)((int32_T)(d << 2) +
          2)] * A2[(int32_T)(i + 8)];
        A3[(int32_T)(i + (int32_T)(d << 2))] += A2[(int32_T)((int32_T)(d << 2) +
          3)] * A2[(int32_T)(i + 12)];
      }
    }

    if ((int32_T)m == 5) {
      for (d = 0; d < 16; d++) {
        F[d] = 420.0F * A2[d] + A3[d];
      }

      F[0] += 15120.0F;
      F[5] += 15120.0F;
      F[10] += 15120.0F;
      F[15] += 15120.0F;
      for (d = 0; d < 4; d++) {
        for (i = 0; i < 4; i++) {
          A_0[(int32_T)(i + (int32_T)(d << 2))] = 0.0F;
          A_0[(int32_T)(i + (int32_T)(d << 2))] += F[(int32_T)(d << 2)] * A[i];
          A_0[(int32_T)(i + (int32_T)(d << 2))] += F[(int32_T)((int32_T)(d << 2)
            + 1)] * A[(int32_T)(i + 4)];
          A_0[(int32_T)(i + (int32_T)(d << 2))] += F[(int32_T)((int32_T)(d << 2)
            + 2)] * A[(int32_T)(i + 8)];
          A_0[(int32_T)(i + (int32_T)(d << 2))] += F[(int32_T)((int32_T)(d << 2)
            + 3)] * A[(int32_T)(i + 12)];
        }
      }

      for (d = 0; d < 4; d++) {
        F[(int32_T)(d << 2)] = A_0[(int32_T)(d << 2)];
        F[(int32_T)(1 + (int32_T)(d << 2))] = A_0[(int32_T)((int32_T)(d << 2) +
          1)];
        F[(int32_T)(2 + (int32_T)(d << 2))] = A_0[(int32_T)((int32_T)(d << 2) +
          2)];
        F[(int32_T)(3 + (int32_T)(d << 2))] = A_0[(int32_T)((int32_T)(d << 2) +
          3)];
      }

      for (d = 0; d < 16; d++) {
        A4[d] = 30.0F * A3[d] + 3360.0F * A2[d];
      }

      d = 30240;
    } else {
      for (d = 0; d < 4; d++) {
        for (i = 0; i < 4; i++) {
          A4[(int32_T)(i + (int32_T)(d << 2))] = 0.0F;
          A4[(int32_T)(i + (int32_T)(d << 2))] += A2[(int32_T)(d << 2)] * A3[i];
          A4[(int32_T)(i + (int32_T)(d << 2))] += A2[(int32_T)((int32_T)(d << 2)
            + 1)] * A3[(int32_T)(i + 4)];
          A4[(int32_T)(i + (int32_T)(d << 2))] += A2[(int32_T)((int32_T)(d << 2)
            + 2)] * A3[(int32_T)(i + 8)];
          A4[(int32_T)(i + (int32_T)(d << 2))] += A2[(int32_T)((int32_T)(d << 2)
            + 3)] * A3[(int32_T)(i + 12)];
        }
      }

      for (d = 0; d < 16; d++) {
        F[d] = (1512.0F * A3[d] + A4[d]) + 277200.0F * A2[d];
      }

      F[0] += 8.64864E+6F;
      F[5] += 8.64864E+6F;
      F[10] += 8.64864E+6F;
      F[15] += 8.64864E+6F;
      for (d = 0; d < 4; d++) {
        for (i = 0; i < 4; i++) {
          A_0[(int32_T)(i + (int32_T)(d << 2))] = 0.0F;
          A_0[(int32_T)(i + (int32_T)(d << 2))] += F[(int32_T)(d << 2)] * A[i];
          A_0[(int32_T)(i + (int32_T)(d << 2))] += F[(int32_T)((int32_T)(d << 2)
            + 1)] * A[(int32_T)(i + 4)];
          A_0[(int32_T)(i + (int32_T)(d << 2))] += F[(int32_T)((int32_T)(d << 2)
            + 2)] * A[(int32_T)(i + 8)];
          A_0[(int32_T)(i + (int32_T)(d << 2))] += F[(int32_T)((int32_T)(d << 2)
            + 3)] * A[(int32_T)(i + 12)];
        }
      }

      for (d = 0; d < 4; d++) {
        F[(int32_T)(d << 2)] = A_0[(int32_T)(d << 2)];
        F[(int32_T)(1 + (int32_T)(d << 2))] = A_0[(int32_T)((int32_T)(d << 2) +
          1)];
        F[(int32_T)(2 + (int32_T)(d << 2))] = A_0[(int32_T)((int32_T)(d << 2) +
          2)];
        F[(int32_T)(3 + (int32_T)(d << 2))] = A_0[(int32_T)((int32_T)(d << 2) +
          3)];
      }

      for (d = 0; d < 16; d++) {
        A4[d] = (56.0F * A4[d] + 25200.0F * A3[d]) + 1.99584E+6F * A2[d];
      }

      d = 17297280;
    }
  }

  A4[0] += (real32_T)d;
  A4[5] += (real32_T)d;
  A4[10] += (real32_T)d;
  A4[15] += (real32_T)d;
  for (d = 0; d < 16; d++) {
    A4[d] -= F[d];
    F[d] *= 2.0F;
  }

  kfkfcbiekfcjmohl_mldivide(A4, F);
  F[0]++;
  F[5]++;
  F[10]++;
  F[15]++;
}

//
// File trailer for generated code.
//
// [EOF]
//
