//
// File: biekbimofknojmop_pinv.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include "jmgljmgdphdjlfcj_svd.h"
#include "biekbimofknojmop_pinv.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void biekbimofknojmop_pinv(const real32_T A[4], real32_T X[4])
{
  real32_T V[4];
  int32_T r;
  int32_T vcol;
  real32_T U[4];
  real32_T S[4];
  real32_T tol;
  int32_T j;
  int32_T ar;
  int32_T ia;
  int32_T b_ic;
  X[0] = 0.0F;
  X[1] = 0.0F;
  X[2] = 0.0F;
  X[3] = 0.0F;
  jmgljmgdphdjlfcj_svd(A, U, S, V);
  tol = 2.0F * S[0] * 1.1920929E-7F;
  r = 0;
  vcol = 0;
  while ((vcol + 1 < 3) && (S[(vcol << 1) + vcol] > tol)) {
    r++;
    vcol++;
  }

  if (r > 0) {
    vcol = 0;
    for (j = 0; j + 1 <= r; j++) {
      tol = 1.0F / S[(j << 1) + j];
      for (ar = vcol; ar + 1 <= vcol + 2; ar++) {
        V[ar] *= tol;
      }

      vcol += 2;
    }

    for (j = 0; j + 1 < 3; j++) {
      X[j] = 0.0F;
    }

    for (j = 2; j + 1 < 5; j++) {
      X[j] = 0.0F;
    }

    ar = -1;
    vcol = ((r - 1) << 1) + 1;
    for (j = 0; j + 1 <= vcol; j += 2) {
      if (U[j] != 0.0F) {
        ia = ar;
        for (b_ic = 0; b_ic + 1 < 3; b_ic++) {
          ia++;
          X[b_ic] += U[j] * V[ia];
        }
      }

      ar += 2;
    }

    ar = -1;
    vcol = ((r - 1) << 1) + 2;
    for (j = 1; j + 1 <= vcol; j += 2) {
      if (U[j] != 0.0F) {
        ia = ar;
        for (b_ic = 2; b_ic + 1 < 5; b_ic++) {
          ia++;
          X[b_ic] += U[j] * V[ia];
        }
      }

      ar += 2;
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
