//
// File: kfcbjeknaaaiiekn_pinv.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include <string.h>
#include "cbimbiekfcjmaaie_svd.h"
#include "kfcbjeknaaaiiekn_pinv.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void kfcbjeknaaaiiekn_pinv(const real32_T A[16], real32_T X[16])
{
  real32_T V[16];
  int32_T r;
  int32_T vcol;
  real32_T U[16];
  real32_T S[16];
  real32_T tol;
  int32_T j;
  int32_T ar;
  int32_T ia;
  int32_T b_ic;
  memset(&X[0], 0, sizeof(real32_T) << 4U);
  cbimbiekfcjmaaie_svd(A, U, S, V);
  tol = 4.0F * S[0] * 1.1920929E-7F;
  r = 0;
  vcol = 0;
  while ((vcol + 1 < 5) && (S[(vcol << 2) + vcol] > tol)) {
    r++;
    vcol++;
  }

  if (r > 0) {
    vcol = 0;
    for (j = 0; j + 1 <= r; j++) {
      tol = 1.0F / S[(j << 2) + j];
      for (ar = vcol; ar + 1 <= vcol + 4; ar++) {
        V[ar] *= tol;
      }

      vcol += 4;
    }

    for (j = 0; j + 1 < 5; j++) {
      X[j] = 0.0F;
    }

    for (j = 4; j + 1 < 9; j++) {
      X[j] = 0.0F;
    }

    for (j = 8; j + 1 < 13; j++) {
      X[j] = 0.0F;
    }

    for (j = 12; j + 1 < 17; j++) {
      X[j] = 0.0F;
    }

    ar = -1;
    vcol = ((r - 1) << 2) + 1;
    for (j = 0; j + 1 <= vcol; j += 4) {
      if (U[j] != 0.0F) {
        ia = ar;
        for (b_ic = 0; b_ic + 1 < 5; b_ic++) {
          ia++;
          X[b_ic] += U[j] * V[ia];
        }
      }

      ar += 4;
    }

    ar = -1;
    vcol = ((r - 1) << 2) + 2;
    for (j = 1; j + 1 <= vcol; j += 4) {
      if (U[j] != 0.0F) {
        ia = ar;
        for (b_ic = 4; b_ic + 1 < 9; b_ic++) {
          ia++;
          X[b_ic] += U[j] * V[ia];
        }
      }

      ar += 4;
    }

    ar = -1;
    vcol = ((r - 1) << 2) + 3;
    for (j = 2; j + 1 <= vcol; j += 4) {
      if (U[j] != 0.0F) {
        ia = ar;
        for (b_ic = 8; b_ic + 1 < 13; b_ic++) {
          ia++;
          X[b_ic] += U[j] * V[ia];
        }
      }

      ar += 4;
    }

    ar = -1;
    vcol = ((r - 1) << 2) + 4;
    for (j = 3; j + 1 <= vcol; j += 4) {
      if (U[j] != 0.0F) {
        ia = ar;
        for (b_ic = 12; b_ic + 1 < 17; b_ic++) {
          ia++;
          X[b_ic] += U[j] * V[ia];
        }
      }

      ar += 4;
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
