//
// File: biecdbieglngohlf_pinv.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Dec  4 08:33:06 2017
//
#include "rtwtypes.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "gdjmjekfdjecpppp_svd.h"
#include "biecdbieglngohlf_pinv.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void biecdbieglngohlf_pinv(const real32_T A[16], real32_T X[16])
{
  real32_T V[16];
  int32_T vcol;
  real32_T tol;
  int32_T j;
  real32_T U[16];
  real32_T S[16];
  boolean_T b_p;
  int32_T ar;
  int32_T ia;
  int32_T b_ic;
  int32_T r;
  b_p = true;
  for (r = 0; r < 16; r++) {
    X[r] = 0.0F;
    if (b_p && ((!rtIsInfF(A[r])) && (!rtIsNaNF(A[r])))) {
    } else {
      b_p = false;
    }
  }

  if (b_p) {
    gdjmjekfdjecpppp_svd(A, U, S, V);
  } else {
    for (r = 0; r < 16; r++) {
      U[r] = (rtNaNF);
      S[r] = 0.0F;
      V[r] = (rtNaNF);
    }

    S[0] = (rtNaNF);
    S[5] = (rtNaNF);
    S[10] = (rtNaNF);
    S[15] = (rtNaNF);
  }

  tol = 4.0F * S[0] * 1.1920929E-7F;
  r = 0;
  vcol = 0;
  while (((int32_T)(vcol + 1) < 5) && (S[(int32_T)((int32_T)(vcol << 2) + vcol)]
          > tol)) {
    r++;
    vcol++;
  }

  if (r > 0) {
    vcol = 0;
    for (j = 0; (int32_T)(j + 1) <= r; j++) {
      tol = 1.0F / S[(int32_T)((int32_T)(j << 2) + j)];
      for (ar = vcol; (int32_T)(ar + 1) <= (int32_T)(vcol + 4); ar++) {
        V[ar] *= tol;
      }

      vcol += 4;
    }

    for (j = 0; (int32_T)(j + 1) < 5; j++) {
      X[j] = 0.0F;
    }

    for (j = 4; (int32_T)(j + 1) < 9; j++) {
      X[j] = 0.0F;
    }

    for (j = 8; (int32_T)(j + 1) < 13; j++) {
      X[j] = 0.0F;
    }

    for (j = 12; (int32_T)(j + 1) < 17; j++) {
      X[j] = 0.0F;
    }

    ar = -1;
    vcol = (int32_T)((int32_T)((int32_T)(r - 1) << 2) + 1);
    for (j = 0; (int32_T)(j + 1) <= vcol; j += 4) {
      if (U[j] != 0.0F) {
        ia = ar;
        for (b_ic = 0; (int32_T)(b_ic + 1) < 5; b_ic++) {
          ia++;
          X[b_ic] += U[j] * V[ia];
        }
      }

      ar += 4;
    }

    ar = -1;
    vcol = (int32_T)((int32_T)((int32_T)(r - 1) << 2) + 2);
    for (j = 1; (int32_T)(j + 1) <= vcol; j += 4) {
      if (U[j] != 0.0F) {
        ia = ar;
        for (b_ic = 4; (int32_T)(b_ic + 1) < 9; b_ic++) {
          ia++;
          X[b_ic] += U[j] * V[ia];
        }
      }

      ar += 4;
    }

    ar = -1;
    vcol = (int32_T)((int32_T)((int32_T)(r - 1) << 2) + 3);
    for (j = 2; (int32_T)(j + 1) <= vcol; j += 4) {
      if (U[j] != 0.0F) {
        ia = ar;
        for (b_ic = 8; (int32_T)(b_ic + 1) < 13; b_ic++) {
          ia++;
          X[b_ic] += U[j] * V[ia];
        }
      }

      ar += 4;
    }

    ar = -1;
    vcol = (int32_T)((int32_T)((int32_T)(r - 1) << 2) + 4);
    for (j = 3; (int32_T)(j + 1) <= vcol; j += 4) {
      if (U[j] != 0.0F) {
        ia = ar;
        for (b_ic = 12; (int32_T)(b_ic + 1) < 17; b_ic++) {
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
