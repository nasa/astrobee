//
// File: iekfiecjknopophd_pinv.cpp
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
#include "moppaaimimgdecjm_svd.h"
#include "iekfiecjknopophd_pinv.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void iekfiecjknopophd_pinv(const real32_T A[4], real32_T X[4])
{
  real32_T V[4];
  int32_T r;
  int32_T vcol;
  real32_T tol;
  int32_T j;
  real32_T U[4];
  real32_T S[4];
  boolean_T b_p;
  int32_T ar;
  int32_T ia;
  int32_T b_ic;
  b_p = true;
  X[0] = 0.0F;
  if (!((!rtIsInfF(A[0])) && (!rtIsNaNF(A[0])))) {
    b_p = false;
  }

  X[1] = 0.0F;
  if (b_p && ((!rtIsInfF(A[1])) && (!rtIsNaNF(A[1])))) {
  } else {
    b_p = false;
  }

  X[2] = 0.0F;
  if (b_p && ((!rtIsInfF(A[2])) && (!rtIsNaNF(A[2])))) {
  } else {
    b_p = false;
  }

  X[3] = 0.0F;
  if (b_p && ((!rtIsInfF(A[3])) && (!rtIsNaNF(A[3])))) {
  } else {
    b_p = false;
  }

  if (b_p) {
    moppaaimimgdecjm_svd(A, U, S, V);
  } else {
    U[0] = (rtNaNF);
    U[1] = (rtNaNF);
    S[1] = 0.0F;
    U[2] = (rtNaNF);
    S[2] = 0.0F;
    U[3] = (rtNaNF);
    S[0] = (rtNaNF);
    S[3] = (rtNaNF);
    V[0] = (rtNaNF);
    V[1] = (rtNaNF);
    V[2] = (rtNaNF);
    V[3] = (rtNaNF);
  }

  tol = 2.0F * S[0] * 1.1920929E-7F;
  r = 0;
  vcol = 0;
  while (((int32_T)(vcol + 1) < 3) && (S[(int32_T)((int32_T)(vcol << 1) + vcol)]
          > tol)) {
    r++;
    vcol++;
  }

  if (r > 0) {
    vcol = 0;
    for (j = 0; (int32_T)(j + 1) <= r; j++) {
      tol = 1.0F / S[(int32_T)((int32_T)(j << 1) + j)];
      for (ar = vcol; (int32_T)(ar + 1) <= (int32_T)(vcol + 2); ar++) {
        V[ar] *= tol;
      }

      vcol += 2;
    }

    for (j = 0; (int32_T)(j + 1) < 3; j++) {
      X[j] = 0.0F;
    }

    for (j = 2; (int32_T)(j + 1) < 5; j++) {
      X[j] = 0.0F;
    }

    ar = -1;
    vcol = (int32_T)((int32_T)((int32_T)(r - 1) << 1) + 1);
    for (j = 0; (int32_T)(j + 1) <= vcol; j += 2) {
      if (U[j] != 0.0F) {
        ia = ar;
        for (b_ic = 0; (int32_T)(b_ic + 1) < 3; b_ic++) {
          ia++;
          X[b_ic] += U[j] * V[ia];
        }
      }

      ar += 2;
    }

    ar = -1;
    vcol = (int32_T)((int32_T)((int32_T)(r - 1) << 1) + 2);
    for (j = 1; (int32_T)(j + 1) <= vcol; j += 2) {
      if (U[j] != 0.0F) {
        ia = ar;
        for (b_ic = 2; (int32_T)(b_ic + 1) < 5; b_ic++) {
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
