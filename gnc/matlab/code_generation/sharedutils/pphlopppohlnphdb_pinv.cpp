//
// File: pphlopppohlnphdb_pinv.cpp
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:07:06 2018
//
#include "rtwtypes.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "lnohcjmoimglcbai_svd.h"
#include "pphlopppohlnphdb_pinv.h"

// Function for MATLAB Function: '<S12>/MATLAB Function'
void pphlopppohlnphdb_pinv(const real32_T A[36], real32_T X[36])
{
  real32_T b_X[36];
  real32_T b_A[36];
  real32_T V[9];
  int32_T vcol;
  real32_T tol;
  int32_T j;
  real32_T U[36];
  real32_T S[9];
  boolean_T b_p;
  int32_T ar;
  int32_T ia;
  int32_T b;
  int32_T ib;
  int32_T b_ic;
  int32_T r;
  for (r = 0; r < 3; r++) {
    for (vcol = 0; vcol < 12; vcol++) {
      b_A[(int32_T)(vcol + (int32_T)(12 * r))] = A[(int32_T)((int32_T)(3 * vcol)
        + r)];
    }
  }

  b_p = true;
  for (r = 0; r < 36; r++) {
    b_X[r] = 0.0F;
    if (b_p && ((!rtIsInfF(b_A[r])) && (!rtIsNaNF(b_A[r])))) {
    } else {
      b_p = false;
    }
  }

  if (b_p) {
    lnohcjmoimglcbai_svd(b_A, U, S, V);
  } else {
    for (r = 0; r < 36; r++) {
      U[r] = (rtNaNF);
    }

    for (r = 0; r < 9; r++) {
      S[r] = 0.0F;
      V[r] = (rtNaNF);
    }

    S[0] = (rtNaNF);
    S[4] = (rtNaNF);
    S[8] = (rtNaNF);
  }

  tol = 12.0F * S[0] * 1.1920929E-7F;
  r = 0;
  vcol = 0;
  while (((int32_T)(vcol + 1) < 4) && (S[(int32_T)((int32_T)(3 * vcol) + vcol)] >
          tol)) {
    r++;
    vcol++;
  }

  if (r > 0) {
    vcol = 0;
    for (j = 0; (int32_T)(j + 1) <= r; j++) {
      tol = 1.0F / S[(int32_T)((int32_T)(3 * j) + j)];
      for (ar = vcol; (int32_T)(ar + 1) <= (int32_T)(vcol + 3); ar++) {
        V[ar] *= tol;
      }

      vcol += 3;
    }

    for (vcol = 0; vcol <= 34; vcol += 3) {
      for (j = vcol; (int32_T)(j + 1) <= (int32_T)(vcol + 3); j++) {
        b_X[j] = 0.0F;
      }
    }

    vcol = -1;
    for (j = 0; j <= 34; j += 3) {
      ar = -1;
      vcol++;
      b = (int32_T)((int32_T)((int32_T)((int32_T)(r - 1) * 12) + vcol) + 1);
      for (ib = vcol; (int32_T)(ib + 1) <= b; ib += 12) {
        if (U[ib] != 0.0F) {
          ia = ar;
          for (b_ic = j; (int32_T)(b_ic + 1) <= (int32_T)(j + 3); b_ic++) {
            ia++;
            b_X[b_ic] += U[ib] * V[ia];
          }
        }

        ar += 3;
      }
    }
  }

  for (r = 0; r < 3; r++) {
    for (vcol = 0; vcol < 12; vcol++) {
      X[(int32_T)(vcol + (int32_T)(12 * r))] = b_X[(int32_T)((int32_T)(3 * vcol)
        + r)];
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
