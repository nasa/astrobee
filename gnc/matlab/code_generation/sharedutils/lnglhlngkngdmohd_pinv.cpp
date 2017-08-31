//
// File: lnglhlngkngdmohd_pinv.cpp
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:39 2017
//
#include "rtwtypes.h"
#include <string.h>
#include "aaaacjecophlngdj_svd.h"
#include "lnglhlngkngdmohd_pinv.h"

// Function for MATLAB Function: '<S9>/MATLAB Function'
void lnglhlngkngdmohd_pinv(const real32_T A[36], real32_T X[36])
{
  real32_T b_X[36];
  real32_T V[9];
  int32_T r;
  int32_T vcol;
  real32_T tol;
  int32_T j;
  real32_T U[36];
  real32_T S[9];
  int32_T ar;
  int32_T ia;
  int32_T b;
  int32_T ib;
  int32_T b_ic;
  real32_T A_0[36];
  memset(&b_X[0], 0, 36U * sizeof(real32_T));
  for (r = 0; r < 3; r++) {
    for (vcol = 0; vcol < 12; vcol++) {
      A_0[vcol + 12 * r] = A[3 * vcol + r];
    }
  }

  aaaacjecophlngdj_svd(A_0, U, S, V);
  tol = 12.0F * S[0] * 1.1920929E-7F;
  r = 0;
  vcol = 0;
  while ((vcol + 1 < 4) && (S[3 * vcol + vcol] > tol)) {
    r++;
    vcol++;
  }

  if (r > 0) {
    vcol = 0;
    for (j = 0; j + 1 <= r; j++) {
      tol = 1.0F / S[3 * j + j];
      for (ar = vcol; ar + 1 <= vcol + 3; ar++) {
        V[ar] *= tol;
      }

      vcol += 3;
    }

    for (vcol = 0; vcol <= 34; vcol += 3) {
      for (j = vcol; j + 1 <= vcol + 3; j++) {
        b_X[j] = 0.0F;
      }
    }

    vcol = -1;
    for (j = 0; j <= 34; j += 3) {
      ar = 0;
      vcol++;
      b = ((r - 1) * 12 + vcol) + 1;
      for (ib = vcol; ib + 1 <= b; ib += 12) {
        if (U[ib] != 0.0F) {
          ia = ar;
          for (b_ic = j; b_ic + 1 <= j + 3; b_ic++) {
            ia++;
            b_X[b_ic] += V[ia - 1] * U[ib];
          }
        }

        ar += 3;
      }
    }
  }

  for (r = 0; r < 3; r++) {
    for (vcol = 0; vcol < 12; vcol++) {
      X[vcol + 12 * r] = b_X[3 * vcol + r];
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
