//
// File: dbaidjecmohdphln_xzlarf.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include "dbaidjecmohdphln_xzlarf.h"

// Function for MATLAB Function: '<S16>/Compute Residual and H'
void dbaidjecmohdphln_xzlarf(int32_T m, int32_T n, int32_T iv0, real32_T tau,
  real32_T C_data[], int32_T ic0, int32_T ldc, real32_T work[6])
{
  int32_T lastv;
  int32_T lastc;
  int32_T ix;
  real32_T c;
  int32_T iy;
  int32_T iac;
  int32_T d;
  int32_T b_ia;
  int32_T jy;
  boolean_T exitg2;
  if (tau != 0.0F) {
    lastv = m;
    lastc = iv0 + m;
    while ((lastv > 0) && (C_data[lastc - 2] == 0.0F)) {
      lastv--;
      lastc--;
    }

    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      iy = (lastc - 1) * ldc + ic0;
      jy = iy;
      do {
        iac = 0;
        if (jy <= (iy + lastv) - 1) {
          if (C_data[jy - 1] != 0.0F) {
            iac = 1;
          } else {
            jy++;
          }
        } else {
          lastc--;
          iac = 2;
        }
      } while (iac == 0);

      if (iac == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = 0;
  }

  if (lastv > 0) {
    if (lastc != 0) {
      for (iy = 1; iy <= lastc; iy++) {
        work[iy - 1] = 0.0F;
      }

      iy = 0;
      jy = (lastc - 1) * ldc + ic0;
      iac = ic0;
      while ((ldc > 0) && (iac <= jy)) {
        ix = iv0;
        c = 0.0F;
        d = (iac + lastv) - 1;
        for (b_ia = iac; b_ia <= d; b_ia++) {
          c += C_data[b_ia - 1] * C_data[ix - 1];
          ix++;
        }

        work[iy] += c;
        iy++;
        iac += ldc;
      }
    }

    if (!(-tau == 0.0F)) {
      iy = ic0 - 1;
      jy = 0;
      for (iac = 1; iac <= lastc; iac++) {
        if (work[jy] != 0.0F) {
          c = work[jy] * -tau;
          ix = iv0;
          d = lastv + iy;
          for (b_ia = iy; b_ia + 1 <= d; b_ia++) {
            C_data[b_ia] += C_data[ix - 1] * c;
            ix++;
          }
        }

        jy++;
        iy += ldc;
      }
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
