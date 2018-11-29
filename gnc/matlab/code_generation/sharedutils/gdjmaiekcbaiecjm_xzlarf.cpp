//
// File: gdjmaiekcbaiecjm_xzlarf.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 11:58:17 2018
//
#include "rtwtypes.h"
#include "gdjmaiekcbaiecjm_xzlarf.h"

// Function for MATLAB Function: '<S16>/Compute Residual and H'
void gdjmaiekcbaiecjm_xzlarf(int32_T m, int32_T n, int32_T iv0, real32_T tau,
  real32_T C_data[], int32_T ic0, int32_T ldc, real32_T work[6])
{
  int32_T lastv;
  int32_T lastc;
  int32_T coltop;
  int32_T ix;
  real32_T c;
  int32_T iac;
  int32_T d;
  int32_T b_ia;
  int32_T jy;
  int32_T exitg1;
  boolean_T exitg2;
  if (tau != 0.0F) {
    lastv = m;
    lastc = (int32_T)(iv0 + m);
    while ((lastv > 0) && (C_data[(int32_T)(lastc - 2)] == 0.0F)) {
      lastv--;
      lastc--;
    }

    lastc = n;
    exitg2 = false;
    while ((!exitg2) && (lastc > 0)) {
      coltop = (int32_T)((int32_T)((int32_T)(lastc - 1) * ldc) + ic0);
      jy = coltop;
      do {
        exitg1 = 0;
        if (jy <= (int32_T)((int32_T)(coltop + lastv) - 1)) {
          if (C_data[(int32_T)(jy - 1)] != 0.0F) {
            exitg1 = 1;
          } else {
            jy++;
          }
        } else {
          lastc--;
          exitg1 = 2;
        }
      } while (exitg1 == 0);

      if (exitg1 == 1) {
        exitg2 = true;
      }
    }
  } else {
    lastv = 0;
    lastc = 0;
  }

  if (lastv > 0) {
    if (lastc != 0) {
      for (coltop = 1; coltop <= lastc; coltop++) {
        work[(int32_T)(coltop - 1)] = 0.0F;
      }

      coltop = 0;
      jy = (int32_T)((int32_T)((int32_T)(lastc - 1) * ldc) + ic0);
      iac = ic0;
      while ((ldc > 0) && (iac <= jy)) {
        ix = iv0;
        c = 0.0F;
        d = (int32_T)((int32_T)(iac + lastv) - 1);
        for (b_ia = iac; b_ia <= d; b_ia++) {
          c += C_data[(int32_T)(b_ia - 1)] * C_data[(int32_T)(ix - 1)];
          ix++;
        }

        work[coltop] += c;
        coltop++;
        iac += ldc;
      }
    }

    if (!(-tau == 0.0F)) {
      coltop = (int32_T)(ic0 - 1);
      jy = 0;
      for (iac = 1; iac <= lastc; iac++) {
        if (work[jy] != 0.0F) {
          c = work[jy] * -tau;
          ix = iv0;
          d = (int32_T)(lastv + coltop);
          for (b_ia = coltop; (int32_T)(b_ia + 1) <= d; b_ia++) {
            C_data[b_ia] += C_data[(int32_T)(ix - 1)] * c;
            ix++;
          }
        }

        jy++;
        coltop += ldc;
      }
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
