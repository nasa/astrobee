//
// File: aaiehlfccbimimgd_xzlarf.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Dec  4 08:33:06 2017
//
#include "rtwtypes.h"
#include "biecdbaijecjlngd_xgemv.h"
#include "lfcjpphlnoppphdb_xgerc.h"
#include "ppppaaaacbimpphd_ilazlc.h"
#include "aaiehlfccbimimgd_xzlarf.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void aaiehlfccbimimgd_xzlarf(int32_T m, int32_T n, int32_T iv0, real32_T tau,
  real32_T C_data[], int32_T ic0, int32_T ldc, real32_T work_data[])
{
  int32_T lastv;
  int32_T lastc;
  if (tau != 0.0F) {
    lastv = m;
    lastc = (int32_T)(iv0 + m);
    while ((lastv > 0) && (C_data[(int32_T)(lastc - 2)] == 0.0F)) {
      lastv--;
      lastc--;
    }

    lastc = ppppaaaacbimpphd_ilazlc(lastv, n, C_data, ic0, ldc);
  } else {
    lastv = 0;
    lastc = 0;
  }

  if (lastv > 0) {
    biecdbaijecjlngd_xgemv(lastv, lastc, C_data, ic0, ldc, C_data, iv0,
      work_data);
    lfcjpphlnoppphdb_xgerc(lastv, lastc, -tau, iv0, work_data, C_data, ic0, ldc);
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
