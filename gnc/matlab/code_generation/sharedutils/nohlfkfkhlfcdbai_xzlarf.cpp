//
// File: nohlfkfkhlfcdbai_xzlarf.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include "biecdbaijecjlngd_xgemv.h"
#include "lfcjpphlnoppphdb_xgerc.h"
#include "ppppmophjmopekno_ilazlc.h"
#include "nohlfkfkhlfcdbai_xzlarf.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void nohlfkfkhlfcdbai_xzlarf(int32_T m, int32_T n, int32_T iv0, real32_T tau,
  real32_T C_data[], int32_T ic0, int32_T ldc, real32_T work_data[])
{
  int32_T lastv;
  int32_T lastc;
  if (tau != 0.0F) {
    lastv = m;
    lastc = iv0 + m;
    while ((lastv > 0) && (C_data[lastc - 2] == 0.0F)) {
      lastv--;
      lastc--;
    }

    lastc = ppppmophjmopekno_ilazlc(lastv, n, C_data, ic0, ldc);
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
