//
// File: ohdbhlnomohlcjek_xgeqrf.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed May 17 14:42:31 2017
//
#include "rtwtypes.h"
#include "fcbagdbapphliecj_xzlarfg.h"
#include "ophdecbiohdblnoh_xzlarf.h"
#include "ohdbhlnomohlcjek_xgeqrf.h"

// Function for MATLAB Function: '<S16>/Compute Residual and H'
void ohdbhlnomohlcjek_xgeqrf(real32_T A_data[], int32_T A_sizes[2], real32_T
  tau_data[], int32_T *tau_sizes)
{
  int32_T m;
  int32_T n;
  int32_T mn;
  int32_T i_i;
  int32_T mmi;
  real32_T b;
  real32_T b_atmp;
  int32_T loop_ub;
  real32_T work_data[150];
  int32_T work_sizes;
  uint8_T c_idx_0;
  m = A_sizes[0];
  n = A_sizes[1];
  work_sizes = A_sizes[0];
  mn = A_sizes[1];
  if (work_sizes <= mn) {
    mn = work_sizes;
  }

  *tau_sizes = (int32_T)(uint8_T)mn;
  if (!((A_sizes[0] == 0) || (A_sizes[1] == 0))) {
    c_idx_0 = (uint8_T)A_sizes[1];
    work_sizes = (int32_T)c_idx_0;
    loop_ub = (int32_T)c_idx_0;
    for (i_i = 0; i_i <= (int32_T)(loop_ub - 1); i_i++) {
      work_data[i_i] = 0.0F;
    }

    for (loop_ub = 0; (int32_T)(loop_ub + 1) <= mn; loop_ub++) {
      i_i = (int32_T)((int32_T)(loop_ub * m) + loop_ub);
      mmi = (int32_T)(m - loop_ub);
      if ((int32_T)(loop_ub + 1) < m) {
        b_atmp = A_data[i_i];
        b = fcbagdbapphliecj_xzlarfg(mmi, &b_atmp, A_data, (int32_T)(i_i + 2));
        tau_data[loop_ub] = b;
        A_data[i_i] = b_atmp;
      } else {
        tau_data[loop_ub] = 0.0F;
      }

      if ((int32_T)(loop_ub + 1) < n) {
        b_atmp = A_data[i_i];
        A_data[i_i] = 1.0F;
        ophdecbiohdblnoh_xzlarf(mmi, (int32_T)((int32_T)(n - loop_ub) - 1),
          (int32_T)(i_i + 1), tau_data[loop_ub], A_data, (int32_T)((int32_T)
          (loop_ub + (int32_T)((int32_T)(loop_ub + 1) * m)) + 1), m, work_data);
        A_data[i_i] = b_atmp;
      }
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
