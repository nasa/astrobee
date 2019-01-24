//
// File: fcbijmgdglngglfc_xgeqrf.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:06:07 2018
//
#include "rtwtypes.h"
#include "aaiehlfccbimimgd_xzlarf.h"
#include "jecjfcjmbaaaaiek_xzlarfg.h"
#include "fcbijmgdglngglfc_xgeqrf.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void fcbijmgdglngglfc_xgeqrf(real32_T A_data[], int32_T A_sizes[2], real32_T
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
  real32_T work_data[100];
  int32_T work_sizes;
  int8_T c_idx_0;
  m = A_sizes[0];
  n = A_sizes[1];
  work_sizes = A_sizes[0];
  mn = A_sizes[1];
  if (work_sizes <= mn) {
    mn = work_sizes;
  }

  *tau_sizes = (int32_T)(int8_T)mn;
  if (!((A_sizes[0] == 0) || (A_sizes[1] == 0))) {
    c_idx_0 = (int8_T)A_sizes[1];
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
        b = jecjfcjmbaaaaiek_xzlarfg(mmi, &b_atmp, A_data, (int32_T)(i_i + 2));
        tau_data[loop_ub] = b;
        A_data[i_i] = b_atmp;
      } else {
        tau_data[loop_ub] = 0.0F;
      }

      if ((int32_T)(loop_ub + 1) < n) {
        b_atmp = A_data[i_i];
        A_data[i_i] = 1.0F;
        aaiehlfccbimimgd_xzlarf(mmi, (int32_T)((int32_T)(n - loop_ub) - 1),
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
