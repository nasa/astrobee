//
// File: mgdjbieccjechlng_xgeqrf.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:06:07 2018
//
#include "rtwtypes.h"
#include "ecbilfcbnglnimoh_xzlarfg.h"
#include "gdjmaiekcbaiecjm_xzlarf.h"
#include "mgdjbieccjechlng_xgeqrf.h"

// Function for MATLAB Function: '<S16>/Compute Residual and H'
void mgdjbieccjechlng_xgeqrf(real32_T A_data[], int32_T A_sizes[2], real32_T
  tau_data[], int32_T *tau_sizes)
{
  int32_T m;
  int32_T mn;
  real32_T work[6];
  int32_T i_i;
  int32_T mmi;
  real32_T b;
  real32_T b_atmp;
  int32_T i;
  m = A_sizes[0];
  mn = A_sizes[0];
  if (!(mn <= 6)) {
    mn = 6;
  }

  *tau_sizes = (int32_T)(int8_T)mn;
  if (A_sizes[0] != 0) {
    for (i = 0; i < 6; i++) {
      work[i] = 0.0F;
    }

    for (i = 0; (int32_T)(i + 1) <= mn; i++) {
      i_i = (int32_T)((int32_T)(i * m) + i);
      mmi = (int32_T)(m - i);
      if ((int32_T)(i + 1) < m) {
        b_atmp = A_data[i_i];
        b = ecbilfcbnglnimoh_xzlarfg(mmi, &b_atmp, A_data, (int32_T)(i_i + 2));
        tau_data[i] = b;
        A_data[i_i] = b_atmp;
      } else {
        tau_data[i] = 0.0F;
      }

      if ((int32_T)(i + 1) < 6) {
        b_atmp = A_data[i_i];
        A_data[i_i] = 1.0F;
        gdjmaiekcbaiecjm_xzlarf(mmi, (int32_T)(5 - i), (int32_T)(i_i + 1),
          tau_data[i], A_data, (int32_T)((int32_T)(i + (int32_T)((int32_T)(i + 1)
          * m)) + 1), m, work);
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
