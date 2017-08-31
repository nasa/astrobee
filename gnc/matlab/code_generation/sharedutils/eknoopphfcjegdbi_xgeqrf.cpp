//
// File: eknoopphfcjegdbi_xgeqrf.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include "dbaidjecmohdphln_xzlarf.h"
#include "ecbilfcbnglnimoh_xzlarfg.h"
#include "eknoopphfcjegdbi_xgeqrf.h"

// Function for MATLAB Function: '<S16>/Compute Residual and H'
void eknoopphfcjegdbi_xgeqrf(real32_T A_data[], int32_T A_sizes[2], real32_T
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

  *tau_sizes = (int8_T)mn;
  if (A_sizes[0] != 0) {
    for (i = 0; i < 6; i++) {
      work[i] = 0.0F;
    }

    for (i = 0; i + 1 <= mn; i++) {
      i_i = i * m + i;
      mmi = m - i;
      if (i + 1 < m) {
        b_atmp = A_data[i_i];
        b = ecbilfcbnglnimoh_xzlarfg(mmi, &b_atmp, A_data, i_i + 2);
        tau_data[i] = b;
        A_data[i_i] = b_atmp;
      } else {
        tau_data[i] = 0.0F;
      }

      if (i + 1 < 6) {
        b_atmp = A_data[i_i];
        A_data[i_i] = 1.0F;
        dbaidjecmohdphln_xzlarf(mmi, 5 - i, i_i + 1, tau_data[i], A_data, (i +
          (i + 1) * m) + 1, m, work);
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
