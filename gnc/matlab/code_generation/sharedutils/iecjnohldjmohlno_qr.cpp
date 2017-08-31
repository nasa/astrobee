//
// File: iecjnohldjmohlno_qr.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include "baimfcjeeknonglf_xzlarf.h"
#include "knohbaiengdbnoph_xscal.h"
#include "nglnkfcbnophmohd_xgeqrf.h"
#include "nohlfkfkhlfcdbai_xzlarf.h"
#include "ppphhdjekfkfdjmg_xgeqrf.h"
#include "iecjnohldjmohlno_qr.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void iecjnohldjmohlno_qr(const real32_T A_data[], const int32_T A_sizes[2],
  real32_T Q_data[], int32_T Q_sizes[2], real32_T R_data[], int32_T R_sizes[2])
{
  int32_T m;
  int32_T i_i;
  int32_T itau;
  real32_T work[6];
  int32_T iaii;
  int32_T c;
  int32_T i;
  real32_T b_A_data[600];
  int32_T b_A_sizes[2];
  real32_T tau_data[6];
  real32_T b_tau_data[100];
  real32_T b_work_data[100];
  int8_T b_idx_0;
  m = A_sizes[0];
  b_idx_0 = (int8_T)A_sizes[0];
  Q_sizes[0] = (int8_T)A_sizes[0];
  Q_sizes[1] = (int8_T)A_sizes[0];
  R_sizes[0] = A_sizes[0];
  R_sizes[1] = A_sizes[1];
  if (A_sizes[0] > 6) {
    for (itau = 0; itau < 6; itau++) {
      for (i_i = 0; i_i + 1 <= m; i_i++) {
        Q_data[i_i + b_idx_0 * itau] = A_data[A_sizes[0] * itau + i_i];
      }
    }

    for (itau = 7; itau <= m; itau++) {
      for (i_i = 1; i_i <= m; i_i++) {
        Q_data[(i_i + b_idx_0 * (itau - 1)) - 1] = 0.0F;
      }
    }

    ppphhdjekfkfdjmg_xgeqrf(Q_data, Q_sizes, b_tau_data, &itau);
    for (itau = 0; itau < 6; itau++) {
      for (i_i = 0; i_i + 1 <= itau + 1; i_i++) {
        R_data[i_i + R_sizes[0] * itau] = Q_data[Q_sizes[0] * itau + i_i];
      }

      for (i_i = itau + 1; i_i + 1 <= m; i_i++) {
        R_data[i_i + R_sizes[0] * itau] = 0.0F;
      }
    }

    for (itau = 6; itau < m; itau++) {
      i_i = itau * m;
      for (iaii = 0; iaii < m; iaii++) {
        Q_data[i_i + iaii] = 0.0F;
      }

      Q_data[i_i + itau] = 1.0F;
    }

    itau = 5;
    b_idx_0 = (int8_T)Q_sizes[1];
    i = b_idx_0;
    i_i = b_idx_0;
    for (iaii = 0; iaii < i_i; iaii++) {
      b_work_data[iaii] = 0.0F;
    }

    for (i_i = 5; i_i >= 0; i_i += -1) {
      iaii = i_i * m + i_i;
      Q_data[iaii] = 1.0F;
      nohlfkfkhlfcdbai_xzlarf(m - i_i, (m - i_i) - 1, iaii + 1, b_tau_data[itau],
        Q_data, (iaii + m) + 1, m, b_work_data);
      knohbaiengdbnoph_xscal((m - i_i) - 1, -b_tau_data[itau], Q_data, iaii + 2);
      Q_data[iaii] = 1.0F - b_tau_data[itau];
      for (c = 1; c <= i_i; c++) {
        Q_data[iaii - c] = 0.0F;
      }

      itau--;
    }
  } else {
    b_A_sizes[0] = A_sizes[0];
    b_A_sizes[1] = 6;
    i_i = A_sizes[0] * A_sizes[1];
    for (iaii = 0; iaii < i_i; iaii++) {
      b_A_data[iaii] = A_data[iaii];
    }

    nglnkfcbnophmohd_xgeqrf(b_A_data, b_A_sizes, tau_data, &itau);
    for (itau = 0; itau + 1 <= m; itau++) {
      for (i_i = 0; i_i + 1 <= itau + 1; i_i++) {
        R_data[i_i + R_sizes[0] * itau] = b_A_data[b_A_sizes[0] * itau + i_i];
      }

      for (i_i = itau + 1; i_i + 1 <= m; i_i++) {
        R_data[i_i + R_sizes[0] * itau] = 0.0F;
      }
    }

    for (itau = A_sizes[0]; itau + 1 < 7; itau++) {
      for (i_i = 0; i_i + 1 <= m; i_i++) {
        R_data[i_i + R_sizes[0] * itau] = b_A_data[b_A_sizes[0] * itau + i_i];
      }
    }

    if (!(A_sizes[0] < 1)) {
      for (itau = A_sizes[0]; itau < m; itau++) {
        i_i = itau * m;
        for (iaii = 0; iaii < m; iaii++) {
          b_A_data[i_i + iaii] = 0.0F;
        }

        b_A_data[i_i + itau] = 1.0F;
      }

      itau = A_sizes[0] - 1;
      for (i = 0; i < 6; i++) {
        work[i] = 0.0F;
      }

      for (i_i = A_sizes[0]; i_i >= 1; i_i--) {
        iaii = (i_i - 1) * m + i_i;
        if (i_i < m) {
          b_A_data[iaii - 1] = 1.0F;
          baimfcjeeknonglf_xzlarf((m - i_i) + 1, m - i_i, iaii, tau_data[itau],
            b_A_data, iaii + m, m, work);
          c = (iaii + m) - i_i;
          for (i = iaii; i + 1 <= c; i++) {
            b_A_data[i] *= -tau_data[itau];
          }
        }

        b_A_data[iaii - 1] = 1.0F - tau_data[itau];
        for (c = 1; c < i_i; c++) {
          b_A_data[(iaii - c) - 1] = 0.0F;
        }

        itau--;
      }
    }

    for (itau = 0; itau + 1 <= m; itau++) {
      for (i_i = 0; i_i + 1 <= m; i_i++) {
        Q_data[i_i + b_idx_0 * itau] = b_A_data[b_A_sizes[0] * itau + i_i];
      }
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
