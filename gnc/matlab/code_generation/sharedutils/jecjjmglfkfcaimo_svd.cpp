//
// File: jecjjmglfkfcaimo_svd.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include <math.h>
#include "cjecbaaifkfkkngd_xaxpy.h"
#include "hdbinophmgdjjmoh_xrotg.h"
#include "imohaimgbaimjmgl_xdotc.h"
#include "jmgljecbphlnngln_xnrm2.h"
#include "jmohnoppbieciecj_xrot.h"
#include "lfcjglnghlfknglf_xscal.h"
#include "ohdjhlfcnohdhdba_xscal.h"
#include "ophdjekfieknglfk_xswap.h"
#include "phlfglfcdbaajmgd_xswap.h"
#include "pphdglfkecjmjekf_xrot.h"
#include "rt_nonfinite.h"
#include "jecjjmglfkfcaimo_svd.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void jecjjmglfkfcaimo_svd(const real32_T A_data[], const int32_T A_sizes[2],
  real32_T U_data[], int32_T U_sizes[2], real32_T S_data[], int32_T S_sizes[2],
  real32_T V_data[], int32_T V_sizes[2])
{
  int32_T p;
  int32_T minnp;
  int32_T nct;
  int32_T q;
  boolean_T apply_transform;
  int32_T iter;
  real32_T snorm;
  real32_T ztest0;
  int32_T kase;
  int32_T qs;
  real32_T ztest;
  real32_T smm1;
  real32_T emm1;
  real32_T sqds;
  real32_T shift;
  int32_T j_ii;
  real32_T varargin_1[5];
  real32_T b_A_data[6];
  int32_T b_A_sizes[2];
  real32_T s_data[2];
  real32_T e_data[2];
  real32_T Vf_data[4];
  real32_T n_data[9];
  int32_T n_sizes[2];
  real32_T o_data[96];
  int32_T o_sizes[2];
  int8_T d_idx_1;
  int8_T d_idx_0;
  int32_T Vf_sizes_idx_0;
  int32_T Vf_sizes_idx_1;
  b_A_sizes[0] = 3;
  b_A_sizes[1] = A_sizes[1];
  kase = A_sizes[0] * A_sizes[1];
  for (qs = 0; qs < kase; qs++) {
    b_A_data[qs] = A_data[qs];
  }

  p = A_sizes[1];
  minnp = A_sizes[1];
  kase = (int8_T)A_sizes[1];
  for (qs = 0; qs < kase; qs++) {
    s_data[qs] = 0.0F;
  }

  kase = (int8_T)A_sizes[1];
  for (qs = 0; qs < kase; qs++) {
    e_data[qs] = 0.0F;
  }

  d_idx_1 = (int8_T)A_sizes[1];
  U_sizes[0] = 3;
  U_sizes[1] = d_idx_1;
  kase = 3 * d_idx_1;
  for (qs = 0; qs < kase; qs++) {
    U_data[qs] = 0.0F;
  }

  d_idx_0 = (int8_T)A_sizes[1];
  d_idx_1 = (int8_T)A_sizes[1];
  Vf_sizes_idx_0 = d_idx_0;
  Vf_sizes_idx_1 = d_idx_1;
  kase = d_idx_0 * d_idx_1;
  for (qs = 0; qs < kase; qs++) {
    Vf_data[qs] = 0.0F;
  }

  if (A_sizes[1] == 0) {
    for (nct = 0; nct + 1 <= minnp; nct++) {
      U_data[nct + 3 * nct] = 1.0F;
    }
  } else {
    nct = A_sizes[1];
    for (q = 0; q + 1 <= nct; q++) {
      iter = 3 * q + q;
      apply_transform = false;
      if (q + 1 <= nct) {
        snorm = jmgljecbphlnngln_xnrm2(3 - q, b_A_data, iter + 1);
        if (snorm > 0.0F) {
          apply_transform = true;
          if (b_A_data[iter] < 0.0F) {
            s_data[q] = -snorm;
          } else {
            s_data[q] = snorm;
          }

          if ((real32_T)fabs((real_T)s_data[q]) >= 9.86076132E-32F) {
            snorm = 1.0F / s_data[q];
            kase = (iter - q) + 3;
            for (qs = iter; qs + 1 <= kase; qs++) {
              b_A_data[qs] *= snorm;
            }
          } else {
            kase = (iter - q) + 3;
            for (qs = iter; qs + 1 <= kase; qs++) {
              b_A_data[qs] /= s_data[q];
            }
          }

          b_A_data[iter]++;
          s_data[q] = -s_data[q];
        } else {
          s_data[q] = 0.0F;
        }
      }

      kase = q + 2;
      while (kase <= p) {
        if (apply_transform) {
          cjecbaaifkfkkngd_xaxpy(3 - q, -(imohaimgbaimjmgl_xdotc(3 - q, b_A_data,
            iter + 1, b_A_data, q + 4) / b_A_data[q + b_A_sizes[0] * q]), iter +
            1, b_A_data, q + 4);
        }

        e_data[1] = b_A_data[q + 3];
        kase = 3;
      }

      if (q + 1 <= nct) {
        for (iter = q; iter + 1 < 4; iter++) {
          U_data[iter + 3 * q] = b_A_data[b_A_sizes[0] * q + iter];
        }
      }
    }

    nct = A_sizes[1];
    if (1 < A_sizes[1]) {
      e_data[0] = b_A_data[b_A_sizes[0]];
    }

    e_data[A_sizes[1] - 1] = 0.0F;
    if (A_sizes[1] + 1 <= A_sizes[1]) {
      iter = A_sizes[1] + 1;
      while (iter <= 2) {
        U_data[3] = 0.0F;
        U_data[5] = 0.0F;
        iter = 3;
        U_data[4] = 1.0F;
      }
    }

    for (q = A_sizes[1] - 1; q + 1 > 0; q--) {
      iter = 3 * q + q;
      if (s_data[q] != 0.0F) {
        kase = q + 2;
        while (kase <= minnp) {
          cjecbaaifkfkkngd_xaxpy(3 - q, -(imohaimgbaimjmgl_xdotc(3 - q, U_data,
            iter + 1, U_data, q + 4) / U_data[iter]), iter + 1, U_data, q + 4);
          kase = 3;
        }

        for (kase = q; kase + 1 < 4; kase++) {
          U_data[kase + U_sizes[0] * q] = -U_data[U_sizes[0] * q + kase];
        }

        U_data[iter]++;
        iter = 1;
        while (iter <= q) {
          U_data[U_sizes[0] * q] = 0.0F;
          iter = 2;
        }
      } else {
        U_data[U_sizes[0] * q] = 0.0F;
        U_data[1 + U_sizes[0] * q] = 0.0F;
        U_data[2 + U_sizes[0] * q] = 0.0F;
        U_data[iter] = 1.0F;
      }
    }

    for (iter = A_sizes[1] - 1; iter + 1 > 0; iter--) {
      for (q = 1; q <= p; q++) {
        Vf_data[(q + d_idx_0 * iter) - 1] = 0.0F;
      }

      Vf_data[iter + d_idx_0 * iter] = 1.0F;
    }

    for (iter = 0; iter + 1 <= nct; iter++) {
      if (s_data[iter] != 0.0F) {
        ztest0 = (real32_T)fabs((real_T)s_data[iter]);
        snorm = s_data[iter] / ztest0;
        s_data[iter] = ztest0;
        if (iter + 1 < nct) {
          e_data[0] /= snorm;
        }

        n_sizes[0] = 3;
        n_sizes[1] = U_sizes[1];
        kase = U_sizes[0] * U_sizes[1];
        for (qs = 0; qs < kase; qs++) {
          n_data[qs] = U_data[qs];
        }

        lfcjglnghlfknglf_xscal(snorm, n_data, 1 + 3 * iter);
        U_sizes[0] = 3;
        U_sizes[1] = n_sizes[1];
        kase = n_sizes[0] * n_sizes[1];
        for (qs = 0; qs < kase; qs++) {
          U_data[qs] = n_data[qs];
        }
      }

      if ((iter + 1 < nct) && (e_data[0] != 0.0F)) {
        ztest0 = (real32_T)fabs((real_T)e_data[0]);
        snorm = ztest0 / e_data[0];
        e_data[0] = ztest0;
        s_data[1] *= snorm;
        o_sizes[0] = Vf_sizes_idx_0;
        o_sizes[1] = Vf_sizes_idx_1;
        kase = Vf_sizes_idx_0 * Vf_sizes_idx_1;
        for (qs = 0; qs < kase; qs++) {
          o_data[qs] = Vf_data[qs];
        }

        ohdjhlfcnohdhdba_xscal(p, snorm, o_data, 1 + p);
        Vf_sizes_idx_0 = o_sizes[0];
        Vf_sizes_idx_1 = o_sizes[1];
        kase = o_sizes[0] * o_sizes[1];
        for (qs = 0; qs < kase; qs++) {
          Vf_data[qs] = o_data[qs];
        }
      }
    }

    iter = 0;
    snorm = 0.0F;
    for (q = 0; q + 1 <= nct; q++) {
      ztest0 = (real32_T)fabs((real_T)s_data[q]);
      ztest = (real32_T)fabs((real_T)e_data[q]);
      if ((ztest0 >= ztest) || rtIsNaNF(ztest)) {
        ztest = ztest0;
      }

      if (!((snorm >= ztest) || rtIsNaNF(ztest))) {
        snorm = ztest;
      }
    }

    while ((nct > 0) && (!(iter >= 75))) {
      kase = nct - 1;
      do {
        qs = 0;
        q = kase;
        if (kase == 0) {
          qs = 1;
        } else {
          ztest0 = (real32_T)fabs((real_T)e_data[0]);
          if ((ztest0 <= ((real32_T)fabs((real_T)s_data[0]) + (real32_T)fabs
                          ((real_T)s_data[1])) * 1.1920929E-7F) || (ztest0 <=
               9.86076132E-32F) || ((iter > 20) && (ztest0 <= 1.1920929E-7F *
                snorm))) {
            e_data[0] = 0.0F;
            qs = 1;
          } else {
            kase = 0;
          }
        }
      } while (qs == 0);

      if (nct - 1 == kase) {
        kase = 4;
      } else {
        qs = nct;
        j_ii = nct;
        apply_transform = false;
        while ((!apply_transform) && (j_ii >= kase)) {
          qs = j_ii;
          if (j_ii == kase) {
            apply_transform = true;
          } else {
            ztest0 = 0.0F;
            if (j_ii < nct) {
              ztest0 = (real32_T)fabs((real_T)e_data[j_ii - 1]);
            }

            if (j_ii > kase + 1) {
              ztest0 += (real32_T)fabs((real_T)e_data[0]);
            }

            ztest = (real32_T)fabs((real_T)s_data[j_ii - 1]);
            if ((ztest <= 1.1920929E-7F * ztest0) || (ztest <= 9.86076132E-32F))
            {
              s_data[j_ii - 1] = 0.0F;
              apply_transform = true;
            } else {
              j_ii--;
            }
          }
        }

        if (qs == kase) {
          kase = 3;
        } else if (qs == nct) {
          kase = 1;
        } else {
          kase = 2;
          q = qs;
        }
      }

      switch (kase) {
       case 1:
        ztest0 = e_data[nct - 2];
        e_data[nct - 2] = 0.0F;
        kase = nct - 1;
        while (kase >= q + 1) {
          ztest = s_data[0];
          hdbinophmgdjjmoh_xrotg(&ztest, &ztest0, &sqds, &smm1);
          s_data[0] = ztest;
          o_sizes[0] = Vf_sizes_idx_0;
          o_sizes[1] = Vf_sizes_idx_1;
          kase = Vf_sizes_idx_0 * Vf_sizes_idx_1;
          for (qs = 0; qs < kase; qs++) {
            o_data[qs] = Vf_data[qs];
          }

          pphdglfkecjmjekf_xrot(p, o_data, 1, 1 + p * (nct - 1), sqds, smm1);
          Vf_sizes_idx_0 = o_sizes[0];
          Vf_sizes_idx_1 = o_sizes[1];
          kase = o_sizes[0] * o_sizes[1];
          for (qs = 0; qs < kase; qs++) {
            Vf_data[qs] = o_data[qs];
          }

          kase = 0;
        }
        break;

       case 2:
        ztest0 = e_data[q - 1];
        e_data[q - 1] = 0.0F;
        for (kase = q; kase + 1 <= nct; kase++) {
          ztest = s_data[kase];
          hdbinophmgdjjmoh_xrotg(&ztest, &ztest0, &sqds, &smm1);
          s_data[kase] = ztest;
          ztest0 = -smm1 * e_data[kase];
          e_data[kase] *= sqds;
          jmohnoppbieciecj_xrot(U_data, 1 + 3 * kase, 1 + 3 * (q - 1), sqds,
                                smm1);
        }
        break;

       case 3:
        varargin_1[0] = (real32_T)fabs((real_T)s_data[nct - 1]);
        varargin_1[1] = (real32_T)fabs((real_T)s_data[nct - 2]);
        varargin_1[2] = (real32_T)fabs((real_T)e_data[nct - 2]);
        varargin_1[3] = (real32_T)fabs((real_T)s_data[q]);
        varargin_1[4] = (real32_T)fabs((real_T)e_data[q]);
        kase = 1;
        ztest = varargin_1[0];
        if (rtIsNaNF(varargin_1[0])) {
          qs = 2;
          apply_transform = false;
          while ((!apply_transform) && (qs < 6)) {
            kase = qs;
            if (!rtIsNaNF(varargin_1[qs - 1])) {
              ztest = varargin_1[qs - 1];
              apply_transform = true;
            } else {
              qs++;
            }
          }
        }

        if (kase < 5) {
          while (kase + 1 < 6) {
            if (varargin_1[kase] > ztest) {
              ztest = varargin_1[kase];
            }

            kase++;
          }
        }

        ztest0 = s_data[nct - 1] / ztest;
        smm1 = s_data[nct - 2] / ztest;
        emm1 = e_data[nct - 2] / ztest;
        sqds = s_data[q] / ztest;
        smm1 = ((smm1 + ztest0) * (smm1 - ztest0) + emm1 * emm1) / 2.0F;
        emm1 *= ztest0;
        emm1 *= emm1;
        if ((smm1 != 0.0F) || (emm1 != 0.0F)) {
          shift = (real32_T)sqrt((real_T)(smm1 * smm1 + emm1));
          if (smm1 < 0.0F) {
            shift = -shift;
          }

          shift = emm1 / (smm1 + shift);
        } else {
          shift = 0.0F;
        }

        ztest0 = (sqds + ztest0) * (sqds - ztest0) + shift;
        ztest = e_data[q] / ztest * sqds;
        while (q + 1 <= nct - 1) {
          hdbinophmgdjjmoh_xrotg(&ztest0, &ztest, &sqds, &smm1);
          ztest0 = sqds * s_data[0] + smm1 * e_data[0];
          e_data[0] = sqds * e_data[0] - smm1 * s_data[0];
          ztest = smm1 * s_data[1];
          s_data[1] *= sqds;
          o_sizes[0] = Vf_sizes_idx_0;
          o_sizes[1] = Vf_sizes_idx_1;
          kase = Vf_sizes_idx_0 * Vf_sizes_idx_1;
          for (qs = 0; qs < kase; qs++) {
            o_data[qs] = Vf_data[qs];
          }

          pphdglfkecjmjekf_xrot(p, o_data, 1, 1 + p, sqds, smm1);
          Vf_sizes_idx_0 = o_sizes[0];
          Vf_sizes_idx_1 = o_sizes[1];
          kase = o_sizes[0] * o_sizes[1];
          for (qs = 0; qs < kase; qs++) {
            Vf_data[qs] = o_data[qs];
          }

          hdbinophmgdjjmoh_xrotg(&ztest0, &ztest, &sqds, &smm1);
          s_data[0] = ztest0;
          ztest0 = sqds * e_data[0] + smm1 * s_data[1];
          s_data[1] = -smm1 * e_data[0] + sqds * s_data[1];
          ztest = smm1 * e_data[1];
          e_data[1] *= sqds;
          jmohnoppbieciecj_xrot(U_data, 1, 4, sqds, smm1);
          q = 1;
        }

        e_data[nct - 2] = ztest0;
        iter++;
        break;

       default:
        if (s_data[q] < 0.0F) {
          s_data[q] = -s_data[q];
          o_sizes[0] = Vf_sizes_idx_0;
          o_sizes[1] = Vf_sizes_idx_1;
          kase = Vf_sizes_idx_0 * Vf_sizes_idx_1;
          for (qs = 0; qs < kase; qs++) {
            o_data[qs] = Vf_data[qs];
          }

          ohdjhlfcnohdhdba_xscal(p, -1.0F, o_data, 1 + p * q);
          Vf_sizes_idx_0 = o_sizes[0];
          Vf_sizes_idx_1 = o_sizes[1];
          kase = o_sizes[0] * o_sizes[1];
          for (qs = 0; qs < kase; qs++) {
            Vf_data[qs] = o_data[qs];
          }
        }

        while ((q + 1 < A_sizes[1]) && (s_data[0] < s_data[1])) {
          ztest0 = s_data[0];
          s_data[0] = s_data[1];
          s_data[1] = ztest0;
          if (1 < p) {
            o_sizes[0] = Vf_sizes_idx_0;
            o_sizes[1] = Vf_sizes_idx_1;
            kase = Vf_sizes_idx_0 * Vf_sizes_idx_1;
            for (qs = 0; qs < kase; qs++) {
              o_data[qs] = Vf_data[qs];
            }

            ophdjekfieknglfk_xswap(2, o_data, 1, 3);
            Vf_sizes_idx_0 = o_sizes[0];
            Vf_sizes_idx_1 = o_sizes[1];
            kase = o_sizes[0] * o_sizes[1];
            for (qs = 0; qs < kase; qs++) {
              Vf_data[qs] = o_data[qs];
            }
          }

          phlfglfcdbaajmgd_xswap(U_data);
          q = 1;
        }

        iter = 0;
        nct--;
        break;
      }
    }
  }

  Vf_sizes_idx_1 = A_sizes[1];
  for (nct = 0; nct + 1 <= minnp; nct++) {
    e_data[nct] = s_data[nct];
  }

  d_idx_0 = (int8_T)A_sizes[1];
  V_sizes[0] = (int8_T)A_sizes[1];
  V_sizes[1] = (int8_T)A_sizes[1];
  for (nct = 0; nct + 1 <= minnp; nct++) {
    for (iter = 0; iter + 1 <= p; iter++) {
      V_data[iter + d_idx_0 * nct] = Vf_data[Vf_sizes_idx_0 * nct + iter];
    }
  }

  S_sizes[0] = A_sizes[1];
  S_sizes[1] = A_sizes[1];
  kase = A_sizes[1] * A_sizes[1];
  for (qs = 0; qs < kase; qs++) {
    S_data[qs] = 0.0F;
  }

  for (p = 0; p < Vf_sizes_idx_1; p++) {
    S_data[p + Vf_sizes_idx_1 * p] = e_data[p];
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
