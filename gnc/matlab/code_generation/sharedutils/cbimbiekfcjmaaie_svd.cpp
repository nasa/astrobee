//
// File: cbimbiekfcjmaaie_svd.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include <math.h>
#include <string.h>
#include "aimgimglphlffkfk_xswap.h"
#include "cjekkfkfmoppfkfc_xdotc.h"
#include "dbainohddbimecba_xnrm2.h"
#include "dbiebiecimopdbaa_xnrm2.h"
#include "glfkdbaibaaakfcb_xscal.h"
#include "hdbinophmgdjjmoh_xrotg.h"
#include "ieknhdjmlngdkfcb_xaxpy.h"
#include "jecbfkfkhdbangdj_xaxpy.h"
#include "moppjecbaaaaimop_xrot.h"
#include "phlniekfphlndjmo_xaxpy.h"
#include "rt_nonfinite.h"
#include "cbimbiekfcjmaaie_svd.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void cbimbiekfcjmaaie_svd(const real32_T A[16], real32_T U[16], real32_T S[16],
  real32_T V[16])
{
  real32_T b_A[16];
  real32_T s[4];
  real32_T e[4];
  real32_T work[4];
  real32_T Vf[16];
  int32_T q;
  boolean_T apply_transform;
  int32_T m;
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
  int32_T k_ii;
  real32_T varargin_1[5];
  e[0] = 0.0F;
  work[0] = 0.0F;
  e[1] = 0.0F;
  work[1] = 0.0F;
  e[2] = 0.0F;
  work[2] = 0.0F;
  e[3] = 0.0F;
  work[3] = 0.0F;
  for (iter = 0; iter < 16; iter++) {
    b_A[iter] = A[iter];
    U[iter] = 0.0F;
    Vf[iter] = 0.0F;
  }

  iter = 0;
  apply_transform = false;
  snorm = dbiebiecimopdbaa_xnrm2(4, b_A, 1);
  if (snorm > 0.0F) {
    apply_transform = true;
    if (b_A[0] < 0.0F) {
      s[0] = -snorm;
    } else {
      s[0] = snorm;
    }

    if ((real32_T)fabs((real_T)s[0]) >= 9.86076132E-32F) {
      snorm = 1.0F / s[0];
      for (kase = 0; kase + 1 < 5; kase++) {
        b_A[kase] *= snorm;
      }
    } else {
      for (kase = 0; kase + 1 < 5; kase++) {
        b_A[kase] /= s[0];
      }
    }

    b_A[0]++;
    s[0] = -s[0];
  } else {
    s[0] = 0.0F;
  }

  for (q = 1; q + 1 < 5; q++) {
    kase = q << 2;
    if (apply_transform) {
      phlniekfphlndjmo_xaxpy(4, -(cjekkfkfmoppfkfc_xdotc(4, b_A, 1, b_A, kase +
        1) / b_A[0]), 1, b_A, kase + 1);
    }

    e[q] = b_A[kase];
  }

  while (iter + 1 < 5) {
    U[iter] = b_A[iter];
    iter++;
  }

  snorm = dbainohddbimecba_xnrm2(3, e, 2);
  if (snorm == 0.0F) {
    e[0] = 0.0F;
  } else {
    if (e[1] < 0.0F) {
      e[0] = -snorm;
    } else {
      e[0] = snorm;
    }

    snorm = e[0];
    if ((real32_T)fabs((real_T)e[0]) >= 9.86076132E-32F) {
      snorm = 1.0F / e[0];
      for (iter = 1; iter + 1 < 5; iter++) {
        e[iter] *= snorm;
      }
    } else {
      for (iter = 1; iter + 1 < 5; iter++) {
        e[iter] /= snorm;
      }
    }

    e[1]++;
    e[0] = -e[0];
    for (iter = 1; iter + 1 < 5; iter++) {
      work[iter] = 0.0F;
    }

    for (iter = 1; iter + 1 < 5; iter++) {
      jecbfkfkhdbangdj_xaxpy(3, e[iter], b_A, (iter << 2) + 2, work, 2);
    }

    for (iter = 1; iter + 1 < 5; iter++) {
      ieknhdjmlngdkfcb_xaxpy(3, -e[iter] / e[1], work, 2, b_A, (iter << 2) + 2);
    }
  }

  for (iter = 1; iter + 1 < 5; iter++) {
    Vf[iter] = e[iter];
  }

  apply_transform = false;
  snorm = dbiebiecimopdbaa_xnrm2(3, b_A, 6);
  if (snorm > 0.0F) {
    apply_transform = true;
    if (b_A[5] < 0.0F) {
      s[1] = -snorm;
    } else {
      s[1] = snorm;
    }

    if ((real32_T)fabs((real_T)s[1]) >= 9.86076132E-32F) {
      snorm = 1.0F / s[1];
      for (kase = 5; kase + 1 < 9; kase++) {
        b_A[kase] *= snorm;
      }
    } else {
      for (kase = 5; kase + 1 < 9; kase++) {
        b_A[kase] /= s[1];
      }
    }

    b_A[5]++;
    s[1] = -s[1];
  } else {
    s[1] = 0.0F;
  }

  for (q = 2; q + 1 < 5; q++) {
    kase = (q << 2) + 1;
    if (apply_transform) {
      phlniekfphlndjmo_xaxpy(3, -(cjekkfkfmoppfkfc_xdotc(3, b_A, 6, b_A, kase +
        1) / b_A[5]), 6, b_A, kase + 1);
    }

    e[q] = b_A[kase];
  }

  for (iter = 1; iter + 1 < 5; iter++) {
    U[iter + 4] = b_A[iter + 4];
  }

  snorm = dbainohddbimecba_xnrm2(2, e, 3);
  if (snorm == 0.0F) {
    e[1] = 0.0F;
  } else {
    if (e[2] < 0.0F) {
      e[1] = -snorm;
    } else {
      e[1] = snorm;
    }

    snorm = e[1];
    if ((real32_T)fabs((real_T)e[1]) >= 9.86076132E-32F) {
      snorm = 1.0F / e[1];
      for (iter = 2; iter + 1 < 5; iter++) {
        e[iter] *= snorm;
      }
    } else {
      for (iter = 2; iter + 1 < 5; iter++) {
        e[iter] /= snorm;
      }
    }

    e[2]++;
    e[1] = -e[1];
    for (iter = 2; iter + 1 < 5; iter++) {
      work[iter] = 0.0F;
    }

    for (iter = 2; iter + 1 < 5; iter++) {
      jecbfkfkhdbangdj_xaxpy(2, e[iter], b_A, (iter << 2) + 3, work, 3);
    }

    for (iter = 2; iter + 1 < 5; iter++) {
      ieknhdjmlngdkfcb_xaxpy(2, -e[iter] / e[2], work, 3, b_A, (iter << 2) + 3);
    }
  }

  for (iter = 2; iter + 1 < 5; iter++) {
    Vf[iter + 4] = e[iter];
  }

  apply_transform = false;
  snorm = dbiebiecimopdbaa_xnrm2(2, b_A, 11);
  if (snorm > 0.0F) {
    apply_transform = true;
    if (b_A[10] < 0.0F) {
      s[2] = -snorm;
    } else {
      s[2] = snorm;
    }

    if ((real32_T)fabs((real_T)s[2]) >= 9.86076132E-32F) {
      snorm = 1.0F / s[2];
      for (kase = 10; kase + 1 < 13; kase++) {
        b_A[kase] *= snorm;
      }
    } else {
      for (kase = 10; kase + 1 < 13; kase++) {
        b_A[kase] /= s[2];
      }
    }

    b_A[10]++;
    s[2] = -s[2];
  } else {
    s[2] = 0.0F;
  }

  for (q = 3; q + 1 < 5; q++) {
    kase = (q << 2) + 2;
    if (apply_transform) {
      phlniekfphlndjmo_xaxpy(2, -(cjekkfkfmoppfkfc_xdotc(2, b_A, 11, b_A, kase +
        1) / b_A[10]), 11, b_A, kase + 1);
    }

    e[q] = b_A[kase];
  }

  for (iter = 2; iter + 1 < 5; iter++) {
    U[iter + 8] = b_A[iter + 8];
  }

  m = 2;
  s[3] = b_A[15];
  e[2] = b_A[14];
  e[3] = 0.0F;
  U[12] = 0.0F;
  U[13] = 0.0F;
  U[14] = 0.0F;
  U[15] = 1.0F;
  for (q = 2; q >= 0; q += -1) {
    iter = (q << 2) + q;
    if (s[q] != 0.0F) {
      for (qs = q + 1; qs + 1 < 5; qs++) {
        kase = ((qs << 2) + q) + 1;
        phlniekfphlndjmo_xaxpy(4 - q, -(cjekkfkfmoppfkfc_xdotc(4 - q, U, iter +
          1, U, kase) / U[iter]), iter + 1, U, kase);
      }

      for (kase = q; kase + 1 < 5; kase++) {
        U[kase + (q << 2)] = -U[(q << 2) + kase];
      }

      U[iter]++;
      for (iter = 1; iter <= q; iter++) {
        U[(iter + (q << 2)) - 1] = 0.0F;
      }
    } else {
      U[q << 2] = 0.0F;
      U[1 + (q << 2)] = 0.0F;
      U[2 + (q << 2)] = 0.0F;
      U[3 + (q << 2)] = 0.0F;
      U[iter] = 1.0F;
    }
  }

  for (iter = 3; iter >= 0; iter += -1) {
    if ((iter + 1 <= 2) && (e[iter] != 0.0F)) {
      q = ((iter << 2) + iter) + 2;
      for (kase = iter + 1; kase + 1 < 5; kase++) {
        qs = ((kase << 2) + iter) + 2;
        phlniekfphlndjmo_xaxpy(3 - iter, -(cjekkfkfmoppfkfc_xdotc(3 - iter, Vf,
          q, Vf, qs) / Vf[q - 1]), q, Vf, qs);
      }
    }

    Vf[iter << 2] = 0.0F;
    Vf[1 + (iter << 2)] = 0.0F;
    Vf[2 + (iter << 2)] = 0.0F;
    Vf[3 + (iter << 2)] = 0.0F;
    Vf[iter + (iter << 2)] = 1.0F;
  }

  ztest = e[0];
  if (s[0] != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)s[0]);
    snorm = s[0] / ztest0;
    s[0] = ztest0;
    ztest = e[0] / snorm;
    glfkdbaibaaakfcb_xscal(snorm, U, 1);
  }

  if (ztest != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)ztest);
    snorm = ztest0 / ztest;
    ztest = ztest0;
    s[1] *= snorm;
    glfkdbaibaaakfcb_xscal(snorm, Vf, 5);
  }

  e[0] = ztest;
  ztest = e[1];
  if (s[1] != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)s[1]);
    snorm = s[1] / ztest0;
    s[1] = ztest0;
    ztest = e[1] / snorm;
    glfkdbaibaaakfcb_xscal(snorm, U, 5);
  }

  if (ztest != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)ztest);
    snorm = ztest0 / ztest;
    ztest = ztest0;
    s[2] *= snorm;
    glfkdbaibaaakfcb_xscal(snorm, Vf, 9);
  }

  e[1] = ztest;
  ztest = b_A[14];
  if (s[2] != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)s[2]);
    snorm = s[2] / ztest0;
    s[2] = ztest0;
    ztest = b_A[14] / snorm;
    glfkdbaibaaakfcb_xscal(snorm, U, 9);
  }

  if (ztest != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)ztest);
    snorm = ztest0 / ztest;
    ztest = ztest0;
    s[3] = b_A[15] * snorm;
    glfkdbaibaaakfcb_xscal(snorm, Vf, 13);
  }

  e[2] = ztest;
  if (s[3] != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)s[3]);
    snorm = s[3] / ztest0;
    s[3] = ztest0;
    glfkdbaibaaakfcb_xscal(snorm, U, 13);
  }

  e[3] = 0.0F;
  iter = 0;
  snorm = 0.0F;
  if ((s[0] >= e[0]) || rtIsNaNF(e[0])) {
    ztest0 = s[0];
  } else {
    ztest0 = e[0];
  }

  if (!((0.0F >= ztest0) || rtIsNaNF(ztest0))) {
    snorm = ztest0;
  }

  if ((s[1] >= e[1]) || rtIsNaNF(e[1])) {
    ztest0 = s[1];
  } else {
    ztest0 = e[1];
  }

  if (!((snorm >= ztest0) || rtIsNaNF(ztest0))) {
    snorm = ztest0;
  }

  if ((s[2] >= ztest) || rtIsNaNF(ztest)) {
    ztest = s[2];
  }

  if (!((snorm >= ztest) || rtIsNaNF(ztest))) {
    snorm = ztest;
  }

  if (s[3] >= 0.0F) {
    ztest0 = s[3];
  } else {
    ztest0 = 0.0F;
  }

  if (!((snorm >= ztest0) || rtIsNaNF(ztest0))) {
    snorm = ztest0;
  }

  while ((m + 2 > 0) && (!(iter >= 75))) {
    kase = m + 1;
    do {
      qs = 0;
      q = kase;
      if (kase == 0) {
        qs = 1;
      } else {
        ztest0 = (real32_T)fabs((real_T)e[kase - 1]);
        if ((ztest0 <= ((real32_T)fabs((real_T)s[kase - 1]) + (real32_T)fabs
                        ((real_T)s[kase])) * 1.1920929E-7F) || (ztest0 <=
             9.86076132E-32F) || ((iter > 20) && (ztest0 <= 1.1920929E-7F *
              snorm))) {
          e[kase - 1] = 0.0F;
          qs = 1;
        } else {
          kase--;
        }
      }
    } while (qs == 0);

    if (m + 1 == kase) {
      kase = 4;
    } else {
      qs = m + 2;
      k_ii = m + 2;
      apply_transform = false;
      while ((!apply_transform) && (k_ii >= kase)) {
        qs = k_ii;
        if (k_ii == kase) {
          apply_transform = true;
        } else {
          ztest0 = 0.0F;
          if (k_ii < m + 2) {
            ztest0 = (real32_T)fabs((real_T)e[k_ii - 1]);
          }

          if (k_ii > kase + 1) {
            ztest0 += (real32_T)fabs((real_T)e[k_ii - 2]);
          }

          ztest = (real32_T)fabs((real_T)s[k_ii - 1]);
          if ((ztest <= 1.1920929E-7F * ztest0) || (ztest <= 9.86076132E-32F)) {
            s[k_ii - 1] = 0.0F;
            apply_transform = true;
          } else {
            k_ii--;
          }
        }
      }

      if (qs == kase) {
        kase = 3;
      } else if (m + 2 == qs) {
        kase = 1;
      } else {
        kase = 2;
        q = qs;
      }
    }

    switch (kase) {
     case 1:
      ztest0 = e[m];
      e[m] = 0.0F;
      for (kase = m; kase + 1 >= q + 1; kase--) {
        ztest = s[kase];
        hdbinophmgdjjmoh_xrotg(&ztest, &ztest0, &sqds, &smm1);
        s[kase] = ztest;
        if (kase + 1 > q + 1) {
          ztest0 = e[kase - 1] * -smm1;
          e[kase - 1] *= sqds;
        }

        moppjecbaaaaimop_xrot(Vf, 1 + (kase << 2), 1 + ((m + 1) << 2), sqds,
                              smm1);
      }
      break;

     case 2:
      ztest0 = e[q - 1];
      e[q - 1] = 0.0F;
      for (kase = q; kase + 1 <= m + 2; kase++) {
        ztest = s[kase];
        hdbinophmgdjjmoh_xrotg(&ztest, &ztest0, &sqds, &smm1);
        s[kase] = ztest;
        ztest0 = -smm1 * e[kase];
        e[kase] *= sqds;
        moppjecbaaaaimop_xrot(U, 1 + (kase << 2), 1 + ((q - 1) << 2), sqds, smm1);
      }
      break;

     case 3:
      varargin_1[0] = (real32_T)fabs((real_T)s[m + 1]);
      varargin_1[1] = (real32_T)fabs((real_T)s[m]);
      varargin_1[2] = (real32_T)fabs((real_T)e[m]);
      varargin_1[3] = (real32_T)fabs((real_T)s[q]);
      varargin_1[4] = (real32_T)fabs((real_T)e[q]);
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

      ztest0 = s[m + 1] / ztest;
      smm1 = s[m] / ztest;
      emm1 = e[m] / ztest;
      sqds = s[q] / ztest;
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
      ztest = e[q] / ztest * sqds;
      for (kase = q + 1; kase <= m + 1; kase++) {
        hdbinophmgdjjmoh_xrotg(&ztest0, &ztest, &sqds, &smm1);
        if (kase > q + 1) {
          e[kase - 2] = ztest0;
        }

        ztest0 = s[kase - 1] * sqds + e[kase - 1] * smm1;
        e[kase - 1] = e[kase - 1] * sqds - s[kase - 1] * smm1;
        ztest = smm1 * s[kase];
        s[kase] *= sqds;
        moppjecbaaaaimop_xrot(Vf, 1 + ((kase - 1) << 2), 1 + (kase << 2), sqds,
                              smm1);
        hdbinophmgdjjmoh_xrotg(&ztest0, &ztest, &sqds, &smm1);
        s[kase - 1] = ztest0;
        ztest0 = e[kase - 1] * sqds + smm1 * s[kase];
        s[kase] = e[kase - 1] * -smm1 + sqds * s[kase];
        ztest = smm1 * e[kase];
        e[kase] *= sqds;
        moppjecbaaaaimop_xrot(U, 1 + ((kase - 1) << 2), 1 + (kase << 2), sqds,
                              smm1);
      }

      e[m] = ztest0;
      iter++;
      break;

     default:
      if (s[q] < 0.0F) {
        s[q] = -s[q];
        glfkdbaibaaakfcb_xscal(-1.0F, Vf, 1 + (q << 2));
      }

      iter = q + 1;
      while ((q + 1 < 4) && (s[q] < s[iter])) {
        ztest0 = s[q];
        s[q] = s[iter];
        s[iter] = ztest0;
        aimgimglphlffkfk_xswap(Vf, 1 + (q << 2), 1 + ((q + 1) << 2));
        aimgimglphlffkfk_xswap(U, 1 + (q << 2), 1 + ((q + 1) << 2));
        q = iter;
        iter++;
      }

      iter = 0;
      m--;
      break;
    }
  }

  memcpy(&V[0], &Vf[0], sizeof(real32_T) << 4U);
  for (m = 0; m < 4; m++) {
    e[m] = s[m];
  }

  memset(&S[0], 0, sizeof(real32_T) << 4U);
  S[0] = e[0];
  S[5] = e[1];
  S[10] = e[2];
  S[15] = e[3];
}

//
// File trailer for generated code.
//
// [EOF]
//
