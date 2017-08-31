//
// File: jmgljmgdphdjlfcj_svd.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include <math.h>
#include "cbimeknolnglpphd_xswap.h"
#include "ecbaphdbnglnimoh_xdotc.h"
#include "fcjejmgdgdbicjek_xaxpy.h"
#include "gdbijekfimglfcje_xrot.h"
#include "hdbinophmgdjjmoh_xrotg.h"
#include "imgdgdbadbaihlng_xnrm2.h"
#include "knopcjmocbaacbaa_xscal.h"
#include "rt_nonfinite.h"
#include "jmgljmgdphdjlfcj_svd.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void jmgljmgdphdjlfcj_svd(const real32_T A[4], real32_T U[4], real32_T S[4],
  real32_T V[4])
{
  real32_T b_A[4];
  real32_T s[2];
  real32_T e[2];
  real32_T Vf[4];
  int32_T q;
  boolean_T apply_transform;
  real32_T nrm;
  int32_T m;
  int32_T iter;
  real32_T ztest0;
  int32_T kase;
  int32_T qs;
  real32_T ztest;
  real32_T smm1;
  real32_T emm1;
  real32_T sqds;
  real32_T shift;
  int32_T h_ii;
  real32_T varargin_1[5];
  b_A[0] = A[0];
  b_A[1] = A[1];
  b_A[2] = A[2];
  b_A[3] = A[3];
  apply_transform = false;
  nrm = imgdgdbadbaihlng_xnrm2(A);
  if (nrm > 0.0F) {
    apply_transform = true;
    if (A[0] < 0.0F) {
      nrm = -nrm;
    }

    if ((real32_T)fabs((real_T)nrm) >= 9.86076132E-32F) {
      ztest0 = 1.0F / nrm;
      b_A[0] = ztest0 * A[0];
      b_A[1] = ztest0 * A[1];
    } else {
      b_A[0] = A[0] / nrm;
      b_A[1] = A[1] / nrm;
    }

    b_A[0]++;
    s[0] = -nrm;
  } else {
    s[0] = 0.0F;
  }

  if (apply_transform) {
    fcjejmgdgdbicjek_xaxpy(-(ecbaphdbnglnimoh_xdotc(b_A, b_A) / b_A[0]), b_A);
  }

  U[0] = b_A[0];
  U[2] = 0.0F;
  U[1] = b_A[1];
  m = 2;
  s[1] = b_A[3];
  e[1] = 0.0F;
  U[3] = 1.0F;
  if (s[0] != 0.0F) {
    fcjejmgdgdbicjek_xaxpy(-(ecbaphdbnglnimoh_xdotc(U, U) / b_A[0]), U);
    U[0] = -U[0];
    U[1] = -U[1];
    U[0]++;
  } else {
    U[1] = 0.0F;
    U[0] = 1.0F;
  }

  Vf[2] = 0.0F;
  Vf[3] = 1.0F;
  Vf[1] = 0.0F;
  Vf[0] = 1.0F;
  ztest = b_A[2];
  if (s[0] != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)s[0]);
    nrm = s[0] / ztest0;
    s[0] = ztest0;
    ztest = b_A[2] / nrm;
    knopcjmocbaacbaa_xscal(nrm, U, 1);
  }

  if (ztest != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)ztest);
    nrm = ztest0 / ztest;
    ztest = ztest0;
    s[1] = b_A[3] * nrm;
    knopcjmocbaacbaa_xscal(nrm, Vf, 3);
  }

  if (s[1] != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)s[1]);
    nrm = s[1] / ztest0;
    s[1] = ztest0;
    knopcjmocbaacbaa_xscal(nrm, U, 3);
  }

  e[0] = ztest;
  iter = 0;
  nrm = 0.0F;
  if ((s[0] >= ztest) || rtIsNaNF(ztest)) {
    ztest = s[0];
  }

  if (!((0.0F >= ztest) || rtIsNaNF(ztest))) {
    nrm = ztest;
  }

  if (s[1] >= 0.0F) {
    ztest0 = s[1];
  } else {
    ztest0 = 0.0F;
  }

  if (!((nrm >= ztest0) || rtIsNaNF(ztest0))) {
    nrm = ztest0;
  }

  while ((m > 0) && (!(iter >= 75))) {
    kase = m - 1;
    do {
      qs = 0;
      q = kase;
      if (kase == 0) {
        qs = 1;
      } else {
        ztest0 = (real32_T)fabs((real_T)e[0]);
        if ((ztest0 <= ((real32_T)fabs((real_T)s[0]) + (real32_T)fabs((real_T)s
               [1])) * 1.1920929E-7F) || (ztest0 <= 9.86076132E-32F) || ((iter >
              20) && (ztest0 <= 1.1920929E-7F * nrm))) {
          e[0] = 0.0F;
          qs = 1;
        } else {
          kase = 0;
        }
      }
    } while (qs == 0);

    if (m - 1 == kase) {
      kase = 4;
    } else {
      qs = m;
      h_ii = m;
      apply_transform = false;
      while ((!apply_transform) && (h_ii >= kase)) {
        qs = h_ii;
        if (h_ii == kase) {
          apply_transform = true;
        } else {
          ztest0 = 0.0F;
          if (h_ii < m) {
            ztest0 = (real32_T)fabs((real_T)e[0]);
          }

          if (h_ii > kase + 1) {
            ztest0 += (real32_T)fabs((real_T)e[0]);
          }

          ztest = (real32_T)fabs((real_T)s[h_ii - 1]);
          if ((ztest <= 1.1920929E-7F * ztest0) || (ztest <= 9.86076132E-32F)) {
            s[h_ii - 1] = 0.0F;
            apply_transform = true;
          } else {
            h_ii--;
          }
        }
      }

      if (qs == kase) {
        kase = 3;
      } else if (qs == m) {
        kase = 1;
      } else {
        kase = 2;
        q = qs;
      }
    }

    switch (kase) {
     case 1:
      ztest0 = e[0];
      e[0] = 0.0F;
      kase = m - 1;
      while (kase >= q + 1) {
        ztest = s[0];
        hdbinophmgdjjmoh_xrotg(&ztest, &ztest0, &sqds, &smm1);
        s[0] = ztest;
        gdbijekfimglfcje_xrot(Vf, 1, 1 + ((m - 1) << 1), sqds, smm1);
        kase = 0;
      }
      break;

     case 2:
      ztest0 = e[q - 1];
      e[q - 1] = 0.0F;
      for (kase = q; kase + 1 <= m; kase++) {
        ztest = s[kase];
        hdbinophmgdjjmoh_xrotg(&ztest, &ztest0, &sqds, &smm1);
        s[kase] = ztest;
        ztest0 = -smm1 * e[kase];
        e[kase] *= sqds;
        gdbijekfimglfcje_xrot(U, 1 + (kase << 1), 1 + ((q - 1) << 1), sqds, smm1);
      }
      break;

     case 3:
      varargin_1[0] = (real32_T)fabs((real_T)s[m - 1]);
      varargin_1[1] = (real32_T)fabs((real_T)s[0]);
      varargin_1[2] = (real32_T)fabs((real_T)e[0]);
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

      ztest0 = s[m - 1] / ztest;
      smm1 = s[0] / ztest;
      emm1 = e[0] / ztest;
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
      while (q + 1 <= 1) {
        hdbinophmgdjjmoh_xrotg(&ztest0, &ztest, &sqds, &smm1);
        ztest0 = sqds * s[0] + smm1 * e[0];
        e[0] = sqds * e[0] - smm1 * s[0];
        ztest = smm1 * s[1];
        s[1] *= sqds;
        gdbijekfimglfcje_xrot(Vf, 1, 3, sqds, smm1);
        hdbinophmgdjjmoh_xrotg(&ztest0, &ztest, &sqds, &smm1);
        s[0] = ztest0;
        ztest0 = sqds * e[0] + smm1 * s[1];
        s[1] = -smm1 * e[0] + sqds * s[1];
        ztest = smm1 * e[1];
        e[1] *= sqds;
        gdbijekfimglfcje_xrot(U, 1, 3, sqds, smm1);
        q = 1;
      }

      e[0] = ztest0;
      iter++;
      break;

     default:
      if (s[q] < 0.0F) {
        s[q] = -s[q];
        knopcjmocbaacbaa_xscal(-1.0F, Vf, 1 + (q << 1));
      }

      while ((q + 1 < 2) && (s[0] < s[1])) {
        ztest0 = s[0];
        s[0] = s[1];
        s[1] = ztest0;
        cbimeknolnglpphd_xswap(Vf);
        cbimeknolnglpphd_xswap(U);
        q = 1;
      }

      iter = 0;
      m--;
      break;
    }
  }

  V[0] = Vf[0];
  V[1] = Vf[1];
  V[2] = Vf[2];
  V[3] = Vf[3];
  S[1] = 0.0F;
  S[2] = 0.0F;
  S[0] = s[0];
  S[3] = s[1];
}

//
// File trailer for generated code.
//
// [EOF]
//
