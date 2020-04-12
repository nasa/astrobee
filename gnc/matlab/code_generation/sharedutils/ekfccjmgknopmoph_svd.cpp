//
// File: ekfccjmgknopmoph_svd.cpp
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:07:06 2018
//
#include "rtwtypes.h"
#include <math.h>
#include <string.h>
#include "bimohlfkbaaidbai_xscal.h"
#include "cbimpphlphlnekfc_xswap.h"
#include "ekfkdjmomglnngln_xdotc.h"
#include "fkngjekfaimgmohd_xaxpy.h"
#include "hdbinophmgdjjmoh_xrotg.h"
#include "imopmohdaaaimgln_xrot.h"
#include "knohlnohphdbkfcj_xnrm2.h"
#include "knoplnopecjmpphd_xaxpy.h"
#include "lfcjdbieaimomoph_xdotc.h"
#include "mgdjglnoimohglno_xaxpy.h"
#include "nglndjmgaimophdj_xswap.h"
#include "nohlkfkngdjmkfcb_xrot.h"
#include "ohlfeknojmopaimg_xscal.h"
#include "opphjmohhlnodjmo_xaxpy.h"
#include "ppppfkfkiekfimop_xnrm2.h"
#include "rt_nonfinite.h"
#include "ekfccjmgknopmoph_svd.h"

// Function for MATLAB Function: '<S12>/MATLAB Function'
void ekfccjmgknopmoph_svd(const real32_T A[72], real32_T U[72], real32_T S[36],
  real32_T V[36])
{
  real32_T b_A[72];
  real32_T s[6];
  real32_T e[6];
  real32_T work[12];
  real32_T Vf[36];
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
  int32_T i;
  boolean_T exitg1;
  boolean_T exitg2;
  int32_T exitg3;
  memcpy(&b_A[0], &A[0], (uint32_T)(72U * sizeof(real32_T)));
  for (i = 0; i < 6; i++) {
    s[i] = 0.0F;
    e[i] = 0.0F;
  }

  for (i = 0; i < 12; i++) {
    work[i] = 0.0F;
  }

  memset(&U[0], 0, (uint32_T)(72U * sizeof(real32_T)));
  memset(&Vf[0], 0, (uint32_T)(36U * sizeof(real32_T)));
  for (i = 0; i < 6; i++) {
    iter = (int32_T)((int32_T)(12 * i) + i);
    apply_transform = false;
    snorm = ppppfkfkiekfimop_xnrm2((int32_T)(12 - i), b_A, (int32_T)(iter + 1));
    if (snorm > 0.0F) {
      apply_transform = true;
      if (b_A[iter] < 0.0F) {
        s[i] = -snorm;
      } else {
        s[i] = snorm;
      }

      if ((real32_T)fabs((real_T)s[i]) >= 9.86076132E-32F) {
        snorm = 1.0F / s[i];
        q = (int32_T)((int32_T)(iter - i) + 12);
        for (qs = iter; (int32_T)(qs + 1) <= q; qs++) {
          b_A[qs] *= snorm;
        }
      } else {
        q = (int32_T)((int32_T)(iter - i) + 12);
        for (qs = iter; (int32_T)(qs + 1) <= q; qs++) {
          b_A[qs] /= s[i];
        }
      }

      b_A[iter]++;
      s[i] = -s[i];
    } else {
      s[i] = 0.0F;
    }

    for (q = (int32_T)(i + 1); (int32_T)(q + 1) < 7; q++) {
      qs = (int32_T)((int32_T)(12 * q) + i);
      if (apply_transform) {
        fkngjekfaimgmohd_xaxpy((int32_T)(12 - i), -(ekfkdjmomglnngln_xdotc
          ((int32_T)(12 - i), b_A, (int32_T)(iter + 1), b_A, (int32_T)(qs + 1)) /
          b_A[(int32_T)(i + (int32_T)(12 * i))]), (int32_T)(iter + 1), b_A,
          (int32_T)(qs + 1));
      }

      e[q] = b_A[qs];
    }

    for (iter = i; (int32_T)(iter + 1) < 13; iter++) {
      U[(int32_T)(iter + (int32_T)(12 * i))] = b_A[(int32_T)((int32_T)(12 * i) +
        iter)];
    }

    if ((int32_T)(i + 1) <= 4) {
      snorm = knohlnohphdbkfcj_xnrm2((int32_T)(5 - i), e, (int32_T)(i + 2));
      if (snorm == 0.0F) {
        e[i] = 0.0F;
      } else {
        if (e[(int32_T)(i + 1)] < 0.0F) {
          e[i] = -snorm;
        } else {
          e[i] = snorm;
        }

        snorm = e[i];
        if ((real32_T)fabs((real_T)e[i]) >= 9.86076132E-32F) {
          snorm = 1.0F / e[i];
          for (iter = (int32_T)(i + 1); (int32_T)(iter + 1) < 7; iter++) {
            e[iter] *= snorm;
          }
        } else {
          for (iter = (int32_T)(i + 1); (int32_T)(iter + 1) < 7; iter++) {
            e[iter] /= snorm;
          }
        }

        e[(int32_T)(i + 1)]++;
        e[i] = -e[i];
        for (iter = (int32_T)(i + 1); (int32_T)(iter + 1) < 13; iter++) {
          work[iter] = 0.0F;
        }

        for (iter = (int32_T)(i + 1); (int32_T)(iter + 1) < 7; iter++) {
          mgdjglnoimohglno_xaxpy((int32_T)(11 - i), e[iter], b_A, (int32_T)
            ((int32_T)(i + (int32_T)(12 * iter)) + 2), work, (int32_T)(i + 2));
        }

        for (iter = (int32_T)(i + 1); (int32_T)(iter + 1) < 7; iter++) {
          opphjmohhlnodjmo_xaxpy((int32_T)(11 - i), -e[iter] / e[(int32_T)(i + 1)],
            work, (int32_T)(i + 2), b_A, (int32_T)((int32_T)(i + (int32_T)(12 *
            iter)) + 2));
        }
      }

      for (iter = (int32_T)(i + 1); (int32_T)(iter + 1) < 7; iter++) {
        Vf[(int32_T)(iter + (int32_T)(6 * i))] = e[iter];
      }
    }
  }

  i = 4;
  e[4] = b_A[64];
  e[5] = 0.0F;
  for (q = 5; q >= 0; q += -1) {
    iter = (int32_T)((int32_T)(12 * q) + q);
    if (s[q] != 0.0F) {
      for (kase = (int32_T)(q + 1); (int32_T)(kase + 1) < 7; kase++) {
        qs = (int32_T)((int32_T)((int32_T)(12 * kase) + q) + 1);
        fkngjekfaimgmohd_xaxpy((int32_T)(12 - q), -(ekfkdjmomglnngln_xdotc
          ((int32_T)(12 - q), U, (int32_T)(iter + 1), U, qs) / U[iter]),
          (int32_T)(iter + 1), U, qs);
      }

      for (qs = q; (int32_T)(qs + 1) < 13; qs++) {
        U[(int32_T)(qs + (int32_T)(12 * q))] = -U[(int32_T)((int32_T)(12 * q) +
          qs)];
      }

      U[iter]++;
      for (iter = 1; iter <= q; iter++) {
        U[(int32_T)((int32_T)(iter + (int32_T)(12 * q)) - 1)] = 0.0F;
      }
    } else {
      for (qs = 0; qs < 12; qs++) {
        U[(int32_T)(qs + (int32_T)(12 * q))] = 0.0F;
      }

      U[iter] = 1.0F;
    }
  }

  for (iter = 5; iter >= 0; iter += -1) {
    if (((int32_T)(iter + 1) <= 4) && (e[iter] != 0.0F)) {
      q = (int32_T)((int32_T)((int32_T)(6 * iter) + iter) + 2);
      for (qs = (int32_T)(iter + 1); (int32_T)(qs + 1) < 7; qs++) {
        kase = (int32_T)((int32_T)((int32_T)(6 * qs) + iter) + 2);
        knoplnopecjmpphd_xaxpy((int32_T)(5 - iter), -(lfcjdbieaimomoph_xdotc
          ((int32_T)(5 - iter), Vf, q, Vf, kase) / Vf[(int32_T)(q - 1)]), q, Vf,
          kase);
      }
    }

    for (q = 0; q < 6; q++) {
      Vf[(int32_T)(q + (int32_T)(6 * iter))] = 0.0F;
    }

    Vf[(int32_T)(iter + (int32_T)(6 * iter))] = 1.0F;
  }

  for (iter = 0; iter < 6; iter++) {
    ztest = e[iter];
    if (s[iter] != 0.0F) {
      ztest0 = (real32_T)fabs((real_T)s[iter]);
      snorm = s[iter] / ztest0;
      s[iter] = ztest0;
      if ((int32_T)(iter + 1) < 6) {
        ztest = e[iter] / snorm;
      }

      bimohlfkbaaidbai_xscal(snorm, U, (int32_T)(1 + (int32_T)(12 * iter)));
    }

    if (((int32_T)(iter + 1) < 6) && (ztest != 0.0F)) {
      ztest0 = (real32_T)fabs((real_T)ztest);
      snorm = ztest0 / ztest;
      ztest = ztest0;
      s[(int32_T)(iter + 1)] *= snorm;
      ohlfeknojmopaimg_xscal(snorm, Vf, (int32_T)(1 + (int32_T)(6 * (int32_T)
        (iter + 1))));
    }

    e[iter] = ztest;
  }

  iter = 0;
  snorm = 0.0F;
  for (q = 0; q < 6; q++) {
    ztest0 = (real32_T)fabs((real_T)s[q]);
    ztest = (real32_T)fabs((real_T)e[q]);
    if ((ztest0 >= ztest) || rtIsNaNF(ztest)) {
      ztest = ztest0;
    }

    if (!((snorm >= ztest) || rtIsNaNF(ztest))) {
      snorm = ztest;
    }
  }

  while (((int32_T)(i + 2) > 0) && (!(iter >= 75))) {
    kase = (int32_T)(i + 1);
    do {
      exitg3 = 0;
      q = kase;
      if (kase == 0) {
        exitg3 = 1;
      } else {
        ztest0 = (real32_T)fabs((real_T)e[(int32_T)(kase - 1)]);
        if ((ztest0 <= ((real32_T)fabs((real_T)s[(int32_T)(kase - 1)]) +
                        (real32_T)fabs((real_T)s[kase])) * 1.1920929E-7F) ||
            ((ztest0 <= 9.86076132E-32F) || ((iter > 20) && (ztest0 <=
               1.1920929E-7F * snorm)))) {
          e[(int32_T)(kase - 1)] = 0.0F;
          exitg3 = 1;
        } else {
          kase--;
        }
      }
    } while (exitg3 == 0);

    if ((int32_T)(i + 1) == kase) {
      kase = 4;
    } else {
      qs = (int32_T)(i + 2);
      j_ii = (int32_T)(i + 2);
      exitg2 = false;
      while ((!exitg2) && (j_ii >= kase)) {
        qs = j_ii;
        if (j_ii == kase) {
          exitg2 = true;
        } else {
          ztest0 = 0.0F;
          if (j_ii < (int32_T)(i + 2)) {
            ztest0 = (real32_T)fabs((real_T)e[(int32_T)(j_ii - 1)]);
          }

          if (j_ii > (int32_T)(kase + 1)) {
            ztest0 += (real32_T)fabs((real_T)e[(int32_T)(j_ii - 2)]);
          }

          ztest = (real32_T)fabs((real_T)s[(int32_T)(j_ii - 1)]);
          if ((ztest <= 1.1920929E-7F * ztest0) || (ztest <= 9.86076132E-32F)) {
            s[(int32_T)(j_ii - 1)] = 0.0F;
            exitg2 = true;
          } else {
            j_ii--;
          }
        }
      }

      if (qs == kase) {
        kase = 3;
      } else if ((int32_T)(i + 2) == qs) {
        kase = 1;
      } else {
        kase = 2;
        q = qs;
      }
    }

    switch (kase) {
     case 1:
      ztest0 = e[i];
      e[i] = 0.0F;
      for (qs = i; (int32_T)(qs + 1) >= (int32_T)(q + 1); qs--) {
        ztest = s[qs];
        hdbinophmgdjjmoh_xrotg(&ztest, &ztest0, &sqds, &smm1);
        s[qs] = ztest;
        if ((int32_T)(qs + 1) > (int32_T)(q + 1)) {
          ztest0 = e[(int32_T)(qs - 1)] * -smm1;
          e[(int32_T)(qs - 1)] *= sqds;
        }

        imopmohdaaaimgln_xrot(Vf, (int32_T)(1 + (int32_T)(6 * qs)), (int32_T)(1
          + (int32_T)(6 * (int32_T)(i + 1))), sqds, smm1);
      }
      break;

     case 2:
      ztest0 = e[(int32_T)(q - 1)];
      e[(int32_T)(q - 1)] = 0.0F;
      for (qs = q; (int32_T)(qs + 1) <= (int32_T)(i + 2); qs++) {
        ztest = s[qs];
        hdbinophmgdjjmoh_xrotg(&ztest, &ztest0, &sqds, &smm1);
        s[qs] = ztest;
        ztest0 = -smm1 * e[qs];
        e[qs] *= sqds;
        nohlkfkngdjmkfcb_xrot(U, (int32_T)(1 + (int32_T)(12 * qs)), (int32_T)(1
          + (int32_T)(12 * (int32_T)(q - 1))), sqds, smm1);
      }
      break;

     case 3:
      varargin_1[0] = (real32_T)fabs((real_T)s[(int32_T)(i + 1)]);
      varargin_1[1] = (real32_T)fabs((real_T)s[i]);
      varargin_1[2] = (real32_T)fabs((real_T)e[i]);
      varargin_1[3] = (real32_T)fabs((real_T)s[q]);
      varargin_1[4] = (real32_T)fabs((real_T)e[q]);
      qs = 1;
      ztest = varargin_1[0];
      if (rtIsNaNF(varargin_1[0])) {
        kase = 2;
        exitg1 = false;
        while ((!exitg1) && (kase < 6)) {
          qs = kase;
          if (!rtIsNaNF(varargin_1[(int32_T)(kase - 1)])) {
            ztest = varargin_1[(int32_T)(kase - 1)];
            exitg1 = true;
          } else {
            kase++;
          }
        }
      }

      if (qs < 5) {
        while ((int32_T)(qs + 1) < 6) {
          if (varargin_1[qs] > ztest) {
            ztest = varargin_1[qs];
          }

          qs++;
        }
      }

      ztest0 = s[(int32_T)(i + 1)] / ztest;
      smm1 = s[i] / ztest;
      emm1 = e[i] / ztest;
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
      for (qs = (int32_T)(q + 1); qs <= (int32_T)(i + 1); qs++) {
        hdbinophmgdjjmoh_xrotg(&ztest0, &ztest, &sqds, &smm1);
        if (qs > (int32_T)(q + 1)) {
          e[(int32_T)(qs - 2)] = ztest0;
        }

        ztest0 = s[(int32_T)(qs - 1)] * sqds + e[(int32_T)(qs - 1)] * smm1;
        e[(int32_T)(qs - 1)] = e[(int32_T)(qs - 1)] * sqds - s[(int32_T)(qs - 1)]
          * smm1;
        ztest = smm1 * s[qs];
        s[qs] *= sqds;
        imopmohdaaaimgln_xrot(Vf, (int32_T)(1 + (int32_T)(6 * (int32_T)(qs - 1))),
                              (int32_T)(1 + (int32_T)(6 * qs)), sqds, smm1);
        hdbinophmgdjjmoh_xrotg(&ztest0, &ztest, &sqds, &smm1);
        s[(int32_T)(qs - 1)] = ztest0;
        ztest0 = e[(int32_T)(qs - 1)] * sqds + smm1 * s[qs];
        s[qs] = e[(int32_T)(qs - 1)] * -smm1 + sqds * s[qs];
        ztest = smm1 * e[qs];
        e[qs] *= sqds;
        nohlkfkngdjmkfcb_xrot(U, (int32_T)(1 + (int32_T)(12 * (int32_T)(qs - 1))),
                              (int32_T)(1 + (int32_T)(12 * qs)), sqds, smm1);
      }

      e[i] = ztest0;
      iter++;
      break;

     default:
      if (s[q] < 0.0F) {
        s[q] = -s[q];
        ohlfeknojmopaimg_xscal(-1.0F, Vf, (int32_T)(1 + (int32_T)(6 * q)));
      }

      iter = (int32_T)(q + 1);
      while (((int32_T)(q + 1) < 6) && (s[q] < s[iter])) {
        ztest0 = s[q];
        s[q] = s[iter];
        s[iter] = ztest0;
        cbimpphlphlnekfc_xswap(Vf, (int32_T)(1 + (int32_T)(6 * q)), (int32_T)(1
          + (int32_T)(6 * (int32_T)(q + 1))));
        nglndjmgaimophdj_xswap(U, (int32_T)(1 + (int32_T)(12 * q)), (int32_T)(1
          + (int32_T)(12 * (int32_T)(q + 1))));
        q = iter;
        iter++;
      }

      iter = 0;
      i--;
      break;
    }
  }

  for (i = 0; i < 6; i++) {
    e[i] = s[i];
    for (iter = 0; iter < 6; iter++) {
      V[(int32_T)(iter + (int32_T)(6 * i))] = Vf[(int32_T)((int32_T)(6 * i) +
        iter)];
    }
  }

  memset(&S[0], 0, (uint32_T)(36U * sizeof(real32_T)));
  for (i = 0; i < 6; i++) {
    S[(int32_T)(i + (int32_T)(6 * i))] = e[i];
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
