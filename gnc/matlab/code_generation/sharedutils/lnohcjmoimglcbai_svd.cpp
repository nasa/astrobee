//
// File: lnohcjmoimglcbai_svd.cpp
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
#include "cbielfcbnophecbi_xaxpy.h"
#include "cjekaaieekfkbimo_xnrm2.h"
#include "cjmgkfkfphdjaaai_xscal.h"
#include "djmgbimooppphlno_xswap.h"
#include "ekfklfkfohdjaimg_xaxpy.h"
#include "eknofcjenophiecj_xswap.h"
#include "fcbiohlnbimolnop_xnrm2.h"
#include "gdjmmgdjekfccjmg_xrot.h"
#include "hdbinophmgdjjmoh_xrotg.h"
#include "hlfkiecjohlfopph_xaxpy.h"
#include "jekncbaiknglnoph_xdotc.h"
#include "mglnohlnekfcekfk_xdotc.h"
#include "ophdimgdcjekekfc_xrot.h"
#include "ophlaaaiophdbaai_xscal.h"
#include "opppmglfglfkdbaa_xaxpy.h"
#include "rt_nonfinite.h"
#include "lnohcjmoimglcbai_svd.h"

// Function for MATLAB Function: '<S12>/MATLAB Function'
void lnohcjmoimglcbai_svd(const real32_T A[36], real32_T U[36], real32_T S[9],
  real32_T V[9])
{
  real32_T b_A[36];
  real32_T s[3];
  real32_T e[3];
  real32_T work[12];
  real32_T Vf[9];
  int32_T q;
  boolean_T apply_transform;
  int32_T m;
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
  memcpy(&b_A[0], &A[0], (uint32_T)(36U * sizeof(real32_T)));
  e[0] = 0.0F;
  e[1] = 0.0F;
  e[2] = 0.0F;
  for (i = 0; i < 12; i++) {
    work[i] = 0.0F;
  }

  memset(&U[0], 0, (uint32_T)(36U * sizeof(real32_T)));
  for (i = 0; i < 9; i++) {
    Vf[i] = 0.0F;
  }

  i = 0;
  apply_transform = false;
  snorm = cjekaaieekfkbimo_xnrm2(12, b_A, 1);
  if (snorm > 0.0F) {
    apply_transform = true;
    if (b_A[0] < 0.0F) {
      s[0] = -snorm;
    } else {
      s[0] = snorm;
    }

    if ((real32_T)fabs((real_T)s[0]) >= 9.86076132E-32F) {
      snorm = 1.0F / s[0];
      for (qs = 0; (int32_T)(qs + 1) < 13; qs++) {
        b_A[qs] *= snorm;
      }
    } else {
      for (qs = 0; (int32_T)(qs + 1) < 13; qs++) {
        b_A[qs] /= s[0];
      }
    }

    b_A[0]++;
    s[0] = -s[0];
  } else {
    s[0] = 0.0F;
  }

  for (q = 1; (int32_T)(q + 1) < 4; q++) {
    qs = (int32_T)(12 * q);
    if (apply_transform) {
      hlfkiecjohlfopph_xaxpy(12, -(jekncbaiknglnoph_xdotc(12, b_A, 1, b_A,
        (int32_T)(qs + 1)) / b_A[0]), 1, b_A, (int32_T)(qs + 1));
    }

    e[q] = b_A[qs];
  }

  while ((int32_T)(i + 1) < 13) {
    U[i] = b_A[i];
    i++;
  }

  snorm = fcbiohlnbimolnop_xnrm2(2, e, 2);
  if (snorm == 0.0F) {
    e[0] = 0.0F;
  } else {
    if (e[1] < 0.0F) {
      snorm = -snorm;
    }

    e[0] = snorm;
    if ((real32_T)fabs((real_T)snorm) >= 9.86076132E-32F) {
      snorm = 1.0F / snorm;
      for (i = 1; (int32_T)(i + 1) < 4; i++) {
        e[i] *= snorm;
      }
    } else {
      for (i = 1; (int32_T)(i + 1) < 4; i++) {
        e[i] /= snorm;
      }
    }

    e[1]++;
    e[0] = -e[0];
    for (i = 2; i < 13; i++) {
      work[(int32_T)(i - 1)] = 0.0F;
    }

    for (i = 1; (int32_T)(i + 1) < 4; i++) {
      opppmglfglfkdbaa_xaxpy(11, e[i], b_A, (int32_T)((int32_T)(12 * i) + 2),
        work, 2);
    }

    for (i = 1; (int32_T)(i + 1) < 4; i++) {
      cbielfcbnophecbi_xaxpy(11, -e[i] / e[1], work, 2, b_A, (int32_T)((int32_T)
        (12 * i) + 2));
    }
  }

  for (i = 1; (int32_T)(i + 1) < 4; i++) {
    Vf[i] = e[i];
  }

  apply_transform = false;
  snorm = cjekaaieekfkbimo_xnrm2(11, b_A, 14);
  if (snorm > 0.0F) {
    apply_transform = true;
    if (b_A[13] < 0.0F) {
      s[1] = -snorm;
    } else {
      s[1] = snorm;
    }

    if ((real32_T)fabs((real_T)s[1]) >= 9.86076132E-32F) {
      snorm = 1.0F / s[1];
      for (qs = 13; (int32_T)(qs + 1) < 25; qs++) {
        b_A[qs] *= snorm;
      }
    } else {
      for (qs = 13; (int32_T)(qs + 1) < 25; qs++) {
        b_A[qs] /= s[1];
      }
    }

    b_A[13]++;
    s[1] = -s[1];
  } else {
    s[1] = 0.0F;
  }

  for (q = 2; (int32_T)(q + 1) < 4; q++) {
    qs = (int32_T)((int32_T)(12 * q) + 1);
    if (apply_transform) {
      hlfkiecjohlfopph_xaxpy(11, -(jekncbaiknglnoph_xdotc(11, b_A, 14, b_A,
        (int32_T)(qs + 1)) / b_A[13]), 14, b_A, (int32_T)(qs + 1));
    }

    e[q] = b_A[qs];
  }

  for (i = 1; (int32_T)(i + 1) < 13; i++) {
    U[(int32_T)(i + 12)] = b_A[(int32_T)(i + 12)];
  }

  snorm = cjekaaieekfkbimo_xnrm2(10, b_A, 27);
  if (snorm > 0.0F) {
    if (b_A[26] < 0.0F) {
      s[2] = -snorm;
    } else {
      s[2] = snorm;
    }

    if ((real32_T)fabs((real_T)s[2]) >= 9.86076132E-32F) {
      snorm = 1.0F / s[2];
      for (qs = 26; (int32_T)(qs + 1) < 37; qs++) {
        b_A[qs] *= snorm;
      }
    } else {
      for (qs = 26; (int32_T)(qs + 1) < 37; qs++) {
        b_A[qs] /= s[2];
      }
    }

    b_A[26]++;
    s[2] = -s[2];
  } else {
    s[2] = 0.0F;
  }

  for (i = 2; (int32_T)(i + 1) < 13; i++) {
    U[(int32_T)(i + 24)] = b_A[(int32_T)(i + 24)];
  }

  m = 1;
  for (q = 2; q >= 0; q += -1) {
    i = (int32_T)((int32_T)(12 * q) + q);
    if (s[q] != 0.0F) {
      for (kase = (int32_T)(q + 1); (int32_T)(kase + 1) < 4; kase++) {
        qs = (int32_T)((int32_T)((int32_T)(12 * kase) + q) + 1);
        hlfkiecjohlfopph_xaxpy((int32_T)(12 - q), -(jekncbaiknglnoph_xdotc
          ((int32_T)(12 - q), U, (int32_T)(i + 1), U, qs) / U[i]), (int32_T)(i +
          1), U, qs);
      }

      for (qs = q; (int32_T)(qs + 1) < 13; qs++) {
        U[(int32_T)(qs + (int32_T)(12 * q))] = -U[(int32_T)((int32_T)(12 * q) +
          qs)];
      }

      U[i]++;
      for (i = 1; i <= q; i++) {
        U[(int32_T)((int32_T)(i + (int32_T)(12 * q)) - 1)] = 0.0F;
      }
    } else {
      for (qs = 0; qs < 12; qs++) {
        U[(int32_T)(qs + (int32_T)(12 * q))] = 0.0F;
      }

      U[i] = 1.0F;
    }
  }

  for (i = 2; i >= 0; i += -1) {
    if (((int32_T)(i + 1) <= 1) && (e[0] != 0.0F)) {
      for (q = 2; q < 4; q++) {
        qs = (int32_T)((int32_T)((int32_T)(q - 1) * 3) + 2);
        ekfklfkfohdjaimg_xaxpy(2, -(mglnohlnekfcekfk_xdotc(2, Vf, 2, Vf, qs) /
          Vf[1]), 2, Vf, qs);
      }
    }

    Vf[(int32_T)(3 * i)] = 0.0F;
    Vf[(int32_T)(1 + (int32_T)(3 * i))] = 0.0F;
    Vf[(int32_T)(2 + (int32_T)(3 * i))] = 0.0F;
    Vf[(int32_T)(i + (int32_T)(3 * i))] = 1.0F;
  }

  ztest = e[0];
  if (s[0] != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)s[0]);
    snorm = s[0] / ztest0;
    s[0] = ztest0;
    ztest = e[0] / snorm;
    cjmgkfkfphdjaaai_xscal(snorm, U, 1);
  }

  if (ztest != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)ztest);
    snorm = ztest0 / ztest;
    ztest = ztest0;
    s[1] *= snorm;
    ophlaaaiophdbaai_xscal(snorm, Vf, 4);
  }

  e[0] = ztest;
  ztest = b_A[25];
  if (s[1] != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)s[1]);
    snorm = s[1] / ztest0;
    s[1] = ztest0;
    ztest = b_A[25] / snorm;
    cjmgkfkfphdjaaai_xscal(snorm, U, 13);
  }

  if (ztest != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)ztest);
    snorm = ztest0 / ztest;
    ztest = ztest0;
    s[2] *= snorm;
    ophlaaaiophdbaai_xscal(snorm, Vf, 7);
  }

  e[1] = ztest;
  if (s[2] != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)s[2]);
    snorm = s[2] / ztest0;
    s[2] = ztest0;
    cjmgkfkfphdjaaai_xscal(snorm, U, 25);
  }

  e[2] = 0.0F;
  i = 0;
  snorm = 0.0F;
  if ((s[0] >= e[0]) || rtIsNaNF(e[0])) {
    ztest0 = s[0];
  } else {
    ztest0 = e[0];
  }

  if (!((0.0F >= ztest0) || rtIsNaNF(ztest0))) {
    snorm = ztest0;
  }

  if ((s[1] >= ztest) || rtIsNaNF(ztest)) {
    ztest = s[1];
  }

  if (!((snorm >= ztest) || rtIsNaNF(ztest))) {
    snorm = ztest;
  }

  if (s[2] >= 0.0F) {
    ztest0 = s[2];
  } else {
    ztest0 = 0.0F;
  }

  if (!((snorm >= ztest0) || rtIsNaNF(ztest0))) {
    snorm = ztest0;
  }

  while (((int32_T)(m + 2) > 0) && (!(i >= 75))) {
    kase = (int32_T)(m + 1);
    do {
      exitg3 = 0;
      q = kase;
      if (kase == 0) {
        exitg3 = 1;
      } else {
        ztest0 = (real32_T)fabs((real_T)e[(int32_T)(kase - 1)]);
        if ((ztest0 <= ((real32_T)fabs((real_T)s[(int32_T)(kase - 1)]) +
                        (real32_T)fabs((real_T)s[kase])) * 1.1920929E-7F) ||
            ((ztest0 <= 9.86076132E-32F) || ((i > 20) && (ztest0 <=
               1.1920929E-7F * snorm)))) {
          e[(int32_T)(kase - 1)] = 0.0F;
          exitg3 = 1;
        } else {
          kase--;
        }
      }
    } while (exitg3 == 0);

    if ((int32_T)(m + 1) == kase) {
      kase = 4;
    } else {
      qs = (int32_T)(m + 2);
      j_ii = (int32_T)(m + 2);
      exitg2 = false;
      while ((!exitg2) && (j_ii >= kase)) {
        qs = j_ii;
        if (j_ii == kase) {
          exitg2 = true;
        } else {
          ztest0 = 0.0F;
          if (j_ii < (int32_T)(m + 2)) {
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
      } else if ((int32_T)(m + 2) == qs) {
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
      for (qs = m; (int32_T)(qs + 1) >= (int32_T)(q + 1); qs--) {
        ztest = s[qs];
        hdbinophmgdjjmoh_xrotg(&ztest, &ztest0, &sqds, &smm1);
        s[qs] = ztest;
        if ((int32_T)(qs + 1) > (int32_T)(q + 1)) {
          ztest0 = -smm1 * e[0];
          e[0] *= sqds;
        }

        gdjmmgdjekfccjmg_xrot(Vf, (int32_T)(1 + (int32_T)(3 * qs)), (int32_T)(1
          + (int32_T)(3 * (int32_T)(m + 1))), sqds, smm1);
      }
      break;

     case 2:
      ztest0 = e[(int32_T)(q - 1)];
      e[(int32_T)(q - 1)] = 0.0F;
      for (qs = q; (int32_T)(qs + 1) <= (int32_T)(m + 2); qs++) {
        ztest = s[qs];
        hdbinophmgdjjmoh_xrotg(&ztest, &ztest0, &sqds, &smm1);
        s[qs] = ztest;
        ztest0 = -smm1 * e[qs];
        e[qs] *= sqds;
        ophdimgdcjekekfc_xrot(U, (int32_T)(1 + (int32_T)(12 * qs)), (int32_T)(1
          + (int32_T)(12 * (int32_T)(q - 1))), sqds, smm1);
      }
      break;

     case 3:
      varargin_1[0] = (real32_T)fabs((real_T)s[(int32_T)(m + 1)]);
      varargin_1[1] = (real32_T)fabs((real_T)s[m]);
      varargin_1[2] = (real32_T)fabs((real_T)e[m]);
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

      ztest0 = s[(int32_T)(m + 1)] / ztest;
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
      for (qs = (int32_T)(q + 1); qs <= (int32_T)(m + 1); qs++) {
        hdbinophmgdjjmoh_xrotg(&ztest0, &ztest, &sqds, &smm1);
        if (qs > (int32_T)(q + 1)) {
          e[0] = ztest0;
        }

        ztest0 = s[(int32_T)(qs - 1)] * sqds + e[(int32_T)(qs - 1)] * smm1;
        e[(int32_T)(qs - 1)] = e[(int32_T)(qs - 1)] * sqds - s[(int32_T)(qs - 1)]
          * smm1;
        ztest = smm1 * s[qs];
        s[qs] *= sqds;
        gdjmmgdjekfccjmg_xrot(Vf, (int32_T)(1 + (int32_T)(3 * (int32_T)(qs - 1))),
                              (int32_T)(1 + (int32_T)(3 * qs)), sqds, smm1);
        hdbinophmgdjjmoh_xrotg(&ztest0, &ztest, &sqds, &smm1);
        s[(int32_T)(qs - 1)] = ztest0;
        ztest0 = e[(int32_T)(qs - 1)] * sqds + smm1 * s[qs];
        s[qs] = e[(int32_T)(qs - 1)] * -smm1 + sqds * s[qs];
        ztest = smm1 * e[qs];
        e[qs] *= sqds;
        ophdimgdcjekekfc_xrot(U, (int32_T)(1 + (int32_T)(12 * (int32_T)(qs - 1))),
                              (int32_T)(1 + (int32_T)(12 * qs)), sqds, smm1);
      }

      e[m] = ztest0;
      i++;
      break;

     default:
      if (s[q] < 0.0F) {
        s[q] = -s[q];
        ophlaaaiophdbaai_xscal(-1.0F, Vf, (int32_T)(1 + (int32_T)(3 * q)));
      }

      i = (int32_T)(q + 1);
      while (((int32_T)(q + 1) < 3) && (s[q] < s[i])) {
        ztest0 = s[q];
        s[q] = s[i];
        s[i] = ztest0;
        djmgbimooppphlno_xswap(Vf, (int32_T)(1 + (int32_T)(3 * q)), (int32_T)(1
          + (int32_T)(3 * (int32_T)(q + 1))));
        eknofcjenophiecj_xswap(U, (int32_T)(1 + (int32_T)(12 * q)), (int32_T)(1
          + (int32_T)(12 * (int32_T)(q + 1))));
        q = i;
        i++;
      }

      i = 0;
      m--;
      break;
    }
  }

  for (m = 0; m < 3; m++) {
    e[m] = s[m];
    V[(int32_T)(3 * m)] = Vf[(int32_T)(3 * m)];
    V[(int32_T)(1 + (int32_T)(3 * m))] = Vf[(int32_T)((int32_T)(3 * m) + 1)];
    V[(int32_T)(2 + (int32_T)(3 * m))] = Vf[(int32_T)((int32_T)(3 * m) + 2)];
  }

  for (i = 0; i < 9; i++) {
    S[i] = 0.0F;
  }

  S[0] = e[0];
  S[4] = e[1];
  S[8] = e[2];
}

//
// File trailer for generated code.
//
// [EOF]
//
