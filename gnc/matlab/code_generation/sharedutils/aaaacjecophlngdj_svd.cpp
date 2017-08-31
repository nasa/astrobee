//
// File: aaaacjecophlngdj_svd.cpp
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:39 2017
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
#include "aaaacjecophlngdj_svd.h"

// Function for MATLAB Function: '<S9>/MATLAB Function'
void aaaacjecophlngdj_svd(const real32_T A[36], real32_T U[36], real32_T S[9],
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
  memcpy(&b_A[0], &A[0], 36U * sizeof(real32_T));
  e[0] = 0.0F;
  e[1] = 0.0F;
  e[2] = 0.0F;
  for (i = 0; i < 12; i++) {
    work[i] = 0.0F;
  }

  memset(&U[0], 0, 36U * sizeof(real32_T));
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
      for (kase = 0; kase + 1 < 13; kase++) {
        b_A[kase] *= snorm;
      }
    } else {
      for (kase = 0; kase + 1 < 13; kase++) {
        b_A[kase] /= s[0];
      }
    }

    b_A[0]++;
    s[0] = -s[0];
  } else {
    s[0] = 0.0F;
  }

  for (q = 1; q + 1 < 4; q++) {
    kase = 12 * q;
    if (apply_transform) {
      hlfkiecjohlfopph_xaxpy(12, -(jekncbaiknglnoph_xdotc(12, b_A, 1, b_A, kase
        + 1) / b_A[0]), 1, b_A, kase + 1);
    }

    e[q] = b_A[kase];
  }

  while (i + 1 < 13) {
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
      for (i = 1; i + 1 < 4; i++) {
        e[i] *= snorm;
      }
    } else {
      for (i = 1; i + 1 < 4; i++) {
        e[i] /= snorm;
      }
    }

    e[1]++;
    e[0] = -e[0];
    for (i = 2; i < 13; i++) {
      work[i - 1] = 0.0F;
    }

    for (i = 1; i + 1 < 4; i++) {
      opppmglfglfkdbaa_xaxpy(11, e[i], b_A, 12 * i + 2, work, 2);
    }

    for (i = 1; i + 1 < 4; i++) {
      cbielfcbnophecbi_xaxpy(11, -e[i] / e[1], work, 2, b_A, 12 * i + 2);
    }
  }

  for (i = 1; i + 1 < 4; i++) {
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
      for (kase = 13; kase + 1 < 25; kase++) {
        b_A[kase] *= snorm;
      }
    } else {
      for (kase = 13; kase + 1 < 25; kase++) {
        b_A[kase] /= s[1];
      }
    }

    b_A[13]++;
    s[1] = -s[1];
  } else {
    s[1] = 0.0F;
  }

  for (q = 2; q + 1 < 4; q++) {
    kase = 12 * q + 1;
    if (apply_transform) {
      hlfkiecjohlfopph_xaxpy(11, -(jekncbaiknglnoph_xdotc(11, b_A, 14, b_A, kase
        + 1) / b_A[13]), 14, b_A, kase + 1);
    }

    e[q] = b_A[kase];
  }

  for (i = 1; i + 1 < 13; i++) {
    U[i + 12] = b_A[i + 12];
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
      for (kase = 26; kase + 1 < 37; kase++) {
        b_A[kase] *= snorm;
      }
    } else {
      for (kase = 26; kase + 1 < 37; kase++) {
        b_A[kase] /= s[2];
      }
    }

    b_A[26]++;
    s[2] = -s[2];
  } else {
    s[2] = 0.0F;
  }

  for (i = 2; i + 1 < 13; i++) {
    U[i + 24] = b_A[i + 24];
  }

  m = 1;
  for (q = 2; q >= 0; q += -1) {
    i = 12 * q + q;
    if (s[q] != 0.0F) {
      for (qs = q + 1; qs + 1 < 4; qs++) {
        kase = (12 * qs + q) + 1;
        hlfkiecjohlfopph_xaxpy(12 - q, -(jekncbaiknglnoph_xdotc(12 - q, U, i + 1,
          U, kase) / U[i]), i + 1, U, kase);
      }

      for (kase = q; kase + 1 < 13; kase++) {
        U[kase + 12 * q] = -U[12 * q + kase];
      }

      U[i]++;
      for (i = 1; i <= q; i++) {
        U[(i + 12 * q) - 1] = 0.0F;
      }
    } else {
      for (kase = 0; kase < 12; kase++) {
        U[kase + 12 * q] = 0.0F;
      }

      U[i] = 1.0F;
    }
  }

  for (i = 2; i >= 0; i += -1) {
    if ((i + 1 <= 1) && (e[0] != 0.0F)) {
      for (q = 2; q < 4; q++) {
        kase = (q - 1) * 3 + 2;
        ekfklfkfohdjaimg_xaxpy(2, -(mglnohlnekfcekfk_xdotc(2, Vf, 2, Vf, kase) /
          Vf[1]), 2, Vf, kase);
      }
    }

    Vf[3 * i] = 0.0F;
    Vf[1 + 3 * i] = 0.0F;
    Vf[2 + 3 * i] = 0.0F;
    Vf[i + 3 * i] = 1.0F;
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

  while ((m + 2 > 0) && (!(i >= 75))) {
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
             9.86076132E-32F) || ((i > 20) && (ztest0 <= 1.1920929E-7F * snorm)))
        {
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
      j_ii = m + 2;
      apply_transform = false;
      while ((!apply_transform) && (j_ii >= kase)) {
        qs = j_ii;
        if (j_ii == kase) {
          apply_transform = true;
        } else {
          ztest0 = 0.0F;
          if (j_ii < m + 2) {
            ztest0 = (real32_T)fabs((real_T)e[j_ii - 1]);
          }

          if (j_ii > kase + 1) {
            ztest0 += (real32_T)fabs((real_T)e[j_ii - 2]);
          }

          ztest = (real32_T)fabs((real_T)s[j_ii - 1]);
          if ((ztest <= 1.1920929E-7F * ztest0) || (ztest <= 9.86076132E-32F)) {
            s[j_ii - 1] = 0.0F;
            apply_transform = true;
          } else {
            j_ii--;
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
          ztest0 = -smm1 * e[0];
          e[0] *= sqds;
        }

        gdjmmgdjekfccjmg_xrot(Vf, 1 + 3 * kase, 1 + 3 * (m + 1), sqds, smm1);
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
        ophdimgdcjekekfc_xrot(U, 1 + 12 * kase, 1 + 12 * (q - 1), sqds, smm1);
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
          e[0] = ztest0;
        }

        ztest0 = s[kase - 1] * sqds + e[kase - 1] * smm1;
        e[kase - 1] = e[kase - 1] * sqds - s[kase - 1] * smm1;
        ztest = smm1 * s[kase];
        s[kase] *= sqds;
        gdjmmgdjekfccjmg_xrot(Vf, 1 + 3 * (kase - 1), 1 + 3 * kase, sqds, smm1);
        hdbinophmgdjjmoh_xrotg(&ztest0, &ztest, &sqds, &smm1);
        s[kase - 1] = ztest0;
        ztest0 = e[kase - 1] * sqds + smm1 * s[kase];
        s[kase] = e[kase - 1] * -smm1 + sqds * s[kase];
        ztest = smm1 * e[kase];
        e[kase] *= sqds;
        ophdimgdcjekekfc_xrot(U, 1 + 12 * (kase - 1), 1 + 12 * kase, sqds, smm1);
      }

      e[m] = ztest0;
      i++;
      break;

     default:
      if (s[q] < 0.0F) {
        s[q] = -s[q];
        ophlaaaiophdbaai_xscal(-1.0F, Vf, 1 + 3 * q);
      }

      i = q + 1;
      while ((q + 1 < 3) && (s[q] < s[i])) {
        ztest0 = s[q];
        s[q] = s[i];
        s[i] = ztest0;
        djmgbimooppphlno_xswap(Vf, 1 + 3 * q, 1 + 3 * (q + 1));
        eknofcjenophiecj_xswap(U, 1 + 12 * q, 1 + 12 * (q + 1));
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
    V[3 * m] = Vf[3 * m];
    V[1 + 3 * m] = Vf[3 * m + 1];
    V[2 + 3 * m] = Vf[3 * m + 2];
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
