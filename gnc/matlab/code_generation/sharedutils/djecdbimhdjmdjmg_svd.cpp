//
// File: djecdbimhdjmdjmg_svd.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include <math.h>
#include "baaicbiepphdbiek_xaxpy.h"
#include "cjmggdbimohlknop_xnrm2.h"
#include "djmgbimooppphlno_xswap.h"
#include "ekfklfkfohdjaimg_xaxpy.h"
#include "fcjebiekeknggdba_xdotc.h"
#include "gdjmmgdjekfccjmg_xrot.h"
#include "hdbinophmgdjjmoh_xrotg.h"
#include "kfkfopppmophfcjm_xaxpy.h"
#include "mglnohlnekfcekfk_xdotc.h"
#include "nohdgdbadbiejmop_xnrm2.h"
#include "ohdbaiekngdjbaaa_xdotc.h"
#include "ohdjhlfcnohdhdba_xscal.h"
#include "ophdjekfieknglfk_xswap.h"
#include "ophlaaaiophdbaai_xscal.h"
#include "opppnohdmglfjekf_xaxpy.h"
#include "pphdglfkecjmjekf_xrot.h"
#include "ppppjmohbaaigdje_xaxpy.h"
#include "rt_nonfinite.h"
#include "djecdbimhdjmdjmg_svd.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void djecdbimhdjmdjmg_svd(const real32_T A_data[], const int32_T A_sizes[2],
  real32_T U_data[], int32_T U_sizes[2], real32_T S_data[], int32_T S_sizes[2],
  real32_T V_data[], int32_T V_sizes[2])
{
  int32_T n;
  real32_T e[3];
  real32_T Vf[9];
  int32_T nct;
  int32_T q;
  boolean_T apply_transform;
  int32_T qjj;
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
  real32_T varargin_1[5];
  int32_T i;
  int32_T loop_ub;
  real32_T b_A_data[96];
  int32_T b_A_sizes[2];
  real32_T s_data[3];
  real32_T work_data[32];
  int32_T work_sizes;
  real32_T u_data[96];
  int32_T u_sizes[2];
  int8_T d_idx_0;
  b_A_sizes[0] = A_sizes[0];
  b_A_sizes[1] = 3;
  loop_ub = A_sizes[0] * A_sizes[1];
  for (i = 0; i < loop_ub; i++) {
    b_A_data[i] = A_data[i];
  }

  n = A_sizes[0];
  s_data[0] = 0.0F;
  e[0] = 0.0F;
  s_data[1] = 0.0F;
  e[1] = 0.0F;
  s_data[2] = 0.0F;
  e[2] = 0.0F;
  d_idx_0 = (int8_T)A_sizes[0];
  work_sizes = d_idx_0;
  loop_ub = d_idx_0;
  for (i = 0; i < loop_ub; i++) {
    work_data[i] = 0.0F;
  }

  d_idx_0 = (int8_T)A_sizes[0];
  U_sizes[0] = d_idx_0;
  U_sizes[1] = 3;
  loop_ub = d_idx_0 * 3;
  for (i = 0; i < loop_ub; i++) {
    U_data[i] = 0.0F;
  }

  for (i = 0; i < 9; i++) {
    Vf[i] = 0.0F;
  }

  if (A_sizes[0] < 1) {
    nct = 0;
  } else {
    nct = A_sizes[0] - 1;
  }

  if (!(nct <= 3)) {
    nct = 3;
  }

  if (nct >= 1) {
    m = nct;
  } else {
    m = 1;
  }

  for (kase = 0; kase + 1 <= m; kase++) {
    qs = n * kase + kase;
    q = n - kase;
    apply_transform = false;
    if (kase + 1 <= nct) {
      snorm = nohdgdbadbiejmop_xnrm2(q, b_A_data, qs + 1);
      if (snorm > 0.0F) {
        apply_transform = true;
        if (b_A_data[qs] < 0.0F) {
          s_data[kase] = -snorm;
        } else {
          s_data[kase] = snorm;
        }

        if ((real32_T)fabs((real_T)s_data[kase]) >= 9.86076132E-32F) {
          snorm = 1.0F / s_data[kase];
          qjj = qs + q;
          for (loop_ub = qs; loop_ub + 1 <= qjj; loop_ub++) {
            b_A_data[loop_ub] *= snorm;
          }
        } else {
          qjj = qs + q;
          for (loop_ub = qs; loop_ub + 1 <= qjj; loop_ub++) {
            b_A_data[loop_ub] /= s_data[kase];
          }
        }

        b_A_data[qs]++;
        s_data[kase] = -s_data[kase];
      } else {
        s_data[kase] = 0.0F;
      }
    }

    for (loop_ub = kase + 1; loop_ub + 1 < 4; loop_ub++) {
      qjj = n * loop_ub + kase;
      if (apply_transform) {
        ppppjmohbaaigdje_xaxpy(q, -(ohdbaiekngdjbaaa_xdotc(q, b_A_data, qs + 1,
          b_A_data, qjj + 1) / b_A_data[kase + b_A_sizes[0] * kase]), qs + 1,
          b_A_data, qjj + 1);
      }

      e[loop_ub] = b_A_data[qjj];
    }

    if (kase + 1 <= nct) {
      for (qs = kase; qs + 1 <= n; qs++) {
        U_data[qs + d_idx_0 * kase] = b_A_data[b_A_sizes[0] * kase + qs];
      }
    }

    if (kase + 1 <= 1) {
      snorm = cjmggdbimohlknop_xnrm2(e, 2);
      if (snorm == 0.0F) {
        e[0] = 0.0F;
      } else {
        if (e[1] < 0.0F) {
          snorm = -snorm;
        }

        e[0] = snorm;
        if ((real32_T)fabs((real_T)snorm) >= 9.86076132E-32F) {
          snorm = 1.0F / snorm;
          for (qs = 1; qs + 1 < 4; qs++) {
            e[qs] *= snorm;
          }
        } else {
          for (qs = 1; qs + 1 < 4; qs++) {
            e[qs] /= snorm;
          }
        }

        e[1]++;
        e[0] = -e[0];
        for (qs = 2; qs <= n; qs++) {
          work_data[qs - 1] = 0.0F;
        }

        for (qs = 1; qs + 1 < 4; qs++) {
          baaicbiepphdbiek_xaxpy(q - 1, e[qs], b_A_data, n * qs + 2, work_data,
            2);
        }

        for (qs = 1; qs + 1 < 4; qs++) {
          kfkfopppmophfcjm_xaxpy(q - 1, -e[qs] / e[1], work_data, 2, b_A_data, n
            * qs + 2);
        }
      }

      for (q = 1; q + 1 < 4; q++) {
        Vf[q] = e[q];
      }
    }
  }

  m = 1;
  if (nct < 3) {
    s_data[nct] = b_A_data[b_A_sizes[0] * nct + nct];
  }

  e[1] = b_A_data[(b_A_sizes[0] << 1) + 1];
  e[2] = 0.0F;
  if (nct + 1 <= 3) {
    for (q = nct; q + 1 < 4; q++) {
      for (kase = 1; kase <= n; kase++) {
        U_data[(kase + d_idx_0 * q) - 1] = 0.0F;
      }

      U_data[q + d_idx_0 * q] = 1.0F;
    }
  }

  for (nct--; nct + 1 > 0; nct--) {
    q = n - nct;
    qs = n * nct + nct;
    if (s_data[nct] != 0.0F) {
      for (kase = nct + 1; kase + 1 < 4; kase++) {
        qjj = (n * kase + nct) + 1;
        u_sizes[0] = U_sizes[0];
        u_sizes[1] = 3;
        loop_ub = U_sizes[0] * U_sizes[1];
        for (i = 0; i < loop_ub; i++) {
          u_data[i] = U_data[i];
        }

        opppnohdmglfjekf_xaxpy(q, -(fcjebiekeknggdba_xdotc(q, U_data, qs + 1,
          U_data, qjj) / U_data[qs]), qs + 1, u_data, qjj);
        U_sizes[0] = u_sizes[0];
        U_sizes[1] = u_sizes[1];
        loop_ub = u_sizes[0] * u_sizes[1];
        for (i = 0; i < loop_ub; i++) {
          U_data[i] = u_data[i];
        }
      }

      for (q = nct; q + 1 <= n; q++) {
        U_data[q + U_sizes[0] * nct] = -U_data[U_sizes[0] * nct + q];
      }

      U_data[qs]++;
      for (q = 1; q <= nct; q++) {
        U_data[(q + U_sizes[0] * nct) - 1] = 0.0F;
      }
    } else {
      for (q = 1; q <= n; q++) {
        U_data[(q + U_sizes[0] * nct) - 1] = 0.0F;
      }

      U_data[qs] = 1.0F;
    }
  }

  for (nct = 2; nct >= 0; nct += -1) {
    if ((nct + 1 <= 1) && (e[0] != 0.0F)) {
      ekfklfkfohdjaimg_xaxpy(2, -(mglnohlnekfcekfk_xdotc(2, Vf, 2, Vf, 5) / Vf[1]),
        2, Vf, 5);
      ekfklfkfohdjaimg_xaxpy(2, -(mglnohlnekfcekfk_xdotc(2, Vf, 2, Vf, 8) / Vf[1]),
        2, Vf, 8);
    }

    Vf[3 * nct] = 0.0F;
    Vf[1 + 3 * nct] = 0.0F;
    Vf[2 + 3 * nct] = 0.0F;
    Vf[nct + 3 * nct] = 1.0F;
  }

  for (nct = 0; nct < 3; nct++) {
    if (s_data[nct] != 0.0F) {
      ztest0 = (real32_T)fabs((real_T)s_data[nct]);
      snorm = s_data[nct] / ztest0;
      s_data[nct] = ztest0;
      if (nct + 1 < 3) {
        e[nct] /= snorm;
      }

      u_sizes[0] = U_sizes[0];
      u_sizes[1] = 3;
      loop_ub = U_sizes[0] * U_sizes[1];
      for (i = 0; i < loop_ub; i++) {
        u_data[i] = U_data[i];
      }

      ohdjhlfcnohdhdba_xscal(n, snorm, u_data, 1 + n * nct);
      U_sizes[0] = u_sizes[0];
      U_sizes[1] = u_sizes[1];
      loop_ub = u_sizes[0] * u_sizes[1];
      for (i = 0; i < loop_ub; i++) {
        U_data[i] = u_data[i];
      }
    }

    if ((nct + 1 < 3) && (e[nct] != 0.0F)) {
      ztest0 = (real32_T)fabs((real_T)e[nct]);
      snorm = ztest0 / e[nct];
      e[nct] = ztest0;
      s_data[nct + 1] *= snorm;
      ophlaaaiophdbaai_xscal(snorm, Vf, 1 + 3 * (nct + 1));
    }
  }

  nct = 0;
  snorm = 0.0F;
  ztest0 = (real32_T)fabs((real_T)s_data[0]);
  ztest = (real32_T)fabs((real_T)e[0]);
  if ((ztest0 >= ztest) || rtIsNaNF(ztest)) {
    ztest = ztest0;
  }

  if (!((0.0F >= ztest) || rtIsNaNF(ztest))) {
    snorm = ztest;
  }

  ztest0 = (real32_T)fabs((real_T)s_data[1]);
  ztest = (real32_T)fabs((real_T)e[1]);
  if ((ztest0 >= ztest) || rtIsNaNF(ztest)) {
    ztest = ztest0;
  }

  if (!((snorm >= ztest) || rtIsNaNF(ztest))) {
    snorm = ztest;
  }

  ztest0 = (real32_T)fabs((real_T)s_data[2]);
  ztest = (real32_T)fabs((real_T)e[2]);
  if ((ztest0 >= ztest) || rtIsNaNF(ztest)) {
    ztest = ztest0;
  }

  if (!((snorm >= ztest) || rtIsNaNF(ztest))) {
    snorm = ztest;
  }

  while ((m + 2 > 0) && (!(nct >= 75))) {
    kase = m + 1;
    do {
      qs = 0;
      q = kase;
      if (kase == 0) {
        qs = 1;
      } else {
        ztest0 = (real32_T)fabs((real_T)e[kase - 1]);
        if ((ztest0 <= ((real32_T)fabs((real_T)s_data[kase - 1]) + (real32_T)
                        fabs((real_T)s_data[kase])) * 1.1920929E-7F) || (ztest0 <=
             9.86076132E-32F) || ((nct > 20) && (ztest0 <= 1.1920929E-7F * snorm)))
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
      qjj = m + 2;
      apply_transform = false;
      while ((!apply_transform) && (qjj >= kase)) {
        qs = qjj;
        if (qjj == kase) {
          apply_transform = true;
        } else {
          ztest0 = 0.0F;
          if (qjj < m + 2) {
            ztest0 = (real32_T)fabs((real_T)e[qjj - 1]);
          }

          if (qjj > kase + 1) {
            ztest0 += (real32_T)fabs((real_T)e[qjj - 2]);
          }

          ztest = (real32_T)fabs((real_T)s_data[qjj - 1]);
          if ((ztest <= 1.1920929E-7F * ztest0) || (ztest <= 9.86076132E-32F)) {
            s_data[qjj - 1] = 0.0F;
            apply_transform = true;
          } else {
            qjj--;
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
        ztest = s_data[kase];
        hdbinophmgdjjmoh_xrotg(&ztest, &ztest0, &sqds, &smm1);
        s_data[kase] = ztest;
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
        ztest = s_data[kase];
        hdbinophmgdjjmoh_xrotg(&ztest, &ztest0, &sqds, &smm1);
        s_data[kase] = ztest;
        ztest0 = -smm1 * e[kase];
        e[kase] *= sqds;
        u_sizes[0] = U_sizes[0];
        u_sizes[1] = 3;
        loop_ub = U_sizes[0] * U_sizes[1];
        for (i = 0; i < loop_ub; i++) {
          u_data[i] = U_data[i];
        }

        pphdglfkecjmjekf_xrot(n, u_data, 1 + n * kase, 1 + n * (q - 1), sqds,
                              smm1);
        U_sizes[0] = u_sizes[0];
        U_sizes[1] = u_sizes[1];
        loop_ub = u_sizes[0] * u_sizes[1];
        for (i = 0; i < loop_ub; i++) {
          U_data[i] = u_data[i];
        }
      }
      break;

     case 3:
      varargin_1[0] = (real32_T)fabs((real_T)s_data[m + 1]);
      varargin_1[1] = (real32_T)fabs((real_T)s_data[m]);
      varargin_1[2] = (real32_T)fabs((real_T)e[m]);
      varargin_1[3] = (real32_T)fabs((real_T)s_data[q]);
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

      ztest0 = s_data[m + 1] / ztest;
      smm1 = s_data[m] / ztest;
      emm1 = e[m] / ztest;
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
      ztest = e[q] / ztest * sqds;
      for (kase = q + 1; kase <= m + 1; kase++) {
        hdbinophmgdjjmoh_xrotg(&ztest0, &ztest, &sqds, &smm1);
        if (kase > q + 1) {
          e[0] = ztest0;
        }

        ztest0 = s_data[kase - 1] * sqds + e[kase - 1] * smm1;
        e[kase - 1] = e[kase - 1] * sqds - s_data[kase - 1] * smm1;
        ztest = smm1 * s_data[kase];
        s_data[kase] *= sqds;
        gdjmmgdjekfccjmg_xrot(Vf, 1 + 3 * (kase - 1), 1 + 3 * kase, sqds, smm1);
        hdbinophmgdjjmoh_xrotg(&ztest0, &ztest, &sqds, &smm1);
        s_data[kase - 1] = ztest0;
        ztest0 = e[kase - 1] * sqds + smm1 * s_data[kase];
        s_data[kase] = e[kase - 1] * -smm1 + sqds * s_data[kase];
        ztest = smm1 * e[kase];
        e[kase] *= sqds;
        u_sizes[0] = U_sizes[0];
        u_sizes[1] = 3;
        loop_ub = U_sizes[0] * U_sizes[1];
        for (i = 0; i < loop_ub; i++) {
          u_data[i] = U_data[i];
        }

        pphdglfkecjmjekf_xrot(n, u_data, 1 + n * (kase - 1), 1 + n * kase, sqds,
                              smm1);
        U_sizes[0] = u_sizes[0];
        U_sizes[1] = u_sizes[1];
        loop_ub = u_sizes[0] * u_sizes[1];
        for (i = 0; i < loop_ub; i++) {
          U_data[i] = u_data[i];
        }
      }

      e[m] = ztest0;
      nct++;
      break;

     default:
      if (s_data[q] < 0.0F) {
        s_data[q] = -s_data[q];
        ophlaaaiophdbaai_xscal(-1.0F, Vf, 1 + 3 * q);
      }

      nct = q + 1;
      while ((q + 1 < 3) && (s_data[q] < s_data[nct])) {
        ztest0 = s_data[q];
        s_data[q] = s_data[nct];
        s_data[nct] = ztest0;
        djmgbimooppphlno_xswap(Vf, 1 + 3 * q, 1 + 3 * (q + 1));
        u_sizes[0] = U_sizes[0];
        u_sizes[1] = 3;
        loop_ub = U_sizes[0] * U_sizes[1];
        for (i = 0; i < loop_ub; i++) {
          u_data[i] = U_data[i];
        }

        ophdjekfieknglfk_xswap(n, u_data, 1 + n * q, 1 + n * (q + 1));
        U_sizes[0] = u_sizes[0];
        U_sizes[1] = u_sizes[1];
        loop_ub = u_sizes[0] * u_sizes[1];
        for (i = 0; i < loop_ub; i++) {
          U_data[i] = u_data[i];
        }

        q = nct;
        nct++;
      }

      nct = 0;
      m--;
      break;
    }
  }

  V_sizes[0] = 3;
  V_sizes[1] = 3;
  for (n = 0; n < 3; n++) {
    V_data[3 * n] = Vf[3 * n];
    V_data[1 + 3 * n] = Vf[3 * n + 1];
    V_data[2 + 3 * n] = Vf[3 * n + 2];
  }

  S_sizes[0] = 3;
  S_sizes[1] = 3;
  for (i = 0; i < 9; i++) {
    S_data[i] = 0.0F;
  }

  S_data[0] = s_data[0];
  S_data[4] = s_data[1];
  S_data[8] = s_data[2];
}

//
// File trailer for generated code.
//
// [EOF]
//
