//
// File: cjmolnohmglfbiec_svd.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:06:07 2018
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
#include "cjmolnohmglfbiec_svd.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void cjmolnohmglfbiec_svd(const real32_T A_data[], const int32_T A_sizes[2],
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
  int32_T g_k;
  real32_T b_A_data[96];
  int32_T b_A_sizes[2];
  real32_T s_data[3];
  real32_T work_data[32];
  int32_T work_sizes;
  int8_T d_idx_0;
  boolean_T exitg1;
  boolean_T exitg2;
  int32_T exitg3;
  b_A_sizes[0] = A_sizes[0];
  b_A_sizes[1] = 3;
  nct = (int32_T)(A_sizes[0] * A_sizes[1]);
  for (m = 0; m <= (int32_T)(nct - 1); m++) {
    b_A_data[m] = A_data[m];
  }

  n = A_sizes[0];
  s_data[0] = 0.0F;
  e[0] = 0.0F;
  s_data[1] = 0.0F;
  e[1] = 0.0F;
  s_data[2] = 0.0F;
  e[2] = 0.0F;
  d_idx_0 = (int8_T)A_sizes[0];
  work_sizes = (int32_T)d_idx_0;
  nct = (int32_T)d_idx_0;
  for (m = 0; m <= (int32_T)(nct - 1); m++) {
    work_data[m] = 0.0F;
  }

  d_idx_0 = (int8_T)A_sizes[0];
  U_sizes[0] = (int32_T)d_idx_0;
  U_sizes[1] = 3;
  nct = (int32_T)((int32_T)d_idx_0 * 3);
  for (m = 0; m <= (int32_T)(nct - 1); m++) {
    U_data[m] = 0.0F;
  }

  for (m = 0; m < 9; m++) {
    Vf[m] = 0.0F;
  }

  if (A_sizes[0] > 1) {
    nct = (int32_T)(A_sizes[0] - 1);
  } else {
    nct = 0;
  }

  if (!(nct <= 3)) {
    nct = 3;
  }

  if (nct >= 1) {
    m = nct;
  } else {
    m = 1;
  }

  for (qs = 0; (int32_T)(qs + 1) <= m; qs++) {
    kase = (int32_T)((int32_T)(n * qs) + qs);
    q = (int32_T)(n - qs);
    apply_transform = false;
    if ((int32_T)(qs + 1) <= nct) {
      snorm = nohdgdbadbiejmop_xnrm2(q, b_A_data, (int32_T)(kase + 1));
      if (snorm > 0.0F) {
        apply_transform = true;
        if (b_A_data[kase] < 0.0F) {
          s_data[qs] = -snorm;
        } else {
          s_data[qs] = snorm;
        }

        if ((real32_T)fabs((real_T)s_data[qs]) >= 9.86076132E-32F) {
          snorm = 1.0F / s_data[qs];
          qjj = (int32_T)(kase + q);
          for (g_k = kase; (int32_T)(g_k + 1) <= qjj; g_k++) {
            b_A_data[g_k] *= snorm;
          }
        } else {
          qjj = (int32_T)(kase + q);
          for (g_k = kase; (int32_T)(g_k + 1) <= qjj; g_k++) {
            b_A_data[g_k] /= s_data[qs];
          }
        }

        b_A_data[kase]++;
        s_data[qs] = -s_data[qs];
      } else {
        s_data[qs] = 0.0F;
      }
    }

    for (g_k = (int32_T)(qs + 1); (int32_T)(g_k + 1) < 4; g_k++) {
      qjj = (int32_T)((int32_T)(n * g_k) + qs);
      if (apply_transform) {
        ppppjmohbaaigdje_xaxpy(q, -(ohdbaiekngdjbaaa_xdotc(q, b_A_data, (int32_T)
          (kase + 1), b_A_data, (int32_T)(qjj + 1)) / b_A_data[(int32_T)(qs +
          (int32_T)(b_A_sizes[0] * qs))]), (int32_T)(kase + 1), b_A_data,
          (int32_T)(qjj + 1));
      }

      e[g_k] = b_A_data[qjj];
    }

    if ((int32_T)(qs + 1) <= nct) {
      for (kase = qs; (int32_T)(kase + 1) <= n; kase++) {
        U_data[(int32_T)(kase + (int32_T)((int32_T)d_idx_0 * qs))] = b_A_data
          [(int32_T)((int32_T)(b_A_sizes[0] * qs) + kase)];
      }
    }

    if ((int32_T)(qs + 1) <= 1) {
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
          for (kase = 1; (int32_T)(kase + 1) < 4; kase++) {
            e[kase] *= snorm;
          }
        } else {
          for (kase = 1; (int32_T)(kase + 1) < 4; kase++) {
            e[kase] /= snorm;
          }
        }

        e[1]++;
        e[0] = -e[0];
        for (kase = 2; kase <= n; kase++) {
          work_data[(int32_T)(kase - 1)] = 0.0F;
        }

        for (kase = 1; (int32_T)(kase + 1) < 4; kase++) {
          baaicbiepphdbiek_xaxpy((int32_T)(q - 1), e[kase], b_A_data, (int32_T)
            ((int32_T)(n * kase) + 2), work_data, 2);
        }

        for (kase = 1; (int32_T)(kase + 1) < 4; kase++) {
          kfkfopppmophfcjm_xaxpy((int32_T)(q - 1), -e[kase] / e[1], work_data, 2,
            b_A_data, (int32_T)((int32_T)(n * kase) + 2));
        }
      }

      for (q = 1; (int32_T)(q + 1) < 4; q++) {
        Vf[q] = e[q];
      }
    }
  }

  m = 1;
  if (nct < 3) {
    s_data[nct] = b_A_data[(int32_T)((int32_T)(b_A_sizes[0] * nct) + nct)];
  }

  if ((int32_T)(nct + 1) <= 3) {
    for (q = nct; (int32_T)(q + 1) < 4; q++) {
      for (qs = 1; qs <= n; qs++) {
        U_data[(int32_T)((int32_T)(qs + (int32_T)((int32_T)d_idx_0 * q)) - 1)] =
          0.0F;
      }

      U_data[(int32_T)(q + (int32_T)((int32_T)d_idx_0 * q))] = 1.0F;
    }
  }

  for (nct--; (int32_T)(nct + 1) > 0; nct--) {
    q = (int32_T)(n - nct);
    kase = (int32_T)((int32_T)(n * nct) + nct);
    if (s_data[nct] != 0.0F) {
      for (qs = (int32_T)(nct + 1); (int32_T)(qs + 1) < 4; qs++) {
        qjj = (int32_T)((int32_T)((int32_T)(n * qs) + nct) + 1);
        opppnohdmglfjekf_xaxpy(q, -(fcjebiekeknggdba_xdotc(q, U_data, (int32_T)
          (kase + 1), U_data, qjj) / U_data[kase]), (int32_T)(kase + 1), U_data,
          qjj);
      }

      for (q = nct; (int32_T)(q + 1) <= n; q++) {
        U_data[(int32_T)(q + (int32_T)(U_sizes[0] * nct))] = -U_data[(int32_T)
          ((int32_T)(U_sizes[0] * nct) + q)];
      }

      U_data[kase]++;
      for (q = 1; q <= nct; q++) {
        U_data[(int32_T)((int32_T)(q + (int32_T)(U_sizes[0] * nct)) - 1)] = 0.0F;
      }
    } else {
      for (q = 1; q <= n; q++) {
        U_data[(int32_T)((int32_T)(q + (int32_T)(U_sizes[0] * nct)) - 1)] = 0.0F;
      }

      U_data[kase] = 1.0F;
    }
  }

  for (nct = 2; nct >= 0; nct += -1) {
    if (((int32_T)(nct + 1) <= 1) && (e[0] != 0.0F)) {
      ekfklfkfohdjaimg_xaxpy(2, -(mglnohlnekfcekfk_xdotc(2, Vf, 2, Vf, 5) / Vf[1]),
        2, Vf, 5);
      ekfklfkfohdjaimg_xaxpy(2, -(mglnohlnekfcekfk_xdotc(2, Vf, 2, Vf, 8) / Vf[1]),
        2, Vf, 8);
    }

    Vf[(int32_T)(3 * nct)] = 0.0F;
    Vf[(int32_T)(1 + (int32_T)(3 * nct))] = 0.0F;
    Vf[(int32_T)(2 + (int32_T)(3 * nct))] = 0.0F;
    Vf[(int32_T)(nct + (int32_T)(3 * nct))] = 1.0F;
  }

  ztest = e[0];
  if (s_data[0] != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)s_data[0]);
    snorm = s_data[0] / ztest0;
    s_data[0] = ztest0;
    ztest = e[0] / snorm;
    ohdjhlfcnohdhdba_xscal(A_sizes[0], snorm, U_data, 1);
  }

  if (ztest != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)ztest);
    snorm = ztest0 / ztest;
    ztest = ztest0;
    s_data[1] *= snorm;
    ophlaaaiophdbaai_xscal(snorm, Vf, 4);
  }

  e[0] = ztest;
  ztest = b_A_data[(int32_T)((int32_T)(b_A_sizes[0] << 1) + 1)];
  if (s_data[1] != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)s_data[1]);
    snorm = s_data[1] / ztest0;
    s_data[1] = ztest0;
    ztest = b_A_data[(int32_T)((int32_T)(b_A_sizes[0] << 1) + 1)] / snorm;
    ohdjhlfcnohdhdba_xscal(A_sizes[0], snorm, U_data, (int32_T)(1 + A_sizes[0]));
  }

  if (ztest != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)ztest);
    snorm = ztest0 / ztest;
    ztest = ztest0;
    s_data[2] *= snorm;
    ophlaaaiophdbaai_xscal(snorm, Vf, 7);
  }

  e[1] = ztest;
  if (s_data[2] != 0.0F) {
    ztest0 = (real32_T)fabs((real_T)s_data[2]);
    snorm = s_data[2] / ztest0;
    s_data[2] = ztest0;
    ohdjhlfcnohdhdba_xscal(A_sizes[0], snorm, U_data, (int32_T)(1 + (int32_T)
      (A_sizes[0] << 1)));
  }

  e[2] = 0.0F;
  nct = 0;
  snorm = 0.0F;
  if ((s_data[0] >= e[0]) || rtIsNaNF(e[0])) {
    ztest0 = s_data[0];
  } else {
    ztest0 = e[0];
  }

  if (!((0.0F >= ztest0) || rtIsNaNF(ztest0))) {
    snorm = ztest0;
  }

  if ((s_data[1] >= ztest) || rtIsNaNF(ztest)) {
    ztest = s_data[1];
  }

  if (!((snorm >= ztest) || rtIsNaNF(ztest))) {
    snorm = ztest;
  }

  if (s_data[2] >= 0.0F) {
    ztest0 = s_data[2];
  } else {
    ztest0 = 0.0F;
  }

  if (!((snorm >= ztest0) || rtIsNaNF(ztest0))) {
    snorm = ztest0;
  }

  while (((int32_T)(m + 2) > 0) && (!(nct >= 75))) {
    kase = (int32_T)(m + 1);
    do {
      exitg3 = 0;
      q = kase;
      if (kase == 0) {
        exitg3 = 1;
      } else {
        ztest0 = (real32_T)fabs((real_T)e[(int32_T)(kase - 1)]);
        if ((ztest0 <= ((real32_T)fabs((real_T)s_data[(int32_T)(kase - 1)]) +
                        (real32_T)fabs((real_T)s_data[kase])) * 1.1920929E-7F) ||
            ((ztest0 <= 9.86076132E-32F) || ((nct > 20) && (ztest0 <=
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
      qjj = (int32_T)(m + 2);
      exitg2 = false;
      while ((!exitg2) && (qjj >= kase)) {
        qs = qjj;
        if (qjj == kase) {
          exitg2 = true;
        } else {
          ztest0 = 0.0F;
          if (qjj < (int32_T)(m + 2)) {
            ztest0 = (real32_T)fabs((real_T)e[(int32_T)(qjj - 1)]);
          }

          if (qjj > (int32_T)(kase + 1)) {
            ztest0 += (real32_T)fabs((real_T)e[(int32_T)(qjj - 2)]);
          }

          ztest = (real32_T)fabs((real_T)s_data[(int32_T)(qjj - 1)]);
          if ((ztest <= 1.1920929E-7F * ztest0) || (ztest <= 9.86076132E-32F)) {
            s_data[(int32_T)(qjj - 1)] = 0.0F;
            exitg2 = true;
          } else {
            qjj--;
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
        ztest = s_data[qs];
        hdbinophmgdjjmoh_xrotg(&ztest, &ztest0, &sqds, &smm1);
        s_data[qs] = ztest;
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
        ztest = s_data[qs];
        hdbinophmgdjjmoh_xrotg(&ztest, &ztest0, &sqds, &smm1);
        s_data[qs] = ztest;
        ztest0 = -smm1 * e[qs];
        e[qs] *= sqds;
        pphdglfkecjmjekf_xrot(n, U_data, (int32_T)(1 + (int32_T)(n * qs)),
                              (int32_T)(1 + (int32_T)(n * (int32_T)(q - 1))),
                              sqds, smm1);
      }
      break;

     case 3:
      varargin_1[0] = (real32_T)fabs((real_T)s_data[(int32_T)(m + 1)]);
      varargin_1[1] = (real32_T)fabs((real_T)s_data[m]);
      varargin_1[2] = (real32_T)fabs((real_T)e[m]);
      varargin_1[3] = (real32_T)fabs((real_T)s_data[q]);
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

      ztest0 = s_data[(int32_T)(m + 1)] / ztest;
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
      for (qs = (int32_T)(q + 1); qs <= (int32_T)(m + 1); qs++) {
        hdbinophmgdjjmoh_xrotg(&ztest0, &ztest, &sqds, &smm1);
        if (qs > (int32_T)(q + 1)) {
          e[0] = ztest0;
        }

        ztest0 = s_data[(int32_T)(qs - 1)] * sqds + e[(int32_T)(qs - 1)] * smm1;
        e[(int32_T)(qs - 1)] = e[(int32_T)(qs - 1)] * sqds - s_data[(int32_T)(qs
          - 1)] * smm1;
        ztest = smm1 * s_data[qs];
        s_data[qs] *= sqds;
        gdjmmgdjekfccjmg_xrot(Vf, (int32_T)(1 + (int32_T)(3 * (int32_T)(qs - 1))),
                              (int32_T)(1 + (int32_T)(3 * qs)), sqds, smm1);
        hdbinophmgdjjmoh_xrotg(&ztest0, &ztest, &sqds, &smm1);
        s_data[(int32_T)(qs - 1)] = ztest0;
        ztest0 = e[(int32_T)(qs - 1)] * sqds + smm1 * s_data[qs];
        s_data[qs] = e[(int32_T)(qs - 1)] * -smm1 + sqds * s_data[qs];
        ztest = smm1 * e[qs];
        e[qs] *= sqds;
        pphdglfkecjmjekf_xrot(n, U_data, (int32_T)(1 + (int32_T)(n * (int32_T)
          (qs - 1))), (int32_T)(1 + (int32_T)(n * qs)), sqds, smm1);
      }

      e[m] = ztest0;
      nct++;
      break;

     default:
      if (s_data[q] < 0.0F) {
        s_data[q] = -s_data[q];
        ophlaaaiophdbaai_xscal(-1.0F, Vf, (int32_T)(1 + (int32_T)(3 * q)));
      }

      nct = (int32_T)(q + 1);
      while (((int32_T)(q + 1) < 3) && (s_data[q] < s_data[nct])) {
        ztest0 = s_data[q];
        s_data[q] = s_data[nct];
        s_data[nct] = ztest0;
        djmgbimooppphlno_xswap(Vf, (int32_T)(1 + (int32_T)(3 * q)), (int32_T)(1
          + (int32_T)(3 * (int32_T)(q + 1))));
        ophdjekfieknglfk_xswap(n, U_data, (int32_T)(1 + (int32_T)(n * q)),
          (int32_T)(1 + (int32_T)(n * (int32_T)(q + 1))));
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
    V_data[(int32_T)(3 * n)] = Vf[(int32_T)(3 * n)];
    V_data[(int32_T)(1 + (int32_T)(3 * n))] = Vf[(int32_T)((int32_T)(3 * n) + 1)];
    V_data[(int32_T)(2 + (int32_T)(3 * n))] = Vf[(int32_T)((int32_T)(3 * n) + 2)];
  }

  S_sizes[0] = 3;
  S_sizes[1] = 3;
  for (m = 0; m < 9; m++) {
    S_data[m] = 0.0F;
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
