//
// File: lfcjmoppknohbimo_svd.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Dec  4 08:33:06 2017
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
#include "lfcjmoppknohbimo_svd.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void lfcjmoppknohbimo_svd(const real32_T A_data[], const int32_T A_sizes[2],
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
  int32_T Vf_sizes_idx_0;
  int32_T Vf_sizes_idx_1;
  int8_T d_idx_0;
  int8_T d_idx_1;
  boolean_T exitg1;
  boolean_T exitg2;
  boolean_T exitg3;
  b_A_sizes[0] = 3;
  b_A_sizes[1] = A_sizes[1];
  qs = (int32_T)(A_sizes[0] * A_sizes[1]);
  for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
    b_A_data[kase] = A_data[kase];
  }

  p = A_sizes[1];
  minnp = A_sizes[1];
  qs = (int32_T)(int8_T)A_sizes[1];
  for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
    s_data[kase] = 0.0F;
  }

  qs = (int32_T)(int8_T)A_sizes[1];
  for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
    e_data[kase] = 0.0F;
  }

  d_idx_1 = (int8_T)A_sizes[1];
  U_sizes[0] = 3;
  U_sizes[1] = (int32_T)d_idx_1;
  qs = (int32_T)(3 * (int32_T)d_idx_1);
  for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
    U_data[kase] = 0.0F;
  }

  d_idx_0 = (int8_T)A_sizes[1];
  d_idx_1 = (int8_T)A_sizes[1];
  Vf_sizes_idx_0 = (int32_T)d_idx_0;
  Vf_sizes_idx_1 = (int32_T)d_idx_1;
  qs = (int32_T)((int32_T)d_idx_0 * (int32_T)d_idx_1);
  for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
    Vf_data[kase] = 0.0F;
  }

  if (A_sizes[1] == 0) {
    for (nct = 0; (int32_T)(nct + 1) <= minnp; nct++) {
      U_data[(int32_T)(nct + (int32_T)(3 * nct))] = 1.0F;
    }
  } else {
    nct = A_sizes[1];
    for (q = 0; (int32_T)(q + 1) <= nct; q++) {
      iter = (int32_T)((int32_T)(3 * q) + q);
      apply_transform = false;
      if ((int32_T)(q + 1) <= nct) {
        snorm = jmgljecbphlnngln_xnrm2((int32_T)(3 - q), b_A_data, (int32_T)
          (iter + 1));
        if (snorm > 0.0F) {
          apply_transform = true;
          if (b_A_data[iter] < 0.0F) {
            s_data[q] = -snorm;
          } else {
            s_data[q] = snorm;
          }

          if ((real32_T)fabs((real_T)s_data[q]) >= 9.86076132E-32F) {
            snorm = 1.0F / s_data[q];
            qs = (int32_T)((int32_T)(iter - q) + 3);
            for (kase = iter; (int32_T)(kase + 1) <= qs; kase++) {
              b_A_data[kase] *= snorm;
            }
          } else {
            qs = (int32_T)((int32_T)(iter - q) + 3);
            for (kase = iter; (int32_T)(kase + 1) <= qs; kase++) {
              b_A_data[kase] /= s_data[q];
            }
          }

          b_A_data[iter]++;
          s_data[q] = -s_data[q];
        } else {
          s_data[q] = 0.0F;
        }
      }

      qs = (int32_T)(q + 2);
      while (qs <= p) {
        if (apply_transform) {
          cjecbaaifkfkkngd_xaxpy((int32_T)(3 - q), -(imohaimgbaimjmgl_xdotc
            ((int32_T)(3 - q), b_A_data, (int32_T)(iter + 1), b_A_data, (int32_T)
             (q + 4)) / b_A_data[(int32_T)(q + (int32_T)(b_A_sizes[0] * q))]),
            (int32_T)(iter + 1), b_A_data, (int32_T)(q + 4));
        }

        e_data[1] = b_A_data[(int32_T)(q + 3)];
        qs = 3;
      }

      if ((int32_T)(q + 1) <= nct) {
        for (iter = q; (int32_T)(iter + 1) < 4; iter++) {
          U_data[(int32_T)(iter + (int32_T)(3 * q))] = b_A_data[(int32_T)
            ((int32_T)(b_A_sizes[0] * q) + iter)];
        }
      }
    }

    nct = A_sizes[1];
    if (1 < A_sizes[1]) {
      e_data[0] = b_A_data[b_A_sizes[0]];
    }

    e_data[(int32_T)(A_sizes[1] - 1)] = 0.0F;
    if ((int32_T)(A_sizes[1] + 1) <= A_sizes[1]) {
      iter = (int32_T)(A_sizes[1] + 1);
      while (iter <= 2) {
        U_data[3] = 0.0F;
        U_data[5] = 0.0F;
        iter = 3;
        U_data[4] = 1.0F;
      }
    }

    for (q = (int32_T)(A_sizes[1] - 1); (int32_T)(q + 1) > 0; q--) {
      iter = (int32_T)((int32_T)(3 * q) + q);
      if (s_data[q] != 0.0F) {
        qs = (int32_T)(q + 2);
        while (qs <= minnp) {
          cjecbaaifkfkkngd_xaxpy((int32_T)(3 - q), -(imohaimgbaimjmgl_xdotc
            ((int32_T)(3 - q), U_data, (int32_T)(iter + 1), U_data, (int32_T)(q
            + 4)) / U_data[iter]), (int32_T)(iter + 1), U_data, (int32_T)(q + 4));
          qs = 3;
        }

        for (qs = q; (int32_T)(qs + 1) < 4; qs++) {
          U_data[(int32_T)(qs + (int32_T)(U_sizes[0] * q))] = -U_data[(int32_T)
            ((int32_T)(U_sizes[0] * q) + qs)];
        }

        U_data[iter]++;
        iter = 1;
        while (iter <= q) {
          U_data[(int32_T)(U_sizes[0] * q)] = 0.0F;
          iter = 2;
        }
      } else {
        U_data[(int32_T)(U_sizes[0] * q)] = 0.0F;
        U_data[(int32_T)(1 + (int32_T)(U_sizes[0] * q))] = 0.0F;
        U_data[(int32_T)(2 + (int32_T)(U_sizes[0] * q))] = 0.0F;
        U_data[iter] = 1.0F;
      }
    }

    for (iter = (int32_T)(A_sizes[1] - 1); (int32_T)(iter + 1) > 0; iter--) {
      for (q = 1; q <= p; q++) {
        Vf_data[(int32_T)((int32_T)(q + (int32_T)((int32_T)d_idx_0 * iter)) - 1)]
          = 0.0F;
      }

      Vf_data[(int32_T)(iter + (int32_T)((int32_T)d_idx_0 * iter))] = 1.0F;
    }

    for (iter = 0; (int32_T)(iter + 1) <= nct; iter++) {
      if (s_data[iter] != 0.0F) {
        ztest0 = (real32_T)fabs((real_T)s_data[iter]);
        snorm = s_data[iter] / ztest0;
        s_data[iter] = ztest0;
        if ((int32_T)(iter + 1) < nct) {
          e_data[0] /= snorm;
        }

        n_sizes[0] = 3;
        n_sizes[1] = U_sizes[1];
        qs = (int32_T)(U_sizes[0] * U_sizes[1]);
        for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
          n_data[kase] = U_data[kase];
        }

        lfcjglnghlfknglf_xscal(snorm, n_data, (int32_T)(1 + (int32_T)(3 * iter)));
        U_sizes[0] = 3;
        U_sizes[1] = n_sizes[1];
        qs = (int32_T)(n_sizes[0] * n_sizes[1]);
        for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
          U_data[kase] = n_data[kase];
        }
      }

      if (((int32_T)(iter + 1) < nct) && (e_data[0] != 0.0F)) {
        ztest0 = (real32_T)fabs((real_T)e_data[0]);
        snorm = ztest0 / e_data[0];
        e_data[0] = ztest0;
        s_data[1] *= snorm;
        o_sizes[0] = Vf_sizes_idx_0;
        o_sizes[1] = Vf_sizes_idx_1;
        qs = (int32_T)(Vf_sizes_idx_0 * Vf_sizes_idx_1);
        for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
          o_data[kase] = Vf_data[kase];
        }

        ohdjhlfcnohdhdba_xscal(p, snorm, o_data, (int32_T)(1 + p));
        Vf_sizes_idx_0 = o_sizes[0];
        Vf_sizes_idx_1 = o_sizes[1];
        qs = (int32_T)(o_sizes[0] * o_sizes[1]);
        for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
          Vf_data[kase] = o_data[kase];
        }
      }
    }

    iter = 0;
    snorm = 0.0F;
    for (q = 0; (int32_T)(q + 1) <= nct; q++) {
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
      q = (int32_T)(nct - 1);
      exitg3 = false;
      while (!(exitg3 || (q == 0))) {
        ztest0 = (real32_T)fabs((real_T)e_data[0]);
        if ((ztest0 <= ((real32_T)fabs((real_T)s_data[0]) + (real32_T)fabs
                        ((real_T)s_data[1])) * 1.1920929E-7F) || ((ztest0 <=
              9.86076132E-32F) || ((iter > 20) && (ztest0 <= 1.1920929E-7F *
               snorm)))) {
          e_data[0] = 0.0F;
          exitg3 = true;
        } else {
          q = 0;
        }
      }

      if ((int32_T)(nct - 1) == q) {
        kase = 4;
      } else {
        qs = nct;
        kase = nct;
        exitg2 = false;
        while ((!exitg2) && (kase >= q)) {
          qs = kase;
          if (kase == q) {
            exitg2 = true;
          } else {
            ztest0 = 0.0F;
            if (kase < nct) {
              ztest0 = (real32_T)fabs((real_T)e_data[0]);
            }

            if (kase > (int32_T)(q + 1)) {
              ztest0 += (real32_T)fabs((real_T)e_data[0]);
            }

            ztest = (real32_T)fabs((real_T)s_data[(int32_T)(kase - 1)]);
            if ((ztest <= 1.1920929E-7F * ztest0) || (ztest <= 9.86076132E-32F))
            {
              s_data[(int32_T)(kase - 1)] = 0.0F;
              exitg2 = true;
            } else {
              kase--;
            }
          }
        }

        if (qs == q) {
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
        ztest0 = e_data[0];
        e_data[0] = 0.0F;
        qs = (int32_T)(nct - 1);
        while (qs >= (int32_T)(q + 1)) {
          ztest = s_data[0];
          hdbinophmgdjjmoh_xrotg(&ztest, &ztest0, &sqds, &smm1);
          s_data[0] = ztest;
          o_sizes[0] = Vf_sizes_idx_0;
          o_sizes[1] = Vf_sizes_idx_1;
          qs = (int32_T)(Vf_sizes_idx_0 * Vf_sizes_idx_1);
          for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
            o_data[kase] = Vf_data[kase];
          }

          pphdglfkecjmjekf_xrot(p, o_data, 1, (int32_T)(1 + (int32_T)(p *
            (int32_T)(nct - 1))), sqds, smm1);
          Vf_sizes_idx_0 = o_sizes[0];
          Vf_sizes_idx_1 = o_sizes[1];
          qs = (int32_T)(o_sizes[0] * o_sizes[1]);
          for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
            Vf_data[kase] = o_data[kase];
          }

          qs = 0;
        }
        break;

       case 2:
        ztest0 = e_data[(int32_T)(q - 1)];
        e_data[(int32_T)(q - 1)] = 0.0F;
        for (qs = q; (int32_T)(qs + 1) <= nct; qs++) {
          ztest = s_data[qs];
          hdbinophmgdjjmoh_xrotg(&ztest, &ztest0, &sqds, &smm1);
          s_data[qs] = ztest;
          ztest0 = -smm1 * e_data[qs];
          e_data[qs] *= sqds;
          jmohnoppbieciecj_xrot(U_data, (int32_T)(1 + (int32_T)(3 * qs)),
                                (int32_T)(1 + (int32_T)(3 * (int32_T)(q - 1))),
                                sqds, smm1);
        }
        break;

       case 3:
        varargin_1[0] = (real32_T)fabs((real_T)s_data[(int32_T)(nct - 1)]);
        varargin_1[1] = (real32_T)fabs((real_T)s_data[0]);
        varargin_1[2] = (real32_T)fabs((real_T)e_data[0]);
        varargin_1[3] = (real32_T)fabs((real_T)s_data[q]);
        varargin_1[4] = (real32_T)fabs((real_T)e_data[q]);
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

        ztest0 = s_data[(int32_T)(nct - 1)] / ztest;
        smm1 = s_data[0] / ztest;
        emm1 = e_data[0] / ztest;
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
        while ((int32_T)(q + 1) <= 1) {
          hdbinophmgdjjmoh_xrotg(&ztest0, &ztest, &sqds, &smm1);
          ztest0 = sqds * s_data[0] + smm1 * e_data[0];
          e_data[0] = sqds * e_data[0] - smm1 * s_data[0];
          ztest = smm1 * s_data[1];
          s_data[1] *= sqds;
          o_sizes[0] = Vf_sizes_idx_0;
          o_sizes[1] = Vf_sizes_idx_1;
          qs = (int32_T)(Vf_sizes_idx_0 * Vf_sizes_idx_1);
          for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
            o_data[kase] = Vf_data[kase];
          }

          pphdglfkecjmjekf_xrot(p, o_data, 1, (int32_T)(1 + p), sqds, smm1);
          Vf_sizes_idx_0 = o_sizes[0];
          Vf_sizes_idx_1 = o_sizes[1];
          qs = (int32_T)(o_sizes[0] * o_sizes[1]);
          for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
            Vf_data[kase] = o_data[kase];
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

        e_data[0] = ztest0;
        iter++;
        break;

       default:
        if (s_data[q] < 0.0F) {
          s_data[q] = -s_data[q];
          o_sizes[0] = Vf_sizes_idx_0;
          o_sizes[1] = Vf_sizes_idx_1;
          qs = (int32_T)(Vf_sizes_idx_0 * Vf_sizes_idx_1);
          for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
            o_data[kase] = Vf_data[kase];
          }

          ohdjhlfcnohdhdba_xscal(p, -1.0F, o_data, (int32_T)(1 + (int32_T)(p * q)));
          Vf_sizes_idx_0 = o_sizes[0];
          Vf_sizes_idx_1 = o_sizes[1];
          qs = (int32_T)(o_sizes[0] * o_sizes[1]);
          for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
            Vf_data[kase] = o_data[kase];
          }
        }

        while (((int32_T)(q + 1) < A_sizes[1]) && (s_data[0] < s_data[1])) {
          ztest0 = s_data[0];
          s_data[0] = s_data[1];
          s_data[1] = ztest0;
          if (1 < p) {
            o_sizes[0] = Vf_sizes_idx_0;
            o_sizes[1] = Vf_sizes_idx_1;
            qs = (int32_T)(Vf_sizes_idx_0 * Vf_sizes_idx_1);
            for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
              o_data[kase] = Vf_data[kase];
            }

            ophdjekfieknglfk_xswap(2, o_data, 1, 3);
            Vf_sizes_idx_0 = o_sizes[0];
            Vf_sizes_idx_1 = o_sizes[1];
            qs = (int32_T)(o_sizes[0] * o_sizes[1]);
            for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
              Vf_data[kase] = o_data[kase];
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
  for (nct = 0; (int32_T)(nct + 1) <= minnp; nct++) {
    e_data[nct] = s_data[nct];
  }

  d_idx_0 = (int8_T)A_sizes[1];
  V_sizes[0] = (int32_T)(int8_T)A_sizes[1];
  V_sizes[1] = (int32_T)(int8_T)A_sizes[1];
  for (nct = 0; (int32_T)(nct + 1) <= minnp; nct++) {
    for (iter = 0; (int32_T)(iter + 1) <= p; iter++) {
      V_data[(int32_T)(iter + (int32_T)((int32_T)d_idx_0 * nct))] = Vf_data
        [(int32_T)((int32_T)(Vf_sizes_idx_0 * nct) + iter)];
    }
  }

  S_sizes[0] = A_sizes[1];
  S_sizes[1] = A_sizes[1];
  qs = (int32_T)(A_sizes[1] * A_sizes[1]);
  for (kase = 0; kase <= (int32_T)(qs - 1); kase++) {
    S_data[kase] = 0.0F;
  }

  for (p = 0; p <= (int32_T)(Vf_sizes_idx_1 - 1); p++) {
    S_data[(int32_T)(p + (int32_T)(Vf_sizes_idx_1 * p))] = e_data[p];
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
