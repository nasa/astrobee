//
// File: glfcngdjgdjmmglf_pinv.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Dec  4 08:33:06 2017
//
#include "rtwtypes.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "cjmolnohmglfbiec_svd.h"
#include "lfcjmoppknohbimo_svd.h"
#include "glfcngdjgdjmmglf_pinv.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void glfcngdjgdjmmglf_pinv(const real32_T A_data[], const int32_T A_sizes[2],
  real32_T X_data[], int32_T X_sizes[2])
{
  int32_T m;
  int32_T vcol;
  real32_T tol;
  int32_T n;
  boolean_T b_p;
  int32_T br;
  int32_T ar;
  int32_T ia;
  int32_T d;
  int32_T ib;
  int32_T b_ic;
  int32_T d_ic;
  int32_T loop_ub;
  real32_T V_data[9];
  int32_T V_sizes[2];
  real32_T U1_data[96];
  int32_T U1_sizes[2];
  real32_T V1_data[9];
  int32_T V1_sizes[2];
  real32_T S_data[9];
  int32_T S_sizes[2];
  real32_T x_data[9];
  real32_T b_X_data[6];
  real32_T b_V_data[4];
  real32_T b_U1_data[6];
  real32_T b_V1_data[4];
  real32_T c_S_data[4];
  real32_T tmp_data[96];
  int32_T tmp_sizes[2];
  real32_T tmp_data_0[6];
  int32_T b_X_sizes_idx_0;
  int8_T S1_idx_0;
  int8_T S1_idx_1;
  int8_T b_idx_0;
  int8_T b_idx_1;
  if (A_sizes[0] < 3) {
    loop_ub = A_sizes[0];
    for (ar = 0; ar <= (int32_T)(loop_ub - 1); ar++) {
      tmp_data[(int32_T)(3 * ar)] = A_data[ar];
      tmp_data[(int32_T)(1 + (int32_T)(3 * ar))] = A_data[(int32_T)(ar +
        A_sizes[0])];
      tmp_data[(int32_T)(2 + (int32_T)(3 * ar))] = A_data[(int32_T)((int32_T)
        (A_sizes[0] << 1) + ar)];
    }

    m = A_sizes[0];
    b_idx_0 = (int8_T)A_sizes[0];
    b_X_sizes_idx_0 = (int32_T)b_idx_0;
    loop_ub = (int32_T)((int32_T)b_idx_0 * 3);
    for (ar = 0; ar <= (int32_T)(loop_ub - 1); ar++) {
      b_X_data[ar] = 0.0F;
    }

    if (A_sizes[0] != 0) {
      b_p = true;
      n = (int32_T)(3 * A_sizes[0]);
      for (vcol = 0; vcol <= (int32_T)(n - 1); vcol++) {
        if (b_p && ((!rtIsInfF(tmp_data[vcol])) && (!rtIsNaNF(tmp_data[vcol]))))
        {
        } else {
          b_p = false;
        }
      }

      if (b_p) {
        tmp_sizes[0] = 3;
        tmp_sizes[1] = A_sizes[0];
        lfcjmoppknohbimo_svd(tmp_data, tmp_sizes, b_U1_data, U1_sizes, c_S_data,
                             V1_sizes, b_V_data, V_sizes);
      } else {
        b_idx_1 = (int8_T)A_sizes[0];
        tmp_sizes[0] = 3;
        tmp_sizes[1] = (int32_T)b_idx_1;
        loop_ub = (int32_T)(3 * (int32_T)b_idx_1);
        for (ar = 0; ar <= (int32_T)(loop_ub - 1); ar++) {
          tmp_data_0[ar] = 0.0F;
        }

        lfcjmoppknohbimo_svd(tmp_data_0, tmp_sizes, b_U1_data, U1_sizes,
                             b_V_data, V_sizes, b_V1_data, S_sizes);
        loop_ub = U1_sizes[1];
        for (ar = 0; ar <= (int32_T)(loop_ub - 1); ar++) {
          b_U1_data[(int32_T)(3 * ar)] = (rtNaNF);
          b_U1_data[(int32_T)(1 + (int32_T)(3 * ar))] = (rtNaNF);
          b_U1_data[(int32_T)(2 + (int32_T)(3 * ar))] = (rtNaNF);
        }

        S1_idx_0 = (int8_T)V_sizes[0];
        b_idx_0 = (int8_T)V_sizes[0];
        S1_idx_1 = (int8_T)V_sizes[1];
        b_idx_1 = (int8_T)V_sizes[1];
        loop_ub = (int32_T)((int32_T)S1_idx_0 * (int32_T)S1_idx_1);
        for (ar = 0; ar <= (int32_T)(loop_ub - 1); ar++) {
          V1_data[ar] = 0.0F;
        }

        if ((int32_T)b_idx_0 <= (int32_T)b_idx_1) {
          n = (int32_T)b_idx_0;
        } else {
          n = (int32_T)b_idx_1;
        }

        for (vcol = 0; (int32_T)(vcol + 1) <= n; vcol++) {
          V1_data[(int32_T)(vcol + (int32_T)((int32_T)S1_idx_0 * vcol))] =
            (rtNaNF);
        }

        V1_sizes[0] = (int32_T)S1_idx_0;
        loop_ub = (int32_T)((int32_T)S1_idx_0 * (int32_T)S1_idx_1);
        for (ar = 0; ar <= (int32_T)(loop_ub - 1); ar++) {
          c_S_data[ar] = V1_data[ar];
        }

        V_sizes[0] = S_sizes[0];
        V_sizes[1] = S_sizes[1];
        loop_ub = (int32_T)(S_sizes[0] * S_sizes[1]);
        for (ar = 0; ar <= (int32_T)(loop_ub - 1); ar++) {
          b_V_data[ar] = (rtNaNF);
        }
      }

      tol = 3.0F * c_S_data[0] * 1.1920929E-7F;
      n = 0;
      vcol = 0;
      while (((int32_T)(vcol + 1) <= m) && (c_S_data[(int32_T)((int32_T)
               (V1_sizes[0] * vcol) + vcol)] > tol)) {
        n++;
        vcol++;
      }

      if (n > 0) {
        vcol = 0;
        for (br = 0; (int32_T)(br + 1) <= n; br++) {
          tol = 1.0F / c_S_data[(int32_T)((int32_T)(V1_sizes[0] * br) + br)];
          loop_ub = (int32_T)(V_sizes[0] * V_sizes[1]);
          for (ar = 0; ar <= (int32_T)(loop_ub - 1); ar++) {
            U1_data[ar] = b_V_data[ar];
          }

          loop_ub = (int32_T)(vcol + m);
          for (ar = vcol; (int32_T)(ar + 1) <= loop_ub; ar++) {
            U1_data[ar] *= tol;
          }

          loop_ub = (int32_T)(V_sizes[0] * V_sizes[1]);
          for (ar = 0; ar <= (int32_T)(loop_ub - 1); ar++) {
            b_V_data[ar] = U1_data[ar];
          }

          vcol += m;
        }

        vcol = (int32_T)(A_sizes[0] << 1);
        br = 0;
        while ((m > 0) && (br <= vcol)) {
          loop_ub = (int32_T)(br + m);
          for (ar = br; (int32_T)(ar + 1) <= loop_ub; ar++) {
            b_X_data[ar] = 0.0F;
          }

          br += m;
        }

        br = -1;
        loop_ub = 0;
        while ((m > 0) && (loop_ub <= vcol)) {
          ar = 0;
          br++;
          d = (int32_T)((int32_T)((int32_T)((int32_T)(n - 1) * 3) + br) + 1);
          for (ib = br; (int32_T)(ib + 1) <= d; ib += 3) {
            if (b_U1_data[ib] != 0.0F) {
              ia = ar;
              b_ic = (int32_T)(loop_ub + m);
              for (d_ic = loop_ub; (int32_T)(d_ic + 1) <= b_ic; d_ic++) {
                ia++;
                b_X_data[d_ic] += b_V_data[(int32_T)(ia - 1)] * b_U1_data[ib];
              }
            }

            ar += m;
          }

          loop_ub += m;
        }
      }
    }

    X_sizes[0] = 3;
    X_sizes[1] = b_X_sizes_idx_0;
    for (ar = 0; ar <= (int32_T)(b_X_sizes_idx_0 - 1); ar++) {
      X_data[(int32_T)(3 * ar)] = b_X_data[ar];
      X_data[(int32_T)(1 + (int32_T)(3 * ar))] = b_X_data[(int32_T)(ar +
        b_X_sizes_idx_0)];
      X_data[(int32_T)(2 + (int32_T)(3 * ar))] = b_X_data[(int32_T)((int32_T)
        (b_X_sizes_idx_0 << 1) + ar)];
    }
  } else {
    m = A_sizes[0];
    b_idx_1 = (int8_T)A_sizes[0];
    X_sizes[0] = 3;
    X_sizes[1] = (int32_T)b_idx_1;
    loop_ub = (int32_T)(3 * (int32_T)b_idx_1);
    for (ar = 0; ar <= (int32_T)(loop_ub - 1); ar++) {
      X_data[ar] = 0.0F;
    }

    b_p = true;
    n = (int32_T)(A_sizes[0] * 3);
    for (vcol = 0; vcol <= (int32_T)(n - 1); vcol++) {
      if (b_p && ((!rtIsInfF(A_data[vcol])) && (!rtIsNaNF(A_data[vcol])))) {
      } else {
        b_p = false;
      }
    }

    if (b_p) {
      cjmolnohmglfbiec_svd(A_data, A_sizes, U1_data, U1_sizes, S_data, S_sizes,
                           V_data, V_sizes);
    } else {
      b_idx_0 = (int8_T)A_sizes[0];
      tmp_sizes[0] = (int32_T)b_idx_0;
      tmp_sizes[1] = 3;
      loop_ub = (int32_T)((int32_T)b_idx_0 * 3);
      for (ar = 0; ar <= (int32_T)(loop_ub - 1); ar++) {
        tmp_data[ar] = 0.0F;
      }

      cjmolnohmglfbiec_svd(tmp_data, tmp_sizes, U1_data, U1_sizes, V_data,
                           V_sizes, V1_data, V1_sizes);
      loop_ub = U1_sizes[1];
      for (ar = 0; ar <= (int32_T)(loop_ub - 1); ar++) {
        n = U1_sizes[0];
        for (vcol = 0; vcol <= (int32_T)(n - 1); vcol++) {
          U1_data[(int32_T)(vcol + (int32_T)(U1_sizes[0] * ar))] = (rtNaNF);
        }
      }

      for (ar = 0; ar < 9; ar++) {
        V1_data[ar] = 0.0F;
      }

      for (n = 0; (int32_T)(n + 1) < 4; n++) {
        V1_data[(int32_T)(n + (int32_T)(3 * n))] = (rtNaNF);
      }

      S_sizes[0] = 3;
      V_sizes[0] = 3;
      V_sizes[1] = 3;
      for (ar = 0; ar < 9; ar++) {
        S_data[ar] = V1_data[ar];
        V_data[ar] = (rtNaNF);
      }
    }

    tol = (real32_T)A_sizes[0] * S_data[0] * 1.1920929E-7F;
    n = 0;
    vcol = 0;
    while (((int32_T)(vcol + 1) < 4) && (S_data[(int32_T)((int32_T)(S_sizes[0] *
              vcol) + vcol)] > tol)) {
      n++;
      vcol++;
    }

    if (n > 0) {
      vcol = 0;
      for (br = 0; (int32_T)(br + 1) <= n; br++) {
        tol = 1.0F / S_data[(int32_T)((int32_T)(S_sizes[0] * br) + br)];
        loop_ub = (int32_T)(V_sizes[0] * V_sizes[1]);
        for (ar = 0; ar <= (int32_T)(loop_ub - 1); ar++) {
          x_data[ar] = V_data[ar];
        }

        for (loop_ub = vcol; (int32_T)(loop_ub + 1) <= (int32_T)(vcol + 3);
             loop_ub++) {
          x_data[loop_ub] *= tol;
        }

        V_sizes[0] = 3;
        V_sizes[1] = 3;
        for (ar = 0; ar < 9; ar++) {
          V_data[ar] = x_data[ar];
        }

        vcol += 3;
      }

      vcol = (int32_T)((int32_T)(A_sizes[0] - 1) * 3);
      for (br = 0; br <= vcol; br += 3) {
        for (loop_ub = (int32_T)(br + 1); loop_ub <= (int32_T)(br + 3); loop_ub
             ++) {
          X_data[(int32_T)(loop_ub - 1)] = 0.0F;
        }
      }

      br = -1;
      for (loop_ub = 0; loop_ub <= vcol; loop_ub += 3) {
        ar = 0;
        br++;
        d = (int32_T)((int32_T)((int32_T)((int32_T)(n - 1) * m) + br) + 1);
        for (ib = br; (int32_T)(ib + 1) <= d; ib += m) {
          if (U1_data[ib] != 0.0F) {
            ia = ar;
            for (b_ic = loop_ub; (int32_T)(b_ic + 1) <= (int32_T)(loop_ub + 3);
                 b_ic++) {
              ia++;
              X_data[b_ic] += V_data[(int32_T)(ia - 1)] * U1_data[ib];
            }
          }

          ar += 3;
        }
      }
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
