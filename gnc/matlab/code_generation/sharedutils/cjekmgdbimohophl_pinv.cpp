//
// File: cjekmgdbimohophl_pinv.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include "djecdbimhdjmdjmg_svd.h"
#include "jecjjmglfkfcaimo_svd.h"
#include "cjekmgdbimohophl_pinv.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void cjekmgdbimohophl_pinv(const real32_T A_data[], const int32_T A_sizes[2],
  real32_T X_data[], int32_T X_sizes[2])
{
  int32_T m;
  int32_T r;
  int32_T vcol;
  real32_T tol;
  int32_T br;
  int32_T ar;
  int32_T ia;
  int32_T c;
  int32_T ib;
  int32_T b_ic;
  int32_T d_ic;
  int32_T loop_ub;
  real32_T V_data[9];
  int32_T V_sizes[2];
  real32_T U_data[96];
  int32_T U_sizes[2];
  real32_T S_data[9];
  int32_T S_sizes[2];
  real32_T x_data[9];
  real32_T b_X_data[6];
  real32_T b_V_data[4];
  int32_T b_V_sizes[2];
  real32_T b_U_data[6];
  real32_T b_S_data[4];
  real32_T b_x_data[96];
  int32_T b_X_sizes_idx_0;
  int8_T b_idx_1;
  if (A_sizes[0] < 3) {
    loop_ub = A_sizes[0];
    for (ar = 0; ar < loop_ub; ar++) {
      U_data[3 * ar] = A_data[ar];
      U_data[1 + 3 * ar] = A_data[ar + A_sizes[0]];
      U_data[2 + 3 * ar] = A_data[(A_sizes[0] << 1) + ar];
    }

    m = A_sizes[0];
    b_idx_1 = (int8_T)A_sizes[0];
    b_X_sizes_idx_0 = b_idx_1;
    loop_ub = b_idx_1 * 3;
    for (ar = 0; ar < loop_ub; ar++) {
      b_X_data[ar] = 0.0F;
    }

    if (A_sizes[0] != 0) {
      U_sizes[0] = 3;
      U_sizes[1] = A_sizes[0];
      jecjjmglfkfcaimo_svd(U_data, U_sizes, b_U_data, S_sizes, b_S_data, V_sizes,
                           b_V_data, b_V_sizes);
      tol = 3.0F * b_S_data[0] * 1.1920929E-7F;
      r = 0;
      vcol = 0;
      while ((vcol + 1 <= m) && (b_S_data[V_sizes[0] * vcol + vcol] > tol)) {
        r++;
        vcol++;
      }

      if (r > 0) {
        vcol = 0;
        for (br = 0; br + 1 <= r; br++) {
          tol = 1.0F / b_S_data[V_sizes[0] * br + br];
          loop_ub = b_V_sizes[0] * b_V_sizes[1];
          for (ar = 0; ar < loop_ub; ar++) {
            b_x_data[ar] = b_V_data[ar];
          }

          loop_ub = vcol + m;
          for (ar = vcol; ar + 1 <= loop_ub; ar++) {
            b_x_data[ar] *= tol;
          }

          loop_ub = b_V_sizes[0] * b_V_sizes[1];
          for (ar = 0; ar < loop_ub; ar++) {
            b_V_data[ar] = b_x_data[ar];
          }

          vcol += m;
        }

        vcol = A_sizes[0] << 1;
        br = 0;
        while ((m > 0) && (br <= vcol)) {
          loop_ub = br + m;
          for (ar = br; ar + 1 <= loop_ub; ar++) {
            b_X_data[ar] = 0.0F;
          }

          br += m;
        }

        br = -1;
        loop_ub = 0;
        while ((m > 0) && (loop_ub <= vcol)) {
          ar = 0;
          br++;
          c = ((r - 1) * 3 + br) + 1;
          for (ib = br; ib + 1 <= c; ib += 3) {
            if (b_U_data[ib] != 0.0F) {
              ia = ar;
              b_ic = loop_ub + m;
              for (d_ic = loop_ub; d_ic + 1 <= b_ic; d_ic++) {
                ia++;
                b_X_data[d_ic] += b_V_data[ia - 1] * b_U_data[ib];
              }
            }

            ar += m;
          }

          loop_ub += m;
        }
      }
    }

    X_sizes[0] = 3;
    X_sizes[1] = b_idx_1;
    for (ar = 0; ar < b_X_sizes_idx_0; ar++) {
      X_data[3 * ar] = b_X_data[ar];
      X_data[1 + 3 * ar] = b_X_data[ar + b_idx_1];
      X_data[2 + 3 * ar] = b_X_data[(b_idx_1 << 1) + ar];
    }
  } else {
    m = A_sizes[0];
    b_idx_1 = (int8_T)A_sizes[0];
    X_sizes[0] = 3;
    X_sizes[1] = b_idx_1;
    loop_ub = 3 * b_idx_1;
    for (ar = 0; ar < loop_ub; ar++) {
      X_data[ar] = 0.0F;
    }

    djecdbimhdjmdjmg_svd(A_data, A_sizes, U_data, U_sizes, S_data, S_sizes,
                         V_data, V_sizes);
    tol = (real32_T)A_sizes[0] * S_data[0] * 1.1920929E-7F;
    r = 0;
    vcol = 0;
    while ((vcol + 1 < 4) && (S_data[S_sizes[0] * vcol + vcol] > tol)) {
      r++;
      vcol++;
    }

    if (r > 0) {
      vcol = 0;
      for (br = 0; br + 1 <= r; br++) {
        tol = 1.0F / S_data[S_sizes[0] * br + br];
        loop_ub = V_sizes[0] * V_sizes[1];
        for (ar = 0; ar < loop_ub; ar++) {
          x_data[ar] = V_data[ar];
        }

        for (loop_ub = vcol; loop_ub + 1 <= vcol + 3; loop_ub++) {
          x_data[loop_ub] *= tol;
        }

        V_sizes[0] = 3;
        V_sizes[1] = 3;
        for (ar = 0; ar < 9; ar++) {
          V_data[ar] = x_data[ar];
        }

        vcol += 3;
      }

      vcol = (A_sizes[0] - 1) * 3;
      for (br = 0; br <= vcol; br += 3) {
        for (loop_ub = br + 1; loop_ub <= br + 3; loop_ub++) {
          X_data[loop_ub - 1] = 0.0F;
        }
      }

      br = -1;
      for (loop_ub = 0; loop_ub <= vcol; loop_ub += 3) {
        ar = 0;
        br++;
        c = ((r - 1) * m + br) + 1;
        for (ib = br; ib + 1 <= c; ib += m) {
          if (U_data[ib] != 0.0F) {
            ia = ar;
            for (b_ic = loop_ub; b_ic + 1 <= loop_ub + 3; b_ic++) {
              ia++;
              X_data[b_ic] += V_data[ia - 1] * U_data[ib];
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
