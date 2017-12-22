//
// File: aaiepphlecjelnoh_permute.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Dec  4 08:33:06 2017
//
#include "rtwtypes.h"
#include "aaiepphlecjelnoh_permute.h"

// Function for MATLAB Function: '<S24>/compute_of_global_points'
void aaiepphlecjelnoh_permute(const real32_T a_data[], const int32_T a_sizes[3],
  real32_T b_data[], int32_T b_sizes[3])
{
  int8_T outsz[3];
  int32_T iwork[3];
  int32_T isrc;
  int32_T c_k;
  boolean_T c_b;
  int32_T plast;
  static const int8_T e[3] = { 3, 1, 2 };

  int8_T b_a_idx_0;
  int32_T inc_idx_0;
  int32_T exitg1;
  boolean_T exitg2;
  boolean_T guard1 = false;
  b_a_idx_0 = (int8_T)a_sizes[0];
  outsz[0] = 16;
  outsz[1] = b_a_idx_0;
  outsz[2] = 2;
  b_sizes[0] = 16;
  b_sizes[1] = (int32_T)b_a_idx_0;
  b_sizes[2] = 2;
  c_b = true;
  if (!(a_sizes[0] == 0)) {
    plast = 0;
    isrc = 0;
    exitg2 = false;
    while ((!exitg2) && ((int32_T)(isrc + 1) < 4)) {
      guard1 = false;
      if (a_sizes[(int32_T)((int32_T)e[isrc] - 1)] != 1) {
        if (plast > (int32_T)e[isrc]) {
          c_b = false;
          exitg2 = true;
        } else {
          plast = (int32_T)e[isrc];
          guard1 = true;
        }
      } else {
        guard1 = true;
      }

      if (guard1) {
        isrc++;
      }
    }
  }

  if (c_b) {
    plast = (int32_T)(a_sizes[0] << 5);
    for (isrc = 0; (int32_T)(isrc + 1) <= plast; isrc++) {
      b_data[isrc] = a_data[isrc];
    }
  } else {
    inc_idx_0 = (int32_T)((int32_T)b_a_idx_0 * (int32_T)(int8_T)a_sizes[1]);
    iwork[0] = 0;
    iwork[1] = 0;
    iwork[2] = 0;
    plast = 0;
    do {
      isrc = (int32_T)((int32_T)(iwork[2] * (int32_T)b_a_idx_0) + iwork[1]);
      for (c_k = 0; c_k < 16; c_k++) {
        b_data[plast] = a_data[isrc];
        plast++;
        isrc += inc_idx_0;
      }

      isrc = 1;
      do {
        exitg1 = 0;
        iwork[isrc]++;
        if (iwork[isrc] < (int32_T)outsz[isrc]) {
          exitg1 = 2;
        } else if ((int32_T)(isrc + 1) == 3) {
          exitg1 = 1;
        } else {
          iwork[1] = 0;
          isrc = 2;
        }
      } while (exitg1 == 0);
    } while (!(exitg1 == 1));
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
