//
// File: hdjmknohknopimgl_sort.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:08:00 2018
//
#include "rtwtypes.h"
#include <string.h>
#include "phlnlfkfphdjohln_merge.h"
#include "rt_nonfinite.h"
#include "hdjmknohknopimgl_sort.h"

// Function for MATLAB Function: '<S83>/generate_output'
void hdjmknohknopimgl_sort(real32_T x[50], int32_T idx[50])
{
  int32_T nNaNs;
  real32_T xwork[50];
  int32_T iwork[50];
  real32_T x4[4];
  int8_T idx4[4];
  int32_T ib;
  int32_T m;
  int8_T perm[4];
  int32_T i3;
  int32_T i4;
  int32_T tailOffset;
  int32_T nTail;
  x4[0] = 0.0F;
  idx4[0] = 0;
  x4[1] = 0.0F;
  idx4[1] = 0;
  x4[2] = 0.0F;
  idx4[2] = 0;
  x4[3] = 0.0F;
  idx4[3] = 0;
  memset(&idx[0], 0, (uint32_T)(50U * sizeof(int32_T)));
  memset(&xwork[0], 0, (uint32_T)(50U * sizeof(real32_T)));
  nNaNs = 0;
  ib = 0;
  for (m = 0; m < 50; m++) {
    if (rtIsNaNF(x[m])) {
      idx[(int32_T)(49 - nNaNs)] = (int32_T)(m + 1);
      xwork[(int32_T)(49 - nNaNs)] = x[m];
      nNaNs++;
    } else {
      ib++;
      idx4[(int32_T)(ib - 1)] = (int8_T)(int32_T)(m + 1);
      x4[(int32_T)(ib - 1)] = x[m];
      if (ib == 4) {
        ib = (int32_T)(m - nNaNs);
        if (x4[0] <= x4[1]) {
          tailOffset = 1;
          nTail = 2;
        } else {
          tailOffset = 2;
          nTail = 1;
        }

        if (x4[2] <= x4[3]) {
          i3 = 3;
          i4 = 4;
        } else {
          i3 = 4;
          i4 = 3;
        }

        if (x4[(int32_T)(tailOffset - 1)] <= x4[(int32_T)(i3 - 1)]) {
          if (x4[(int32_T)(nTail - 1)] <= x4[(int32_T)(i3 - 1)]) {
            perm[0] = (int8_T)tailOffset;
            perm[1] = (int8_T)nTail;
            perm[2] = (int8_T)i3;
            perm[3] = (int8_T)i4;
          } else if (x4[(int32_T)(nTail - 1)] <= x4[(int32_T)(i4 - 1)]) {
            perm[0] = (int8_T)tailOffset;
            perm[1] = (int8_T)i3;
            perm[2] = (int8_T)nTail;
            perm[3] = (int8_T)i4;
          } else {
            perm[0] = (int8_T)tailOffset;
            perm[1] = (int8_T)i3;
            perm[2] = (int8_T)i4;
            perm[3] = (int8_T)nTail;
          }
        } else if (x4[(int32_T)(tailOffset - 1)] <= x4[(int32_T)(i4 - 1)]) {
          if (x4[(int32_T)(nTail - 1)] <= x4[(int32_T)(i4 - 1)]) {
            perm[0] = (int8_T)i3;
            perm[1] = (int8_T)tailOffset;
            perm[2] = (int8_T)nTail;
            perm[3] = (int8_T)i4;
          } else {
            perm[0] = (int8_T)i3;
            perm[1] = (int8_T)tailOffset;
            perm[2] = (int8_T)i4;
            perm[3] = (int8_T)nTail;
          }
        } else {
          perm[0] = (int8_T)i3;
          perm[1] = (int8_T)i4;
          perm[2] = (int8_T)tailOffset;
          perm[3] = (int8_T)nTail;
        }

        idx[(int32_T)(ib - 3)] = (int32_T)idx4[(int32_T)((int32_T)perm[0] - 1)];
        idx[(int32_T)(ib - 2)] = (int32_T)idx4[(int32_T)((int32_T)perm[1] - 1)];
        idx[(int32_T)(ib - 1)] = (int32_T)idx4[(int32_T)((int32_T)perm[2] - 1)];
        idx[ib] = (int32_T)idx4[(int32_T)((int32_T)perm[3] - 1)];
        x[(int32_T)(ib - 3)] = x4[(int32_T)((int32_T)perm[0] - 1)];
        x[(int32_T)(ib - 2)] = x4[(int32_T)((int32_T)perm[1] - 1)];
        x[(int32_T)(ib - 1)] = x4[(int32_T)((int32_T)perm[2] - 1)];
        x[ib] = x4[(int32_T)((int32_T)perm[3] - 1)];
        ib = 0;
      }
    }
  }

  if (ib > 0) {
    perm[1] = 0;
    perm[2] = 0;
    perm[3] = 0;
    if (ib == 1) {
      perm[0] = 1;
    } else if (ib == 2) {
      if (x4[0] <= x4[1]) {
        perm[0] = 1;
        perm[1] = 2;
      } else {
        perm[0] = 2;
        perm[1] = 1;
      }
    } else if (x4[0] <= x4[1]) {
      if (x4[1] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 2;
        perm[2] = 3;
      } else if (x4[0] <= x4[2]) {
        perm[0] = 1;
        perm[1] = 3;
        perm[2] = 2;
      } else {
        perm[0] = 3;
        perm[1] = 1;
        perm[2] = 2;
      }
    } else if (x4[0] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 1;
      perm[2] = 3;
    } else if (x4[1] <= x4[2]) {
      perm[0] = 2;
      perm[1] = 3;
      perm[2] = 1;
    } else {
      perm[0] = 3;
      perm[1] = 2;
      perm[2] = 1;
    }

    for (m = 50; (int32_T)(m - 49) <= ib; m++) {
      idx[(int32_T)((int32_T)(m - nNaNs) - ib)] = (int32_T)idx4[(int32_T)
        ((int32_T)perm[(int32_T)(m - 50)] - 1)];
      x[(int32_T)((int32_T)(m - nNaNs) - ib)] = x4[(int32_T)((int32_T)perm
        [(int32_T)(m - 50)] - 1)];
    }
  }

  m = (int32_T)(nNaNs >> 1);
  for (ib = 1; ib <= m; ib++) {
    tailOffset = idx[(int32_T)((int32_T)(ib - nNaNs) + 49)];
    idx[(int32_T)((int32_T)(ib - nNaNs) + 49)] = idx[(int32_T)(50 - ib)];
    idx[(int32_T)(50 - ib)] = tailOffset;
    x[(int32_T)((int32_T)(ib - nNaNs) + 49)] = xwork[(int32_T)(50 - ib)];
    x[(int32_T)(50 - ib)] = xwork[(int32_T)((int32_T)(ib - nNaNs) + 49)];
  }

  if ((int32_T)(nNaNs & 1) != 0) {
    x[(int32_T)((int32_T)(m - nNaNs) + 50)] = xwork[(int32_T)((int32_T)(m -
      nNaNs) + 50)];
  }

  if ((int32_T)(50 - nNaNs) > 1) {
    memset(&iwork[0], 0, (uint32_T)(50U * sizeof(int32_T)));
    ib = (int32_T)((int32_T)(50 - nNaNs) >> 2);
    m = 4;
    while (ib > 1) {
      if ((int32_T)(ib & 1) != 0) {
        ib--;
        tailOffset = (int32_T)(m * ib);
        nTail = (int32_T)(50 - (int32_T)(nNaNs + tailOffset));
        if (nTail > m) {
          phlnlfkfphdjohln_merge(idx, x, tailOffset, m, (int32_T)(nTail - m),
            iwork, xwork);
        }
      }

      tailOffset = (int32_T)(m << 1);
      ib >>= 1;
      for (nTail = 1; nTail <= ib; nTail++) {
        phlnlfkfphdjohln_merge(idx, x, (int32_T)((int32_T)(nTail - 1) *
          tailOffset), m, m, iwork, xwork);
      }

      m = tailOffset;
    }

    if ((int32_T)(50 - nNaNs) > m) {
      phlnlfkfphdjohln_merge(idx, x, 0, m, (int32_T)(50 - (int32_T)(nNaNs + m)),
        iwork, xwork);
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
