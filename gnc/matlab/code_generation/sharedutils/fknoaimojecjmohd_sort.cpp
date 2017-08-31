//
// File: fknoaimojecjmohd_sort.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:14:10 2017
//
#include "rtwtypes.h"
#include <string.h>
#include "gdbafcbifcbapphd_merge.h"
#include "rt_nonfinite.h"
#include "fknoaimojecjmohd_sort.h"

// Function for MATLAB Function: '<S47>/generate_output'
void fknoaimojecjmohd_sort(real32_T x[50], int32_T idx[50])
{
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
  int32_T i;
  x4[0] = 0.0F;
  idx4[0] = 0;
  x4[1] = 0.0F;
  idx4[1] = 0;
  x4[2] = 0.0F;
  idx4[2] = 0;
  x4[3] = 0.0F;
  idx4[3] = 0;
  for (i = 0; i < 50; i++) {
    idx[i] = 0;
    xwork[i] = 0.0F;
  }

  i = 0;
  ib = 0;
  for (m = 0; m < 50; m++) {
    if (rtIsNaNF(x[m])) {
      idx[49 - i] = m + 1;
      xwork[49 - i] = x[m];
      i++;
    } else {
      ib++;
      idx4[ib - 1] = (int8_T)(m + 1);
      x4[ib - 1] = x[m];
      if (ib == 4) {
        ib = m - i;
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

        if (x4[tailOffset - 1] <= x4[i3 - 1]) {
          if (x4[nTail - 1] <= x4[i3 - 1]) {
            perm[0] = (int8_T)tailOffset;
            perm[1] = (int8_T)nTail;
            perm[2] = (int8_T)i3;
            perm[3] = (int8_T)i4;
          } else if (x4[nTail - 1] <= x4[i4 - 1]) {
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
        } else if (x4[tailOffset - 1] <= x4[i4 - 1]) {
          if (x4[nTail - 1] <= x4[i4 - 1]) {
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

        idx[ib - 3] = idx4[perm[0] - 1];
        idx[ib - 2] = idx4[perm[1] - 1];
        idx[ib - 1] = idx4[perm[2] - 1];
        idx[ib] = idx4[perm[3] - 1];
        x[ib - 3] = x4[perm[0] - 1];
        x[ib - 2] = x4[perm[1] - 1];
        x[ib - 1] = x4[perm[2] - 1];
        x[ib] = x4[perm[3] - 1];
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

    for (m = 50; m - 49 <= ib; m++) {
      idx[(m - i) - ib] = idx4[perm[m - 50] - 1];
      x[(m - i) - ib] = x4[perm[m - 50] - 1];
    }
  }

  m = i >> 1;
  for (ib = 1; ib <= m; ib++) {
    tailOffset = idx[(ib - i) + 49];
    idx[(ib - i) + 49] = idx[50 - ib];
    idx[50 - ib] = tailOffset;
    x[(ib - i) + 49] = xwork[50 - ib];
    x[50 - ib] = xwork[(ib - i) + 49];
  }

  if ((i & 1) != 0) {
    x[(m - i) + 50] = xwork[(m - i) + 50];
  }

  if (50 - i > 1) {
    memset(&iwork[0], 0, 50U * sizeof(int32_T));
    ib = (50 - i) >> 2;
    m = 4;
    while (ib > 1) {
      if ((ib & 1) != 0) {
        ib--;
        tailOffset = m * ib;
        nTail = 50 - (i + tailOffset);
        if (nTail > m) {
          gdbafcbifcbapphd_merge(idx, x, tailOffset, m, nTail - m, iwork, xwork);
        }
      }

      tailOffset = m << 1;
      ib >>= 1;
      for (nTail = 1; nTail <= ib; nTail++) {
        gdbafcbifcbapphd_merge(idx, x, (nTail - 1) * tailOffset, m, m, iwork,
          xwork);
      }

      m = tailOffset;
    }

    if (50 - i > m) {
      gdbafcbifcbapphd_merge(idx, x, 0, m, 50 - (i + m), iwork, xwork);
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
