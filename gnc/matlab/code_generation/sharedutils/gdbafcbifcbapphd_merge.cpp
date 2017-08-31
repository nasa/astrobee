//
// File: gdbafcbifcbapphd_merge.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:14:10 2017
//
#include "rtwtypes.h"
#include "gdbafcbifcbapphd_merge.h"

// Function for MATLAB Function: '<S47>/generate_output'
void gdbafcbifcbapphd_merge(int32_T idx[50], real32_T x[50], int32_T offset,
  int32_T np, int32_T nq, int32_T iwork[50], real32_T xwork[50])
{
  int32_T n;
  int32_T q;
  int32_T qend;
  int32_T iout;
  int32_T offset1;
  int32_T b_j;
  if (nq != 0) {
    n = np + nq;
    for (q = 0; q + 1 <= n; q++) {
      iwork[q] = idx[offset + q];
      xwork[q] = x[offset + q];
    }

    n = 0;
    q = np;
    qend = np + nq;
    iout = offset - 1;
    do {
      offset1 = 0;
      iout++;
      if (xwork[n] <= xwork[q]) {
        idx[iout] = iwork[n];
        x[iout] = xwork[n];
        if (n + 1 < np) {
          n++;
        } else {
          offset1 = 1;
        }
      } else {
        idx[iout] = iwork[q];
        x[iout] = xwork[q];
        if (q + 1 < qend) {
          q++;
        } else {
          offset1 = (iout - n) + 1;
          for (b_j = n; b_j + 1 <= np; b_j++) {
            idx[offset1 + b_j] = iwork[b_j];
            x[offset1 + b_j] = xwork[b_j];
          }

          offset1 = 1;
        }
      }
    } while (offset1 == 0);
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
