//
// File: phlnlfkfphdjohln_merge.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 12:00:33 2018
//
#include "rtwtypes.h"
#include "phlnlfkfphdjohln_merge.h"

// Function for MATLAB Function: '<S83>/generate_output'
void phlnlfkfphdjohln_merge(int32_T idx[50], real32_T x[50], int32_T offset,
  int32_T np, int32_T nq, int32_T iwork[50], real32_T xwork[50])
{
  int32_T n;
  int32_T q;
  int32_T qend;
  int32_T iout;
  int32_T exitg1;
  if (nq != 0) {
    n = (int32_T)(np + nq);
    for (q = 0; (int32_T)(q + 1) <= n; q++) {
      iwork[q] = idx[(int32_T)(offset + q)];
      xwork[q] = x[(int32_T)(offset + q)];
    }

    n = 0;
    q = np;
    qend = (int32_T)(np + nq);
    iout = (int32_T)(offset - 1);
    do {
      exitg1 = 0;
      iout++;
      if (xwork[n] <= xwork[q]) {
        idx[iout] = iwork[n];
        x[iout] = xwork[n];
        if ((int32_T)(n + 1) < np) {
          n++;
        } else {
          exitg1 = 1;
        }
      } else {
        idx[iout] = iwork[q];
        x[iout] = xwork[q];
        if ((int32_T)(q + 1) < qend) {
          q++;
        } else {
          q = (int32_T)((int32_T)(iout - n) + 1);
          while ((int32_T)(n + 1) <= np) {
            idx[(int32_T)(q + n)] = iwork[n];
            x[(int32_T)(q + n)] = xwork[n];
            n++;
          }

          exitg1 = 1;
        }
      }
    } while (exitg1 == 0);
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
