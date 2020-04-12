//
// File: baaiimglmglniekf_sortIdx.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:08:00 2018
//
#include "rtwtypes.h"
#include "baaiimglmglniekf_sortIdx.h"

// Function for MATLAB Function: '<S81>/generate_output'
void baaiimglmglniekf_sortIdx(const real_T x_data[], const int32_T x_sizes[2],
  int32_T idx_data[], int32_T idx_sizes[2])
{
  int32_T n;
  int32_T k;
  int32_T i;
  int32_T i2;
  int32_T j;
  int32_T pEnd;
  int32_T p;
  int32_T q;
  int32_T qEnd;
  int32_T kEnd;
  int32_T iwork_data[504];
  n = (int32_T)(x_sizes[1] + 1);
  idx_sizes[0] = 1;
  idx_sizes[1] = x_sizes[1];
  i = x_sizes[1];
  for (i2 = 0; i2 <= (int32_T)(i - 1); i2++) {
    idx_data[i2] = 0;
  }

  if (x_sizes[1] != 0) {
    for (i = 1; i <= (int32_T)(n - 2); i += 2) {
      if (x_data[(int32_T)(i - 1)] <= x_data[i]) {
        idx_data[(int32_T)(i - 1)] = i;
        idx_data[i] = (int32_T)(i + 1);
      } else {
        idx_data[(int32_T)(i - 1)] = (int32_T)(i + 1);
        idx_data[i] = i;
      }
    }

    if ((int32_T)(x_sizes[1] & 1) != 0) {
      idx_data[(int32_T)(x_sizes[1] - 1)] = x_sizes[1];
    }

    i = 2;
    while (i < (int32_T)(n - 1)) {
      i2 = (int32_T)(i << 1);
      j = 1;
      pEnd = (int32_T)(1 + i);
      while (pEnd < n) {
        p = j;
        q = pEnd;
        qEnd = (int32_T)(j + i2);
        if (qEnd > n) {
          qEnd = n;
        }

        k = 0;
        kEnd = (int32_T)(qEnd - j);
        while ((int32_T)(k + 1) <= kEnd) {
          if (x_data[(int32_T)(idx_data[(int32_T)(p - 1)] - 1)] <= x_data
              [(int32_T)(idx_data[(int32_T)(q - 1)] - 1)]) {
            iwork_data[k] = idx_data[(int32_T)(p - 1)];
            p++;
            if (p == pEnd) {
              while (q < qEnd) {
                k++;
                iwork_data[k] = idx_data[(int32_T)(q - 1)];
                q++;
              }
            }
          } else {
            iwork_data[k] = idx_data[(int32_T)(q - 1)];
            q++;
            if (q == qEnd) {
              while (p < pEnd) {
                k++;
                iwork_data[k] = idx_data[(int32_T)(p - 1)];
                p++;
              }
            }
          }

          k++;
        }

        for (pEnd = 0; (int32_T)(pEnd + 1) <= kEnd; pEnd++) {
          idx_data[(int32_T)((int32_T)(j + pEnd) - 1)] = iwork_data[pEnd];
        }

        j = qEnd;
        pEnd = (int32_T)(qEnd + i);
      }

      i = i2;
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
