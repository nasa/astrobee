//
// File: mohdaaielnohnohl_sortIdx.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:14:10 2017
//
#include "rtwtypes.h"
#include "mohdaaielnohnohl_sortIdx.h"

// Function for MATLAB Function: '<S45>/generate_output'
void mohdaaielnohnohl_sortIdx(const real_T x_data[], const int32_T x_sizes[2],
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
  idx_sizes[0] = 1;
  idx_sizes[1] = x_sizes[1];
  n = x_sizes[1];
  for (i = 0; i < n; i++) {
    idx_data[i] = 0;
  }

  if (x_sizes[1] != 0) {
    n = x_sizes[1] + 1;
    for (i = 1; i <= n - 2; i += 2) {
      if (x_data[i - 1] <= x_data[i]) {
        idx_data[i - 1] = i;
        idx_data[i] = i + 1;
      } else {
        idx_data[i - 1] = i + 1;
        idx_data[i] = i;
      }
    }

    if ((x_sizes[1] & 1) != 0) {
      idx_data[x_sizes[1] - 1] = x_sizes[1];
    }

    i = 2;
    while (i < n - 1) {
      i2 = i << 1;
      j = 1;
      pEnd = 1 + i;
      while (pEnd < n) {
        p = j;
        q = pEnd;
        qEnd = j + i2;
        if (qEnd > n) {
          qEnd = n;
        }

        k = 0;
        kEnd = qEnd - j;
        while (k + 1 <= kEnd) {
          if (x_data[idx_data[p - 1] - 1] <= x_data[idx_data[q - 1] - 1]) {
            iwork_data[k] = idx_data[p - 1];
            p++;
            if (p == pEnd) {
              while (q < qEnd) {
                k++;
                iwork_data[k] = idx_data[q - 1];
                q++;
              }
            }
          } else {
            iwork_data[k] = idx_data[q - 1];
            q++;
            if (q == qEnd) {
              while (p < pEnd) {
                k++;
                iwork_data[k] = idx_data[p - 1];
                p++;
              }
            }
          }

          k++;
        }

        for (pEnd = 0; pEnd + 1 <= kEnd; pEnd++) {
          idx_data[(j + pEnd) - 1] = iwork_data[pEnd];
        }

        j = qEnd;
        pEnd = qEnd + i;
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
