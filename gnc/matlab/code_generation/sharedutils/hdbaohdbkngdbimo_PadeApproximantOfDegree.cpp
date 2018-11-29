//
// File: hdbaohdbkngdbimo_PadeApproximantOfDegree.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 11:58:17 2018
//
#include "rtwtypes.h"
#include <string.h>
#include "djecfknojekfgdba_mldivide.h"
#include "hdbaohdbkngdbimo_PadeApproximantOfDegree.h"

// Function for MATLAB Function: '<S94>/MATLAB Function'
void hdbaohdbkngdbimo_PadeApproximantOfDegree(const real32_T A[225], uint8_T m,
  real32_T F[225])
{
  real32_T A2[225];
  int32_T d;
  real32_T A3[225];
  real32_T A4[225];
  int32_T d_k;
  real32_T A_0[225];
  int32_T i;
  for (d = 0; d < 15; d++) {
    for (d_k = 0; d_k < 15; d_k++) {
      A2[(int32_T)(d_k + (int32_T)(15 * d))] = 0.0F;
      for (i = 0; i < 15; i++) {
        A2[(int32_T)(d_k + (int32_T)(15 * d))] += A[(int32_T)((int32_T)(15 * i)
          + d_k)] * A[(int32_T)((int32_T)(15 * d) + i)];
      }
    }
  }

  if ((int32_T)m == 3) {
    memcpy(&F[0], &A2[0], (uint32_T)(225U * sizeof(real32_T)));
    for (d = 0; d < 15; d++) {
      F[(int32_T)(d + (int32_T)(15 * d))] += 60.0F;
    }

    for (d = 0; d < 15; d++) {
      for (d_k = 0; d_k < 15; d_k++) {
        A_0[(int32_T)(d_k + (int32_T)(15 * d))] = 0.0F;
        for (i = 0; i < 15; i++) {
          A_0[(int32_T)(d_k + (int32_T)(15 * d))] += A[(int32_T)((int32_T)(15 *
            i) + d_k)] * F[(int32_T)((int32_T)(15 * d) + i)];
        }
      }
    }

    for (d = 0; d < 15; d++) {
      for (d_k = 0; d_k < 15; d_k++) {
        F[(int32_T)(d_k + (int32_T)(15 * d))] = A_0[(int32_T)((int32_T)(15 * d)
          + d_k)];
      }
    }

    for (d = 0; d < 225; d++) {
      A4[d] = 12.0F * A2[d];
    }

    d = 120;
  } else {
    for (d = 0; d < 15; d++) {
      for (d_k = 0; d_k < 15; d_k++) {
        A3[(int32_T)(d_k + (int32_T)(15 * d))] = 0.0F;
        for (i = 0; i < 15; i++) {
          A3[(int32_T)(d_k + (int32_T)(15 * d))] += A2[(int32_T)((int32_T)(15 *
            i) + d_k)] * A2[(int32_T)((int32_T)(15 * d) + i)];
        }
      }
    }

    if ((int32_T)m == 5) {
      for (d = 0; d < 225; d++) {
        F[d] = 420.0F * A2[d] + A3[d];
      }

      for (d = 0; d < 15; d++) {
        F[(int32_T)(d + (int32_T)(15 * d))] += 15120.0F;
      }

      for (d = 0; d < 15; d++) {
        for (d_k = 0; d_k < 15; d_k++) {
          A_0[(int32_T)(d_k + (int32_T)(15 * d))] = 0.0F;
          for (i = 0; i < 15; i++) {
            A_0[(int32_T)(d_k + (int32_T)(15 * d))] += A[(int32_T)((int32_T)(15 *
              i) + d_k)] * F[(int32_T)((int32_T)(15 * d) + i)];
          }
        }
      }

      for (d = 0; d < 15; d++) {
        for (d_k = 0; d_k < 15; d_k++) {
          F[(int32_T)(d_k + (int32_T)(15 * d))] = A_0[(int32_T)((int32_T)(15 * d)
            + d_k)];
        }
      }

      for (d = 0; d < 225; d++) {
        A4[d] = 30.0F * A3[d] + 3360.0F * A2[d];
      }

      d = 30240;
    } else {
      for (d = 0; d < 15; d++) {
        for (d_k = 0; d_k < 15; d_k++) {
          A4[(int32_T)(d_k + (int32_T)(15 * d))] = 0.0F;
          for (i = 0; i < 15; i++) {
            A4[(int32_T)(d_k + (int32_T)(15 * d))] += A3[(int32_T)((int32_T)(15 *
              i) + d_k)] * A2[(int32_T)((int32_T)(15 * d) + i)];
          }
        }
      }

      for (d = 0; d < 225; d++) {
        F[d] = (1512.0F * A3[d] + A4[d]) + 277200.0F * A2[d];
      }

      for (d = 0; d < 15; d++) {
        F[(int32_T)(d + (int32_T)(15 * d))] += 8.64864E+6F;
      }

      for (d = 0; d < 15; d++) {
        for (d_k = 0; d_k < 15; d_k++) {
          A_0[(int32_T)(d_k + (int32_T)(15 * d))] = 0.0F;
          for (i = 0; i < 15; i++) {
            A_0[(int32_T)(d_k + (int32_T)(15 * d))] += A[(int32_T)((int32_T)(15 *
              i) + d_k)] * F[(int32_T)((int32_T)(15 * d) + i)];
          }
        }
      }

      for (d = 0; d < 15; d++) {
        for (d_k = 0; d_k < 15; d_k++) {
          F[(int32_T)(d_k + (int32_T)(15 * d))] = A_0[(int32_T)((int32_T)(15 * d)
            + d_k)];
        }
      }

      for (d = 0; d < 225; d++) {
        A4[d] = (56.0F * A4[d] + 25200.0F * A3[d]) + 1.99584E+6F * A2[d];
      }

      d = 17297280;
    }
  }

  for (d_k = 0; d_k < 15; d_k++) {
    A4[(int32_T)(d_k + (int32_T)(15 * d_k))] += (real32_T)d;
  }

  for (d = 0; d < 225; d++) {
    A4[d] -= F[d];
    F[d] *= 2.0F;
  }

  djecfknojekfgdba_mldivide(A4, F);
  for (d = 0; d < 15; d++) {
    F[(int32_T)(d + (int32_T)(15 * d))]++;
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
