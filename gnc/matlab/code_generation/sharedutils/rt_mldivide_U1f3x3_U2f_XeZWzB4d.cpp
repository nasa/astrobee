//
// File: rt_mldivide_U1f3x3_U2f_XeZWzB4d.cpp
//
// Code generated for Simulink model 'ctl_controller0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:06:40 2018
//
#include "rtwtypes.h"
#include <math.h>
#include "rt_mldivide_U1f3x3_U2f_XeZWzB4d.h"

void rt_mldivide_U1f3x3_U2f_XeZWzB4d(const real32_T u0[9], const real32_T u1[3],
  real32_T y[3])
{
  real32_T A[9];
  int32_T r1;
  int32_T r2;
  int32_T r3;
  real32_T maxval;
  real32_T a21;
  int32_T rtemp;
  for (r1 = 0; r1 < 9; r1++) {
    A[r1] = u0[r1];
  }

  r1 = 0;
  r2 = 1;
  r3 = 2;
  maxval = (real32_T)fabs((real_T)u0[0]);
  a21 = (real32_T)fabs((real_T)u0[1]);
  if (a21 > maxval) {
    maxval = a21;
    r1 = 1;
    r2 = 0;
  }

  if ((real32_T)fabs((real_T)u0[2]) > maxval) {
    r1 = 2;
    r2 = 1;
    r3 = 0;
  }

  A[r2] = u0[r2] / u0[r1];
  A[r3] /= A[r1];
  A[(int32_T)(3 + r2)] -= A[(int32_T)(3 + r1)] * A[r2];
  A[(int32_T)(3 + r3)] -= A[(int32_T)(3 + r1)] * A[r3];
  A[(int32_T)(6 + r2)] -= A[(int32_T)(6 + r1)] * A[r2];
  A[(int32_T)(6 + r3)] -= A[(int32_T)(6 + r1)] * A[r3];
  if ((real32_T)fabs((real_T)A[(int32_T)(3 + r3)]) > (real32_T)fabs((real_T)A
       [(int32_T)(3 + r2)])) {
    rtemp = (int32_T)(r2 + 1);
    r2 = r3;
    r3 = (int32_T)(rtemp - 1);
  }

  A[(int32_T)(3 + r3)] /= A[(int32_T)(3 + r2)];
  A[(int32_T)(6 + r3)] -= A[(int32_T)(3 + r3)] * A[(int32_T)(6 + r2)];
  maxval = u1[r2] - u1[r1] * A[r2];
  a21 = ((u1[r3] - u1[r1] * A[r3]) - A[(int32_T)(3 + r3)] * maxval) / A[(int32_T)
    (6 + r3)];
  maxval -= A[(int32_T)(6 + r2)] * a21;
  maxval /= A[(int32_T)(3 + r2)];
  y[0] = ((u1[r1] - A[(int32_T)(6 + r1)] * a21) - A[(int32_T)(3 + r1)] * maxval)
    / A[r1];
  y[1] = maxval;
  y[2] = a21;
}

//
// File trailer for generated code.
//
// [EOF]
//
