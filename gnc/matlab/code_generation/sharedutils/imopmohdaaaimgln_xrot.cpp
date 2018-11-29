//
// File: imopmohdaaaimgln_xrot.cpp
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 11:59:30 2018
//
#include "rtwtypes.h"
#include "imopmohdaaaimgln_xrot.h"

// Function for MATLAB Function: '<S12>/MATLAB Function'
void imopmohdaaaimgln_xrot(real32_T x[36], int32_T ix0, int32_T iy0, real32_T c,
  real32_T s)
{
  int32_T ix;
  int32_T iy;
  real32_T temp;
  int32_T k;
  ix = (int32_T)(ix0 - 1);
  iy = (int32_T)(iy0 - 1);
  for (k = 0; k < 6; k++) {
    temp = c * x[ix] + s * x[iy];
    x[iy] = c * x[iy] - s * x[ix];
    x[ix] = temp;
    iy++;
    ix++;
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
