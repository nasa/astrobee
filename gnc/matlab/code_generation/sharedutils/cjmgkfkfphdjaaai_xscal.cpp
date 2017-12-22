//
// File: cjmgkfkfphdjaaai_xscal.cpp
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Dec  4 08:34:01 2017
//
#include "rtwtypes.h"
#include "cjmgkfkfphdjaaai_xscal.h"

// Function for MATLAB Function: '<S12>/MATLAB Function'
void cjmgkfkfphdjaaai_xscal(real32_T a, real32_T x[36], int32_T ix0)
{
  int32_T k;
  for (k = ix0; k <= (int32_T)(ix0 + 11); k++) {
    x[(int32_T)(k - 1)] *= a;
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
