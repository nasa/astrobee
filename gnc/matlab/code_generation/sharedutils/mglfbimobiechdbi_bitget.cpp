//
// File: mglfbimobiechdbi_bitget.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Dec  4 08:33:06 2017
//
#include "rtwtypes.h"
#include "mglfbimobiechdbi_bitget.h"

// Function for MATLAB Function: '<S94>/MATLAB Function2'
void mglfbimobiechdbi_bitget(uint32_T a, uint32_T c[17])
{
  int32_T k;
  for (k = 0; k < 17; k++) {
    c[k] = (uint32_T)((int32_T)(uint32_T)((uint32_T)(1U << k) & a) != 0);
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
