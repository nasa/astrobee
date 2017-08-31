//
// File: kfcjlnglmgdbgdjm_all.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed May 17 14:43:55 2017
//
#include "rtwtypes.h"
#include "kfcjlnglmgdbgdjm_all.h"

// Function for MATLAB Function: '<S43>/generate_output'
boolean_T kfcjlnglmgdbgdjm_all(const boolean_T x[4])
{
  boolean_T y;
  int32_T k;
  boolean_T exitg1;
  y = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 4)) {
    if (!x[k]) {
      y = false;
      exitg1 = true;
    } else {
      k++;
    }
  }

  return y;
}

//
// File trailer for generated code.
//
// [EOF]
//
