//
// File: ngdjjecbgdbaglfc_eye.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:06:07 2018
//
#include "rtwtypes.h"
#include <string.h>
#include "ngdjjecbgdbaglfc_eye.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void ngdjjecbgdbaglfc_eye(real32_T I[702])
{
  int32_T k;
  memset(&I[0], 0, (uint32_T)(702U * sizeof(real32_T)));
  for (k = 0; k < 6; k++) {
    I[(int32_T)(k + (int32_T)(6 * k))] = 1.0F;
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
