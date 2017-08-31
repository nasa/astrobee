//
// File: ppppmophjmopekno_ilazlc.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include "ppppmophjmopekno_ilazlc.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
int32_T ppppmophjmopekno_ilazlc(int32_T m, int32_T n, const real32_T A_data[],
  int32_T ia0, int32_T lda)
{
  int32_T j;
  int32_T coltop;
  int32_T ia;
  int32_T exitg1;
  boolean_T exitg2;
  j = n;
  exitg2 = false;
  while ((!exitg2) && (j > 0)) {
    coltop = (j - 1) * lda + ia0;
    ia = coltop;
    do {
      exitg1 = 0;
      if (ia <= (coltop + m) - 1) {
        if (A_data[ia - 1] != 0.0F) {
          exitg1 = 1;
        } else {
          ia++;
        }
      } else {
        j--;
        exitg1 = 2;
      }
    } while (exitg1 == 0);

    if (exitg1 == 1) {
      exitg2 = true;
    }
  }

  return j;
}

//
// File trailer for generated code.
//
// [EOF]
//
