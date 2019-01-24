//
// File: knohaiececjedbai_ilazlc.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:06:07 2018
//
#include "rtwtypes.h"
#include "knohaiececjedbai_ilazlc.h"

// Function for MATLAB Function: '<S16>/Compute Residual and H'
int32_T knohaiececjedbai_ilazlc(int32_T m, int32_T n, const real32_T A_data[],
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
    coltop = (int32_T)((int32_T)((int32_T)(j - 1) * lda) + ia0);
    ia = coltop;
    do {
      exitg1 = 0;
      if (ia <= (int32_T)((int32_T)(coltop + m) - 1)) {
        if (A_data[(int32_T)(ia - 1)] != 0.0F) {
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
