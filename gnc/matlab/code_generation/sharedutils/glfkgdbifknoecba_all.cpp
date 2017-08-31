//
// File: glfkgdbifknoecba_all.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:14:10 2017
//
#include "rtwtypes.h"
#include "glfkgdbifknoecba_all.h"

// Function for MATLAB Function: '<S43>/generate_output'
boolean_T glfkgdbifknoecba_all(const boolean_T x[4])
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
