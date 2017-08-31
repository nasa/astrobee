//
// File: imohimopmglfaaim_sortLE.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:14:10 2017
//
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "imohimopmglfaaim_sortLE.h"

// Function for MATLAB Function: '<S46>/generate_output'
boolean_T imohimopmglfaaim_sortLE(const real32_T v[17264], const int32_T col[2],
  int32_T irow1, int32_T irow2)
{
  boolean_T p;
  int32_T coloffset;
  int32_T k;
  boolean_T exitg1;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    coloffset = (col[k] - 1) * 8632 - 1;
    if (!((v[coloffset + irow1] == v[coloffset + irow2]) || (rtIsNaNF
          (v[coloffset + irow1]) && rtIsNaNF(v[coloffset + irow2])))) {
      p = ((v[coloffset + irow1] <= v[coloffset + irow2]) || rtIsNaNF
           (v[coloffset + irow2]));
      exitg1 = true;
    } else {
      k++;
    }
  }

  return p;
}

//
// File trailer for generated code.
//
// [EOF]
//
