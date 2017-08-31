//
// File: mohdcjmgnohlknoh_sortLE.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed May 17 14:43:55 2017
//
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "mohdcjmgnohlknoh_sortLE.h"

// Function for MATLAB Function: '<S46>/generate_output'
boolean_T mohdcjmgnohlknoh_sortLE(const real32_T v[17264], const int32_T col[2],
  int32_T irow1, int32_T irow2)
{
  boolean_T p;
  int32_T coloffset;
  boolean_T b;
  int32_T k;
  boolean_T exitg1;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    coloffset = (int32_T)((int32_T)((int32_T)(col[k] - 1) * 8632) - 1);
    if ((v[(int32_T)(coloffset + irow1)] == v[(int32_T)(coloffset + irow2)]) ||
        (rtIsNaNF(v[(int32_T)(coloffset + irow1)]) && rtIsNaNF(v[(int32_T)
          (coloffset + irow2)]))) {
      b = true;
    } else {
      b = false;
    }

    if (!b) {
      if ((v[(int32_T)(coloffset + irow1)] <= v[(int32_T)(coloffset + irow2)]) ||
          rtIsNaNF(v[(int32_T)(coloffset + irow2)])) {
      } else {
        p = false;
      }

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
