//
// File: nglnglnoohdjmgdj_sortLE.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:14:10 2017
//
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "nglnglnoohdjmgdj_sortLE.h"

// Function for MATLAB Function: '<S47>/generate_output'
boolean_T nglnglnoohdjmgdj_sortLE(const real32_T v_data[], const int32_T
  v_sizes[2], const int32_T col[2], int32_T irow1, int32_T irow2)
{
  boolean_T p;
  int32_T coloffset;
  int32_T k;
  boolean_T exitg1;
  p = true;
  k = 0;
  exitg1 = false;
  while ((!exitg1) && (k < 2)) {
    coloffset = (col[k] - 1) * v_sizes[0] - 1;
    if (!((v_data[coloffset + irow1] == v_data[coloffset + irow2]) || (rtIsNaNF
          (v_data[coloffset + irow1]) && rtIsNaNF(v_data[coloffset + irow2]))))
    {
      p = ((v_data[coloffset + irow1] <= v_data[coloffset + irow2]) || rtIsNaNF
           (v_data[coloffset + irow2]));
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
