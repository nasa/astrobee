//
// File: dbiekfcbglfkkfcb_sortLE.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed May 17 14:43:55 2017
//
#include "rtwtypes.h"
#include "rt_nonfinite.h"
#include "dbiekfcbglfkkfcb_sortLE.h"

// Function for MATLAB Function: '<S47>/generate_output'
boolean_T dbiekfcbglfkkfcb_sortLE(const real32_T v_data[], const int32_T
  v_sizes[2], const int32_T col[2], int32_T irow1, int32_T irow2)
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
    coloffset = (int32_T)((int32_T)((int32_T)(col[k] - 1) * v_sizes[0]) - 1);
    if ((v_data[(int32_T)(coloffset + irow1)] == v_data[(int32_T)(coloffset +
          irow2)]) || (rtIsNaNF(v_data[(int32_T)(coloffset + irow1)]) &&
                       rtIsNaNF(v_data[(int32_T)(coloffset + irow2)]))) {
      b = true;
    } else {
      b = false;
    }

    if (!b) {
      if ((v_data[(int32_T)(coloffset + irow1)] <= v_data[(int32_T)(coloffset +
            irow2)]) || rtIsNaNF(v_data[(int32_T)(coloffset + irow2)])) {
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
