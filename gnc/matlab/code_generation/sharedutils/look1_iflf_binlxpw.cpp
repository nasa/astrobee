//
// File: look1_iflf_binlxpw.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 12:00:33 2018
//
#include "rtwtypes.h"
#include "look1_iflf_binlxpw.h"

real32_T look1_iflf_binlxpw(real32_T u0, const real32_T bp0[], const real32_T
  table[], uint32_T maxIndex)
{
  real32_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;

  // Lookup 1-D
  // Search method: 'binary'
  // Use previous index: 'off'
  // Interpolation method: 'Linear'
  // Extrapolation method: 'Linear'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Extrapolation method: 'Linear'
  // Use previous index: 'off'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u0 <= bp0[0U]) {
    iLeft = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    // Binary Search
    bpIdx = (uint32_T)(maxIndex >> 1U);
    iLeft = 0U;
    iRght = maxIndex;
    while ((uint32_T)(iRght - iLeft) > 1U) {
      if (u0 < bp0[bpIdx]) {
        iRght = bpIdx;
      } else {
        iLeft = bpIdx;
      }

      bpIdx = (uint32_T)((uint32_T)(iRght + iLeft) >> 1U);
    }

    frac = (u0 - bp0[iLeft]) / (bp0[(uint32_T)(iLeft + 1U)] - bp0[iLeft]);
  } else {
    iLeft = (uint32_T)(maxIndex - 1U);
    frac = (u0 - bp0[(uint32_T)(maxIndex - 1U)]) / (bp0[maxIndex] - bp0
      [(uint32_T)(maxIndex - 1U)]);
  }

  // Interpolation 1-D
  // Interpolation method: 'Linear'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Overflow mode: 'portable wrapping'

  return (table[(uint32_T)(iLeft + 1U)] - table[iLeft]) * frac + table[iLeft];
}

//
// File trailer for generated code.
//
// [EOF]
//
