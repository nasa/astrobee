//
// File: look1_iflf_pbinlxpw.cpp
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1051
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Fri Dec  2 10:48:20 2016
//
#include "rtwtypes.h"
#include "look1_iflf_pbinlxpw.h"

real32_T look1_iflf_pbinlxpw(real32_T u0, const real32_T bp0[], const real32_T
  table[], uint32_T prevIndex[], uint32_T maxIndex)
{
  real32_T frac;
  uint32_T startIndex;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T found;

  // Lookup 1-D
  // Search method: 'binary'
  // Use previous index: 'on'
  // Interpolation method: 'Linear'
  // Extrapolation method: 'Linear'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Extrapolation method: 'Linear'
  // Use previous index: 'on'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u0 <= bp0[0U]) {
    startIndex = 0U;
    frac = (u0 - bp0[0U]) / (bp0[1U] - bp0[0U]);
  } else if (u0 < bp0[maxIndex]) {
    startIndex = prevIndex[0U];

    // Binary Search using Previous Index
    iLeft = 0U;
    iRght = maxIndex;
    found = 0U;
    while (found == 0U) {
      if (u0 < bp0[startIndex]) {
        iRght = startIndex - 1U;
        startIndex = (iRght + iLeft) >> 1U;
      } else if (u0 < bp0[startIndex + 1U]) {
        found = 1U;
      } else {
        iLeft = startIndex + 1U;
        startIndex = (iRght + iLeft) >> 1U;
      }
    }

    frac = (u0 - bp0[startIndex]) / (bp0[startIndex + 1U] - bp0[startIndex]);
  } else {
    startIndex = maxIndex - 1U;
    frac = (u0 - bp0[maxIndex - 1U]) / (bp0[maxIndex] - bp0[maxIndex - 1U]);
  }

  prevIndex[0U] = startIndex;

  // Interpolation 1-D
  // Interpolation method: 'Linear'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Overflow mode: 'portable wrapping'

  return (table[startIndex + 1U] - table[startIndex]) * frac + table[startIndex];
}

//
// File trailer for generated code.
//
// [EOF]
//
