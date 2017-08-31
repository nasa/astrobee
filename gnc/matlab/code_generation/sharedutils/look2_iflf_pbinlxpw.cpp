//
// File: look2_iflf_pbinlxpw.cpp
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1051
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Fri Dec  2 10:48:20 2016
//
#include "rtwtypes.h"
#include "look2_iflf_pbinlxpw.h"

real32_T look2_iflf_pbinlxpw(real32_T u0, real32_T u1, const real32_T bp0[],
  const real32_T bp1[], const real32_T table[], uint32_T prevIndex[], const
  uint32_T maxIndex[], uint32_T stride)
{
  real32_T frac;
  uint32_T bpIndices[2];
  real32_T fractions[2];
  uint32_T startIndex;
  real32_T yL_1d;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T found;

  // Lookup 2-D
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
  } else if (u0 < bp0[maxIndex[0U]]) {
    startIndex = prevIndex[0U];

    // Binary Search using Previous Index
    iLeft = 0U;
    iRght = maxIndex[0U];
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
    startIndex = maxIndex[0U] - 1U;
    frac = (u0 - bp0[maxIndex[0U] - 1U]) / (bp0[maxIndex[0U]] - bp0[maxIndex[0U]
      - 1U]);
  }

  prevIndex[0U] = startIndex;
  fractions[0U] = frac;
  bpIndices[0U] = startIndex;

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Extrapolation method: 'Linear'
  // Use previous index: 'on'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u1 <= bp1[0U]) {
    startIndex = 0U;
    frac = (u1 - bp1[0U]) / (bp1[1U] - bp1[0U]);
  } else if (u1 < bp1[maxIndex[1U]]) {
    startIndex = prevIndex[1U];

    // Binary Search using Previous Index
    iLeft = 0U;
    iRght = maxIndex[1U];
    found = 0U;
    while (found == 0U) {
      if (u1 < bp1[startIndex]) {
        iRght = startIndex - 1U;
        startIndex = (iRght + iLeft) >> 1U;
      } else if (u1 < bp1[startIndex + 1U]) {
        found = 1U;
      } else {
        iLeft = startIndex + 1U;
        startIndex = (iRght + iLeft) >> 1U;
      }
    }

    frac = (u1 - bp1[startIndex]) / (bp1[startIndex + 1U] - bp1[startIndex]);
  } else {
    startIndex = maxIndex[1U] - 1U;
    frac = (u1 - bp1[maxIndex[1U] - 1U]) / (bp1[maxIndex[1U]] - bp1[maxIndex[1U]
      - 1U]);
  }

  prevIndex[1U] = startIndex;

  // Interpolation 2-D
  // Interpolation method: 'Linear'
  // Use last breakpoint for index at or above upper limit: 'off'
  // Overflow mode: 'portable wrapping'

  iLeft = startIndex * stride + bpIndices[0U];
  yL_1d = (table[iLeft + 1U] - table[iLeft]) * fractions[0U] + table[iLeft];
  iLeft += stride;
  return (((table[iLeft + 1U] - table[iLeft]) * fractions[0U] + table[iLeft]) -
          yL_1d) * frac + yL_1d;
}

//
// File trailer for generated code.
//
// [EOF]
//
