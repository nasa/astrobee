//
// File: look1_iflf_pbinlcapw.cpp
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:07:06 2018
//
#include "rtwtypes.h"
#include "look1_iflf_pbinlcapw.h"

real32_T look1_iflf_pbinlcapw(real32_T u0, const real32_T bp0[], const real32_T
  table[], uint32_T prevIndex[], uint32_T maxIndex)
{
  real32_T y;
  real32_T frac;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T bpIdx;
  uint32_T found;

  // Lookup 1-D
  // Search method: 'binary'
  // Use previous index: 'on'
  // Interpolation method: 'Linear'
  // Extrapolation method: 'Clip'
  // Use last breakpoint for index at or above upper limit: 'on'
  // Remove protection against out-of-range input in generated code: 'off'

  // Prelookup - Index and Fraction
  // Index Search method: 'binary'
  // Extrapolation method: 'Clip'
  // Use previous index: 'on'
  // Use last breakpoint for index at or above upper limit: 'on'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u0 <= bp0[0U]) {
    bpIdx = 0U;
    frac = 0.0F;
  } else if (u0 < bp0[maxIndex]) {
    // Binary Search using Previous Index
    bpIdx = prevIndex[0U];
    iLeft = 0U;
    iRght = maxIndex;
    found = 0U;
    while (found == 0U) {
      if (u0 < bp0[bpIdx]) {
        iRght = (uint32_T)(bpIdx - 1U);
        bpIdx = (uint32_T)((uint32_T)(iRght + iLeft) >> 1U);
      } else if (u0 < bp0[(uint32_T)(bpIdx + 1U)]) {
        found = 1U;
      } else {
        iLeft = (uint32_T)(bpIdx + 1U);
        bpIdx = (uint32_T)((uint32_T)(iRght + iLeft) >> 1U);
      }
    }

    frac = (u0 - bp0[bpIdx]) / (bp0[(uint32_T)(bpIdx + 1U)] - bp0[bpIdx]);
  } else {
    bpIdx = maxIndex;
    frac = 0.0F;
  }

  prevIndex[0U] = bpIdx;

  // Interpolation 1-D
  // Interpolation method: 'Linear'
  // Use last breakpoint for index at or above upper limit: 'on'
  // Overflow mode: 'portable wrapping'

  if (bpIdx == maxIndex) {
    y = table[bpIdx];
  } else {
    y = (table[(uint32_T)(bpIdx + 1U)] - table[bpIdx]) * frac + table[bpIdx];
  }

  return y;
}

//
// File trailer for generated code.
//
// [EOF]
//
