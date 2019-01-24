//
// File: plook_u32f_binckpan.cpp
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:07:06 2018
//
#include "rtwtypes.h"
#include "binsearch_u32f_prevIdx.h"
#include "plook_u32f_binckpan.h"

uint32_T plook_u32f_binckpan(real32_T u, const real32_T bp[], uint32_T maxIndex,
  uint32_T *prevIndex)
{
  uint32_T bpIndex;

  // Prelookup - Index only
  // Index Search method: 'binary'
  // Interpolation method: 'Use nearest'
  // Extrapolation method: 'Clip'
  // Use previous index: 'on'
  // Use last breakpoint for index at or above upper limit: 'on'
  // Remove protection against out-of-range input in generated code: 'off'

  if (u <= bp[0U]) {
    bpIndex = 0U;
  } else if (u < bp[maxIndex]) {
    bpIndex = binsearch_u32f_prevIdx(u, bp, *prevIndex, maxIndex);
    if (bp[(uint32_T)(bpIndex + 1U)] - u <= u - bp[bpIndex]) {
      bpIndex++;
    }
  } else {
    bpIndex = maxIndex;
  }

  *prevIndex = bpIndex;
  return bpIndex;
}

//
// File trailer for generated code.
//
// [EOF]
//
