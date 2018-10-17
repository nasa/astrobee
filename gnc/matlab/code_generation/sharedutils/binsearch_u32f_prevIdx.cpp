//
// File: binsearch_u32f_prevIdx.cpp
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 11:59:30 2018
//
#include "rtwtypes.h"
#include "binsearch_u32f_prevIdx.h"

uint32_T binsearch_u32f_prevIdx(real32_T u, const real32_T bp[], uint32_T
  startIndex, uint32_T maxIndex)
{
  uint32_T bpIndex;
  uint32_T iRght;
  uint32_T iLeft;
  uint32_T found;

  // Binary Search using Previous Index
  bpIndex = startIndex;
  iLeft = 0U;
  iRght = maxIndex;
  found = 0U;
  while (found == 0U) {
    if (u < bp[bpIndex]) {
      iRght = (uint32_T)(bpIndex - 1U);
      bpIndex = (uint32_T)((uint32_T)(iRght + iLeft) >> 1U);
    } else if (u < bp[(uint32_T)(bpIndex + 1U)]) {
      found = 1U;
    } else {
      iLeft = (uint32_T)(bpIndex + 1U);
      bpIndex = (uint32_T)((uint32_T)(iRght + iLeft) >> 1U);
    }
  }

  return bpIndex;
}

//
// File trailer for generated code.
//
// [EOF]
//
