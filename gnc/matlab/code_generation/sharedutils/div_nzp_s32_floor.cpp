//
// File: div_nzp_s32_floor.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed May 17 14:42:31 2017
//
#include "rtwtypes.h"
#include "div_nzp_s32_floor.h"

int32_T div_nzp_s32_floor(int32_T numerator, int32_T denominator)
{
  uint32_T absNumerator;
  uint32_T absDenominator;
  uint32_T tempAbsQuotient;
  boolean_T quotientNeedsNegation;
  absNumerator = numerator < 0 ? (uint32_T)((uint32_T)~(uint32_T)numerator + 1U)
    : (uint32_T)numerator;
  absDenominator = denominator < 0 ? (uint32_T)((uint32_T)~(uint32_T)denominator
    + 1U) : (uint32_T)denominator;
  quotientNeedsNegation = ((numerator < 0) != (denominator < 0));
  tempAbsQuotient = (uint32_T)(absNumerator / absDenominator);
  if (quotientNeedsNegation) {
    absNumerator %= absDenominator;
    if (absNumerator > 0U) {
      tempAbsQuotient++;
    }
  }

  return quotientNeedsNegation ? (int32_T)-(int32_T)tempAbsQuotient : (int32_T)
    tempAbsQuotient;
}

//
// File trailer for generated code.
//
// [EOF]
//
