//
// File: rt_urand_Upu32_Yd_f_pw_snf.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Oct 16 10:08:00 2018
//
#include "rtwtypes.h"
#include "rt_urand_Upu32_Yd_f_pw_snf.h"

real_T rt_urand_Upu32_Yd_f_pw_snf(uint32_T *u)
{
  uint32_T lo;
  uint32_T hi;

  // Uniform random number generator (random number between 0 and 1)

  // #define IA      16807                      magic multiplier = 7^5
  // #define IM      2147483647                 modulus = 2^31-1
  // #define IQ      127773                     IM div IA
  // #define IR      2836                       IM modulo IA
  // #define S       4.656612875245797e-10      reciprocal of 2^31-1
  // test = IA * (seed % IQ) - IR * (seed/IQ)
  // seed = test < 0 ? (test + IM) : test
  // return (seed*S)

  lo = (uint32_T)(*u % 127773U * 16807U);
  hi = (uint32_T)((uint32_T)(*u / 127773U) * 2836U);
  if (lo < hi) {
    *u = (uint32_T)(2147483647U - (uint32_T)(hi - lo));
  } else {
    *u = (uint32_T)(lo - hi);
  }

  return (real_T)*u * 4.6566128752457969E-10;
}

//
// File trailer for generated code.
//
// [EOF]
//
