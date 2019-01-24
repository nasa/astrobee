//
// File: bpm_blower_2_propulsion_module_private.h
//
// Code generated for Simulink model 'bpm_blower_2_propulsion_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Thu Dec  6 14:21:36 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_bpm_blower_2_propulsion_module_private_h_
#define RTW_HEADER_bpm_blower_2_propulsion_module_private_h_
#include "rtwtypes.h"
#if !defined(rt_VALIDATE_MEMORY)
#define rt_VALIDATE_MEMORY(S, ptr)     if(!(ptr)) {\
 rtmSetErrorStatus(bpm_blower_2_propulsion_modu_M, RT_MEMORY_ALLOCATION_ERROR);\
 }
#endif

#if !defined(rt_FREE)
#if !defined(_WIN32)
#define rt_FREE(ptr)                   if((ptr) != (NULL)) {\
 free((ptr));\
 (ptr) = (NULL);\
 }
#else

// Visual and other windows compilers declare free without const
#define rt_FREE(ptr)                   if((ptr) != (NULL)) {\
 free((void *)(ptr));\
 (ptr) = (NULL);\
 }
#endif
#endif

extern const real_T rtCP_pooled_15LYgUQHWYtd[6];

#define rtCP_random_noise_rtw_collapsed rtCP_pooled_15LYgUQHWYtd // Expression: noz_randn_seed +[1:6]
                                                                 //  Referenced by: '<S2>/random_noise'

#endif                                 // RTW_HEADER_bpm_blower_2_propulsion_module_private_h_ 

//
// File trailer for generated code.
//
// [EOF]
//
