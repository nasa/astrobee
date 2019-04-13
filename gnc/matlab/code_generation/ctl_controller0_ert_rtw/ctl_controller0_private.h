//
// File: ctl_controller0_private.h
//
// Code generated for Simulink model 'ctl_controller0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Thu Mar  7 13:22:34 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_ctl_controller0_private_h_
#define RTW_HEADER_ctl_controller0_private_h_
#include "rtwtypes.h"
#include "ctl_controller0.h"
#if !defined(rt_VALIDATE_MEMORY)
#define rt_VALIDATE_MEMORY(S, ptr)     if(!(ptr)) {\
 rtmSetErrorStatus(ctl_controller0_M, RT_MEMORY_ALLOCATION_ERROR);\
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

void ctl_controller0_Normalize(const real32_T rtu_q_in[4], real32_T
  rty_positive_scalar_q[4], P_Normalize_ctl_controller0_T *localP);
void ctl_controller0_Normalize_e(const real32_T rtu_Vec[4], real32_T
  rtu_Magnitude, real32_T rty_Normalized_Vec[4]);
void ctl_contr_ForEachSubsystem_Init(int32_T NumIters,
  DW_ForEachSubsystem_ctl_contr_T localDW[3], P_ForEachSubsystem_ctl_contro_T
  *localP);
void ctl_cont_ForEachSubsystem_Start(int32_T NumIters,
  DW_ForEachSubsystem_ctl_contr_T localDW[3]);
void ctl_controller_ForEachSubsystem(int32_T NumIters, const real32_T rtu_X[3],
  real32_T rty_Y[3], DW_ForEachSubsystem_ctl_contr_T localDW[3],
  P_ForEachSubsystem_ctl_contro_T *localP, real32_T rtp_filt_enable);

#endif                                 // RTW_HEADER_ctl_controller0_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
