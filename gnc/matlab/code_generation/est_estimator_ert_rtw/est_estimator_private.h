//
// File: est_estimator_private.h
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Dec 18 10:14:23 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_est_estimator_private_h_
#define RTW_HEADER_est_estimator_private_h_
#include "rtwtypes.h"
#include "est_estimator.h"
#if !defined(rt_VALIDATE_MEMORY)
#define rt_VALIDATE_MEMORY(S, ptr)     if(!(ptr)) {\
 rtmSetErrorStatus(est_estimator_M, RT_MEMORY_ALLOCATION_ERROR);\
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

void est_estima_MATLABFunction1_Init(DW_MATLABFunction1_est_estima_T *localDW);
void est_estim_MATLABFunction1_Start(B_MATLABFunction1_est_estimat_T *localB);
void est_estimator_MATLABFunction1(real_T rtu_Ts, real32_T rtu_impeller_speed,
  B_MATLABFunction1_est_estimat_T *localB, DW_MATLABFunction1_est_estima_T
  *localDW);
void est_estimator_MATLABFunction(const real32_T rtu_u[16],
  B_MATLABFunction_est_estimato_T *localB);
void est_estimator_Normalize(const real32_T rtu_q_in[4], real32_T
  rty_positive_scalar_q[4], P_Normalize_est_estimator_T *localP);
void est_estimator_Normalize_p(const real32_T rtu_Vec[4], real32_T rtu_Magnitude,
  real32_T rty_Normalized_Vec[4]);
void est_estimato_IfActionSubsystem1(const kfl_msg *rtu_state_in, const
  ase_cov_datatype rtu_P_in[13689], const real_T rtu_aug_velocity[3], const
  real_T rtu_aug_velocity_p[3], const real_T rtu_aug_velocity_l[48], const
  real_T rtu_aug_velocity_o[48], kfl_msg *rty_state_out, ase_cov_datatype
  rty_P_out[13689], real_T rty_aug_velocity_out[3], real_T
  rty_aug_velocity_out_p[3], real_T rty_aug_velocity_out_l[48], real_T
  rty_aug_velocity_out_o[48]);

#endif                                 // RTW_HEADER_est_estimator_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
