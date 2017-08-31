//
// File: ctl_controller0.h
//
// Code generated for Simulink model 'ctl_controller0'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Thu Aug 31 10:21:18 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_ctl_controller0_h_
#define RTW_HEADER_ctl_controller0_h_
#include <math.h>
#include <stddef.h>
#include <string.h>
#ifndef ctl_controller0_COMMON_INCLUDES_
# define ctl_controller0_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#endif                                 // ctl_controller0_COMMON_INCLUDES_

#include "ctl_controller0_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "mglnkfkfmglfjekn_PadeApproximantOfDegree.h"
#include "rt_mldivide_U1f3x3_U2f_XeZWzB4d.h"
#include "rt_powf_snf.h"
#include "rtGetInf.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block states (auto storage) for system '<S91>/CoreSubsys'
typedef struct {
  real32_T uHzLowPass_states;          // '<S91>/3 Hz Low Pass'
} DW_CoreSubsys_ctl_controller0_T;

// Block states (auto storage) for system '<S5>/For Each Subsystem'
typedef struct {
  DW_CoreSubsys_ctl_controller0_T CoreSubsys;// '<S91>/CoreSubsys'
} DW_ForEachSubsystem_ctl_contr_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real32_T UD_DSTATE[4];               // '<S93>/UD'
  real32_T UD_DSTATE_e[3];             // '<S90>/UD'
  real32_T Delay11_DSTATE[3];          // '<S10>/Delay11'
  real32_T Delay11_DSTATE_i[3];        // '<S11>/Delay11'
  real32_T UnitDelay1_DSTATE[3];       // '<S2>/Unit Delay1'
  real32_T UnitDelay2_DSTATE[4];       // '<S2>/Unit Delay2'
  real32_T DiscreteTimeIntegrator1_DSTATE[3];// '<S12>/Discrete-Time Integrator1' 
  real32_T DiscreteTimeIntegrator_DSTATE[3];// '<S13>/Discrete-Time Integrator'
  int8_T DiscreteTimeIntegrator1_PrevRes;// '<S12>/Discrete-Time Integrator1'
  int8_T DiscreteTimeIntegrator_PrevRese;// '<S13>/Discrete-Time Integrator'
  DW_ForEachSubsystem_ctl_contr_T ForEachSubsystem1[3];// '<S5>/For Each Subsystem1' 
  DW_ForEachSubsystem_ctl_contr_T ForEachSubsystem[3];// '<S5>/For Each Subsystem' 
} DW_ctl_controller0_T;

// Parameters for system: '<S44>/Normalize'
struct P_Normalize_ctl_controller0_T_ {
  real_T Constant1_Value;              // Expression: -1
                                       //  Referenced by: '<S50>/Constant1'

};

// Parameters for system: '<S91>/CoreSubsys'
struct P_CoreSubsys_ctl_controller0_T_ {
  real32_T uHzLowPass_NumCoef[2];      // Expression: [omega*T/(2+omega*T) omega*T/(2+omega*T)]
                                       //  Referenced by: '<S91>/3 Hz Low Pass'

  real32_T uHzLowPass_DenCoef[2];      // Expression: [1 (omega*T-2)/(2+omega*T)]
                                       //  Referenced by: '<S91>/3 Hz Low Pass'

  real32_T uHzLowPass_InitialStates;   // Computed Parameter: uHzLowPass_InitialStates
                                       //  Referenced by: '<S91>/3 Hz Low Pass'

};

// Parameters for system: '<S5>/For Each Subsystem'
struct P_ForEachSubsystem_ctl_contro_T_ {
  P_CoreSubsys_ctl_controller0_T CoreSubsys;// '<S91>/CoreSubsys'
};

// Parameters (auto storage)
struct P_ctl_controller0_T_ {
  real32_T ctl_stopping_omega_thresh;  // Variable: ctl_stopping_omega_thresh
                                       //  Referenced by: '<S7>/Constant'

  real32_T ctl_stopping_vel_thresh;    // Variable: ctl_stopping_vel_thresh
                                       //  Referenced by: '<S6>/Constant'

  real32_T tun_accel_gain[3];          // Variable: tun_accel_gain
                                       //  Referenced by: '<S12>/Gain'

  real32_T tun_alpha_gain[3];          // Variable: tun_alpha_gain
                                       //  Referenced by: '<S13>/Gain'

  real32_T tun_ctl_att_sat_lower;      // Variable: tun_ctl_att_sat_lower
                                       //  Referenced by: '<S13>/Discrete-Time Integrator'

  real32_T tun_ctl_att_sat_upper;      // Variable: tun_ctl_att_sat_upper
                                       //  Referenced by: '<S13>/Discrete-Time Integrator'

  real32_T tun_ctl_linear_force_limit; // Variable: tun_ctl_linear_force_limit
                                       //  Referenced by: '<S12>/Constant2'

  real32_T tun_ctl_pos_sat_lower;      // Variable: tun_ctl_pos_sat_lower
                                       //  Referenced by: '<S12>/Discrete-Time Integrator1'

  real32_T tun_ctl_pos_sat_upper;      // Variable: tun_ctl_pos_sat_upper
                                       //  Referenced by: '<S12>/Discrete-Time Integrator1'

  real32_T tun_truth_q_omega_filt_enable;// Variable: tun_truth_q_omega_filt_enable
                                         //  Referenced by: '<S5>/For Each Subsystem'

  real32_T tun_truth_velocity_filt_enable;// Variable: tun_truth_velocity_filt_enable
                                          //  Referenced by: '<S5>/For Each Subsystem1'

  uint8_T ase_status_converged;        // Variable: ase_status_converged
                                       //  Referenced by: '<S9>/Constant'

  uint8_T ctl_idle_mode;               // Variable: ctl_idle_mode
                                       //  Referenced by: '<S2>/Constant4'

  uint8_T ctl_stopped_mode;            // Variable: ctl_stopped_mode
                                       //  Referenced by: '<S2>/Constant'

  uint8_T ctl_stopping_mode;           // Variable: ctl_stopping_mode
                                       //  Referenced by: '<S8>/Constant'

  uint8_T tun_debug_ctl_use_truth;     // Variable: tun_debug_ctl_use_truth
                                       //  Referenced by: '<S1>/Constant'

  real32_T DiscreteDerivative_ICPrevScaled;// Mask Parameter: DiscreteDerivative_ICPrevScaled
                                           //  Referenced by: '<S93>/UD'

  real32_T DiscreteDerivative_ICPrevScal_k;// Mask Parameter: DiscreteDerivative_ICPrevScal_k
                                           //  Referenced by: '<S90>/UD'

  uint8_T CompareToConstant2_const;    // Mask Parameter: CompareToConstant2_const
                                       //  Referenced by: '<S16>/Constant'

  uint8_T CompareToConstant1_const;    // Mask Parameter: CompareToConstant1_const
                                       //  Referenced by: '<S15>/Constant'

  uint8_T CompareToConstant_const;     // Mask Parameter: CompareToConstant_const
                                       //  Referenced by: '<S14>/Constant'

  uint8_T CompareToConstant2_const_o;  // Mask Parameter: CompareToConstant2_const_o
                                       //  Referenced by: '<S37>/Constant'

  uint8_T CompareToConstant1_const_m;  // Mask Parameter: CompareToConstant1_const_m
                                       //  Referenced by: '<S36>/Constant'

  uint8_T CompareToConstant_const_f;   // Mask Parameter: CompareToConstant_const_f
                                       //  Referenced by: '<S35>/Constant'

  real_T Constant2_Value[9];           // Expression: zeros(9,1)
                                       //  Referenced by: '<S94>/Constant2'

  real_T Constant3_Value;              // Expression: 0
                                       //  Referenced by: '<S96>/Constant3'

  real_T constant28_Value[50];         // Expression: [0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0]
                                       //  Referenced by: '<S5>/constant28'

  real_T constant29_Value[50];         // Expression: [0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0]
                                       //  Referenced by: '<S5>/constant29'

  real_T s1_Gain;                      // Expression: 0.0031317642291927056
                                       //  Referenced by: '<S10>/s(1)'

  real_T a21_Gain;                     // Expression: -0.993736471541614597
                                       //  Referenced by: '<S10>/a(2)(1)'

  real_T s1_Gain_b;                    // Expression: 0.0031317642291927056
                                       //  Referenced by: '<S11>/s(1)'

  real_T a21_Gain_l;                   // Expression: -0.993736471541614597
                                       //  Referenced by: '<S11>/a(2)(1)'

  real_T Constant2_Value_p[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S25>/Constant2'

  real_T Constant1_Value;              // Expression: 1
                                       //  Referenced by: '<S25>/Constant1'

  real_T Constant3_Value_o;            // Expression: 0
                                       //  Referenced by: '<S28>/Constant3'

  real_T Constant2_Value_k[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S20>/Constant2'

  real_T Constant1_Value_f;            // Expression: 1
                                       //  Referenced by: '<S20>/Constant1'

  real_T Constant3_Value_b;            // Expression: 0
                                       //  Referenced by: '<S23>/Constant3'

  real_T Constant2_Value_g[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S45>/Constant2'

  real_T Constant3_Value_a;            // Expression: 0
                                       //  Referenced by: '<S47>/Constant3'

  real_T Constant3_Value_c;            // Expression: 0
                                       //  Referenced by: '<S40>/Constant3'

  real_T Constant2_Value_i[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S78>/Constant2'

  real_T Constant3_Value_n;            // Expression: 0
                                       //  Referenced by: '<S80>/Constant3'

  real32_T Gain2_Gain;                 // Computed Parameter: Gain2_Gain
                                       //  Referenced by: '<S40>/Gain2'

  real32_T Gain1_Gain;                 // Computed Parameter: Gain1_Gain
                                       //  Referenced by: '<S40>/Gain1'

  real32_T Gain_Gain;                  // Computed Parameter: Gain_Gain
                                       //  Referenced by: '<S40>/Gain'

  real32_T Gain_Gain_p;                // Computed Parameter: Gain_Gain_p
                                       //  Referenced by: '<S96>/Gain'

  real32_T Gain1_Gain_e;               // Computed Parameter: Gain1_Gain_e
                                       //  Referenced by: '<S96>/Gain1'

  real32_T Gain2_Gain_k;               // Computed Parameter: Gain2_Gain_k
                                       //  Referenced by: '<S96>/Gain2'

  real32_T Gain1_Gain_o;               // Computed Parameter: Gain1_Gain_o
                                       //  Referenced by: '<S94>/Gain1'

  real32_T Gain_Gain_m;                // Computed Parameter: Gain_Gain_m
                                       //  Referenced by: '<S89>/Gain'

  real32_T TSamp_WtEt;                 // Computed Parameter: TSamp_WtEt
                                       //  Referenced by: '<S93>/TSamp'

  real32_T constant34_Value[3];        // Computed Parameter: constant34_Value
                                       //  Referenced by: '<S5>/constant34'

  real32_T TSamp_WtEt_k;               // Computed Parameter: TSamp_WtEt_k
                                       //  Referenced by: '<S90>/TSamp'

  real32_T constant36_Value[3];        // Computed Parameter: constant36_Value
                                       //  Referenced by: '<S5>/constant36'

  real32_T constant37_Value[3];        // Computed Parameter: constant37_Value
                                       //  Referenced by: '<S5>/constant37'

  real32_T constant4_Value[4];         // Computed Parameter: constant4_Value
                                       //  Referenced by: '<S5>/constant4'

  real32_T constant7_Value[3];         // Computed Parameter: constant7_Value
                                       //  Referenced by: '<S5>/constant7'

  real32_T constant22_Value[64];       // Computed Parameter: constant22_Value
                                       //  Referenced by: '<S5>/constant22'

  real32_T constant23_Value[48];       // Computed Parameter: constant23_Value
                                       //  Referenced by: '<S5>/constant23'

  ase_cov_datatype constant24_Value[117];// Computed Parameter: constant24_Value
                                         //  Referenced by: '<S5>/constant24'

  real32_T constant31_Value[3];        // Computed Parameter: constant31_Value
                                       //  Referenced by: '<S5>/constant31'

  real32_T constant32_Value[4];        // Computed Parameter: constant32_Value
                                       //  Referenced by: '<S5>/constant32'

  real32_T constant33_Value[3];        // Computed Parameter: constant33_Value
                                       //  Referenced by: '<S5>/constant33'

  real32_T Delay11_InitialCondition;   // Computed Parameter: Delay11_InitialCondition
                                       //  Referenced by: '<S10>/Delay11'

  real32_T Delay11_InitialCondition_k; // Computed Parameter: Delay11_InitialCondition_k
                                       //  Referenced by: '<S11>/Delay11'

  real32_T Constant_Value;             // Expression: single(0.5)
                                       //  Referenced by: '<S58>/Constant'

  real32_T Gain_Gain_h;                // Expression: single(1e-9)
                                       //  Referenced by: '<S56>/Gain'

  real32_T Constant1_Value_g;          // Computed Parameter: Constant1_Value_g
                                       //  Referenced by: '<S57>/Constant1'

  real32_T Constant3_Value_d;          // Computed Parameter: Constant3_Value_d
                                       //  Referenced by: '<S57>/Constant3'

  real32_T Constant3_Value_dz;         // Computed Parameter: Constant3_Value_dz
                                       //  Referenced by: '<S62>/Constant3'

  real32_T Gain_Gain_n;                // Computed Parameter: Gain_Gain_n
                                       //  Referenced by: '<S62>/Gain'

  real32_T Gain1_Gain_b;               // Computed Parameter: Gain1_Gain_b
                                       //  Referenced by: '<S62>/Gain1'

  real32_T Constant2_Value_d;          // Computed Parameter: Constant2_Value_d
                                       //  Referenced by: '<S62>/Constant2'

  real32_T Gain2_Gain_kx;              // Computed Parameter: Gain2_Gain_kx
                                       //  Referenced by: '<S62>/Gain2'

  real32_T Gain3_Gain;                 // Computed Parameter: Gain3_Gain
                                       //  Referenced by: '<S62>/Gain3'

  real32_T Gain4_Gain;                 // Computed Parameter: Gain4_Gain
                                       //  Referenced by: '<S62>/Gain4'

  real32_T Constant1_Value_l;          // Computed Parameter: Constant1_Value_l
                                       //  Referenced by: '<S62>/Constant1'

  real32_T Gain5_Gain;                 // Computed Parameter: Gain5_Gain
                                       //  Referenced by: '<S62>/Gain5'

  real32_T Constant_Value_g;           // Computed Parameter: Constant_Value_g
                                       //  Referenced by: '<S62>/Constant'

  real32_T Constant3_Value_f;          // Computed Parameter: Constant3_Value_f
                                       //  Referenced by: '<S61>/Constant3'

  real32_T Gain_Gain_g;                // Computed Parameter: Gain_Gain_g
                                       //  Referenced by: '<S61>/Gain'

  real32_T Gain1_Gain_j;               // Computed Parameter: Gain1_Gain_j
                                       //  Referenced by: '<S61>/Gain1'

  real32_T Constant2_Value_gq;         // Computed Parameter: Constant2_Value_gq
                                       //  Referenced by: '<S61>/Constant2'

  real32_T Gain2_Gain_c;               // Computed Parameter: Gain2_Gain_c
                                       //  Referenced by: '<S61>/Gain2'

  real32_T Gain3_Gain_o;               // Computed Parameter: Gain3_Gain_o
                                       //  Referenced by: '<S61>/Gain3'

  real32_T Gain4_Gain_c;               // Computed Parameter: Gain4_Gain_c
                                       //  Referenced by: '<S61>/Gain4'

  real32_T Constant1_Value_gk;         // Computed Parameter: Constant1_Value_gk
                                       //  Referenced by: '<S61>/Constant1'

  real32_T Gain5_Gain_a;               // Computed Parameter: Gain5_Gain_a
                                       //  Referenced by: '<S61>/Gain5'

  real32_T Constant_Value_m;           // Computed Parameter: Constant_Value_m
                                       //  Referenced by: '<S61>/Constant'

  real32_T Constant2_Value_it;         // Computed Parameter: Constant2_Value_it
                                       //  Referenced by: '<S57>/Constant2'

  real32_T Constant1_Value_a[3];       // Expression: single([0 0 0])
                                       //  Referenced by: '<S2>/Constant1'

  real32_T Constant2_Value_n[3];       // Expression: single([0 0 0])
                                       //  Referenced by: '<S2>/Constant2'

  real32_T Constant3_Value_j[3];       // Expression: single([0 0 0])
                                       //  Referenced by: '<S2>/Constant3'

  real32_T Constant5_Value[3];         // Expression: single([0 0 0])
                                       //  Referenced by: '<S2>/Constant5'

  real32_T UnitDelay1_InitialCondition;// Computed Parameter: UnitDelay1_InitialCondition
                                       //  Referenced by: '<S2>/Unit Delay1'

  real32_T UnitDelay2_InitialCondition;// Computed Parameter: UnitDelay2_InitialCondition
                                       //  Referenced by: '<S2>/Unit Delay2'

  real32_T Gain_Gain_l;                // Computed Parameter: Gain_Gain_l
                                       //  Referenced by: '<S25>/Gain'

  real32_T Gain1_Gain_ej;              // Computed Parameter: Gain1_Gain_ej
                                       //  Referenced by: '<S25>/Gain1'

  real32_T Gain_Gain_i;                // Computed Parameter: Gain_Gain_i
                                       //  Referenced by: '<S28>/Gain'

  real32_T Gain1_Gain_k;               // Computed Parameter: Gain1_Gain_k
                                       //  Referenced by: '<S28>/Gain1'

  real32_T Gain2_Gain_b;               // Computed Parameter: Gain2_Gain_b
                                       //  Referenced by: '<S28>/Gain2'

  real32_T Gain2_Gain_k1;              // Computed Parameter: Gain2_Gain_k1
                                       //  Referenced by: '<S25>/Gain2'

  real32_T Gain_Gain_b;                // Computed Parameter: Gain_Gain_b
                                       //  Referenced by: '<S20>/Gain'

  real32_T Gain1_Gain_g;               // Computed Parameter: Gain1_Gain_g
                                       //  Referenced by: '<S20>/Gain1'

  real32_T Gain_Gain_d;                // Computed Parameter: Gain_Gain_d
                                       //  Referenced by: '<S23>/Gain'

  real32_T Gain1_Gain_ji;              // Computed Parameter: Gain1_Gain_ji
                                       //  Referenced by: '<S23>/Gain1'

  real32_T Gain2_Gain_bs;              // Computed Parameter: Gain2_Gain_bs
                                       //  Referenced by: '<S23>/Gain2'

  real32_T Gain2_Gain_kh;              // Computed Parameter: Gain2_Gain_kh
                                       //  Referenced by: '<S20>/Gain2'

  real32_T Constant4_Value[3];         // Expression: single([0 0 0])
                                       //  Referenced by: '<S12>/Constant4'

  real32_T Constant1_Value_k[3];       // Expression: single([0 0 0])
                                       //  Referenced by: '<S12>/Constant1'

  real32_T DiscreteTimeIntegrator1_gainval;// Computed Parameter: DiscreteTimeIntegrator1_gainval
                                           //  Referenced by: '<S12>/Discrete-Time Integrator1'

  real32_T DiscreteTimeIntegrator1_IC[3];// Expression: single([0 0 0])
                                         //  Referenced by: '<S12>/Discrete-Time Integrator1'

  real32_T Constant3_Value_e[3];       // Expression: single([0 0 0])
                                       //  Referenced by: '<S12>/Constant3'

  real32_T Constant2_Value_o[3];       // Expression: single([0 0 0])
                                       //  Referenced by: '<S13>/Constant2'

  real32_T Constant1_Value_o[3];       // Expression: single([0 0 0])
                                       //  Referenced by: '<S13>/Constant1'

  real32_T Gain_Gain_pf;               // Computed Parameter: Gain_Gain_pf
                                       //  Referenced by: '<S42>/Gain'

  real32_T Gain_Gain_k;                // Computed Parameter: Gain_Gain_k
                                       //  Referenced by: '<S47>/Gain'

  real32_T Gain1_Gain_jz;              // Computed Parameter: Gain1_Gain_jz
                                       //  Referenced by: '<S47>/Gain1'

  real32_T Gain2_Gain_n;               // Computed Parameter: Gain2_Gain_n
                                       //  Referenced by: '<S47>/Gain2'

  real32_T Gain1_Gain_od;              // Computed Parameter: Gain1_Gain_od
                                       //  Referenced by: '<S45>/Gain1'

  real32_T DiscreteTimeIntegrator_gainval;// Computed Parameter: DiscreteTimeIntegrator_gainval
                                          //  Referenced by: '<S13>/Discrete-Time Integrator'

  real32_T DiscreteTimeIntegrator_IC[3];// Expression: single([0 0 0])
                                        //  Referenced by: '<S13>/Discrete-Time Integrator'

  real32_T Constant4_Value_f[3];       // Expression: single([0 0 0])
                                       //  Referenced by: '<S13>/Constant4'

  real32_T Gain2_Gain_j;               // Computed Parameter: Gain2_Gain_j
                                       //  Referenced by: '<S13>/Gain2'

  real32_T Gain_Gain_is;               // Computed Parameter: Gain_Gain_is
                                       //  Referenced by: '<S75>/Gain'

  real32_T Gain_Gain_m2;               // Computed Parameter: Gain_Gain_m2
                                       //  Referenced by: '<S80>/Gain'

  real32_T Gain1_Gain_bs;              // Computed Parameter: Gain1_Gain_bs
                                       //  Referenced by: '<S80>/Gain1'

  real32_T Gain2_Gain_o;               // Computed Parameter: Gain2_Gain_o
                                       //  Referenced by: '<S80>/Gain2'

  real32_T Gain1_Gain_ed;              // Computed Parameter: Gain1_Gain_ed
                                       //  Referenced by: '<S78>/Gain1'

  real32_T Gain_Gain_hg;               // Computed Parameter: Gain_Gain_hg
                                       //  Referenced by: '<S59>/Gain'

  uint32_T constant40_Value;           // Computed Parameter: constant40_Value
                                       //  Referenced by: '<S5>/constant40'

  uint32_T Delay11_DelayLength;        // Computed Parameter: Delay11_DelayLength
                                       //  Referenced by: '<S10>/Delay11'

  uint32_T Delay11_DelayLength_g;      // Computed Parameter: Delay11_DelayLength_g
                                       //  Referenced by: '<S11>/Delay11'

  uint16_T constant25_Value;           // Computed Parameter: constant25_Value
                                       //  Referenced by: '<S5>/constant25'

  uint8_T constant39_Value;            // Computed Parameter: constant39_Value
                                       //  Referenced by: '<S5>/constant39'

  uint8_T constant26_Value;            // Computed Parameter: constant26_Value
                                       //  Referenced by: '<S5>/constant26'

  uint8_T constant27_Value;            // Computed Parameter: constant27_Value
                                       //  Referenced by: '<S5>/constant27'

  P_ForEachSubsystem_ctl_contro_T ForEachSubsystem1;// '<S5>/For Each Subsystem1' 
  P_ForEachSubsystem_ctl_contro_T ForEachSubsystem;// '<S5>/For Each Subsystem'
  P_Normalize_ctl_controller0_T Normalize_k;// '<S77>/Normalize'
  P_Normalize_ctl_controller0_T Normalize;// '<S63>/Normalize'
  P_Normalize_ctl_controller0_T Normalize_l;// '<S44>/Normalize'
};

// Real-time Model Data Structure
struct tag_RTM_ctl_controller0_T {
  const char_T * volatile errorStatus;
  P_ctl_controller0_T *defaultParam;
  boolean_T paramIsMalloced;
  DW_ctl_controller0_T *dwork;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// External data declarations for dependent source files
extern const kfl_msg ctl_controller0_rtZkfl_msg;// kfl_msg ground
extern const cmc_msg ctl_controller0_rtZcmc_msg;// cmc_msg ground
extern const ex_time_msg ctl_controller0_rtZex_time_msg;// ex_time_msg ground
extern const env_msg ctl_controller0_rtZenv_msg;// env_msg ground
extern const cmd_msg ctl_controller0_rtZcmd_msg;// cmd_msg ground
extern const ctl_msg ctl_controller0_rtZctl_msg;// ctl_msg ground

#ifdef __cplusplus

extern "C" {

#endif

  extern const char *RT_MEMORY_ALLOCATION_ERROR;

#ifdef __cplusplus

}
#endif

extern P_ctl_controller0_T ctl_controller0_P;// parameters

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern RT_MODEL_ctl_controller0_T *ctl_controller0(kfl_msg
    *ctl_controller0_U_kfl_msg_l, cmc_msg *ctl_controller0_U_cmc_msg_f,
    ex_time_msg *ctl_controller0_U_ex_time, env_msg *ctl_controller0_U_env_msg_h,
    cmd_msg *ctl_controller0_Y_cmd_msg_c, ctl_msg *ctl_controller0_Y_ctl_msg_o);
  extern void ctl_controller0_initialize(RT_MODEL_ctl_controller0_T *const
    ctl_controller0_M, kfl_msg *ctl_controller0_U_kfl_msg_l, cmc_msg
    *ctl_controller0_U_cmc_msg_f, ex_time_msg *ctl_controller0_U_ex_time,
    env_msg *ctl_controller0_U_env_msg_h, cmd_msg *ctl_controller0_Y_cmd_msg_c,
    ctl_msg *ctl_controller0_Y_ctl_msg_o);
  extern void ctl_controller0_step(RT_MODEL_ctl_controller0_T *const
    ctl_controller0_M, kfl_msg *ctl_controller0_U_kfl_msg_l, cmc_msg
    *ctl_controller0_U_cmc_msg_f, ex_time_msg *ctl_controller0_U_ex_time,
    env_msg *ctl_controller0_U_env_msg_h, cmd_msg *ctl_controller0_Y_cmd_msg_c,
    ctl_msg *ctl_controller0_Y_ctl_msg_o);
  extern void ctl_controller0_terminate(RT_MODEL_ctl_controller0_T
    * ctl_controller0_M);

#ifdef __cplusplus

}
#endif

//-
//  The generated code includes comments that allow you to trace directly
//  back to the appropriate location in the model.  The basic format
//  is <system>/block_name, where system is the system number (uniquely
//  assigned by Simulink) and block_name is the name of the block.
//
//  Note that this particular code originates from a subsystem build,
//  and has its own system numbers different from the parent model.
//  Refer to the system hierarchy for this subsystem below, and use the
//  MATLAB hilite_system command to trace the generated code back
//  to the parent model.  For example,
//
//  hilite_system('astrobee/fsw_lib/ctl_controller')    - opens subsystem astrobee/fsw_lib/ctl_controller
//  hilite_system('astrobee/fsw_lib/ctl_controller/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'astrobee/fsw_lib'
//  '<S1>'   : 'astrobee/fsw_lib/ctl_controller'
//  '<S2>'   : 'astrobee/fsw_lib/ctl_controller/cex_control_executive'
//  '<S3>'   : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib'
//  '<S4>'   : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper'
//  '<S5>'   : 'astrobee/fsw_lib/ctl_controller/debug_truth_builder'
//  '<S6>'   : 'astrobee/fsw_lib/ctl_controller/cex_control_executive/Compare To Constant1'
//  '<S7>'   : 'astrobee/fsw_lib/ctl_controller/cex_control_executive/Compare To Constant2'
//  '<S8>'   : 'astrobee/fsw_lib/ctl_controller/cex_control_executive/Compare To Constant3'
//  '<S9>'   : 'astrobee/fsw_lib/ctl_controller/cex_control_executive/Compare To Constant5'
//  '<S10>'  : 'astrobee/fsw_lib/ctl_controller/cex_control_executive/Filter'
//  '<S11>'  : 'astrobee/fsw_lib/ctl_controller/cex_control_executive/Filter1'
//  '<S12>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control'
//  '<S13>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control'
//  '<S14>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/Compare To Constant'
//  '<S15>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/Compare To Constant1'
//  '<S16>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/Compare To Constant2'
//  '<S17>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/rotate_vec_A2B'
//  '<S18>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/rotate_vec_A2B1'
//  '<S19>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/saturate_vector_mag'
//  '<S20>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/rotate_vec_A2B/quaternion_to_DCM'
//  '<S21>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/rotate_vec_A2B/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S22>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/rotate_vec_A2B/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S23>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/rotate_vec_A2B/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S24>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/rotate_vec_A2B/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S25>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/rotate_vec_A2B1/quaternion_to_DCM'
//  '<S26>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/rotate_vec_A2B1/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S27>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/rotate_vec_A2B1/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S28>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/rotate_vec_A2B1/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S29>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/rotate_vec_A2B1/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S30>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/saturate_vector_mag/vector_magnitude1'
//  '<S31>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/saturate_vector_mag/vector_normalize'
//  '<S32>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/saturate_vector_mag/vector_normalize/No-op'
//  '<S33>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/saturate_vector_mag/vector_normalize/Normalize'
//  '<S34>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/linear control/saturate_vector_mag/vector_normalize/vector_magnitude'
//  '<S35>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/Compare To Constant'
//  '<S36>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/Compare To Constant1'
//  '<S37>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/Compare To Constant2'
//  '<S38>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/cross_product'
//  '<S39>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/quaternion_error1'
//  '<S40>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/cross_product/skew_symetric_matrix_operator'
//  '<S41>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/cross_product/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S42>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/quaternion_error1/Quaternion Inverse'
//  '<S43>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/quaternion_error1/Quaternion_Multiplication'
//  '<S44>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/quaternion_error1/quat_normalize_and_enforce_positive_scalar'
//  '<S45>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/quaternion_error1/Quaternion_Multiplication/Quaternion Xi'
//  '<S46>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/quaternion_error1/Quaternion_Multiplication/Quaternion Xi/Data Type Conversion Inherited'
//  '<S47>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/quaternion_error1/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator'
//  '<S48>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/quaternion_error1/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S49>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/quaternion_error1/quat_normalize_and_enforce_positive_scalar/No-op'
//  '<S50>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/quaternion_error1/quat_normalize_and_enforce_positive_scalar/Normalize'
//  '<S51>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/quaternion_error1/quat_normalize_and_enforce_positive_scalar/vector_normalize'
//  '<S52>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/quaternion_error1/quat_normalize_and_enforce_positive_scalar/Normalize/Data Type Conversion Inherited'
//  '<S53>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/quaternion_error1/quat_normalize_and_enforce_positive_scalar/vector_normalize/No-op'
//  '<S54>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/quaternion_error1/quat_normalize_and_enforce_positive_scalar/vector_normalize/Normalize'
//  '<S55>'  : 'astrobee/fsw_lib/ctl_controller/clc_closed_loop_controller_lib/rotational control/quaternion_error1/quat_normalize_and_enforce_positive_scalar/vector_normalize/vector_magnitude'
//  '<S56>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/cmd_selector'
//  '<S57>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/generate_cmd_attitude'
//  '<S58>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/generate_cmd_path'
//  '<S59>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors'
//  '<S60>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/generate_cmd_attitude/MATLAB Function'
//  '<S61>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/generate_cmd_attitude/create_omega_matrix'
//  '<S62>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/generate_cmd_attitude/create_omega_matrix1'
//  '<S63>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/generate_cmd_attitude/quat_normalize_and_enforce_positive_scalar'
//  '<S64>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/generate_cmd_attitude/quat_normalize_and_enforce_positive_scalar/No-op'
//  '<S65>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/generate_cmd_attitude/quat_normalize_and_enforce_positive_scalar/Normalize'
//  '<S66>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/generate_cmd_attitude/quat_normalize_and_enforce_positive_scalar/vector_normalize'
//  '<S67>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/generate_cmd_attitude/quat_normalize_and_enforce_positive_scalar/Normalize/Data Type Conversion Inherited'
//  '<S68>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/generate_cmd_attitude/quat_normalize_and_enforce_positive_scalar/vector_normalize/No-op'
//  '<S69>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/generate_cmd_attitude/quat_normalize_and_enforce_positive_scalar/vector_normalize/Normalize'
//  '<S70>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/generate_cmd_attitude/quat_normalize_and_enforce_positive_scalar/vector_normalize/vector_magnitude'
//  '<S71>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/quaternion_error1'
//  '<S72>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/vector_magnitude'
//  '<S73>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/vector_magnitude1'
//  '<S74>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/vector_magnitude2'
//  '<S75>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/quaternion_error1/Quaternion Inverse'
//  '<S76>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/quaternion_error1/Quaternion_Multiplication'
//  '<S77>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/quaternion_error1/quat_normalize_and_enforce_positive_scalar'
//  '<S78>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/quaternion_error1/Quaternion_Multiplication/Quaternion Xi'
//  '<S79>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/quaternion_error1/Quaternion_Multiplication/Quaternion Xi/Data Type Conversion Inherited'
//  '<S80>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/quaternion_error1/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator'
//  '<S81>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/quaternion_error1/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S82>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/quaternion_error1/quat_normalize_and_enforce_positive_scalar/No-op'
//  '<S83>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/quaternion_error1/quat_normalize_and_enforce_positive_scalar/Normalize'
//  '<S84>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/quaternion_error1/quat_normalize_and_enforce_positive_scalar/vector_normalize'
//  '<S85>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/quaternion_error1/quat_normalize_and_enforce_positive_scalar/Normalize/Data Type Conversion Inherited'
//  '<S86>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/quaternion_error1/quat_normalize_and_enforce_positive_scalar/vector_normalize/No-op'
//  '<S87>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/quaternion_error1/quat_normalize_and_enforce_positive_scalar/vector_normalize/Normalize'
//  '<S88>'  : 'astrobee/fsw_lib/ctl_controller/cmd_command_shaper/traj_errors/quaternion_error1/quat_normalize_and_enforce_positive_scalar/vector_normalize/vector_magnitude'
//  '<S89>'  : 'astrobee/fsw_lib/ctl_controller/debug_truth_builder/Compute Rates From Quat'
//  '<S90>'  : 'astrobee/fsw_lib/ctl_controller/debug_truth_builder/Discrete Derivative'
//  '<S91>'  : 'astrobee/fsw_lib/ctl_controller/debug_truth_builder/For Each Subsystem'
//  '<S92>'  : 'astrobee/fsw_lib/ctl_controller/debug_truth_builder/For Each Subsystem1'
//  '<S93>'  : 'astrobee/fsw_lib/ctl_controller/debug_truth_builder/Compute Rates From Quat/Discrete Derivative'
//  '<S94>'  : 'astrobee/fsw_lib/ctl_controller/debug_truth_builder/Compute Rates From Quat/Quaternion Xi'
//  '<S95>'  : 'astrobee/fsw_lib/ctl_controller/debug_truth_builder/Compute Rates From Quat/Quaternion Xi/Data Type Conversion Inherited'
//  '<S96>'  : 'astrobee/fsw_lib/ctl_controller/debug_truth_builder/Compute Rates From Quat/Quaternion Xi/skew_symetric_matrix_operator'
//  '<S97>'  : 'astrobee/fsw_lib/ctl_controller/debug_truth_builder/Compute Rates From Quat/Quaternion Xi/skew_symetric_matrix_operator/Data Type Conversion Inherited'

#endif                                 // RTW_HEADER_ctl_controller0_h_

//
// File trailer for generated code.
//
// [EOF]
//
