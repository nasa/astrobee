//
// File: ctl_controller0_data.cpp
//
// Code generated for Simulink model 'ctl_controller0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed Jan 31 12:32:36 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "ctl_controller0.h"
#include "ctl_controller0_private.h"

// Block parameters (auto storage)
P_ctl_controller0_T ctl_controller0_P = {
  0.0001F,                             // Variable: ctl_stopping_omega_thresh
                                       //  Referenced by: '<S7>/Constant'

  0.0001F,                             // Variable: ctl_stopping_vel_thresh
                                       //  Referenced by: '<S6>/Constant'


  //  Variable: tun_accel_gain
  //  Referenced by: '<S30>/Gain'

  { 1.0F, 1.0F, 1.0F },

  //  Variable: tun_alpha_gain
  //  Referenced by: '<S31>/Gain'

  { 1.0F, 1.0F, 1.0F },
  -0.5F,                               // Variable: tun_ctl_att_sat_lower
                                       //  Referenced by: '<S31>/Discrete-Time Integrator'

  0.5F,                                // Variable: tun_ctl_att_sat_upper
                                       //  Referenced by: '<S31>/Discrete-Time Integrator'

  100.0F,                              // Variable: tun_ctl_linear_force_limit
                                       //  Referenced by: '<S30>/Constant2'

  -0.1F,                               // Variable: tun_ctl_pos_sat_lower
                                       //  Referenced by: '<S30>/Discrete-Time Integrator1'

  0.1F,                                // Variable: tun_ctl_pos_sat_upper
                                       //  Referenced by: '<S30>/Discrete-Time Integrator1'

  0.1F,                                // Variable: tun_ctl_stopped_pos_thresh
                                       //  Referenced by: '<S11>/Constant'

  0.174533F,                           // Variable: tun_ctl_stopped_quat_thresh
                                       //  Referenced by: '<S12>/Constant'

  0.0F,                                // Variable: tun_truth_q_omega_filt_enable
                                       //  Referenced by: '<S5>/For Each Subsystem'

  0.0F,                                // Variable: tun_truth_velocity_filt_enable
                                       //  Referenced by: '<S5>/For Each Subsystem1'

  0U,                                  // Variable: ase_status_converged
                                       //  Referenced by: '<S10>/Constant'

  0U,                                  // Variable: ctl_idle_mode
                                       //  Referenced by: '<S2>/Constant4'

  3U,                                  // Variable: ctl_stopped_mode
                                       //  Referenced by:
                                       //    '<S2>/Constant'
                                       //    '<S9>/Constant'

  1U,                                  // Variable: ctl_stopping_mode
                                       //  Referenced by:
                                       //    '<S2>/Constant6'
                                       //    '<S8>/Constant'

  0U,                                  // Variable: tun_debug_ctl_use_truth
                                       //  Referenced by: '<S1>/Constant'

  0.0F,                                // Mask Parameter: DiscreteDerivative_ICPrevScaled
                                       //  Referenced by: '<S115>/UD'

  0.0F,                                // Mask Parameter: DiscreteDerivative_ICPrevScal_k
                                       //  Referenced by: '<S112>/UD'

  1U,                                  // Mask Parameter: CompareToConstant2_const
                                       //  Referenced by: '<S34>/Constant'

  1U,                                  // Mask Parameter: CompareToConstant1_const
                                       //  Referenced by: '<S33>/Constant'

  1U,                                  // Mask Parameter: CompareToConstant_const
                                       //  Referenced by: '<S32>/Constant'

  1U,                                  // Mask Parameter: CompareToConstant2_const_o
                                       //  Referenced by: '<S57>/Constant'

  1U,                                  // Mask Parameter: CompareToConstant1_const_m
                                       //  Referenced by: '<S56>/Constant'

  1U,                                  // Mask Parameter: CompareToConstant_const_f
                                       //  Referenced by: '<S55>/Constant'


  //  Expression: zeros(9,1)
  //  Referenced by: '<S116>/Constant2'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S118>/Constant3'


  //  Expression: zeros(9,1)
  //  Referenced by: '<S19>/Constant2'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S21>/Constant3'

  0.0031317642291927056,               // Expression: 0.0031317642291927056
                                       //  Referenced by: '<S13>/s(1)'

  -0.9937364715416146,                 // Expression: -0.993736471541614597
                                       //  Referenced by: '<S13>/a(2)(1)'

  0.0031317642291927056,               // Expression: 0.0031317642291927056
                                       //  Referenced by: '<S14>/s(1)'

  -0.9937364715416146,                 // Expression: -0.993736471541614597
                                       //  Referenced by: '<S14>/a(2)(1)'


  //  Expression: zeros(9,1)
  //  Referenced by: '<S45>/Constant2'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
  1.0,                                 // Expression: 1
                                       //  Referenced by: '<S45>/Constant1'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S48>/Constant3'


  //  Expression: zeros(9,1)
  //  Referenced by: '<S40>/Constant2'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
  1.0,                                 // Expression: 1
                                       //  Referenced by: '<S40>/Constant1'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S43>/Constant3'


  //  Expression: zeros(9,1)
  //  Referenced by: '<S67>/Constant2'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S69>/Constant3'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S62>/Constant3'


  //  Expression: zeros(9,1)
  //  Referenced by: '<S100>/Constant2'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S102>/Constant3'

  -1.0F,                               // Computed Parameter: Gain2_Gain
                                       //  Referenced by: '<S62>/Gain2'

  -1.0F,                               // Computed Parameter: Gain1_Gain
                                       //  Referenced by: '<S62>/Gain1'

  -1.0F,                               // Computed Parameter: Gain_Gain
                                       //  Referenced by: '<S62>/Gain'

  -1.0F,                               // Computed Parameter: Gain_Gain_p
                                       //  Referenced by: '<S118>/Gain'

  -1.0F,                               // Computed Parameter: Gain1_Gain_e
                                       //  Referenced by: '<S118>/Gain1'

  -1.0F,                               // Computed Parameter: Gain2_Gain_k
                                       //  Referenced by: '<S118>/Gain2'

  -1.0F,                               // Computed Parameter: Gain1_Gain_o
                                       //  Referenced by: '<S116>/Gain1'

  2.0F,                                // Computed Parameter: Gain_Gain_m
                                       //  Referenced by: '<S111>/Gain'

  62.5F,                               // Computed Parameter: TSamp_WtEt
                                       //  Referenced by: '<S115>/TSamp'

  62.5F,                               // Computed Parameter: TSamp_WtEt_k
                                       //  Referenced by: '<S112>/TSamp'

  0.0F,                                // Computed Parameter: UnitDelay2_InitialCondition
                                       //  Referenced by: '<S2>/Unit Delay2'

  -1.0F,                               // Computed Parameter: Gain_Gain_a
                                       //  Referenced by: '<S16>/Gain'

  -1.0F,                               // Computed Parameter: Gain_Gain_i
                                       //  Referenced by: '<S21>/Gain'

  -1.0F,                               // Computed Parameter: Gain1_Gain_n
                                       //  Referenced by: '<S21>/Gain1'

  -1.0F,                               // Computed Parameter: Gain2_Gain_a
                                       //  Referenced by: '<S21>/Gain2'

  -1.0F,                               // Computed Parameter: Gain1_Gain_j
                                       //  Referenced by: '<S19>/Gain1'

  2.0F,                                // Computed Parameter: Gain2_Gain_h
                                       //  Referenced by: '<S2>/Gain2'

  0.0F,                                // Computed Parameter: Delay11_InitialCondition
                                       //  Referenced by: '<S13>/Delay11'

  0.0F,                                // Computed Parameter: Delay11_InitialCondition_k
                                       //  Referenced by: '<S14>/Delay11'

  0.5F,                                // Expression: single(0.5)
                                       //  Referenced by: '<S80>/Constant'

  1.0E-9F,                             // Expression: single(1e-9)
                                       //  Referenced by: '<S78>/Gain'

  0.5F,                                // Computed Parameter: Constant1_Value_g
                                       //  Referenced by: '<S79>/Constant1'

  0.5F,                                // Computed Parameter: Constant3_Value_d
                                       //  Referenced by: '<S79>/Constant3'

  0.0F,                                // Computed Parameter: Constant3_Value_dz
                                       //  Referenced by: '<S84>/Constant3'

  -1.0F,                               // Computed Parameter: Gain_Gain_n
                                       //  Referenced by: '<S84>/Gain'

  -1.0F,                               // Computed Parameter: Gain1_Gain_b
                                       //  Referenced by: '<S84>/Gain1'

  0.0F,                                // Computed Parameter: Constant2_Value_d
                                       //  Referenced by: '<S84>/Constant2'

  -1.0F,                               // Computed Parameter: Gain2_Gain_kx
                                       //  Referenced by: '<S84>/Gain2'

  -1.0F,                               // Computed Parameter: Gain3_Gain
                                       //  Referenced by: '<S84>/Gain3'

  -1.0F,                               // Computed Parameter: Gain4_Gain
                                       //  Referenced by: '<S84>/Gain4'

  0.0F,                                // Computed Parameter: Constant1_Value_l
                                       //  Referenced by: '<S84>/Constant1'

  -1.0F,                               // Computed Parameter: Gain5_Gain
                                       //  Referenced by: '<S84>/Gain5'

  0.0F,                                // Computed Parameter: Constant_Value_g
                                       //  Referenced by: '<S84>/Constant'

  0.0F,                                // Computed Parameter: Constant3_Value_f
                                       //  Referenced by: '<S83>/Constant3'

  -1.0F,                               // Computed Parameter: Gain_Gain_g
                                       //  Referenced by: '<S83>/Gain'

  -1.0F,                               // Computed Parameter: Gain1_Gain_ja
                                       //  Referenced by: '<S83>/Gain1'

  0.0F,                                // Computed Parameter: Constant2_Value_gq
                                       //  Referenced by: '<S83>/Constant2'

  -1.0F,                               // Computed Parameter: Gain2_Gain_c
                                       //  Referenced by: '<S83>/Gain2'

  -1.0F,                               // Computed Parameter: Gain3_Gain_o
                                       //  Referenced by: '<S83>/Gain3'

  -1.0F,                               // Computed Parameter: Gain4_Gain_c
                                       //  Referenced by: '<S83>/Gain4'

  0.0F,                                // Computed Parameter: Constant1_Value_gk
                                       //  Referenced by: '<S83>/Constant1'

  -1.0F,                               // Computed Parameter: Gain5_Gain_a
                                       //  Referenced by: '<S83>/Gain5'

  0.0F,                                // Computed Parameter: Constant_Value_m
                                       //  Referenced by: '<S83>/Constant'

  48.0F,                               // Computed Parameter: Constant2_Value_it
                                       //  Referenced by: '<S79>/Constant2'

  0.0F,                                // Computed Parameter: UnitDelay1_InitialCondition
                                       //  Referenced by: '<S2>/Unit Delay1'


  //  Expression: single([0 0 0])
  //  Referenced by: '<S2>/Constant1'

  { 0.0F, 0.0F, 0.0F },

  //  Expression: single([0 0 0])
  //  Referenced by: '<S2>/Constant2'

  { 0.0F, 0.0F, 0.0F },

  //  Expression: single([0 0 0])
  //  Referenced by: '<S2>/Constant3'

  { 0.0F, 0.0F, 0.0F },

  //  Expression: single([0 0 0])
  //  Referenced by: '<S2>/Constant5'

  { 0.0F, 0.0F, 0.0F },
  2.0F,                                // Computed Parameter: Gain_Gain_l
                                       //  Referenced by: '<S45>/Gain'

  2.0F,                                // Computed Parameter: Gain1_Gain_ej
                                       //  Referenced by: '<S45>/Gain1'

  -1.0F,                               // Computed Parameter: Gain_Gain_ib
                                       //  Referenced by: '<S48>/Gain'

  -1.0F,                               // Computed Parameter: Gain1_Gain_k
                                       //  Referenced by: '<S48>/Gain1'

  -1.0F,                               // Computed Parameter: Gain2_Gain_b
                                       //  Referenced by: '<S48>/Gain2'

  2.0F,                                // Computed Parameter: Gain2_Gain_k1
                                       //  Referenced by: '<S45>/Gain2'

  2.0F,                                // Computed Parameter: Gain_Gain_b
                                       //  Referenced by: '<S40>/Gain'

  2.0F,                                // Computed Parameter: Gain1_Gain_g
                                       //  Referenced by: '<S40>/Gain1'

  -1.0F,                               // Computed Parameter: Gain_Gain_d
                                       //  Referenced by: '<S43>/Gain'

  -1.0F,                               // Computed Parameter: Gain1_Gain_ji
                                       //  Referenced by: '<S43>/Gain1'

  -1.0F,                               // Computed Parameter: Gain2_Gain_bs
                                       //  Referenced by: '<S43>/Gain2'

  2.0F,                                // Computed Parameter: Gain2_Gain_kh
                                       //  Referenced by: '<S40>/Gain2'


  //  Expression: single([0 0 0])
  //  Referenced by: '<S30>/Constant4'

  { 0.0F, 0.0F, 0.0F },

  //  Expression: single([0 0 0])
  //  Referenced by: '<S30>/Constant1'

  { 0.0F, 0.0F, 0.0F },
  0.016F,                              // Computed Parameter: DiscreteTimeIntegrator1_gainval
                                       //  Referenced by: '<S30>/Discrete-Time Integrator1'


  //  Expression: single([0 0 0])
  //  Referenced by: '<S30>/Discrete-Time Integrator1'

  { 0.0F, 0.0F, 0.0F },

  //  Expression: single([0 0 0])
  //  Referenced by: '<S30>/Constant3'

  { 0.0F, 0.0F, 0.0F },

  //  Expression: single([0 0 0])
  //  Referenced by: '<S31>/Constant2'

  { 0.0F, 0.0F, 0.0F },

  //  Expression: single([0 0 0])
  //  Referenced by: '<S31>/Constant1'

  { 0.0F, 0.0F, 0.0F },
  -1.0F,                               // Computed Parameter: Gain_Gain_pf
                                       //  Referenced by: '<S64>/Gain'

  -1.0F,                               // Computed Parameter: Gain_Gain_k
                                       //  Referenced by: '<S69>/Gain'

  -1.0F,                               // Computed Parameter: Gain1_Gain_jz
                                       //  Referenced by: '<S69>/Gain1'

  -1.0F,                               // Computed Parameter: Gain2_Gain_n
                                       //  Referenced by: '<S69>/Gain2'

  -1.0F,                               // Computed Parameter: Gain1_Gain_od
                                       //  Referenced by: '<S67>/Gain1'

  0.016F,                              // Computed Parameter: DiscreteTimeIntegrator_gainval
                                       //  Referenced by: '<S31>/Discrete-Time Integrator'


  //  Expression: single([0 0 0])
  //  Referenced by: '<S31>/Discrete-Time Integrator'

  { 0.0F, 0.0F, 0.0F },

  //  Expression: single([0 0 0])
  //  Referenced by: '<S31>/Constant4'

  { 0.0F, 0.0F, 0.0F },
  2.0F,                                // Computed Parameter: Gain2_Gain_j
                                       //  Referenced by: '<S31>/Gain2'

  -1.0F,                               // Computed Parameter: Gain_Gain_is
                                       //  Referenced by: '<S97>/Gain'

  -1.0F,                               // Computed Parameter: Gain_Gain_m2
                                       //  Referenced by: '<S102>/Gain'

  -1.0F,                               // Computed Parameter: Gain1_Gain_bs
                                       //  Referenced by: '<S102>/Gain1'

  -1.0F,                               // Computed Parameter: Gain2_Gain_o
                                       //  Referenced by: '<S102>/Gain2'

  -1.0F,                               // Computed Parameter: Gain1_Gain_ed
                                       //  Referenced by: '<S100>/Gain1'

  2.0F,                                // Computed Parameter: Gain_Gain_hg
                                       //  Referenced by: '<S81>/Gain'

  1U,                                  // Computed Parameter: Delay11_DelayLength
                                       //  Referenced by: '<S13>/Delay11'

  1U,                                  // Computed Parameter: Delay11_DelayLength_g
                                       //  Referenced by: '<S14>/Delay11'


  // Start of '<S5>/For Each Subsystem1'
  {
    // Start of '<S113>/CoreSubsys'
    {
      //  Expression: [omega*T/(2+omega*T) omega*T/(2+omega*T)]
      //  Referenced by: '<S114>/3 Hz Low Pass'

      { 0.00209001475F, 0.00209001475F },

      //  Expression: [1 (omega*T-2)/(2+omega*T)]
      //  Referenced by: '<S114>/3 Hz Low Pass'

      { 1.0F, -0.99582F },
      0.0F                             // Computed Parameter: uHzLowPass_InitialStates
                                       //  Referenced by: '<S114>/3 Hz Low Pass'

    }
    // End of '<S113>/CoreSubsys'
  }
  // End of '<S5>/For Each Subsystem1'
  ,

  // Start of '<S5>/For Each Subsystem'
  {
    // Start of '<S113>/CoreSubsys'
    {
      //  Expression: [omega*T/(2+omega*T) omega*T/(2+omega*T)]
      //  Referenced by: '<S113>/3 Hz Low Pass'

      { 0.00209001475F, 0.00209001475F },

      //  Expression: [1 (omega*T-2)/(2+omega*T)]
      //  Referenced by: '<S113>/3 Hz Low Pass'

      { 1.0F, -0.99582F },
      0.0F                             // Computed Parameter: uHzLowPass_InitialStates
                                       //  Referenced by: '<S113>/3 Hz Low Pass'

    }
    // End of '<S113>/CoreSubsys'
  }
  // End of '<S5>/For Each Subsystem'
  ,

  // Start of '<S99>/Normalize'
  {
    -1.0                               // Expression: -1
                                       //  Referenced by: '<S105>/Constant1'

  }
  // End of '<S99>/Normalize'
  ,

  // Start of '<S85>/Normalize'
  {
    -1.0                               // Expression: -1
                                       //  Referenced by: '<S87>/Constant1'

  }
  // End of '<S85>/Normalize'
  ,

  // Start of '<S66>/Normalize'
  {
    -1.0                               // Expression: -1
                                       //  Referenced by: '<S72>/Constant1'

  }
  // End of '<S66>/Normalize'
  ,

  // Start of '<S18>/Normalize'
  {
    -1.0                               // Expression: -1
                                       //  Referenced by: '<S24>/Constant1'

  }
  // End of '<S18>/Normalize'
};

//
// File trailer for generated code.
//
// [EOF]
//
