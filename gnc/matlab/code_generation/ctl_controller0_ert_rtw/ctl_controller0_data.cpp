//
// File: ctl_controller0_data.cpp
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
#include "ctl_controller0.h"
#include "ctl_controller0_private.h"

// Block parameters (auto storage)
P_ctl_controller0_T ctl_controller0_P = {
  0.0001F,                             // Variable: ctl_stopping_omega_thresh
                                       //  Referenced by: '<S7>/Constant'

  0.0001F,                             // Variable: ctl_stopping_vel_thresh
                                       //  Referenced by: '<S6>/Constant'


  //  Variable: tun_accel_gain
  //  Referenced by: '<S12>/Gain'

  { 1.0F, 1.0F, 1.0F },

  //  Variable: tun_alpha_gain
  //  Referenced by: '<S13>/Gain'

  { 1.0F, 1.0F, 1.0F },
  -0.5F,                               // Variable: tun_ctl_att_sat_lower
                                       //  Referenced by: '<S13>/Discrete-Time Integrator'

  0.5F,                                // Variable: tun_ctl_att_sat_upper
                                       //  Referenced by: '<S13>/Discrete-Time Integrator'

  100.0F,                              // Variable: tun_ctl_linear_force_limit
                                       //  Referenced by: '<S12>/Constant2'

  -0.1F,                               // Variable: tun_ctl_pos_sat_lower
                                       //  Referenced by: '<S12>/Discrete-Time Integrator1'

  0.1F,                                // Variable: tun_ctl_pos_sat_upper
                                       //  Referenced by: '<S12>/Discrete-Time Integrator1'

  0.0F,                                // Variable: tun_truth_q_omega_filt_enable
                                       //  Referenced by: '<S5>/For Each Subsystem'

  0.0F,                                // Variable: tun_truth_velocity_filt_enable
                                       //  Referenced by: '<S5>/For Each Subsystem1'

  0U,                                  // Variable: ase_status_converged
                                       //  Referenced by: '<S9>/Constant'

  0U,                                  // Variable: ctl_idle_mode
                                       //  Referenced by: '<S2>/Constant4'

  3U,                                  // Variable: ctl_stopped_mode
                                       //  Referenced by: '<S2>/Constant'

  1U,                                  // Variable: ctl_stopping_mode
                                       //  Referenced by: '<S8>/Constant'

  0U,                                  // Variable: tun_debug_ctl_use_truth
                                       //  Referenced by: '<S1>/Constant'

  0.0F,                                // Mask Parameter: DiscreteDerivative_ICPrevScaled
                                       //  Referenced by: '<S93>/UD'

  0.0F,                                // Mask Parameter: DiscreteDerivative_ICPrevScal_k
                                       //  Referenced by: '<S90>/UD'

  1U,                                  // Mask Parameter: CompareToConstant2_const
                                       //  Referenced by: '<S16>/Constant'

  1U,                                  // Mask Parameter: CompareToConstant1_const
                                       //  Referenced by: '<S15>/Constant'

  1U,                                  // Mask Parameter: CompareToConstant_const
                                       //  Referenced by: '<S14>/Constant'

  1U,                                  // Mask Parameter: CompareToConstant2_const_o
                                       //  Referenced by: '<S37>/Constant'

  1U,                                  // Mask Parameter: CompareToConstant1_const_m
                                       //  Referenced by: '<S36>/Constant'

  1U,                                  // Mask Parameter: CompareToConstant_const_f
                                       //  Referenced by: '<S35>/Constant'


  //  Expression: zeros(9,1)
  //  Referenced by: '<S94>/Constant2'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S96>/Constant3'


  //  Expression: [0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0]
  //  Referenced by: '<S5>/constant28'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0 },

  //  Expression: [0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0  0]
  //  Referenced by: '<S5>/constant29'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0 },
  0.0031317642291927056,               // Expression: 0.0031317642291927056
                                       //  Referenced by: '<S10>/s(1)'

  -0.9937364715416146,                 // Expression: -0.993736471541614597
                                       //  Referenced by: '<S10>/a(2)(1)'

  0.0031317642291927056,               // Expression: 0.0031317642291927056
                                       //  Referenced by: '<S11>/s(1)'

  -0.9937364715416146,                 // Expression: -0.993736471541614597
                                       //  Referenced by: '<S11>/a(2)(1)'


  //  Expression: zeros(9,1)
  //  Referenced by: '<S25>/Constant2'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
  1.0,                                 // Expression: 1
                                       //  Referenced by: '<S25>/Constant1'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S28>/Constant3'


  //  Expression: zeros(9,1)
  //  Referenced by: '<S20>/Constant2'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
  1.0,                                 // Expression: 1
                                       //  Referenced by: '<S20>/Constant1'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S23>/Constant3'


  //  Expression: zeros(9,1)
  //  Referenced by: '<S45>/Constant2'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S47>/Constant3'

  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S40>/Constant3'


  //  Expression: zeros(9,1)
  //  Referenced by: '<S78>/Constant2'

  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 },
  0.0,                                 // Expression: 0
                                       //  Referenced by: '<S80>/Constant3'

  -1.0F,                               // Computed Parameter: Gain2_Gain
                                       //  Referenced by: '<S40>/Gain2'

  -1.0F,                               // Computed Parameter: Gain1_Gain
                                       //  Referenced by: '<S40>/Gain1'

  -1.0F,                               // Computed Parameter: Gain_Gain
                                       //  Referenced by: '<S40>/Gain'

  -1.0F,                               // Computed Parameter: Gain_Gain_p
                                       //  Referenced by: '<S96>/Gain'

  -1.0F,                               // Computed Parameter: Gain1_Gain_e
                                       //  Referenced by: '<S96>/Gain1'

  -1.0F,                               // Computed Parameter: Gain2_Gain_k
                                       //  Referenced by: '<S96>/Gain2'

  -1.0F,                               // Computed Parameter: Gain1_Gain_o
                                       //  Referenced by: '<S94>/Gain1'

  2.0F,                                // Computed Parameter: Gain_Gain_m
                                       //  Referenced by: '<S89>/Gain'

  62.5F,                               // Computed Parameter: TSamp_WtEt
                                       //  Referenced by: '<S93>/TSamp'


  //  Computed Parameter: constant34_Value
  //  Referenced by: '<S5>/constant34'

  { 0.0F, 0.0F, 0.0F },
  62.5F,                               // Computed Parameter: TSamp_WtEt_k
                                       //  Referenced by: '<S90>/TSamp'


  //  Computed Parameter: constant36_Value
  //  Referenced by: '<S5>/constant36'

  { 0.0F, 0.0F, 0.0F },

  //  Computed Parameter: constant37_Value
  //  Referenced by: '<S5>/constant37'

  { 0.0F, 0.0F, 0.0F },

  //  Computed Parameter: constant4_Value
  //  Referenced by: '<S5>/constant4'

  { 0.0F, 0.0F, 0.0F, 0.0F },

  //  Computed Parameter: constant7_Value
  //  Referenced by: '<S5>/constant7'

  { 0.0F, 0.0F, 0.0F },

  //  Computed Parameter: constant22_Value
  //  Referenced by: '<S5>/constant22'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

  //  Computed Parameter: constant23_Value
  //  Referenced by: '<S5>/constant23'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },

  //  Computed Parameter: constant24_Value
  //  Referenced by: '<S5>/constant24'

  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F
  },

  //  Computed Parameter: constant31_Value
  //  Referenced by: '<S5>/constant31'

  { 0.0F, 0.0F, 0.0F },

  //  Computed Parameter: constant32_Value
  //  Referenced by: '<S5>/constant32'

  { 0.0F, 0.0F, 0.0F, 0.0F },

  //  Computed Parameter: constant33_Value
  //  Referenced by: '<S5>/constant33'

  { 0.0F, 0.0F, 0.0F },
  0.0F,                                // Computed Parameter: Delay11_InitialCondition
                                       //  Referenced by: '<S10>/Delay11'

  0.0F,                                // Computed Parameter: Delay11_InitialCondition_k
                                       //  Referenced by: '<S11>/Delay11'

  0.5F,                                // Expression: single(0.5)
                                       //  Referenced by: '<S58>/Constant'

  1.0E-9F,                             // Expression: single(1e-9)
                                       //  Referenced by: '<S56>/Gain'

  0.5F,                                // Computed Parameter: Constant1_Value_g
                                       //  Referenced by: '<S57>/Constant1'

  0.5F,                                // Computed Parameter: Constant3_Value_d
                                       //  Referenced by: '<S57>/Constant3'

  0.0F,                                // Computed Parameter: Constant3_Value_dz
                                       //  Referenced by: '<S62>/Constant3'

  -1.0F,                               // Computed Parameter: Gain_Gain_n
                                       //  Referenced by: '<S62>/Gain'

  -1.0F,                               // Computed Parameter: Gain1_Gain_b
                                       //  Referenced by: '<S62>/Gain1'

  0.0F,                                // Computed Parameter: Constant2_Value_d
                                       //  Referenced by: '<S62>/Constant2'

  -1.0F,                               // Computed Parameter: Gain2_Gain_kx
                                       //  Referenced by: '<S62>/Gain2'

  -1.0F,                               // Computed Parameter: Gain3_Gain
                                       //  Referenced by: '<S62>/Gain3'

  -1.0F,                               // Computed Parameter: Gain4_Gain
                                       //  Referenced by: '<S62>/Gain4'

  0.0F,                                // Computed Parameter: Constant1_Value_l
                                       //  Referenced by: '<S62>/Constant1'

  -1.0F,                               // Computed Parameter: Gain5_Gain
                                       //  Referenced by: '<S62>/Gain5'

  0.0F,                                // Computed Parameter: Constant_Value_g
                                       //  Referenced by: '<S62>/Constant'

  0.0F,                                // Computed Parameter: Constant3_Value_f
                                       //  Referenced by: '<S61>/Constant3'

  -1.0F,                               // Computed Parameter: Gain_Gain_g
                                       //  Referenced by: '<S61>/Gain'

  -1.0F,                               // Computed Parameter: Gain1_Gain_j
                                       //  Referenced by: '<S61>/Gain1'

  0.0F,                                // Computed Parameter: Constant2_Value_gq
                                       //  Referenced by: '<S61>/Constant2'

  -1.0F,                               // Computed Parameter: Gain2_Gain_c
                                       //  Referenced by: '<S61>/Gain2'

  -1.0F,                               // Computed Parameter: Gain3_Gain_o
                                       //  Referenced by: '<S61>/Gain3'

  -1.0F,                               // Computed Parameter: Gain4_Gain_c
                                       //  Referenced by: '<S61>/Gain4'

  0.0F,                                // Computed Parameter: Constant1_Value_gk
                                       //  Referenced by: '<S61>/Constant1'

  -1.0F,                               // Computed Parameter: Gain5_Gain_a
                                       //  Referenced by: '<S61>/Gain5'

  0.0F,                                // Computed Parameter: Constant_Value_m
                                       //  Referenced by: '<S61>/Constant'

  48.0F,                               // Computed Parameter: Constant2_Value_it
                                       //  Referenced by: '<S57>/Constant2'


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
  0.0F,                                // Computed Parameter: UnitDelay1_InitialCondition
                                       //  Referenced by: '<S2>/Unit Delay1'

  0.0F,                                // Computed Parameter: UnitDelay2_InitialCondition
                                       //  Referenced by: '<S2>/Unit Delay2'

  2.0F,                                // Computed Parameter: Gain_Gain_l
                                       //  Referenced by: '<S25>/Gain'

  2.0F,                                // Computed Parameter: Gain1_Gain_ej
                                       //  Referenced by: '<S25>/Gain1'

  -1.0F,                               // Computed Parameter: Gain_Gain_i
                                       //  Referenced by: '<S28>/Gain'

  -1.0F,                               // Computed Parameter: Gain1_Gain_k
                                       //  Referenced by: '<S28>/Gain1'

  -1.0F,                               // Computed Parameter: Gain2_Gain_b
                                       //  Referenced by: '<S28>/Gain2'

  2.0F,                                // Computed Parameter: Gain2_Gain_k1
                                       //  Referenced by: '<S25>/Gain2'

  2.0F,                                // Computed Parameter: Gain_Gain_b
                                       //  Referenced by: '<S20>/Gain'

  2.0F,                                // Computed Parameter: Gain1_Gain_g
                                       //  Referenced by: '<S20>/Gain1'

  -1.0F,                               // Computed Parameter: Gain_Gain_d
                                       //  Referenced by: '<S23>/Gain'

  -1.0F,                               // Computed Parameter: Gain1_Gain_ji
                                       //  Referenced by: '<S23>/Gain1'

  -1.0F,                               // Computed Parameter: Gain2_Gain_bs
                                       //  Referenced by: '<S23>/Gain2'

  2.0F,                                // Computed Parameter: Gain2_Gain_kh
                                       //  Referenced by: '<S20>/Gain2'


  //  Expression: single([0 0 0])
  //  Referenced by: '<S12>/Constant4'

  { 0.0F, 0.0F, 0.0F },

  //  Expression: single([0 0 0])
  //  Referenced by: '<S12>/Constant1'

  { 0.0F, 0.0F, 0.0F },
  0.016F,                              // Computed Parameter: DiscreteTimeIntegrator1_gainval
                                       //  Referenced by: '<S12>/Discrete-Time Integrator1'


  //  Expression: single([0 0 0])
  //  Referenced by: '<S12>/Discrete-Time Integrator1'

  { 0.0F, 0.0F, 0.0F },

  //  Expression: single([0 0 0])
  //  Referenced by: '<S12>/Constant3'

  { 0.0F, 0.0F, 0.0F },

  //  Expression: single([0 0 0])
  //  Referenced by: '<S13>/Constant2'

  { 0.0F, 0.0F, 0.0F },

  //  Expression: single([0 0 0])
  //  Referenced by: '<S13>/Constant1'

  { 0.0F, 0.0F, 0.0F },
  -1.0F,                               // Computed Parameter: Gain_Gain_pf
                                       //  Referenced by: '<S42>/Gain'

  -1.0F,                               // Computed Parameter: Gain_Gain_k
                                       //  Referenced by: '<S47>/Gain'

  -1.0F,                               // Computed Parameter: Gain1_Gain_jz
                                       //  Referenced by: '<S47>/Gain1'

  -1.0F,                               // Computed Parameter: Gain2_Gain_n
                                       //  Referenced by: '<S47>/Gain2'

  -1.0F,                               // Computed Parameter: Gain1_Gain_od
                                       //  Referenced by: '<S45>/Gain1'

  0.016F,                              // Computed Parameter: DiscreteTimeIntegrator_gainval
                                       //  Referenced by: '<S13>/Discrete-Time Integrator'


  //  Expression: single([0 0 0])
  //  Referenced by: '<S13>/Discrete-Time Integrator'

  { 0.0F, 0.0F, 0.0F },

  //  Expression: single([0 0 0])
  //  Referenced by: '<S13>/Constant4'

  { 0.0F, 0.0F, 0.0F },
  2.0F,                                // Computed Parameter: Gain2_Gain_j
                                       //  Referenced by: '<S13>/Gain2'

  -1.0F,                               // Computed Parameter: Gain_Gain_is
                                       //  Referenced by: '<S75>/Gain'

  -1.0F,                               // Computed Parameter: Gain_Gain_m2
                                       //  Referenced by: '<S80>/Gain'

  -1.0F,                               // Computed Parameter: Gain1_Gain_bs
                                       //  Referenced by: '<S80>/Gain1'

  -1.0F,                               // Computed Parameter: Gain2_Gain_o
                                       //  Referenced by: '<S80>/Gain2'

  -1.0F,                               // Computed Parameter: Gain1_Gain_ed
                                       //  Referenced by: '<S78>/Gain1'

  2.0F,                                // Computed Parameter: Gain_Gain_hg
                                       //  Referenced by: '<S59>/Gain'

  0U,                                  // Computed Parameter: constant40_Value
                                       //  Referenced by: '<S5>/constant40'

  1U,                                  // Computed Parameter: Delay11_DelayLength
                                       //  Referenced by: '<S10>/Delay11'

  1U,                                  // Computed Parameter: Delay11_DelayLength_g
                                       //  Referenced by: '<S11>/Delay11'

  0U,                                  // Computed Parameter: constant25_Value
                                       //  Referenced by: '<S5>/constant25'

  0U,                                  // Computed Parameter: constant39_Value
                                       //  Referenced by: '<S5>/constant39'

  0U,                                  // Computed Parameter: constant26_Value
                                       //  Referenced by: '<S5>/constant26'

  0U,                                  // Computed Parameter: constant27_Value
                                       //  Referenced by: '<S5>/constant27'


  // Start of '<S5>/For Each Subsystem1'
  {
    // Start of '<S91>/CoreSubsys'
    {
      //  Expression: [omega*T/(2+omega*T) omega*T/(2+omega*T)]
      //  Referenced by: '<S92>/3 Hz Low Pass'

      { 0.00209001475F, 0.00209001475F },

      //  Expression: [1 (omega*T-2)/(2+omega*T)]
      //  Referenced by: '<S92>/3 Hz Low Pass'

      { 1.0F, -0.99582F },
      0.0F                             // Computed Parameter: uHzLowPass_InitialStates
                                       //  Referenced by: '<S92>/3 Hz Low Pass'

    }
    // End of '<S91>/CoreSubsys'
  }
  // End of '<S5>/For Each Subsystem1'
  ,

  // Start of '<S5>/For Each Subsystem'
  {
    // Start of '<S91>/CoreSubsys'
    {
      //  Expression: [omega*T/(2+omega*T) omega*T/(2+omega*T)]
      //  Referenced by: '<S91>/3 Hz Low Pass'

      { 0.00209001475F, 0.00209001475F },

      //  Expression: [1 (omega*T-2)/(2+omega*T)]
      //  Referenced by: '<S91>/3 Hz Low Pass'

      { 1.0F, -0.99582F },
      0.0F                             // Computed Parameter: uHzLowPass_InitialStates
                                       //  Referenced by: '<S91>/3 Hz Low Pass'

    }
    // End of '<S91>/CoreSubsys'
  }
  // End of '<S5>/For Each Subsystem'
  ,

  // Start of '<S77>/Normalize'
  {
    -1.0                               // Expression: -1
                                       //  Referenced by: '<S83>/Constant1'

  }
  // End of '<S77>/Normalize'
  ,

  // Start of '<S63>/Normalize'
  {
    -1.0                               // Expression: -1
                                       //  Referenced by: '<S65>/Constant1'

  }
  // End of '<S63>/Normalize'
  ,

  // Start of '<S44>/Normalize'
  {
    -1.0                               // Expression: -1
                                       //  Referenced by: '<S50>/Constant1'

  }
  // End of '<S44>/Normalize'
};

//
// File trailer for generated code.
//
// [EOF]
//
