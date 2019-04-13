//
// File: sim_model_lib0.h
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Thu Mar  7 13:23:50 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_sim_model_lib0_h_
#define RTW_HEADER_sim_model_lib0_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef sim_model_lib0_COMMON_INCLUDES_
# define sim_model_lib0_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#endif                                 // sim_model_lib0_COMMON_INCLUDES_

#include "sim_model_lib0_types.h"

// Child system includes
#include "blower_aerodynamics.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "aimgnophaiekjecb_eml_rand_mt19937ar.h"
#include "baaiimglmglniekf_sortIdx.h"
#include "cbiephdblnopophl_cat.h"
#include "dbiehlnglnohaaai_cat.h"
#include "dbiekfcbglfkkfcb_sortLE.h"
#include "fcbadjmgohlnpphl_rows_differ.h"
#include "fkfkbiecjecjbaie_cat.h"
#include "hdbiaaaabiecmgdb_do_vectors.h"
#include "hdjmknohknopimgl_sort.h"
#include "jecjcjmgfkfkophl_sum.h"
#include "kfcjlnglmgdbgdjm_all.h"
#include "kngdbaaaiecjbiec_skip_to_last_equal_value.h"
#include "mgdjimgdopphcjek_skip_to_last_equal_value.h"
#include "mglflfcbcbimecje_cat.h"
#include "mglncbaaiekflfcb_cat.h"
#include "mohdcjmgnohlknoh_sortLE.h"
#include "ohdbiekfdjechlno_rows_differ.h"
#include "ophlngdbgdjmgdje_cat.h"
#include "rt_mldivide_U1f3x3_U2f_XeZWzB4d.h"
#include "rt_nrand_Upu32_Yd_f_pw_snf.h"
#include "rt_roundd_snf.h"
#include "rtGetInf.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block states (auto storage) for system '<S89>/CoreSubsys'
typedef struct {
  real_T NextOutput;                   // '<S106>/pixel_noise'
  real_T NextOutput_n;                 // '<S106>/pixel_noise1'
  uint32_T RandSeed;                   // '<S106>/pixel_noise'
  uint32_T RandSeed_k;                 // '<S106>/pixel_noise1'
} DW_CoreSubsys_sim_model_lib0_T;

// Block states (auto storage) for system '<S85>/pinhole_projection_model'
typedef struct {
  DW_CoreSubsys_sim_model_lib0_T CoreSubsys;// '<S89>/CoreSubsys'
} DW_pinhole_projection_model_s_T;

// Block signals for system '<S188>/latch_nozzle_thrust_matricies'
typedef struct {
  real32_T OutportBufferForthrust2force_B[18];// '<S196>/MATLAB Function'
  real32_T OutportBufferForthrust2torque_B[18];// '<S196>/MATLAB Function'
} B_latch_nozzle_thrust_matrici_T;

// Block signals for system '<S184>/calc_nozzle_area'
typedef struct {
  real32_T Gain12[6];                  // '<S189>/Gain12'
  real32_T Product2[6];                // '<S189>/Product2'
} B_calc_nozzle_area_sim_model__T;

// Block signals for system '<S190>/dc_motor_model'
typedef struct {
  real32_T current;                    // '<S211>/Gain5'
  real32_T output_torque;              // '<S211>/Add1'
} B_dc_motor_model_sim_model_li_T;

// Block signals for system '<S190>/speed_controller'
typedef struct {
  real32_T Switch2;                    // '<S214>/Switch2'
} B_speed_controller_sim_model__T;

// Block states (auto storage) for system '<S190>/speed_controller'
typedef struct {
  real32_T Integrator_DSTATE;          // '<S213>/Integrator'
  real32_T Filter_DSTATE;              // '<S213>/Filter'
  real32_T PrevY;                      // '<S212>/Rate Limiter'
} DW_speed_controller_sim_model_T;

// Block signals for system '<S184>/servo_model'
typedef struct {
  real32_T Backlash1[6];               // '<S216>/Backlash1'
  real32_T Gain1[6];                   // '<S216>/Gain1'
  real32_T current[6];                 // '<S216>/Gain5'
} B_servo_model_sim_model_lib0_T;

// Block states (auto storage) for system '<S184>/servo_model'
typedef struct {
  real32_T DiscreteTimeIntegrator4_DSTATE[6];// '<S216>/Discrete-Time Integrator4' 
  real32_T Integrator_DSTATE[6];       // '<S218>/Integrator'
  real32_T Filter_DSTATE[6];           // '<S218>/Filter'
  real32_T DiscreteTimeIntegrator3_DSTATE[6];// '<S216>/Discrete-Time Integrator3' 
  real32_T PrevY[6];                   // '<S216>/Backlash1'
} DW_servo_model_sim_model_lib0_T;

// Block signals (auto storage)
typedef struct {
  real32_T b_points_in_iss_data[26046];
  real32_T P_point_B_iss_p4[25896];    // '<S140>/Matrix Concatenate'
  real32_T Add_o[25896];               // '<S140>/Add'
  real32_T rtb_Add_o_m[25896];
  real32_T b_points_in_iss_data_c[25896];
  real32_T b_points_in_iss_data_k[25896];
  real32_T b_data[17364];
  real32_T b_b[17264];
  real32_T b_data_c[17264];
  real_T idx[8632];
  real32_T P_point_B_iss_k[11937];     // '<S164>/Matrix Concatenate'
  real32_T Add_f[11937];               // '<S164>/Add'
  real32_T rtb_Add_f_b[11937];
  real32_T ycol[8632];
  int32_T unique_rows_data[8632];
  int32_T indx_data[8632];
  int32_T iwork[8632];
  int32_T idx_data[8632];
  int32_T idx_p[8632];
  int32_T iwork_data[8632];
  real_T random_order_data[3979];
  real_T idx_data_c[3979];
  real32_T valid_points_in_cam_data[7958];
  real32_T valid_points_in_cam_data_f[7958];
  real32_T b_c_data[4029];
  real32_T c_data[4029];
  int32_T shuffled_ids_data[3979];
  int32_T indx_data_g[3979];
  int32_T r_data[3979];
  int32_T ib_data[3979];
  int32_T idx_data_g[3979];
  int32_T idx_data_m[3979];
  int32_T iwork_data_n[3979];
  int32_T iwork_data_p[3979];
  int32_T iwork_data_l[3979];
  real32_T valid_ids_data[3979];
  real32_T ycol_data[3979];
  int8_T b_valid_in_data[8682];
  boolean_T b_valid_in_data_j[8632];
  int16_T tmp_data[3979];
  real32_T points_out_2D_cam_e[1600];  // '<S83>/generate_output'
  real32_T new_observations_data[1600];
  real32_T tmp_data_d[1600];
  real32_T P_point_B_iss_p[1512];      // '<S116>/Matrix Concatenate'
  real32_T Add_l[1512];                // '<S116>/Add'
  real32_T rtb_Add_l_g[1512];
  real32_T points_in_iss_data[1512];
  real32_T tmp_data_l[1500];
  real32_T tmp_data_dh[1200];
  env_msg RateTransition2;             // '<S79>/Rate Transition2'
  env_msg RateTransition2_o;           // '<S81>/Rate Transition2'
  env_msg RateTransition2_j;           // '<S82>/Rate Transition2'
  env_msg RateTransition2_b;           // '<S83>/Rate Transition2'
  ex_time_msg RateTransition4;         // '<S79>/Rate Transition4'
  ex_time_msg RateTransition4_m;       // '<S81>/Rate Transition4'
  ex_time_msg RateTransition4_e;       // '<S82>/Rate Transition4'
  ex_time_msg RateTransition4_f;       // '<S83>/Rate Transition4'
  real32_T RateTransition5[150];       // '<S81>/Rate Transition5'
  real32_T RateTransition9[150];       // '<S81>/Rate Transition9'
  real32_T RateTransition6[1600];      // '<S83>/Rate Transition6'
  real32_T RateTransition8[50];        // '<S83>/Rate Transition8'
  real32_T RateTransition5_b[150];     // '<S82>/Rate Transition5'
  real32_T RateTransition6_n[100];     // '<S82>/Rate Transition6'
  real32_T RateTransition5_p[150];     // '<S79>/Rate Transition5'
  real32_T RateTransition6_p[100];     // '<S79>/Rate Transition6'
  real32_T id_tag_m[50];               // '<S83>/generate_output'
  real32_T ImpAsg_InsertedFor_P_points_2D_[7958];// '<S165>/pinhole_camera_projection' 
  real32_T points_out_iss_j[150];      // '<S82>/generate_output'
  real32_T points_out_2D_cam_g[100];   // '<S82>/generate_output'
  real32_T ImpAsg_InsertedFor_P_points_2_f[17264];// '<S141>/pinhole_camera_projection' 
  real32_T points_out_iss_h[150];      // '<S81>/generate_output'
  real32_T points_out_3D_cam_n[150];   // '<S81>/generate_output'
  uint32_T RateTransition3;            // '<S81>/Rate Transition3'
  uint32_T RateTransition1;            // '<S81>/Rate Transition1'
  uint32_T RateTransition3_j;          // '<S83>/Rate Transition3'
  uint32_T RateTransition1_j;          // '<S83>/Rate Transition1'
  uint32_T RateTransition3_o;          // '<S82>/Rate Transition3'
  uint32_T RateTransition1_a;          // '<S82>/Rate Transition1'
  uint32_T RateTransition3_e;          // '<S79>/Rate Transition3'
  uint32_T RateTransition1_b;          // '<S79>/Rate Transition1'
  uint8_T RateTransition7[50];         // '<S81>/Rate Transition7'
  uint8_T RateTransition10;            // '<S83>/Rate Transition10'
  uint8_T RateTransition7_f[800];      // '<S83>/Rate Transition7'
  uint8_T RateTransition7_g[50];       // '<S82>/Rate Transition7'
  uint8_T RateTransition7_k[50];       // '<S79>/Rate Transition7'
  uint8_T registration_id_h;           // '<S83>/generate_output'
  boolean_T ImpAsg_InsertedFor_points_in_FO[3979];// '<S165>/determine_points_in_FOV' 
  boolean_T ImpAsg_InsertedFor_points_in__p[8632];// '<S141>/determine_points_in_FOV' 
  boolean_T ImpAsg_InsertedFor_points_in__n[504];// '<S117>/determine_points_in_FOV' 
  B_servo_model_sim_model_lib0_T servo_model_f;// '<S185>/servo_model'
  B_speed_controller_sim_model__T speed_controller_c;// '<S223>/speed_controller' 
  B_dc_motor_model_sim_model_li_T dc_motor_model_g;// '<S223>/dc_motor_model'
  B_calc_nozzle_area_sim_model__T calc_nozzle_area_c;// '<S185>/calc_nozzle_area' 
  B_latch_nozzle_thrust_matrici_T latch_nozzle_thrust_matricies_p;// '<S221>/latch_nozzle_thrust_matricies' 
  B_blower_aerodynamics_sim_m_f_T blower_aerodynamics_j;// '<S185>/blower_aerodynamics' 
  B_servo_model_sim_model_lib0_T servo_model;// '<S184>/servo_model'
  B_speed_controller_sim_model__T speed_controller;// '<S190>/speed_controller'
  B_dc_motor_model_sim_model_li_T dc_motor_model;// '<S190>/dc_motor_model'
  B_calc_nozzle_area_sim_model__T calc_nozzle_area;// '<S184>/calc_nozzle_area'
  B_latch_nozzle_thrust_matrici_T latch_nozzle_thrust_matricies;// '<S188>/latch_nozzle_thrust_matricies' 
  B_blower_aerodynamics_sim_mod_T blower_aerodynamics;// '<S184>/blower_aerodynamics' 
} B_sim_model_lib0_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  bpm_msg Delay1_DSTATE;               // '<S1>/Delay1'
  real_T DiscreteTimeIntegrator2_DSTATE[3];// '<S254>/Discrete-Time Integrator2' 
  real_T DiscreteTimeIntegrator1_DSTATE[3];// '<S254>/Discrete-Time Integrator1' 
  real_T DiscreteTimeIntegrator2_DSTAT_j[3];// '<S255>/Discrete-Time Integrator2' 
  real_T DiscreteTimeIntegrator1_DSTAT_c[3];// '<S255>/Discrete-Time Integrator1' 
  real_T NextOutput[3];                // '<S254>/random_noise'
  real_T NextOutput_e[3];              // '<S255>/random_noise'
  real_T NextOutput_l[3];              // '<S6>/Random Number'
  real_T NextOutput_f[3];              // '<S6>/Random Number1'
  real_T NextOutput_o;                 // '<S113>/pixel_noise'
  real_T NextOutput_i[3];              // '<S254>/random_noise1'
  real_T NextOutput_j[3];              // '<S254>/random_noise2'
  real_T NextOutput_h[3];              // '<S255>/random_noise1'
  real_T NextOutput_p[3];              // '<S255>/random_noise2'
  real_T int_initalization_complete;   // '<S83>/generate_output'
  real32_T DiscreteTimeIntegrator1_DSTAT_p[3];// '<S7>/Discrete-Time Integrator1' 
  real32_T DiscreteTimeIntegrator_DSTATE[3];// '<S7>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator_DSTATE_b[3];// '<S5>/Discrete-Time Integrator' 
  real32_T Delay_DSTATE[4];            // '<S11>/Delay'
  real32_T DiscreteTimeIntegrator_DSTATE_d[3];// '<S6>/Discrete-Time Integrator' 
  real32_T DiscreteTimeIntegrator1_DSTAT_b[3];// '<S6>/Discrete-Time Integrator1' 
  real32_T DiscreteTimeIntegrator_DSTAT_dp;// '<S191>/Discrete-Time Integrator'
  real32_T DiscreteTimeIntegrator_DSTATE_e;// '<S224>/Discrete-Time Integrator'
  real32_T DelayInput1_DSTATE[3];      // '<S194>/Delay Input1'
  real32_T DelayInput1_DSTATE_m[3];    // '<S227>/Delay Input1'
  uint32_T UnitDelay1_DSTATE;          // '<S8>/Unit Delay1'
  uint32_T UnitDelay_DSTATE;           // '<S8>/Unit Delay'
  uint32_T DelayInput1_DSTATE_p;       // '<S110>/Delay Input1'
  uint32_T DelayInput1_DSTATE_l;       // '<S111>/Delay Input1'
  real32_T RateTransition5_Buffer0[150];// '<S81>/Rate Transition5'
  real32_T RateTransition9_Buffer0[150];// '<S81>/Rate Transition9'
  real32_T RateTransition6_Buffer0[1600];// '<S83>/Rate Transition6'
  real32_T RateTransition8_Buffer0[50];// '<S83>/Rate Transition8'
  real32_T RateTransition5_Buffer0_i[150];// '<S82>/Rate Transition5'
  real32_T RateTransition6_Buffer0_h[100];// '<S82>/Rate Transition6'
  real32_T RateTransition5_Buffer0_f[150];// '<S79>/Rate Transition5'
  real32_T RateTransition6_Buffer0_e[100];// '<S79>/Rate Transition6'
  real32_T int_id_hist[50];            // '<S83>/generate_output'
  real32_T int_observations[1600];     // '<S83>/generate_output'
  int32_T clockTickCounter;            // '<S80>/AR_pulse_gen'
  int32_T clockTickCounter_c;          // '<S80>/landmark_pulse_gen'
  int32_T clockTickCounter_n;          // '<S80>/handrail_pulse_gen'
  uint32_T RateTransition3_Buffer0;    // '<S81>/Rate Transition3'
  uint32_T RateTransition1_Buffer0;    // '<S81>/Rate Transition1'
  uint32_T RandSeed[3];                // '<S254>/random_noise'
  uint32_T RandSeed_k[3];              // '<S255>/random_noise'
  uint32_T RateTransition3_Buffer0_g;  // '<S83>/Rate Transition3'
  uint32_T RateTransition1_Buffer0_b;  // '<S83>/Rate Transition1'
  uint32_T RateTransition3_Buffer0_o;  // '<S82>/Rate Transition3'
  uint32_T RateTransition1_Buffer0_l;  // '<S82>/Rate Transition1'
  uint32_T RateTransition3_Buffer0_n;  // '<S79>/Rate Transition3'
  uint32_T RateTransition1_Buffer0_i;  // '<S79>/Rate Transition1'
  uint32_T RandSeed_i[3];              // '<S6>/Random Number'
  uint32_T RandSeed_m[3];              // '<S6>/Random Number1'
  uint32_T RandSeed_mb;                // '<S85>/pixel_noise'
  uint32_T RandSeed_d;                 // '<S113>/pixel_noise'
  uint32_T RandSeed_mi;                // '<S137>/pixel_noise'
  uint32_T RandSeed_o;                 // '<S161>/pixel_noise'
  uint32_T RandSeed_ip[3];             // '<S254>/random_noise1'
  uint32_T RandSeed_e[3];              // '<S254>/random_noise2'
  uint32_T RandSeed_p[3];              // '<S255>/random_noise1'
  uint32_T RandSeed_n[3];              // '<S255>/random_noise2'
  uint32_T state[625];                 // '<S83>/generate_output'
  uint32_T state_d[625];               // '<S82>/generate_output'
  uint32_T state_m[625];               // '<S81>/generate_output'
  uint8_T UnitDelay_DSTATE_i;          // '<S70>/Unit Delay'
  uint8_T UnitDelay_DSTATE_if;         // '<S71>/Unit Delay'
  uint8_T UnitDelay_DSTATE_h;          // '<S73>/Unit Delay'
  uint8_T UnitDelay_DSTATE_ht;         // '<S72>/Unit Delay'
  uint8_T UnitDelay_DSTATE_hj;         // '<S84>/Unit Delay'
  uint8_T UnitDelay_DSTATE_g;          // '<S112>/Unit Delay'
  uint8_T UnitDelay_DSTATE_a;          // '<S136>/Unit Delay'
  uint8_T UnitDelay_DSTATE_it;         // '<S160>/Unit Delay'
  boolean_T UnitDelay_DSTATE_m;        // '<S188>/Unit Delay'
  boolean_T UnitDelay_DSTATE_c;        // '<S221>/Unit Delay'
  uint8_T RateTransition7_Buffer0[50]; // '<S81>/Rate Transition7'
  uint8_T icLoad;                      // '<S11>/Delay'
  uint8_T RateTransition10_Buffer0;    // '<S83>/Rate Transition10'
  uint8_T RateTransition7_Buffer0_a[800];// '<S83>/Rate Transition7'
  uint8_T RateTransition7_Buffer0_an[50];// '<S82>/Rate Transition7'
  uint8_T RateTransition7_Buffer0_j[50];// '<S79>/Rate Transition7'
  uint8_T int_valid_flag[800];         // '<S83>/generate_output'
  uint8_T int_registration_number;     // '<S83>/generate_output'
  boolean_T int_id_hist_not_empty;     // '<S83>/generate_output'
  DW_servo_model_sim_model_lib0_T servo_model_f;// '<S185>/servo_model'
  DW_speed_controller_sim_model_T speed_controller_c;// '<S223>/speed_controller' 
  DW_blower_aerodynamics_sim__c_T blower_aerodynamics_j;// '<S185>/blower_aerodynamics' 
  DW_servo_model_sim_model_lib0_T servo_model;// '<S184>/servo_model'
  DW_speed_controller_sim_model_T speed_controller;// '<S190>/speed_controller'
  DW_blower_aerodynamics_sim_mo_T blower_aerodynamics;// '<S184>/blower_aerodynamics' 
  DW_pinhole_projection_model_s_T pinhole_projection_model_o[3979];// '<S161>/pinhole_projection_model' 
  DW_pinhole_projection_model_s_T pinhole_projection_model_id[8632];// '<S137>/pinhole_projection_model' 
  DW_pinhole_projection_model_s_T pinhole_projection_model_i[504];// '<S113>/pinhole_projection_model' 
  DW_pinhole_projection_model_s_T pinhole_projection_model[36];// '<S85>/pinhole_projection_model' 
} DW_sim_model_lib0_T;

// Parameters for system: '<S89>/CoreSubsys'
struct P_CoreSubsys_sim_model_lib0_T_ {
  real_T pixel_noise_Mean;             // Expression: 0
                                       //  Referenced by: '<S106>/pixel_noise'

  real_T pixel_noise1_Mean;            // Expression: 0
                                       //  Referenced by: '<S106>/pixel_noise1'

  real32_T Constant_Value[2];          // Expression: single([0;0])
                                       //  Referenced by: '<S106>/Constant'

  real32_T Switch_Threshold;           // Computed Parameter: Switch_Threshold
                                       //  Referenced by: '<S106>/Switch'

  real32_T Constant_Value_n;           // Computed Parameter: Constant_Value_n
                                       //  Referenced by: '<S107>/Constant'

};

// Parameters for system: '<S85>/pinhole_projection_model'
struct P_pinhole_projection_model_si_T_ {
  P_CoreSubsys_sim_model_lib0_T CoreSubsys;// '<S89>/CoreSubsys'
};

// Parameters for system: '<S200>/CoreSubsys'
struct P_CoreSubsys_sim_model_lib0_a_T_ {
  real_T Constant2_Value[9];           // Expression: zeros(9,1)
                                       //  Referenced by: '<S203>/Constant2'

  real_T Constant1_Value;              // Expression: 1
                                       //  Referenced by: '<S203>/Constant1'

  real_T Constant3_Value;              // Expression: 0
                                       //  Referenced by: '<S206>/Constant3'

  real32_T Gain_Gain;                  // Computed Parameter: Gain_Gain
                                       //  Referenced by: '<S203>/Gain'

  real32_T Gain1_Gain;                 // Computed Parameter: Gain1_Gain
                                       //  Referenced by: '<S203>/Gain1'

  real32_T Gain_Gain_k;                // Computed Parameter: Gain_Gain_k
                                       //  Referenced by: '<S206>/Gain'

  real32_T Gain1_Gain_f;               // Computed Parameter: Gain1_Gain_f
                                       //  Referenced by: '<S206>/Gain1'

  real32_T Gain2_Gain;                 // Computed Parameter: Gain2_Gain
                                       //  Referenced by: '<S206>/Gain2'

  real32_T Gain2_Gain_k;               // Computed Parameter: Gain2_Gain_k
                                       //  Referenced by: '<S203>/Gain2'

};

// Parameters for system: '<S188>/latch_nozzle_thrust_matricies'
struct P_latch_nozzle_thrust_matrici_T_ {
  real32_T Constant2_Value[24];        // Expression: [zeros(6,3,'single'), ones(6,1,'single')]
                                       //  Referenced by: '<S196>/Constant2'

  real32_T thrust2torque_B_Y0;         // Computed Parameter: thrust2torque_B_Y0
                                       //  Referenced by: '<S196>/thrust2torque_B'

  real32_T thrust2force_B_Y0;          // Computed Parameter: thrust2force_B_Y0
                                       //  Referenced by: '<S196>/thrust2force_B'

  real32_T Switch_Threshold;           // Computed Parameter: Switch_Threshold
                                       //  Referenced by: '<S196>/Switch'

  P_CoreSubsys_sim_model_lib0_a_T CoreSubsys;// '<S200>/CoreSubsys'
};

// Parameters for system: '<S190>/speed_controller'
struct P_speed_controller_sim_model__T_ {
  real32_T DiscretePIDController_LowerSatu;// Mask Parameter: DiscretePIDController_LowerSatu
                                           //  Referenced by:
                                           //    '<S213>/Saturate'
                                           //    '<S215>/DeadZone'

  real32_T Gain_Gain;                  // Computed Parameter: Gain_Gain
                                       //  Referenced by: '<S212>/Gain'

  real32_T DiscreteTransferFcn1_InitialSta;// Computed Parameter: DiscreteTransferFcn1_InitialSta
                                           //  Referenced by: '<S212>/Discrete Transfer Fcn1'

  real32_T RateLimiter_RisingLim;      // Computed Parameter: RateLimiter_RisingLim
                                       //  Referenced by: '<S212>/Rate Limiter'

  real32_T RateLimiter_FallingLim;     // Computed Parameter: RateLimiter_FallingLim
                                       //  Referenced by: '<S212>/Rate Limiter'

  real32_T RateLimiter_IC;             // Computed Parameter: RateLimiter_IC
                                       //  Referenced by: '<S212>/Rate Limiter'

  real32_T Integrator_gainval;         // Computed Parameter: Integrator_gainval
                                       //  Referenced by: '<S213>/Integrator'

  real32_T Integrator_IC;              // Computed Parameter: Integrator_IC
                                       //  Referenced by: '<S213>/Integrator'

  real32_T Filter_gainval;             // Computed Parameter: Filter_gainval
                                       //  Referenced by: '<S213>/Filter'

  real32_T Filter_IC;                  // Computed Parameter: Filter_IC
                                       //  Referenced by: '<S213>/Filter'

  real32_T ZeroGain_Gain;              // Computed Parameter: ZeroGain_Gain
                                       //  Referenced by: '<S215>/ZeroGain'

  real32_T Constant_Value;             // Computed Parameter: Constant_Value
                                       //  Referenced by: '<S213>/Constant'

};

// Parameters for system: '<S184>/servo_model'
struct P_servo_model_sim_model_lib0_T_ {
  real32_T DiscretePIDController_LowerSatu;// Mask Parameter: DiscretePIDController_LowerSatu
                                           //  Referenced by:
                                           //    '<S218>/Saturate'
                                           //    '<S219>/DeadZone'

  real32_T DiscreteTimeIntegrator4_gainval;// Computed Parameter: DiscreteTimeIntegrator4_gainval
                                           //  Referenced by: '<S216>/Discrete-Time Integrator4'

  real32_T Backlash1_InitialOutput;    // Computed Parameter: Backlash1_InitialOutput
                                       //  Referenced by: '<S216>/Backlash1'

  real32_T Integrator_gainval;         // Computed Parameter: Integrator_gainval
                                       //  Referenced by: '<S218>/Integrator'

  real32_T Integrator_IC;              // Computed Parameter: Integrator_IC
                                       //  Referenced by: '<S218>/Integrator'

  real32_T Filter_gainval;             // Computed Parameter: Filter_gainval
                                       //  Referenced by: '<S218>/Filter'

  real32_T Filter_IC;                  // Computed Parameter: Filter_IC
                                       //  Referenced by: '<S218>/Filter'

  real32_T DiscreteTimeIntegrator3_gainval;// Computed Parameter: DiscreteTimeIntegrator3_gainval
                                           //  Referenced by: '<S216>/Discrete-Time Integrator3'

  real32_T DiscreteTimeIntegrator3_IC; // Expression: single(0)
                                       //  Referenced by: '<S216>/Discrete-Time Integrator3'

  real32_T ZeroGain_Gain;              // Computed Parameter: ZeroGain_Gain
                                       //  Referenced by: '<S219>/ZeroGain'

  real32_T Constant_Value;             // Computed Parameter: Constant_Value
                                       //  Referenced by: '<S218>/Constant'

};

// Parameters (auto storage)
struct P_sim_model_lib0_T_ {
  real_T abp_P_CG_B_B_error[3];        // Variable: abp_P_CG_B_B_error
                                       //  Referenced by:
                                       //    '<S188>/latch_nozzle_thrust_matricies'
                                       //    '<S221>/latch_nozzle_thrust_matricies'

  real_T astrobee_time_step_size;      // Variable: astrobee_time_step_size
                                       //  Referenced by:
                                       //    '<S8>/Constant'
                                       //    '<S80>/AR_pulse_gen'
                                       //    '<S80>/handrail_pulse_gen'
                                       //    '<S80>/landmark_pulse_gen'
                                       //    '<S254>/Gain1'
                                       //    '<S254>/Gain4'
                                       //    '<S255>/Gain3'
                                       //    '<S255>/Gain4'

  real_T bpm_PM1_nozzle_noise_feedback_gain[6];// Variable: bpm_PM1_nozzle_noise_feedback_gain
                                               //  Referenced by: '<S184>/blower_aerodynamics'

  real_T bpm_PM1_randn_noise_seed;     // Variable: bpm_PM1_randn_noise_seed
                                       //  Referenced by: '<S184>/blower_aerodynamics'

  real_T bpm_PM2_nozzle_noise_feedback_gain[6];// Variable: bpm_PM2_nozzle_noise_feedback_gain
                                               //  Referenced by: '<S185>/blower_aerodynamics'

  real_T bpm_PM2_randn_noise_seed;     // Variable: bpm_PM2_randn_noise_seed
                                       //  Referenced by: '<S185>/blower_aerodynamics'

  real_T bpm_servo_min_theta;          // Variable: bpm_servo_min_theta
                                       //  Referenced by:
                                       //    '<S184>/servo_model'
                                       //    '<S185>/servo_model'

  real_T cvs_AR_pixel_noise;           // Variable: cvs_AR_pixel_noise
                                       //  Referenced by: '<S85>/pinhole_projection_model'

  real_T cvs_AR_process_time;          // Variable: cvs_AR_process_time
                                       //  Referenced by: '<S80>/AR_pulse_gen'

  real_T cvs_handrail_noise_var;       // Variable: cvs_handrail_noise_var
                                       //  Referenced by: '<S113>/pixel_noise'

  real_T cvs_handrail_process_time;    // Variable: cvs_handrail_process_time
                                       //  Referenced by: '<S80>/handrail_pulse_gen'

  real_T cvs_landmark_process_time;    // Variable: cvs_landmark_process_time
                                       //  Referenced by: '<S80>/landmark_pulse_gen'

  real_T cvs_noise_seed;               // Variable: cvs_noise_seed
                                       //  Referenced by:
                                       //    '<S85>/pinhole_projection_model'
                                       //    '<S113>/pinhole_projection_model'
                                       //    '<S113>/pixel_noise'
                                       //    '<S137>/pinhole_projection_model'
                                       //    '<S161>/pinhole_projection_model'

  real_T env_ext_air_omega_variance[3];// Variable: env_ext_air_omega_variance
                                       //  Referenced by: '<S6>/Random Number1'

  real_T env_ext_air_vel_seed[3];      // Variable: env_ext_air_vel_seed
                                       //  Referenced by:
                                       //    '<S6>/Random Number'
                                       //    '<S6>/Random Number1'

  real_T env_ext_air_vel_variance[3];  // Variable: env_ext_air_vel_variance
                                       //  Referenced by: '<S6>/Random Number'

  real_T env_rotational_drag_coeff;    // Variable: env_rotational_drag_coeff
                                       //  Referenced by: '<S6>/Constant11'

  real_T epson_P_sensor_B_B_error[3];  // Variable: epson_P_sensor_B_B_error
                                       //  Referenced by: '<S257>/ 2'

  real_T epson_Q_B2accel_error[4];     // Variable: epson_Q_B2accel_error
                                       //  Referenced by: '<S257>/Constant13'

  real_T epson_Q_B2gyro_error[4];      // Variable: epson_Q_B2gyro_error
                                       //  Referenced by: '<S257>/Constant9'

  real_T epson_accel_filt_den;         // Variable: epson_accel_filt_den
                                       //  Referenced by: '<S256>/Discrete Transfer Fcn1'

  real_T epson_accel_filt_num;         // Variable: epson_accel_filt_num
                                       //  Referenced by: '<S256>/Discrete Transfer Fcn1'

  real_T epson_accel_lower_sat;        // Variable: epson_accel_lower_sat
                                       //  Referenced by: '<S256>/Saturation'

  real_T epson_accel_markov_tau;       // Variable: epson_accel_markov_tau
                                       //  Referenced by: '<S254>/Gain2'

  real_T epson_accel_noise_seed[3];    // Variable: epson_accel_noise_seed
                                       //  Referenced by:
                                       //    '<S254>/random_noise'
                                       //    '<S254>/random_noise1'
                                       //    '<S254>/random_noise2'

  real_T epson_accel_nonlinearity_coeff;// Variable: epson_accel_nonlinearity_coeff
                                        //  Referenced by: '<S254>/Constant6'

  real_T epson_accel_resolution;       // Variable: epson_accel_resolution
                                       //  Referenced by: '<S256>/Quantizer'

  real_T epson_accel_sf_coef;          // Variable: epson_accel_sf_coef
                                       //  Referenced by: '<S254>/scale_factor'

  real_T epson_accel_temp_bias_coeff[3];// Variable: epson_accel_temp_bias_coeff
                                        //  Referenced by: '<S254>/Constant2'

  real_T epson_accel_upper_sat;        // Variable: epson_accel_upper_sat
                                       //  Referenced by:
                                       //    '<S254>/Constant11'
                                       //    '<S256>/Saturation'

  real_T epson_gyro_bias_ic[3];        // Variable: epson_gyro_bias_ic
                                       //  Referenced by: '<S255>/Discrete-Time Integrator2'

  real_T epson_gyro_filt_den;          // Variable: epson_gyro_filt_den
                                       //  Referenced by: '<S256>/Discrete Transfer Fcn'

  real_T epson_gyro_filt_num;          // Variable: epson_gyro_filt_num
                                       //  Referenced by: '<S256>/Discrete Transfer Fcn'

  real_T epson_gyro_linear_accel_bias_coeff;// Variable: epson_gyro_linear_accel_bias_coeff
                                            //  Referenced by: '<S255>/Constant3'

  real_T epson_gyro_lower_sat;         // Variable: epson_gyro_lower_sat
                                       //  Referenced by: '<S256>/Saturation1'

  real_T epson_gyro_markov_tau;        // Variable: epson_gyro_markov_tau
                                       //  Referenced by: '<S255>/Gain2'

  real_T epson_gyro_noise_seed[3];     // Variable: epson_gyro_noise_seed
                                       //  Referenced by:
                                       //    '<S255>/random_noise'
                                       //    '<S255>/random_noise1'
                                       //    '<S255>/random_noise2'

  real_T epson_gyro_nonlinearity_coeff;// Variable: epson_gyro_nonlinearity_coeff
                                       //  Referenced by: '<S255>/Constant9'

  real_T epson_gyro_resolution;        // Variable: epson_gyro_resolution
                                       //  Referenced by: '<S256>/Quantizer1'

  real_T epson_gyro_sf_coef;           // Variable: epson_gyro_sf_coef
                                       //  Referenced by: '<S255>/scale_factor'

  real_T epson_gyro_temp_bias_coeff[3];// Variable: epson_gyro_temp_bias_coeff
                                       //  Referenced by: '<S255>/Constant8'

  real_T epson_gyro_upper_sat;         // Variable: epson_gyro_upper_sat
                                       //  Referenced by:
                                       //    '<S255>/Constant6'
                                       //    '<S256>/Saturation1'

  real_T epson_no_rot_effects;         // Variable: epson_no_rot_effects
                                       //  Referenced by: '<S257>/Constant4'

  real32_T abp_PM1_P_nozzle_B_B[18];   // Variable: abp_PM1_P_nozzle_B_B
                                       //  Referenced by: '<S188>/latch_nozzle_thrust_matricies'

  real32_T abp_PM1_discharge_coeff[6]; // Variable: abp_PM1_discharge_coeff
                                       //  Referenced by: '<S184>/blower_aerodynamics'

  real32_T abp_PM1_nozzle_orientations[18];// Variable: abp_PM1_nozzle_orientations
                                           //  Referenced by: '<S188>/latch_nozzle_thrust_matricies'

  real32_T abp_PM1_nozzle_widths[6];   // Variable: abp_PM1_nozzle_widths
                                       //  Referenced by: '<S184>/calc_nozzle_area'

  real32_T abp_PM2_P_nozzle_B_B[18];   // Variable: abp_PM2_P_nozzle_B_B
                                       //  Referenced by: '<S221>/latch_nozzle_thrust_matricies'

  real32_T abp_PM2_discharge_coeff[6]; // Variable: abp_PM2_discharge_coeff
                                       //  Referenced by: '<S185>/blower_aerodynamics'

  real32_T abp_PM2_nozzle_orientations[18];// Variable: abp_PM2_nozzle_orientations
                                           //  Referenced by: '<S221>/latch_nozzle_thrust_matricies'

  real32_T abp_PM2_nozzle_widths[6];   // Variable: abp_PM2_nozzle_widths
                                       //  Referenced by: '<S185>/calc_nozzle_area'

  real32_T abp_impeller_diameter;      // Variable: abp_impeller_diameter
                                       //  Referenced by:
                                       //    '<S184>/blower_aerodynamics'
                                       //    '<S185>/blower_aerodynamics'

  real32_T abp_nozzle_flap_count;      // Variable: abp_nozzle_flap_count
                                       //  Referenced by:
                                       //    '<S184>/calc_nozzle_area'
                                       //    '<S185>/calc_nozzle_area'

  real32_T abp_nozzle_flap_length;     // Variable: abp_nozzle_flap_length
                                       //  Referenced by:
                                       //    '<S184>/calc_nozzle_area'
                                       //    '<S185>/calc_nozzle_area'

  real32_T abp_nozzle_gear_ratio;      // Variable: abp_nozzle_gear_ratio
                                       //  Referenced by:
                                       //    '<S184>/calc_nozzle_area'
                                       //    '<S185>/calc_nozzle_area'

  real32_T abp_nozzle_intake_height;   // Variable: abp_nozzle_intake_height
                                       //  Referenced by:
                                       //    '<S184>/calc_nozzle_area'
                                       //    '<S185>/calc_nozzle_area'

  real32_T abp_nozzle_min_open_angle;  // Variable: abp_nozzle_min_open_angle
                                       //  Referenced by:
                                       //    '<S184>/calc_nozzle_area'
                                       //    '<S185>/calc_nozzle_area'

  real32_T abp_pm1_impeller_orientation[3];// Variable: abp_pm1_impeller_orientation
                                           //  Referenced by: '<S188>/Constant1'

  real32_T abp_pm1_zero_thrust_area;   // Variable: abp_pm1_zero_thrust_area
                                       //  Referenced by: '<S184>/blower_aerodynamics'

  real32_T abp_pm2_impeller_orientation[3];// Variable: abp_pm2_impeller_orientation
                                           //  Referenced by: '<S221>/Constant1'

  real32_T abp_pm2_zero_thrust_area;   // Variable: abp_pm2_zero_thrust_area
                                       //  Referenced by: '<S185>/blower_aerodynamics'

  real32_T bmp_PM1_impeller_orientation_error[3];// Variable: bmp_PM1_impeller_orientation_error
                                                 //  Referenced by: '<S188>/Constant3'

  real32_T bmp_PM2_impeller_orientation_error[3];// Variable: bmp_PM2_impeller_orientation_error
                                                 //  Referenced by: '<S221>/Constant3'

  real32_T bpm_PM1_P_nozzle_B_B_error[18];// Variable: bpm_PM1_P_nozzle_B_B_error
                                          //  Referenced by: '<S188>/latch_nozzle_thrust_matricies'

  real32_T bpm_PM1_Q_nozzle2misaligned[24];// Variable: bpm_PM1_Q_nozzle2misaligned
                                           //  Referenced by: '<S188>/latch_nozzle_thrust_matricies'

  real32_T bpm_PM1_nozzle_discharge_coeff_error[6];// Variable: bpm_PM1_nozzle_discharge_coeff_error
                                                   //  Referenced by: '<S184>/blower_aerodynamics'

  real32_T bpm_PM1_zero_thrust_area_error;// Variable: bpm_PM1_zero_thrust_area_error
                                          //  Referenced by: '<S184>/blower_aerodynamics'

  real32_T bpm_PM2_P_nozzle_B_B_error[18];// Variable: bpm_PM2_P_nozzle_B_B_error
                                          //  Referenced by: '<S221>/latch_nozzle_thrust_matricies'

  real32_T bpm_PM2_Q_nozzle2misaligned[24];// Variable: bpm_PM2_Q_nozzle2misaligned
                                           //  Referenced by: '<S221>/latch_nozzle_thrust_matricies'

  real32_T bpm_PM2_nozzle_discharge_coeff_error[6];// Variable: bpm_PM2_nozzle_discharge_coeff_error
                                                   //  Referenced by: '<S185>/blower_aerodynamics'

  real32_T bpm_PM2_zero_thrust_area_error;// Variable: bpm_PM2_zero_thrust_area_error
                                          //  Referenced by: '<S185>/blower_aerodynamics'

  real32_T bpm_imp_ctl_filt_n;         // Variable: bpm_imp_ctl_filt_n
                                       //  Referenced by:
                                       //    '<S190>/speed_controller'
                                       //    '<S223>/speed_controller'

  real32_T bpm_imp_ctl_kd;             // Variable: bpm_imp_ctl_kd
                                       //  Referenced by:
                                       //    '<S190>/speed_controller'
                                       //    '<S223>/speed_controller'

  real32_T bpm_imp_ctl_ki;             // Variable: bpm_imp_ctl_ki
                                       //  Referenced by:
                                       //    '<S190>/speed_controller'
                                       //    '<S223>/speed_controller'

  real32_T bpm_imp_ctl_kp;             // Variable: bpm_imp_ctl_kp
                                       //  Referenced by:
                                       //    '<S190>/speed_controller'
                                       //    '<S223>/speed_controller'

  real32_T bpm_imp_max_voltage;        // Variable: bpm_imp_max_voltage
                                       //  Referenced by:
                                       //    '<S190>/speed_controller'
                                       //    '<S223>/speed_controller'

  real32_T bpm_imp_motor_friction_coeff;// Variable: bpm_imp_motor_friction_coeff
                                        //  Referenced by:
                                        //    '<S190>/dc_motor_model'
                                        //    '<S223>/dc_motor_model'

  real32_T bpm_imp_motor_r;            // Variable: bpm_imp_motor_r
                                       //  Referenced by:
                                       //    '<S190>/dc_motor_model'
                                       //    '<S223>/dc_motor_model'

  real32_T bpm_imp_motor_speed_k;      // Variable: bpm_imp_motor_speed_k
                                       //  Referenced by:
                                       //    '<S190>/dc_motor_model'
                                       //    '<S223>/dc_motor_model'

  real32_T bpm_imp_motor_torque_k;     // Variable: bpm_imp_motor_torque_k
                                       //  Referenced by:
                                       //    '<S190>/dc_motor_model'
                                       //    '<S223>/dc_motor_model'

  real32_T bpm_imp_speed_filt_den;     // Variable: bpm_imp_speed_filt_den
                                       //  Referenced by:
                                       //    '<S190>/speed_controller'
                                       //    '<S223>/speed_controller'

  real32_T bpm_imp_speed_filt_num;     // Variable: bpm_imp_speed_filt_num
                                       //  Referenced by:
                                       //    '<S190>/speed_controller'
                                       //    '<S223>/speed_controller'

  real32_T bpm_impeller_inertia;       // Variable: bpm_impeller_inertia
                                       //  Referenced by:
                                       //    '<S188>/Constant2'
                                       //    '<S191>/Gain'
                                       //    '<S191>/Gain2'
                                       //    '<S221>/Constant2'
                                       //    '<S224>/Gain'
                                       //    '<S224>/Gain2'

  real32_T bpm_impeller_inertia_error; // Variable: bpm_impeller_inertia_error
                                       //  Referenced by:
                                       //    '<S188>/Constant4'
                                       //    '<S191>/Gain2'
                                       //    '<S221>/Constant4'
                                       //    '<S224>/Gain2'

  real32_T bpm_impeller_init_speed;    // Variable: bpm_impeller_init_speed
                                       //  Referenced by:
                                       //    '<S191>/Discrete-Time Integrator'
                                       //    '<S224>/Discrete-Time Integrator'

  real32_T bpm_lookup_Cdp_data[334];   // Variable: bpm_lookup_Cdp_data
                                       //  Referenced by:
                                       //    '<S184>/blower_aerodynamics'
                                       //    '<S185>/blower_aerodynamics'

  real32_T bpm_lookup_totalarea_breakpoints[334];// Variable: bpm_lookup_totalarea_breakpoints
                                                 //  Referenced by:
                                                 //    '<S184>/blower_aerodynamics'
                                                 //    '<S185>/blower_aerodynamics'

  real32_T bpm_servo_ctl_filt_n;       // Variable: bpm_servo_ctl_filt_n
                                       //  Referenced by:
                                       //    '<S184>/servo_model'
                                       //    '<S185>/servo_model'

  real32_T bpm_servo_ctl_kd;           // Variable: bpm_servo_ctl_kd
                                       //  Referenced by:
                                       //    '<S184>/servo_model'
                                       //    '<S185>/servo_model'

  real32_T bpm_servo_ctl_ki;           // Variable: bpm_servo_ctl_ki
                                       //  Referenced by:
                                       //    '<S184>/servo_model'
                                       //    '<S185>/servo_model'

  real32_T bpm_servo_ctl_kp;           // Variable: bpm_servo_ctl_kp
                                       //  Referenced by:
                                       //    '<S184>/servo_model'
                                       //    '<S185>/servo_model'

  real32_T bpm_servo_max_theta;        // Variable: bpm_servo_max_theta
                                       //  Referenced by:
                                       //    '<S184>/servo_model'
                                       //    '<S185>/servo_model'

  real32_T bpm_servo_max_voltage;      // Variable: bpm_servo_max_voltage
                                       //  Referenced by:
                                       //    '<S184>/servo_model'
                                       //    '<S185>/servo_model'

  real32_T bpm_servo_motor_backlash_deadband;// Variable: bpm_servo_motor_backlash_deadband
                                             //  Referenced by:
                                             //    '<S184>/servo_model'
                                             //    '<S185>/servo_model'

  real32_T bpm_servo_motor_friction_coeff;// Variable: bpm_servo_motor_friction_coeff
                                          //  Referenced by:
                                          //    '<S184>/servo_model'
                                          //    '<S185>/servo_model'

  real32_T bpm_servo_motor_gear_box_inertia;// Variable: bpm_servo_motor_gear_box_inertia
                                            //  Referenced by:
                                            //    '<S184>/servo_model'
                                            //    '<S185>/servo_model'

  real32_T bpm_servo_motor_gear_ratio; // Variable: bpm_servo_motor_gear_ratio
                                       //  Referenced by:
                                       //    '<S184>/servo_model'
                                       //    '<S185>/servo_model'

  real32_T bpm_servo_motor_k;          // Variable: bpm_servo_motor_k
                                       //  Referenced by:
                                       //    '<S184>/servo_model'
                                       //    '<S185>/servo_model'

  real32_T bpm_servo_motor_r;          // Variable: bpm_servo_motor_r
                                       //  Referenced by:
                                       //    '<S184>/servo_model'
                                       //    '<S185>/servo_model'

  real32_T bpm_servo_pwm2angle;        // Variable: bpm_servo_pwm2angle
                                       //  Referenced by:
                                       //    '<S184>/servo_model'
                                       //    '<S185>/servo_model'

  real32_T bpm_servo_pwm2angle_bias;   // Variable: bpm_servo_pwm2angle_bias
                                       //  Referenced by:
                                       //    '<S184>/servo_model'
                                       //    '<S185>/servo_model'

  real32_T const_air_density;          // Variable: const_air_density
                                       //  Referenced by:
                                       //    '<S184>/blower_aerodynamics'
                                       //    '<S185>/blower_aerodynamics'

  real32_T const_gravity_local[3];     // Variable: const_gravity_local
                                       //  Referenced by: '<S6>/Constant2'

  real32_T cvs_AR_map_error[108];      // Variable: cvs_AR_map_error
                                       //  Referenced by: '<S88>/Constant4'

  real32_T cvs_AR_map_iss[108];        // Variable: cvs_AR_map_iss
                                       //  Referenced by: '<S85>/Constant3'

  real32_T cvs_dockcam_P_B_B_error[3]; // Variable: cvs_dockcam_P_B_B_error
                                       //  Referenced by: '<S88>/Constant1'

  real32_T cvs_dockcam_Q_B2dockcam_error[4];// Variable: cvs_dockcam_Q_B2dockcam_error
                                            //  Referenced by: '<S88>/Constant6'

  real32_T cvs_dockcam_max_dist;       // Variable: cvs_dockcam_max_dist
                                       //  Referenced by: '<S85>/pinhole_projection_model'

  real32_T cvs_dockcam_min_dist;       // Variable: cvs_dockcam_min_dist
                                       //  Referenced by: '<S85>/pinhole_projection_model'

  real32_T cvs_dockcam_pointing[3];    // Variable: cvs_dockcam_pointing
                                       //  Referenced by: '<S85>/pinhole_projection_model'

  real32_T cvs_handrail_map_error[1512];// Variable: cvs_handrail_map_error
                                        //  Referenced by: '<S116>/Constant4'

  real32_T cvs_handrail_map_iss[1512]; // Variable: cvs_handrail_map_iss
                                       //  Referenced by: '<S113>/Constant3'

  real32_T cvs_landmark_map_error[25896];// Variable: cvs_landmark_map_error
                                         //  Referenced by: '<S140>/Constant4'

  real32_T cvs_landmark_map_iss[25896];// Variable: cvs_landmark_map_iss
                                       //  Referenced by: '<S137>/Constant3'

  real32_T cvs_navcam_P_B_B_error[3];  // Variable: cvs_navcam_P_B_B_error
                                       //  Referenced by:
                                       //    '<S140>/Constant1'
                                       //    '<S164>/Constant1'

  real32_T cvs_navcam_Q_B2navcan_error[4];// Variable: cvs_navcam_Q_B2navcan_error
                                          //  Referenced by:
                                          //    '<S140>/Constant6'
                                          //    '<S164>/Constant6'

  real32_T cvs_navcam_max_dist;        // Variable: cvs_navcam_max_dist
                                       //  Referenced by:
                                       //    '<S137>/pinhole_projection_model'
                                       //    '<S161>/pinhole_projection_model'

  real32_T cvs_navcam_min_dist;        // Variable: cvs_navcam_min_dist
                                       //  Referenced by:
                                       //    '<S137>/pinhole_projection_model'
                                       //    '<S161>/pinhole_projection_model'

  real32_T cvs_navcam_pointing[3];     // Variable: cvs_navcam_pointing
                                       //  Referenced by:
                                       //    '<S137>/pinhole_projection_model'
                                       //    '<S161>/pinhole_projection_model'

  real32_T cvs_optflow_map_error[11937];// Variable: cvs_optflow_map_error
                                        //  Referenced by: '<S164>/Constant4'

  real32_T cvs_optflow_map_iss[11937]; // Variable: cvs_optflow_map_iss
                                       //  Referenced by: '<S161>/Constant3'

  real32_T cvs_perchcam_P_B_B_error[3];// Variable: cvs_perchcam_P_B_B_error
                                       //  Referenced by: '<S116>/Constant1'

  real32_T cvs_perchcam_Q_B2perchcam_error[4];// Variable: cvs_perchcam_Q_B2perchcam_error
                                              //  Referenced by: '<S116>/Constant6'

  real32_T cvs_perchcam_max_dist;      // Variable: cvs_perchcam_max_dist
                                       //  Referenced by: '<S113>/pinhole_projection_model'

  real32_T cvs_perchcam_min_dist;      // Variable: cvs_perchcam_min_dist
                                       //  Referenced by: '<S113>/pinhole_projection_model'

  real32_T cvs_perchcam_pointing[3];   // Variable: cvs_perchcam_pointing
                                       //  Referenced by: '<S113>/pinhole_projection_model'

  real32_T env_avg_drag_coeff;         // Variable: env_avg_drag_coeff
                                       //  Referenced by: '<S6>/Constant1'

  real32_T env_max_ext_air_omega;      // Variable: env_max_ext_air_omega
                                       //  Referenced by: '<S6>/Discrete-Time Integrator1'

  real32_T env_max_ext_air_vel;        // Variable: env_max_ext_air_vel
                                       //  Referenced by:
                                       //    '<S6>/Constant9'
                                       //    '<S6>/Discrete-Time Integrator'

  real32_T epson_accel_bias_ic[3];     // Variable: epson_accel_bias_ic
                                       //  Referenced by: '<S254>/Discrete-Time Integrator2'

  real32_T mlp_command_list[228];      // Variable: mlp_command_list
                                       //  Referenced by: '<S68>/Constant1'

  real32_T mlp_dummy_state_cmd[19];    // Variable: mlp_dummy_state_cmd
                                       //  Referenced by: '<S74>/Constant'

  real32_T mlp_num_commands;           // Variable: mlp_num_commands
                                       //  Referenced by: '<S74>/Constant1'

  real32_T tun_abp_p_dockcam_body_body_sim[3];// Variable: tun_abp_p_dockcam_body_body_sim
                                              //  Referenced by: '<S88>/Constant2'

  real32_T tun_abp_p_imu_body_body[3]; // Variable: tun_abp_p_imu_body_body
                                       //  Referenced by: '<S257>/ 1'

  real32_T tun_abp_p_navcam_body_body_sim[3];// Variable: tun_abp_p_navcam_body_body_sim
                                             //  Referenced by:
                                             //    '<S140>/Constant2'
                                             //    '<S164>/Constant2'

  real32_T tun_abp_p_perchcam_body_body_sim[3];// Variable: tun_abp_p_perchcam_body_body_sim
                                               //  Referenced by: '<S116>/Constant2'

  real32_T tun_abp_q_body2dockcam[4];  // Variable: tun_abp_q_body2dockcam
                                       //  Referenced by: '<S88>/Constant3'

  real32_T tun_abp_q_body2navcam[4];   // Variable: tun_abp_q_body2navcam
                                       //  Referenced by:
                                       //    '<S140>/Constant3'
                                       //    '<S164>/Constant3'

  real32_T tun_abp_q_body2perchcam[4]; // Variable: tun_abp_q_body2perchcam
                                       //  Referenced by: '<S116>/Constant3'

  real32_T tun_abp_quat_body2imu[4];   // Variable: tun_abp_quat_body2imu
                                       //  Referenced by:
                                       //    '<S257>/Constant2'
                                       //    '<S257>/Constant8'

  real32_T tun_ase_gravity_accel[3];   // Variable: tun_ase_gravity_accel
                                       //  Referenced by: '<S186>/Constant4'

  real32_T tun_bpm_PM1_thrust_error_sf;// Variable: tun_bpm_PM1_thrust_error_sf
                                       //  Referenced by: '<S184>/blower_aerodynamics'

  real32_T tun_bpm_PM2_thrust_error_sf;// Variable: tun_bpm_PM2_thrust_error_sf
                                       //  Referenced by: '<S185>/blower_aerodynamics'

  real32_T tun_bpm_noise_on_flag;      // Variable: tun_bpm_noise_on_flag
                                       //  Referenced by:
                                       //    '<S184>/blower_aerodynamics'
                                       //    '<S185>/blower_aerodynamics'
                                       //    '<S188>/latch_nozzle_thrust_matricies'
                                       //    '<S188>/Gain'
                                       //    '<S188>/Gain1'
                                       //    '<S191>/Constant'
                                       //    '<S221>/latch_nozzle_thrust_matricies'
                                       //    '<S221>/Gain'
                                       //    '<S221>/Gain1'
                                       //    '<S224>/Constant'

  real32_T tun_cvs_dockcam_focal_length_X;// Variable: tun_cvs_dockcam_focal_length_X
                                          //  Referenced by: '<S85>/pinhole_projection_model'

  real32_T tun_cvs_dockcam_focal_length_Y;// Variable: tun_cvs_dockcam_focal_length_Y
                                          //  Referenced by: '<S85>/pinhole_projection_model'

  real32_T tun_cvs_dockcam_num_pixels_X;// Variable: tun_cvs_dockcam_num_pixels_X
                                        //  Referenced by: '<S85>/pinhole_projection_model'

  real32_T tun_cvs_dockcam_num_pixels_Y;// Variable: tun_cvs_dockcam_num_pixels_Y
                                        //  Referenced by: '<S85>/pinhole_projection_model'

  real32_T tun_cvs_navcam_focal_length_X;// Variable: tun_cvs_navcam_focal_length_X
                                         //  Referenced by:
                                         //    '<S137>/pinhole_projection_model'
                                         //    '<S161>/pinhole_projection_model'

  real32_T tun_cvs_navcam_focal_length_Y;// Variable: tun_cvs_navcam_focal_length_Y
                                         //  Referenced by:
                                         //    '<S137>/pinhole_projection_model'
                                         //    '<S161>/pinhole_projection_model'

  real32_T tun_cvs_navcam_num_pixels_X;// Variable: tun_cvs_navcam_num_pixels_X
                                       //  Referenced by:
                                       //    '<S137>/pinhole_projection_model'
                                       //    '<S161>/pinhole_projection_model'

  real32_T tun_cvs_navcam_num_pixels_Y;// Variable: tun_cvs_navcam_num_pixels_Y
                                       //  Referenced by:
                                       //    '<S137>/pinhole_projection_model'
                                       //    '<S161>/pinhole_projection_model'

  real32_T tun_cvs_perchcam_focal_length_X;// Variable: tun_cvs_perchcam_focal_length_X
                                           //  Referenced by: '<S113>/pinhole_projection_model'

  real32_T tun_cvs_perchcam_focal_length_Y;// Variable: tun_cvs_perchcam_focal_length_Y
                                           //  Referenced by: '<S113>/pinhole_projection_model'

  real32_T tun_cvs_perchcam_num_pixels_X;// Variable: tun_cvs_perchcam_num_pixels_X
                                         //  Referenced by: '<S113>/pinhole_projection_model'

  real32_T tun_cvs_perchcam_num_pixels_Y;// Variable: tun_cvs_perchcam_num_pixels_Y
                                         //  Referenced by: '<S113>/pinhole_projection_model'

  real32_T tun_env_accel_dof_gain[3];  // Variable: tun_env_accel_dof_gain
                                       //  Referenced by: '<S7>/Gain'

  real32_T tun_env_alpha_dof_gain[3];  // Variable: tun_env_alpha_dof_gain
                                       //  Referenced by: '<S5>/Gain'

  real32_T tun_inertia_error_mat[9];   // Variable: tun_inertia_error_mat
                                       //  Referenced by: '<S2>/Constant1'

  real32_T tun_ini_P_B_ISS_ISS[3];     // Variable: tun_ini_P_B_ISS_ISS
                                       //  Referenced by: '<S7>/Constant11'

  real32_T tun_ini_Q_ISS2B[4];         // Variable: tun_ini_Q_ISS2B
                                       //  Referenced by: '<S5>/Constant1'

  real32_T tun_ini_V_B_ISS_ISS[3];     // Variable: tun_ini_V_B_ISS_ISS
                                       //  Referenced by: '<S7>/Constant12'

  real32_T tun_ini_omega_B_ISS_B[3];   // Variable: tun_ini_omega_B_ISS_B
                                       //  Referenced by: '<S5>/Constant3'

  real32_T tun_iss_omega_ISS_ECI_ISS[3];// Variable: tun_iss_omega_ISS_ECI_ISS
                                        //  Referenced by:
                                        //    '<S5>/Constant4'
                                        //    '<S7>/Constant1'

  real32_T tun_mass_error;             // Variable: tun_mass_error
                                       //  Referenced by: '<S2>/Constant'

  real32_T tun_sim_cg_error[3];        // Variable: tun_sim_cg_error
                                       //  Referenced by: '<S4>/Constant'

  uint32_T cvs_AR_valid_times[6];      // Variable: cvs_AR_valid_times
                                       //  Referenced by: '<S84>/Constant1'

  uint32_T cvs_handrail_valid_times[6];// Variable: cvs_handrail_valid_times
                                       //  Referenced by: '<S112>/Constant1'

  uint32_T cvs_landmark_valid_times[6];// Variable: cvs_landmark_valid_times
                                       //  Referenced by: '<S136>/Constant1'

  uint32_T cvs_optflow_valid_times[6]; // Variable: cvs_optflow_valid_times
                                       //  Referenced by: '<S160>/Constant1'

  uint32_T ini_time_nanoseconds;       // Variable: ini_time_nanoseconds
                                       //  Referenced by: '<S8>/Unit Delay'

  uint32_T ini_time_seconds;           // Variable: ini_time_seconds
                                       //  Referenced by: '<S8>/Unit Delay1'

  uint32_T mlp_command_times[24];      // Variable: mlp_command_times
                                       //  Referenced by: '<S68>/Constant2'

  uint32_T mlp_loc_mode_cmd_times[6];  // Variable: mlp_loc_mode_cmd_times
                                       //  Referenced by: '<S68>/Constant5'

  uint32_T mlp_mode_cmd_times[6];      // Variable: mlp_mode_cmd_times
                                       //  Referenced by: '<S68>/Constant3'

  uint32_T mlp_speed_gain_cmd_times[6];// Variable: mlp_speed_gain_cmd_times
                                       //  Referenced by: '<S68>/Constant7'

  uint8_T mlp_loc_mode_cmd_list[3];    // Variable: mlp_loc_mode_cmd_list
                                       //  Referenced by: '<S68>/Constant6'

  uint8_T mlp_mode_cmd_list[3];        // Variable: mlp_mode_cmd_list
                                       //  Referenced by: '<S68>/Constant4'

  uint8_T mlp_speed_gain_cmd_list[3];  // Variable: mlp_speed_gain_cmd_list
                                       //  Referenced by: '<S68>/Constant8'

  uint8_T tun_ase_gravity_removal;     // Variable: tun_ase_gravity_removal
                                       //  Referenced by: '<S257>/Constant6'

  uint8_T tun_cvs_noise_on;            // Variable: tun_cvs_noise_on
                                       //  Referenced by:
                                       //    '<S85>/pinhole_projection_model'
                                       //    '<S85>/Gain'
                                       //    '<S113>/pinhole_projection_model'
                                       //    '<S113>/Gain'
                                       //    '<S137>/pinhole_projection_model'
                                       //    '<S137>/Gain'
                                       //    '<S161>/pinhole_projection_model'
                                       //    '<S161>/Gain'
                                       //    '<S88>/Constant'
                                       //    '<S88>/Gain'
                                       //    '<S88>/Gain1'
                                       //    '<S116>/Constant'
                                       //    '<S116>/Gain'
                                       //    '<S116>/Gain1'
                                       //    '<S140>/Constant'
                                       //    '<S140>/Gain'
                                       //    '<S140>/Gain1'
                                       //    '<S164>/Constant'
                                       //    '<S164>/Gain'
                                       //    '<S164>/Gain1'

  uint8_T tun_env_drag_disturb_on;     // Variable: tun_env_drag_disturb_on
                                       //  Referenced by:
                                       //    '<S6>/Constant6'
                                       //    '<S6>/Constant8'

  uint8_T tun_epson_report_truth;      // Variable: tun_epson_report_truth
                                       //  Referenced by:
                                       //    '<S254>/Constant3'
                                       //    '<S255>/Constant1'
                                       //    '<S257>/Constant11'
                                       //    '<S257>/Constant3'
                                       //    '<S257>/Gain'

  boolean_T cvs_AR_valid_mask[3];      // Variable: cvs_AR_valid_mask
                                       //  Referenced by: '<S84>/Constant3'

  boolean_T cvs_handrail_valid_mask[3];// Variable: cvs_handrail_valid_mask
                                       //  Referenced by: '<S112>/Constant3'

  boolean_T cvs_landmark_valid_mask[3];// Variable: cvs_landmark_valid_mask
                                       //  Referenced by: '<S136>/Constant3'

  boolean_T cvs_optflow_valid_mask[3]; // Variable: cvs_optflow_valid_mask
                                       //  Referenced by: '<S160>/Constant3'

  boolean_T env_grav_disturb_on;       // Variable: env_grav_disturb_on
                                       //  Referenced by: '<S6>/Constant3'

  real_T ARtag_image_processing_handrail;// Mask Parameter: ARtag_image_processing_handrail
                                         //  Referenced by: '<S85>/pixel_noise'

  real_T landmark_image_processing_handr;// Mask Parameter: landmark_image_processing_handr
                                         //  Referenced by: '<S137>/pixel_noise'

  real_T optical_flow_image_processing_h;// Mask Parameter: optical_flow_image_processing_h
                                         //  Referenced by: '<S161>/pixel_noise'

  real_T ARtag_image_processing_handra_k;// Mask Parameter: ARtag_image_processing_handra_k
                                         //  Referenced by: '<S85>/pixel_noise'

  real_T landmark_image_processing_han_h;// Mask Parameter: landmark_image_processing_han_h
                                         //  Referenced by: '<S137>/pixel_noise'

  real_T optical_flow_image_processing_b;// Mask Parameter: optical_flow_image_processing_b
                                         //  Referenced by: '<S161>/pixel_noise'

  real_T handrail_image_processing_pixel;// Mask Parameter: handrail_image_processing_pixel
                                         //  Referenced by: '<S113>/pinhole_projection_model'

  real_T landmark_image_processing_pixel;// Mask Parameter: landmark_image_processing_pixel
                                         //  Referenced by: '<S137>/pinhole_projection_model'

  real_T optical_flow_image_processing_p;// Mask Parameter: optical_flow_image_processing_p
                                         //  Referenced by: '<S161>/pinhole_projection_model'

  real32_T CompareToConstant1_const;   // Mask Parameter: CompareToConstant1_const
                                       //  Referenced by: '<S23>/Constant'

  real32_T CompareToConstant_const;    // Mask Parameter: CompareToConstant_const
                                       //  Referenced by: '<S22>/Constant'

  real32_T bpm_blower_1_propulsion_module_;// Mask Parameter: bpm_blower_1_propulsion_module_
                                           //  Referenced by: '<S190>/speed_controller'

  real32_T bpm_blower_2_propulsion_module_;// Mask Parameter: bpm_blower_2_propulsion_module_
                                           //  Referenced by: '<S223>/speed_controller'

  real32_T DetectChange_vinit;         // Mask Parameter: DetectChange_vinit
                                       //  Referenced by: '<S194>/Delay Input1'

  real32_T DetectChange_vinit_o;       // Mask Parameter: DetectChange_vinit_o
                                       //  Referenced by: '<S227>/Delay Input1'

  uint32_T CompareToConstant_const_d;  // Mask Parameter: CompareToConstant_const_d
                                       //  Referenced by: '<S67>/Constant'

  uint32_T DetectChange_vinit_j;       // Mask Parameter: DetectChange_vinit_j
                                       //  Referenced by: '<S110>/Delay Input1'

  uint32_T DetectChange1_vinit;        // Mask Parameter: DetectChange1_vinit
                                       //  Referenced by: '<S111>/Delay Input1'

  bpm_msg Delay1_InitialCondition;     // Computed Parameter: Delay1_InitialCondition
                                       //  Referenced by: '<S1>/Delay1'

  real_T Constant1_Value;              // Expression: -1
                                       //  Referenced by: '<S16>/Constant1'

  real_T WeightedSampleTimeMath_WtEt;  // Computed Parameter: WeightedSampleTimeMath_WtEt
                                       //  Referenced by: '<S26>/Weighted Sample Time Math'

  real_T Gain_Gain;                    // Expression: .5
                                       //  Referenced by: '<S26>/Gain'

  real_T Constant2_Value[16];          // Expression: zeros(16,1)
                                       //  Referenced by: '<S25>/Constant2'

  real_T WeightedSampleTimeMath_WtEt_o;// Computed Parameter: WeightedSampleTimeMath_WtEt_o
                                       //  Referenced by: '<S25>/Weighted Sample Time Math'

  real_T Gain_Gain_j;                  // Expression: .5
                                       //  Referenced by: '<S25>/Gain'

  real_T Constant2_Value_m[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S284>/Constant2'

  real_T Constant2_Value_i[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S270>/Constant2'

  real_T Constant3_Value;              // Expression: 0
                                       //  Referenced by: '<S272>/Constant3'

  real_T Gain_Gain_p;                  // Expression: -1
                                       //  Referenced by: '<S272>/Gain'

  real_T Gain1_Gain;                   // Expression: -1
                                       //  Referenced by: '<S272>/Gain1'

  real_T Gain2_Gain;                   // Expression: -1
                                       //  Referenced by: '<S272>/Gain2'

  real_T Gain1_Gain_a;                 // Expression: -1
                                       //  Referenced by: '<S270>/Gain1'

  real_T Constant1_Value_o[4];         // Expression: [0 0 0 1]
                                       //  Referenced by: '<S257>/Constant1'

  real_T Gain_Gain_f;                  // Expression: 2
                                       //  Referenced by: '<S284>/Gain'

  real_T Constant1_Value_c;            // Expression: 1
                                       //  Referenced by: '<S284>/Constant1'

  real_T Gain1_Gain_d;                 // Expression: 2
                                       //  Referenced by: '<S284>/Gain1'

  real_T Constant3_Value_n;            // Expression: 0
                                       //  Referenced by: '<S287>/Constant3'

  real_T Gain_Gain_fd;                 // Expression: -1
                                       //  Referenced by: '<S287>/Gain'

  real_T Gain1_Gain_l;                 // Expression: -1
                                       //  Referenced by: '<S287>/Gain1'

  real_T Gain2_Gain_a;                 // Expression: -1
                                       //  Referenced by: '<S287>/Gain2'

  real_T Gain2_Gain_d;                 // Expression: 2
                                       //  Referenced by: '<S284>/Gain2'

  real_T Constant5_Value;              // Expression: 0
                                       //  Referenced by: '<S257>/Constant5'

  real_T Constant2_Value_m1[9];        // Expression: zeros(9,1)
                                       //  Referenced by: '<S62>/Constant2'

  real_T Constant1_Value_oo;           // Expression: 1
                                       //  Referenced by: '<S62>/Constant1'

  real_T Constant3_Value_k;            // Expression: 0
                                       //  Referenced by: '<S65>/Constant3'

  real_T Constant2_Value_d[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S57>/Constant2'

  real_T Constant1_Value_i;            // Expression: 1
                                       //  Referenced by: '<S57>/Constant1'

  real_T Constant3_Value_a;            // Expression: 0
                                       //  Referenced by: '<S60>/Constant3'

  real_T Constant2_Value_p[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S52>/Constant2'

  real_T Constant1_Value_c4;           // Expression: 1
                                       //  Referenced by: '<S52>/Constant1'

  real_T Constant3_Value_b;            // Expression: 0
                                       //  Referenced by: '<S55>/Constant3'

  real_T Constant2_Value_c[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S35>/Constant2'

  real_T Constant1_Value_o5;           // Expression: 1
                                       //  Referenced by: '<S35>/Constant1'

  real_T Constant3_Value_j;            // Expression: 0
                                       //  Referenced by: '<S38>/Constant3'

  real_T Constant3_Value_e;            // Expression: 0
                                       //  Referenced by: '<S13>/Constant3'

  real_T Constant2_Value_cr[9];        // Expression: zeros(9,1)
                                       //  Referenced by: '<S279>/Constant2'

  real_T Gain_Gain_e;                  // Expression: 2
                                       //  Referenced by: '<S279>/Gain'

  real_T Constant1_Value_b;            // Expression: 1
                                       //  Referenced by: '<S279>/Constant1'

  real_T Gain1_Gain_o;                 // Expression: 2
                                       //  Referenced by: '<S279>/Gain1'

  real_T Constant3_Value_d;            // Expression: 0
                                       //  Referenced by: '<S282>/Constant3'

  real_T Gain_Gain_l;                  // Expression: -1
                                       //  Referenced by: '<S282>/Gain'

  real_T Gain1_Gain_oh;                // Expression: -1
                                       //  Referenced by: '<S282>/Gain1'

  real_T Gain2_Gain_o;                 // Expression: -1
                                       //  Referenced by: '<S282>/Gain2'

  real_T Gain2_Gain_ar;                // Expression: 2
                                       //  Referenced by: '<S279>/Gain2'

  real_T Constant1_Value_p;            // Expression: 2*pi
                                       //  Referenced by: '<S254>/Constant1'

  real_T random_noise_Mean;            // Expression: 0
                                       //  Referenced by: '<S254>/random_noise'

  real_T random_noise_StdDev;          // Computed Parameter: random_noise_StdDev
                                       //  Referenced by: '<S254>/random_noise'

  real_T Constant_Value;               // Expression: 25
                                       //  Referenced by: '<S254>/Constant'

  real_T DiscreteTimeIntegrator2_gainval;// Computed Parameter: DiscreteTimeIntegrator2_gainval
                                         //  Referenced by: '<S254>/Discrete-Time Integrator2'

  real_T DiscreteTimeIntegrator1_gainval;// Computed Parameter: DiscreteTimeIntegrator1_gainval
                                         //  Referenced by: '<S254>/Discrete-Time Integrator1'

  real_T DiscreteTimeIntegrator1_IC;   // Expression: 0
                                       //  Referenced by: '<S254>/Discrete-Time Integrator1'

  real_T DiscreteTransferFcn1_InitialSta;// Expression: 0
                                         //  Referenced by: '<S256>/Discrete Transfer Fcn1'

  real_T Delay2_InitialCondition;      // Expression: 0.0
                                       //  Referenced by: '<S256>/Delay2'

  real_T Delay2_InitialCondition_o;    // Expression: 0.0
                                       //  Referenced by: '<S253>/Delay2'

  real_T Constant2_Value_dr[9];        // Expression: zeros(9,1)
                                       //  Referenced by: '<S274>/Constant2'

  real_T Constant2_Value_n[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S266>/Constant2'

  real_T Constant3_Value_i;            // Expression: 0
                                       //  Referenced by: '<S268>/Constant3'

  real_T Gain_Gain_d;                  // Expression: -1
                                       //  Referenced by: '<S268>/Gain'

  real_T Gain1_Gain_j;                 // Expression: -1
                                       //  Referenced by: '<S268>/Gain1'

  real_T Gain2_Gain_k;                 // Expression: -1
                                       //  Referenced by: '<S268>/Gain2'

  real_T Gain1_Gain_h;                 // Expression: -1
                                       //  Referenced by: '<S266>/Gain1'

  real_T Constant10_Value[4];          // Expression: [0 0 0 1]
                                       //  Referenced by: '<S257>/Constant10'

  real_T Gain_Gain_ee;                 // Expression: 2
                                       //  Referenced by: '<S274>/Gain'

  real_T Constant1_Value_k;            // Expression: 1
                                       //  Referenced by: '<S274>/Constant1'

  real_T Gain1_Gain_dy;                // Expression: 2
                                       //  Referenced by: '<S274>/Gain1'

  real_T Constant3_Value_iy;           // Expression: 0
                                       //  Referenced by: '<S277>/Constant3'

  real_T Gain_Gain_k;                  // Expression: -1
                                       //  Referenced by: '<S277>/Gain'

  real_T Gain1_Gain_p;                 // Expression: -1
                                       //  Referenced by: '<S277>/Gain1'

  real_T Gain2_Gain_dz;                // Expression: -1
                                       //  Referenced by: '<S277>/Gain2'

  real_T Gain2_Gain_g;                 // Expression: 2
                                       //  Referenced by: '<S274>/Gain2'

  real_T Constant5_Value_a;            // Expression: 2*pi
                                       //  Referenced by: '<S255>/Constant5'

  real_T random_noise_Mean_k;          // Expression: 0
                                       //  Referenced by: '<S255>/random_noise'

  real_T random_noise_StdDev_c;        // Computed Parameter: random_noise_StdDev_c
                                       //  Referenced by: '<S255>/random_noise'

  real_T Constant4_Value;              // Expression: 25
                                       //  Referenced by: '<S255>/Constant4'

  real_T DiscreteTimeIntegrator2_gainv_k;// Computed Parameter: DiscreteTimeIntegrator2_gainv_k
                                         //  Referenced by: '<S255>/Discrete-Time Integrator2'

  real_T DiscreteTimeIntegrator1_gainv_l;// Computed Parameter: DiscreteTimeIntegrator1_gainv_l
                                         //  Referenced by: '<S255>/Discrete-Time Integrator1'

  real_T DiscreteTimeIntegrator1_IC_k; // Expression: 0
                                       //  Referenced by: '<S255>/Discrete-Time Integrator1'

  real_T DiscreteTransferFcn_InitialStat;// Expression: 0
                                         //  Referenced by: '<S256>/Discrete Transfer Fcn'

  real_T Delay1_InitialCondition_m;    // Expression: 0.0
                                       //  Referenced by: '<S256>/Delay1'

  real_T Delay1_InitialCondition_b;    // Expression: 0.0
                                       //  Referenced by: '<S253>/Delay1'

  real_T Constant3_Value_ao;           // Expression: 0
                                       //  Referenced by: '<S198>/Constant3'

  real_T Constant3_Value_c;            // Expression: 0
                                       //  Referenced by: '<S231>/Constant3'

  real_T AR_pulse_gen_Amp;             // Expression: 1
                                       //  Referenced by: '<S80>/AR_pulse_gen'

  real_T AR_pulse_gen_Duty;            // Expression: 1
                                       //  Referenced by: '<S80>/AR_pulse_gen'

  real_T AR_pulse_gen_PhaseDelay;      // Expression: 0
                                       //  Referenced by: '<S80>/AR_pulse_gen'

  real_T landmark_pulse_gen_Amp;       // Expression: 1
                                       //  Referenced by: '<S80>/landmark_pulse_gen'

  real_T landmark_pulse_gen_Duty;      // Expression: 1
                                       //  Referenced by: '<S80>/landmark_pulse_gen'

  real_T landmark_pulse_gen_PhaseDelay;// Expression: 0
                                       //  Referenced by: '<S80>/landmark_pulse_gen'

  real_T Constant1_Value_ot;           // Expression: 0
                                       //  Referenced by: '<S80>/Constant1'

  real_T handrail_pulse_gen_Amp;       // Expression: 1
                                       //  Referenced by: '<S80>/handrail_pulse_gen'

  real_T handrail_pulse_gen_Duty;      // Expression: 1
                                       //  Referenced by: '<S80>/handrail_pulse_gen'

  real_T handrail_pulse_gen_PhaseDelay;// Expression: 0
                                       //  Referenced by: '<S80>/handrail_pulse_gen'

  real_T RandomNumber_Mean;            // Expression: 0
                                       //  Referenced by: '<S6>/Random Number'

  real_T RandomNumber1_Mean;           // Expression: 0
                                       //  Referenced by: '<S6>/Random Number1'

  real_T Constant2_Value_ci[9];        // Expression: zeros(9,1)
                                       //  Referenced by: '<S92>/Constant2'

  real_T Constant2_Value_n1[9];        // Expression: zeros(9,1)
                                       //  Referenced by: '<S93>/Constant2'

  real_T Constant3_Value_eu;           // Expression: 0
                                       //  Referenced by: '<S95>/Constant3'

  real_T Constant1_Value_g;            // Expression: 1
                                       //  Referenced by: '<S92>/Constant1'

  real_T Constant3_Value_p;            // Expression: 0
                                       //  Referenced by: '<S103>/Constant3'

  real_T Constant2_Value_a[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S91>/Constant2'

  real_T Constant1_Value_m;            // Expression: 1
                                       //  Referenced by: '<S91>/Constant1'

  real_T Constant3_Value_az;           // Expression: 0
                                       //  Referenced by: '<S99>/Constant3'

  real_T pixel_noise_Mean;             // Expression: 0
                                       //  Referenced by: '<S85>/pixel_noise'

  real_T Constant2_Value_ab[9];        // Expression: zeros(9,1)
                                       //  Referenced by: '<S120>/Constant2'

  real_T Constant2_Value_nv[9];        // Expression: zeros(9,1)
                                       //  Referenced by: '<S121>/Constant2'

  real_T Constant3_Value_h;            // Expression: 0
                                       //  Referenced by: '<S123>/Constant3'

  real_T Constant1_Value_oc;           // Expression: 1
                                       //  Referenced by: '<S120>/Constant1'

  real_T Constant3_Value_f;            // Expression: 0
                                       //  Referenced by: '<S131>/Constant3'

  real_T Constant2_Value_o[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S119>/Constant2'

  real_T Constant1_Value_e;            // Expression: 1
                                       //  Referenced by: '<S119>/Constant1'

  real_T Constant3_Value_j0;           // Expression: 0
                                       //  Referenced by: '<S127>/Constant3'

  real_T pixel_noise_Mean_a;           // Expression: 0
                                       //  Referenced by: '<S113>/pixel_noise'

  real_T Constant2_Value_id[9];        // Expression: zeros(9,1)
                                       //  Referenced by: '<S144>/Constant2'

  real_T Constant2_Value_j[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S145>/Constant2'

  real_T Constant3_Value_hp;           // Expression: 0
                                       //  Referenced by: '<S147>/Constant3'

  real_T Constant1_Value_f;            // Expression: 1
                                       //  Referenced by: '<S144>/Constant1'

  real_T Constant3_Value_o;            // Expression: 0
                                       //  Referenced by: '<S155>/Constant3'

  real_T Constant2_Value_k[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S143>/Constant2'

  real_T Constant1_Value_l;            // Expression: 1
                                       //  Referenced by: '<S143>/Constant1'

  real_T Constant3_Value_p2;           // Expression: 0
                                       //  Referenced by: '<S151>/Constant3'

  real_T pixel_noise_Mean_n;           // Expression: 0
                                       //  Referenced by: '<S137>/pixel_noise'

  real_T Constant2_Value_kp[9];        // Expression: zeros(9,1)
                                       //  Referenced by: '<S168>/Constant2'

  real_T Constant2_Value_d4[9];        // Expression: zeros(9,1)
                                       //  Referenced by: '<S169>/Constant2'

  real_T Constant3_Value_g;            // Expression: 0
                                       //  Referenced by: '<S171>/Constant3'

  real_T Constant1_Value_h;            // Expression: 1
                                       //  Referenced by: '<S168>/Constant1'

  real_T Constant3_Value_c2;           // Expression: 0
                                       //  Referenced by: '<S179>/Constant3'

  real_T Constant2_Value_e[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S167>/Constant2'

  real_T Constant1_Value_es;           // Expression: 1
                                       //  Referenced by: '<S167>/Constant1'

  real_T Constant3_Value_kj;           // Expression: 0
                                       //  Referenced by: '<S175>/Constant3'

  real_T pixel_noise_Mean_d;           // Expression: 0
                                       //  Referenced by: '<S161>/pixel_noise'

  real_T random_noise1_Mean;           // Expression: 0
                                       //  Referenced by: '<S254>/random_noise1'

  real_T random_noise1_StdDev;         // Computed Parameter: random_noise1_StdDev
                                       //  Referenced by: '<S254>/random_noise1'

  real_T random_noise2_Mean;           // Expression: 0
                                       //  Referenced by: '<S254>/random_noise2'

  real_T random_noise2_StdDev;         // Computed Parameter: random_noise2_StdDev
                                       //  Referenced by: '<S254>/random_noise2'

  real_T random_noise1_Mean_d;         // Expression: 0
                                       //  Referenced by: '<S255>/random_noise1'

  real_T random_noise1_StdDev_l;       // Computed Parameter: random_noise1_StdDev_l
                                       //  Referenced by: '<S255>/random_noise1'

  real_T random_noise2_Mean_f;         // Expression: 0
                                       //  Referenced by: '<S255>/random_noise2'

  real_T random_noise2_StdDev_k;       // Computed Parameter: random_noise2_StdDev_k
                                       //  Referenced by: '<S255>/random_noise2'

  real32_T Constant_Value_c[16];       // Computed Parameter: Constant_Value_c
                                       //  Referenced by: '<S24>/Constant'

  real32_T Constant_Value_p[16];       // Computed Parameter: Constant_Value_p
                                       //  Referenced by: '<S26>/Constant'

  real32_T Constant3_Value_g2;         // Computed Parameter: Constant3_Value_g2
                                       //  Referenced by: '<S31>/Constant3'

  real32_T Gain_Gain_dy;               // Computed Parameter: Gain_Gain_dy
                                       //  Referenced by: '<S31>/Gain'

  real32_T Gain1_Gain_ow;              // Computed Parameter: Gain1_Gain_ow
                                       //  Referenced by: '<S31>/Gain1'

  real32_T Constant2_Value_l;          // Computed Parameter: Constant2_Value_l
                                       //  Referenced by: '<S31>/Constant2'

  real32_T Gain2_Gain_oe;              // Computed Parameter: Gain2_Gain_oe
                                       //  Referenced by: '<S31>/Gain2'

  real32_T Gain3_Gain;                 // Computed Parameter: Gain3_Gain
                                       //  Referenced by: '<S31>/Gain3'

  real32_T Gain4_Gain;                 // Computed Parameter: Gain4_Gain
                                       //  Referenced by: '<S31>/Gain4'

  real32_T Constant1_Value_p4;         // Computed Parameter: Constant1_Value_p4
                                       //  Referenced by: '<S31>/Constant1'

  real32_T Gain5_Gain;                 // Computed Parameter: Gain5_Gain
                                       //  Referenced by: '<S31>/Gain5'

  real32_T Constant_Value_d;           // Computed Parameter: Constant_Value_d
                                       //  Referenced by: '<S31>/Constant'

  real32_T Constant3_Value_pa;         // Computed Parameter: Constant3_Value_pa
                                       //  Referenced by: '<S30>/Constant3'

  real32_T Gain_Gain_g;                // Computed Parameter: Gain_Gain_g
                                       //  Referenced by: '<S30>/Gain'

  real32_T Gain1_Gain_c;               // Computed Parameter: Gain1_Gain_c
                                       //  Referenced by: '<S30>/Gain1'

  real32_T Constant2_Value_dl;         // Computed Parameter: Constant2_Value_dl
                                       //  Referenced by: '<S30>/Constant2'

  real32_T Gain2_Gain_l;               // Computed Parameter: Gain2_Gain_l
                                       //  Referenced by: '<S30>/Gain2'

  real32_T Gain3_Gain_i;               // Computed Parameter: Gain3_Gain_i
                                       //  Referenced by: '<S30>/Gain3'

  real32_T Gain4_Gain_f;               // Computed Parameter: Gain4_Gain_f
                                       //  Referenced by: '<S30>/Gain4'

  real32_T Constant1_Value_of;         // Computed Parameter: Constant1_Value_of
                                       //  Referenced by: '<S30>/Constant1'

  real32_T Gain5_Gain_l;               // Computed Parameter: Gain5_Gain_l
                                       //  Referenced by: '<S30>/Gain5'

  real32_T Constant_Value_f;           // Computed Parameter: Constant_Value_f
                                       //  Referenced by: '<S30>/Constant'

  real32_T Gain2_Gain_f;               // Computed Parameter: Gain2_Gain_f
                                       //  Referenced by: '<S6>/Gain2'

  real32_T Gain_Gain_go;               // Computed Parameter: Gain_Gain_go
                                       //  Referenced by: '<S6>/Gain'

  real32_T RateTransition5_X0;         // Computed Parameter: RateTransition5_X0
                                       //  Referenced by: '<S81>/Rate Transition5'

  real32_T RateTransition9_X0;         // Computed Parameter: RateTransition9_X0
                                       //  Referenced by: '<S81>/Rate Transition9'

  real32_T DiscreteTimeIntegrator1_gainv_i;// Computed Parameter: DiscreteTimeIntegrator1_gainv_i
                                           //  Referenced by: '<S7>/Discrete-Time Integrator1'

  real32_T DiscreteTimeIntegrator_gainval;// Computed Parameter: DiscreteTimeIntegrator_gainval
                                          //  Referenced by: '<S7>/Discrete-Time Integrator'

  real32_T DiscreteTimeIntegrator_gainva_p;// Computed Parameter: DiscreteTimeIntegrator_gainva_p
                                           //  Referenced by: '<S5>/Discrete-Time Integrator'

  real32_T Gain_Gain_m;                // Computed Parameter: Gain_Gain_m
                                       //  Referenced by: '<S62>/Gain'

  real32_T Gain1_Gain_oi;              // Computed Parameter: Gain1_Gain_oi
                                       //  Referenced by: '<S62>/Gain1'

  real32_T Gain_Gain_j1;               // Computed Parameter: Gain_Gain_j1
                                       //  Referenced by: '<S65>/Gain'

  real32_T Gain1_Gain_e;               // Computed Parameter: Gain1_Gain_e
                                       //  Referenced by: '<S65>/Gain1'

  real32_T Gain2_Gain_p;               // Computed Parameter: Gain2_Gain_p
                                       //  Referenced by: '<S65>/Gain2'

  real32_T Gain2_Gain_h;               // Computed Parameter: Gain2_Gain_h
                                       //  Referenced by: '<S62>/Gain2'

  real32_T DiscreteTimeIntegrator_gainva_j;// Computed Parameter: DiscreteTimeIntegrator_gainva_j
                                           //  Referenced by: '<S6>/Discrete-Time Integrator'

  real32_T DiscreteTimeIntegrator_IC;  // Computed Parameter: DiscreteTimeIntegrator_IC
                                       //  Referenced by: '<S6>/Discrete-Time Integrator'

  real32_T Constant7_Value[3];         // Expression: single([0 0 0])
                                       //  Referenced by: '<S6>/Constant7'

  real32_T Constant5_Value_av[3];      // Expression: single([0 0 0])
                                       //  Referenced by: '<S6>/Constant5'

  real32_T Constant10_Value_a;         // Expression: single(2)
                                       //  Referenced by: '<S7>/Constant10'

  real32_T Gain_Gain_b;                // Computed Parameter: Gain_Gain_b
                                       //  Referenced by: '<S57>/Gain'

  real32_T Gain1_Gain_m;               // Computed Parameter: Gain1_Gain_m
                                       //  Referenced by: '<S57>/Gain1'

  real32_T Gain_Gain_la;               // Computed Parameter: Gain_Gain_la
                                       //  Referenced by: '<S60>/Gain'

  real32_T Gain1_Gain_hb;              // Computed Parameter: Gain1_Gain_hb
                                       //  Referenced by: '<S60>/Gain1'

  real32_T Gain2_Gain_gu;              // Computed Parameter: Gain2_Gain_gu
                                       //  Referenced by: '<S60>/Gain2'

  real32_T Gain2_Gain_lk;              // Computed Parameter: Gain2_Gain_lk
                                       //  Referenced by: '<S57>/Gain2'

  real32_T Gain_Gain_h;                // Computed Parameter: Gain_Gain_h
                                       //  Referenced by: '<S52>/Gain'

  real32_T Gain1_Gain_al;              // Computed Parameter: Gain1_Gain_al
                                       //  Referenced by: '<S52>/Gain1'

  real32_T Gain_Gain_kk;               // Computed Parameter: Gain_Gain_kk
                                       //  Referenced by: '<S55>/Gain'

  real32_T Gain1_Gain_aj;              // Computed Parameter: Gain1_Gain_aj
                                       //  Referenced by: '<S55>/Gain1'

  real32_T Gain2_Gain_m;               // Computed Parameter: Gain2_Gain_m
                                       //  Referenced by: '<S55>/Gain2'

  real32_T Gain2_Gain_h4;              // Computed Parameter: Gain2_Gain_h4
                                       //  Referenced by: '<S52>/Gain2'

  real32_T Gain_Gain_c;                // Computed Parameter: Gain_Gain_c
                                       //  Referenced by: '<S35>/Gain'

  real32_T Gain1_Gain_de;              // Computed Parameter: Gain1_Gain_de
                                       //  Referenced by: '<S35>/Gain1'

  real32_T Gain_Gain_g5;               // Computed Parameter: Gain_Gain_g5
                                       //  Referenced by: '<S38>/Gain'

  real32_T Gain1_Gain_k;               // Computed Parameter: Gain1_Gain_k
                                       //  Referenced by: '<S38>/Gain1'

  real32_T Gain2_Gain_ku;              // Computed Parameter: Gain2_Gain_ku
                                       //  Referenced by: '<S38>/Gain2'

  real32_T Gain2_Gain_ax;              // Computed Parameter: Gain2_Gain_ax
                                       //  Referenced by: '<S35>/Gain2'

  real32_T DiscreteTimeIntegrator1_gainv_a;// Computed Parameter: DiscreteTimeIntegrator1_gainv_a
                                           //  Referenced by: '<S6>/Discrete-Time Integrator1'

  real32_T DiscreteTimeIntegrator1_IC_b;// Computed Parameter: DiscreteTimeIntegrator1_IC_b
                                        //  Referenced by: '<S6>/Discrete-Time Integrator1'

  real32_T Constant10_Value_g[3];      // Expression: single([0 0 0])
                                       //  Referenced by: '<S6>/Constant10'

  real32_T Gain_Gain_hs;               // Computed Parameter: Gain_Gain_hs
                                       //  Referenced by: '<S13>/Gain'

  real32_T Gain1_Gain_ep;              // Computed Parameter: Gain1_Gain_ep
                                       //  Referenced by: '<S13>/Gain1'

  real32_T Gain2_Gain_c;               // Computed Parameter: Gain2_Gain_c
                                       //  Referenced by: '<S13>/Gain2'

  real32_T Constant7_Value_j[3];       // Expression: single([0 0 0])
                                       //  Referenced by: '<S257>/Constant7'

  real32_T Constant_Value_g;           // Expression: single(25)
                                       //  Referenced by: '<S186>/Constant'

  real32_T Constant1_Value_fq;         // Expression: single(14)
                                       //  Referenced by: '<S4>/Constant1'

  real32_T DiscreteTimeIntegrator_gainv_j1;// Computed Parameter: DiscreteTimeIntegrator_gainv_j1
                                           //  Referenced by: '<S191>/Discrete-Time Integrator'

  real32_T Constant2_Value_cw;         // Expression: single(14)
                                       //  Referenced by: '<S4>/Constant2'

  real32_T DiscreteTimeIntegrator_gainva_o;// Computed Parameter: DiscreteTimeIntegrator_gainva_o
                                           //  Referenced by: '<S224>/Discrete-Time Integrator'

  real32_T Gain1_Gain_k3;              // Computed Parameter: Gain1_Gain_k3
                                       //  Referenced by: '<S191>/Gain1'

  real32_T Gain_Gain_i;                // Computed Parameter: Gain_Gain_i
                                       //  Referenced by: '<S198>/Gain'

  real32_T Gain1_Gain_me;              // Computed Parameter: Gain1_Gain_me
                                       //  Referenced by: '<S198>/Gain1'

  real32_T Gain2_Gain_b;               // Computed Parameter: Gain2_Gain_b
                                       //  Referenced by: '<S198>/Gain2'

  real32_T Gain1_Gain_hf;              // Computed Parameter: Gain1_Gain_hf
                                       //  Referenced by: '<S224>/Gain1'

  real32_T Gain_Gain_ib;               // Computed Parameter: Gain_Gain_ib
                                       //  Referenced by: '<S231>/Gain'

  real32_T Gain1_Gain_ak;              // Computed Parameter: Gain1_Gain_ak
                                       //  Referenced by: '<S231>/Gain1'

  real32_T Gain2_Gain_al;              // Computed Parameter: Gain2_Gain_al
                                       //  Referenced by: '<S231>/Gain2'

  real32_T RateTransition6_X0;         // Computed Parameter: RateTransition6_X0
                                       //  Referenced by: '<S83>/Rate Transition6'

  real32_T RateTransition8_X0;         // Computed Parameter: RateTransition8_X0
                                       //  Referenced by: '<S83>/Rate Transition8'

  real32_T RateTransition5_X0_c;       // Computed Parameter: RateTransition5_X0_c
                                       //  Referenced by: '<S82>/Rate Transition5'

  real32_T RateTransition6_X0_j;       // Computed Parameter: RateTransition6_X0_j
                                       //  Referenced by: '<S82>/Rate Transition6'

  real32_T cvs_handrail_local_pos_Value[3];// Expression: single([0 0 0])
                                           //  Referenced by: '<S69>/cvs_handrail_local_pos'

  real32_T cvs_handrail_local_quat_Value[4];// Expression: single([0 0 0 1])
                                            //  Referenced by: '<S69>/cvs_handrail_local_quat'

  real32_T RateTransition5_X0_o;       // Computed Parameter: RateTransition5_X0_o
                                       //  Referenced by: '<S79>/Rate Transition5'

  real32_T RateTransition6_X0_g;       // Computed Parameter: RateTransition6_X0_g
                                       //  Referenced by: '<S79>/Rate Transition6'

  real32_T Gain_Gain_o;                // Computed Parameter: Gain_Gain_o
                                       //  Referenced by: '<S95>/Gain'

  real32_T Gain1_Gain_mr;              // Computed Parameter: Gain1_Gain_mr
                                       //  Referenced by: '<S95>/Gain1'

  real32_T Gain2_Gain_og;              // Computed Parameter: Gain2_Gain_og
                                       //  Referenced by: '<S95>/Gain2'

  real32_T Gain1_Gain_cz;              // Computed Parameter: Gain1_Gain_cz
                                       //  Referenced by: '<S93>/Gain1'

  real32_T Constant7_Value_n[4];       // Expression: single([0 0 0 1])
                                       //  Referenced by: '<S88>/Constant7'

  real32_T Gain_Gain_oe;               // Computed Parameter: Gain_Gain_oe
                                       //  Referenced by: '<S92>/Gain'

  real32_T Gain1_Gain_mm;              // Computed Parameter: Gain1_Gain_mm
                                       //  Referenced by: '<S92>/Gain1'

  real32_T Gain_Gain_ee4;              // Computed Parameter: Gain_Gain_ee4
                                       //  Referenced by: '<S103>/Gain'

  real32_T Gain1_Gain_g;               // Computed Parameter: Gain1_Gain_g
                                       //  Referenced by: '<S103>/Gain1'

  real32_T Gain2_Gain_l0;              // Computed Parameter: Gain2_Gain_l0
                                       //  Referenced by: '<S103>/Gain2'

  real32_T Gain2_Gain_fv;              // Computed Parameter: Gain2_Gain_fv
                                       //  Referenced by: '<S92>/Gain2'

  real32_T Gain_Gain_a;                // Computed Parameter: Gain_Gain_a
                                       //  Referenced by: '<S91>/Gain'

  real32_T Gain1_Gain_al2;             // Computed Parameter: Gain1_Gain_al2
                                       //  Referenced by: '<S91>/Gain1'

  real32_T Gain_Gain_od;               // Computed Parameter: Gain_Gain_od
                                       //  Referenced by: '<S99>/Gain'

  real32_T Gain1_Gain_f;               // Computed Parameter: Gain1_Gain_f
                                       //  Referenced by: '<S99>/Gain1'

  real32_T Gain2_Gain_hy;              // Computed Parameter: Gain2_Gain_hy
                                       //  Referenced by: '<S99>/Gain2'

  real32_T Gain2_Gain_lw;              // Computed Parameter: Gain2_Gain_lw
                                       //  Referenced by: '<S91>/Gain2'

  real32_T Gain_Gain_op;               // Computed Parameter: Gain_Gain_op
                                       //  Referenced by: '<S123>/Gain'

  real32_T Gain1_Gain_da;              // Computed Parameter: Gain1_Gain_da
                                       //  Referenced by: '<S123>/Gain1'

  real32_T Gain2_Gain_fs;              // Computed Parameter: Gain2_Gain_fs
                                       //  Referenced by: '<S123>/Gain2'

  real32_T Gain1_Gain_jq;              // Computed Parameter: Gain1_Gain_jq
                                       //  Referenced by: '<S121>/Gain1'

  real32_T Constant7_Value_h[4];       // Expression: single([0 0 0 1])
                                       //  Referenced by: '<S116>/Constant7'

  real32_T Gain_Gain_pb;               // Computed Parameter: Gain_Gain_pb
                                       //  Referenced by: '<S120>/Gain'

  real32_T Gain1_Gain_i;               // Computed Parameter: Gain1_Gain_i
                                       //  Referenced by: '<S120>/Gain1'

  real32_T Gain_Gain_hw;               // Computed Parameter: Gain_Gain_hw
                                       //  Referenced by: '<S131>/Gain'

  real32_T Gain1_Gain_dx;              // Computed Parameter: Gain1_Gain_dx
                                       //  Referenced by: '<S131>/Gain1'

  real32_T Gain2_Gain_j;               // Computed Parameter: Gain2_Gain_j
                                       //  Referenced by: '<S131>/Gain2'

  real32_T Gain2_Gain_dy;              // Computed Parameter: Gain2_Gain_dy
                                       //  Referenced by: '<S120>/Gain2'

  real32_T Gain_Gain_cl;               // Computed Parameter: Gain_Gain_cl
                                       //  Referenced by: '<S119>/Gain'

  real32_T Gain1_Gain_ap;              // Computed Parameter: Gain1_Gain_ap
                                       //  Referenced by: '<S119>/Gain1'

  real32_T Gain_Gain_mu;               // Computed Parameter: Gain_Gain_mu
                                       //  Referenced by: '<S127>/Gain'

  real32_T Gain1_Gain_d3;              // Computed Parameter: Gain1_Gain_d3
                                       //  Referenced by: '<S127>/Gain1'

  real32_T Gain2_Gain_e;               // Computed Parameter: Gain2_Gain_e
                                       //  Referenced by: '<S127>/Gain2'

  real32_T Gain2_Gain_hf;              // Computed Parameter: Gain2_Gain_hf
                                       //  Referenced by: '<S119>/Gain2'

  real32_T Gain_Gain_kc;               // Computed Parameter: Gain_Gain_kc
                                       //  Referenced by: '<S147>/Gain'

  real32_T Gain1_Gain_pi;              // Computed Parameter: Gain1_Gain_pi
                                       //  Referenced by: '<S147>/Gain1'

  real32_T Gain2_Gain_pj;              // Computed Parameter: Gain2_Gain_pj
                                       //  Referenced by: '<S147>/Gain2'

  real32_T Gain1_Gain_hr;              // Computed Parameter: Gain1_Gain_hr
                                       //  Referenced by: '<S145>/Gain1'

  real32_T Constant7_Value_b[4];       // Expression: single([0 0 0 1])
                                       //  Referenced by: '<S140>/Constant7'

  real32_T Gain_Gain_cy;               // Computed Parameter: Gain_Gain_cy
                                       //  Referenced by: '<S144>/Gain'

  real32_T Gain1_Gain_l0;              // Computed Parameter: Gain1_Gain_l0
                                       //  Referenced by: '<S144>/Gain1'

  real32_T Gain_Gain_bv;               // Computed Parameter: Gain_Gain_bv
                                       //  Referenced by: '<S155>/Gain'

  real32_T Gain1_Gain_m5;              // Computed Parameter: Gain1_Gain_m5
                                       //  Referenced by: '<S155>/Gain1'

  real32_T Gain2_Gain_l1;              // Computed Parameter: Gain2_Gain_l1
                                       //  Referenced by: '<S155>/Gain2'

  real32_T Gain2_Gain_hs;              // Computed Parameter: Gain2_Gain_hs
                                       //  Referenced by: '<S144>/Gain2'

  real32_T Gain_Gain_bvs;              // Computed Parameter: Gain_Gain_bvs
                                       //  Referenced by: '<S143>/Gain'

  real32_T Gain1_Gain_hd;              // Computed Parameter: Gain1_Gain_hd
                                       //  Referenced by: '<S143>/Gain1'

  real32_T Gain_Gain_jw;               // Computed Parameter: Gain_Gain_jw
                                       //  Referenced by: '<S151>/Gain'

  real32_T Gain1_Gain_om;              // Computed Parameter: Gain1_Gain_om
                                       //  Referenced by: '<S151>/Gain1'

  real32_T Gain2_Gain_pf;              // Computed Parameter: Gain2_Gain_pf
                                       //  Referenced by: '<S151>/Gain2'

  real32_T Gain2_Gain_n;               // Computed Parameter: Gain2_Gain_n
                                       //  Referenced by: '<S143>/Gain2'

  real32_T Gain_Gain_mw;               // Computed Parameter: Gain_Gain_mw
                                       //  Referenced by: '<S171>/Gain'

  real32_T Gain1_Gain_b;               // Computed Parameter: Gain1_Gain_b
                                       //  Referenced by: '<S171>/Gain1'

  real32_T Gain2_Gain_ch;              // Computed Parameter: Gain2_Gain_ch
                                       //  Referenced by: '<S171>/Gain2'

  real32_T Gain1_Gain_ll;              // Computed Parameter: Gain1_Gain_ll
                                       //  Referenced by: '<S169>/Gain1'

  real32_T Constant7_Value_m[4];       // Expression: single([0 0 0 1])
                                       //  Referenced by: '<S164>/Constant7'

  real32_T Gain_Gain_ci;               // Computed Parameter: Gain_Gain_ci
                                       //  Referenced by: '<S168>/Gain'

  real32_T Gain1_Gain_fm;              // Computed Parameter: Gain1_Gain_fm
                                       //  Referenced by: '<S168>/Gain1'

  real32_T Gain_Gain_d5;               // Computed Parameter: Gain_Gain_d5
                                       //  Referenced by: '<S179>/Gain'

  real32_T Gain1_Gain_hfz;             // Computed Parameter: Gain1_Gain_hfz
                                       //  Referenced by: '<S179>/Gain1'

  real32_T Gain2_Gain_ee;              // Computed Parameter: Gain2_Gain_ee
                                       //  Referenced by: '<S179>/Gain2'

  real32_T Gain2_Gain_he;              // Computed Parameter: Gain2_Gain_he
                                       //  Referenced by: '<S168>/Gain2'

  real32_T Gain_Gain_fp;               // Computed Parameter: Gain_Gain_fp
                                       //  Referenced by: '<S167>/Gain'

  real32_T Gain1_Gain_in;              // Computed Parameter: Gain1_Gain_in
                                       //  Referenced by: '<S167>/Gain1'

  real32_T Gain_Gain_lw;               // Computed Parameter: Gain_Gain_lw
                                       //  Referenced by: '<S175>/Gain'

  real32_T Gain1_Gain_jqn;             // Computed Parameter: Gain1_Gain_jqn
                                       //  Referenced by: '<S175>/Gain1'

  real32_T Gain2_Gain_jj;              // Computed Parameter: Gain2_Gain_jj
                                       //  Referenced by: '<S175>/Gain2'

  real32_T Gain2_Gain_dk;              // Computed Parameter: Gain2_Gain_dk
                                       //  Referenced by: '<S167>/Gain2'

  real32_T Switch_Threshold;           // Computed Parameter: Switch_Threshold
                                       //  Referenced by: '<S191>/Switch'

  real32_T Switch_Threshold_o;         // Computed Parameter: Switch_Threshold_o
                                       //  Referenced by: '<S224>/Switch'

  uint32_T RateTransition3_X0;         // Computed Parameter: RateTransition3_X0
                                       //  Referenced by: '<S81>/Rate Transition3'

  uint32_T RateTransition1_X0;         // Computed Parameter: RateTransition1_X0
                                       //  Referenced by: '<S81>/Rate Transition1'

  uint32_T Constant2_Value_p1[2];      // Expression: uint32([2^31 2^31])
                                       //  Referenced by: '<S74>/Constant2'

  uint32_T Constant2_Value_jp;         // Expression: uint32(1)
                                       //  Referenced by: '<S8>/Constant2'

  uint32_T Constant1_Value_hc;         // Expression: uint32(1E9)
                                       //  Referenced by: '<S8>/Constant1'

  uint32_T Delay_DelayLength;          // Computed Parameter: Delay_DelayLength
                                       //  Referenced by: '<S11>/Delay'

  uint32_T Delay1_DelayLength;         // Computed Parameter: Delay1_DelayLength
                                       //  Referenced by: '<S1>/Delay1'

  uint32_T RateTransition3_X0_o;       // Computed Parameter: RateTransition3_X0_o
                                       //  Referenced by: '<S83>/Rate Transition3'

  uint32_T RateTransition1_X0_e;       // Computed Parameter: RateTransition1_X0_e
                                       //  Referenced by: '<S83>/Rate Transition1'

  uint32_T RateTransition3_X0_m;       // Computed Parameter: RateTransition3_X0_m
                                       //  Referenced by: '<S82>/Rate Transition3'

  uint32_T RateTransition1_X0_eo;      // Computed Parameter: RateTransition1_X0_eo
                                       //  Referenced by: '<S82>/Rate Transition1'

  uint32_T RateTransition3_X0_c;       // Computed Parameter: RateTransition3_X0_c
                                       //  Referenced by: '<S79>/Rate Transition3'

  uint32_T RateTransition1_X0_a;       // Computed Parameter: RateTransition1_X0_a
                                       //  Referenced by: '<S79>/Rate Transition1'

  uint32_T Constant_Value_n;           // Computed Parameter: Constant_Value_n
                                       //  Referenced by: '<S75>/Constant'

  uint32_T Constant_Value_j;           // Computed Parameter: Constant_Value_j
                                       //  Referenced by: '<S76>/Constant'

  uint32_T Constant_Value_m;           // Computed Parameter: Constant_Value_m
                                       //  Referenced by: '<S77>/Constant'

  uint32_T Constant_Value_nc;          // Computed Parameter: Constant_Value_nc
                                       //  Referenced by: '<S78>/Constant'

  uint32_T Constant_Value_e;           // Computed Parameter: Constant_Value_e
                                       //  Referenced by: '<S87>/Constant'

  uint32_T Constant_Value_c0;          // Computed Parameter: Constant_Value_c0
                                       //  Referenced by: '<S115>/Constant'

  uint32_T Constant_Value_eq;          // Computed Parameter: Constant_Value_eq
                                       //  Referenced by: '<S139>/Constant'

  uint32_T Constant_Value_mi;          // Computed Parameter: Constant_Value_mi
                                       //  Referenced by: '<S163>/Constant'

  uint8_T RateTransition7_X0;          // Computed Parameter: RateTransition7_X0
                                       //  Referenced by: '<S81>/Rate Transition7'

  uint8_T UnitDelay_InitialCondition;  // Computed Parameter: UnitDelay_InitialCondition
                                       //  Referenced by: '<S70>/Unit Delay'

  uint8_T Bias_Bias;                   // Computed Parameter: Bias_Bias
                                       //  Referenced by: '<S70>/Bias'

  uint8_T UnitDelay_InitialCondition_c;// Computed Parameter: UnitDelay_InitialCondition_c
                                       //  Referenced by: '<S71>/Unit Delay'

  uint8_T UnitDelay_InitialCondition_n;// Computed Parameter: UnitDelay_InitialCondition_n
                                       //  Referenced by: '<S73>/Unit Delay'

  uint8_T UnitDelay_InitialCondition_l;// Computed Parameter: UnitDelay_InitialCondition_l
                                       //  Referenced by: '<S72>/Unit Delay'

  uint8_T Switch_Threshold_p;          // Computed Parameter: Switch_Threshold_p
                                       //  Referenced by: '<S257>/Switch'

  uint8_T Delay2_DelayLength;          // Computed Parameter: Delay2_DelayLength
                                       //  Referenced by: '<S256>/Delay2'

  uint8_T Delay2_DelayLength_h;        // Computed Parameter: Delay2_DelayLength_h
                                       //  Referenced by: '<S253>/Delay2'

  uint8_T Delay1_DelayLength_b;        // Computed Parameter: Delay1_DelayLength_b
                                       //  Referenced by: '<S256>/Delay1'

  uint8_T Delay1_DelayLength_n;        // Computed Parameter: Delay1_DelayLength_n
                                       //  Referenced by: '<S253>/Delay1'

  uint8_T Constant1_Value_ol;          // Expression: uint8(1)
                                       //  Referenced by: '<S186>/Constant1'

  uint8_T Constant2_Value_cc;          // Expression: uint8(0)
                                       //  Referenced by: '<S186>/Constant2'

  uint8_T RateTransition10_X0;         // Computed Parameter: RateTransition10_X0
                                       //  Referenced by: '<S83>/Rate Transition10'

  uint8_T RateTransition7_X0_m;        // Computed Parameter: RateTransition7_X0_m
                                       //  Referenced by: '<S83>/Rate Transition7'

  uint8_T RateTransition7_X0_k;        // Computed Parameter: RateTransition7_X0_k
                                       //  Referenced by: '<S82>/Rate Transition7'

  uint8_T cvs_3d_knowledge_flag_Value; // Expression: uint8(0)
                                       //  Referenced by: '<S69>/cvs_3d_knowledge_flag'

  uint8_T cvs_handrail_update_global_pose;// Expression: uint8(0)
                                          //  Referenced by: '<S69>/cvs_handrail_update_global_pose_flag'

  uint8_T RateTransition7_X0_n;        // Computed Parameter: RateTransition7_X0_n
                                       //  Referenced by: '<S79>/Rate Transition7'

  uint8_T Saturation_UpperSat;         // Expression: mlp_num_commands
                                       //  Referenced by: '<S70>/Saturation'

  uint8_T Saturation_LowerSat;         // Computed Parameter: Saturation_LowerSat
                                       //  Referenced by: '<S70>/Saturation'

  uint8_T Saturation_UpperSat_b;       // Expression: mlp_num_mode_cmds
                                       //  Referenced by: '<S71>/Saturation'

  uint8_T Saturation_LowerSat_e;       // Computed Parameter: Saturation_LowerSat_e
                                       //  Referenced by: '<S71>/Saturation'

  uint8_T Saturation_UpperSat_d;       // Expression: mlp_num_mode_cmds
                                       //  Referenced by: '<S72>/Saturation'

  uint8_T Saturation_LowerSat_j;       // Computed Parameter: Saturation_LowerSat_j
                                       //  Referenced by: '<S72>/Saturation'

  uint8_T Saturation_UpperSat_f;       // Expression: mlp_num_mode_cmds
                                       //  Referenced by: '<S73>/Saturation'

  uint8_T Saturation_LowerSat_k;       // Computed Parameter: Saturation_LowerSat_k
                                       //  Referenced by: '<S73>/Saturation'

  uint8_T UnitDelay_InitialCondition_d;// Computed Parameter: UnitDelay_InitialCondition_d
                                       //  Referenced by: '<S84>/Unit Delay'

  uint8_T Saturation_UpperSat_i;       // Computed Parameter: Saturation_UpperSat_i
                                       //  Referenced by: '<S84>/Saturation'

  uint8_T Saturation_LowerSat_b;       // Computed Parameter: Saturation_LowerSat_b
                                       //  Referenced by: '<S84>/Saturation'

  uint8_T UnitDelay_InitialCondition_m;// Computed Parameter: UnitDelay_InitialCondition_m
                                       //  Referenced by: '<S112>/Unit Delay'

  uint8_T Saturation_UpperSat_n;       // Computed Parameter: Saturation_UpperSat_n
                                       //  Referenced by: '<S112>/Saturation'

  uint8_T Saturation_LowerSat_f;       // Computed Parameter: Saturation_LowerSat_f
                                       //  Referenced by: '<S112>/Saturation'

  uint8_T UnitDelay_InitialCondition_g;// Computed Parameter: UnitDelay_InitialCondition_g
                                       //  Referenced by: '<S136>/Unit Delay'

  uint8_T Saturation_UpperSat_k;       // Computed Parameter: Saturation_UpperSat_k
                                       //  Referenced by: '<S136>/Saturation'

  uint8_T Saturation_LowerSat_a;       // Computed Parameter: Saturation_LowerSat_a
                                       //  Referenced by: '<S136>/Saturation'

  uint8_T UnitDelay_InitialCondition_a;// Computed Parameter: UnitDelay_InitialCondition_a
                                       //  Referenced by: '<S160>/Unit Delay'

  uint8_T Saturation_UpperSat_p;       // Computed Parameter: Saturation_UpperSat_p
                                       //  Referenced by: '<S160>/Saturation'

  uint8_T Saturation_LowerSat_h;       // Computed Parameter: Saturation_LowerSat_h
                                       //  Referenced by: '<S160>/Saturation'

  boolean_T UnitDelay_InitialCondition_o;// Computed Parameter: UnitDelay_InitialCondition_o
                                         //  Referenced by: '<S188>/Unit Delay'

  boolean_T UnitDelay_InitialCondition_gb;// Computed Parameter: UnitDelay_InitialCondition_gb
                                          //  Referenced by: '<S221>/Unit Delay'

  P_servo_model_sim_model_lib0_T servo_model_f;// '<S185>/servo_model'
  P_speed_controller_sim_model__T speed_controller_c;// '<S223>/speed_controller' 
  P_latch_nozzle_thrust_matrici_T latch_nozzle_thrust_matricies_p;// '<S221>/latch_nozzle_thrust_matricies' 
  P_blower_aerodynamics_sim_m_m_T blower_aerodynamics_j;// '<S185>/blower_aerodynamics' 
  P_servo_model_sim_model_lib0_T servo_model;// '<S184>/servo_model'
  P_speed_controller_sim_model__T speed_controller;// '<S190>/speed_controller'
  P_latch_nozzle_thrust_matrici_T latch_nozzle_thrust_matricies;// '<S188>/latch_nozzle_thrust_matricies' 
  P_blower_aerodynamics_sim_mod_T blower_aerodynamics;// '<S184>/blower_aerodynamics' 
  P_pinhole_projection_model_si_T pinhole_projection_model_o;// '<S161>/pinhole_projection_model' 
  P_pinhole_projection_model_si_T pinhole_projection_model_id;// '<S137>/pinhole_projection_model' 
  P_pinhole_projection_model_si_T pinhole_projection_model_i;// '<S113>/pinhole_projection_model' 
  P_pinhole_projection_model_si_T pinhole_projection_model;// '<S85>/pinhole_projection_model' 
};

// Real-time Model Data Structure
struct tag_RTM_sim_model_lib0_T {
  const char_T * volatile errorStatus;
  B_sim_model_lib0_T *blockIO;
  P_sim_model_lib0_T *defaultParam;
  boolean_T paramIsMalloced;
  DW_sim_model_lib0_T *dwork;

  //
  //  Timing:
  //  The following substructure contains information regarding
  //  the timing information for the model.

  struct {
    struct {
      uint8_T TID0_1;
      uint8_T TID0_2;
      uint8_T TID0_3;
      uint8_T TID0_4;
    } RateInteraction;
  } Timing;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// External data declarations for dependent source files
extern const act_msg sim_model_lib0_rtZact_msg;// act_msg ground
extern const cmc_msg sim_model_lib0_rtZcmc_msg;// cmc_msg ground
extern const env_msg sim_model_lib0_rtZenv_msg;// env_msg ground
extern const ex_time_msg sim_model_lib0_rtZex_time_msg;// ex_time_msg ground
extern const cvs_landmark_msg sim_model_lib0_rtZcvs_landmark_msg;// cvs_landmark_msg ground 
extern const cvs_optical_flow_msg sim_model_lib0_rtZcvs_optical_flow_msg;// cvs_optical_flow_msg ground 
extern const cvs_handrail_msg sim_model_lib0_rtZcvs_handrail_msg;// cvs_handrail_msg ground 
extern const cvs_registration_pulse sim_model_lib0_rtZcvs_registration_pulse;// cvs_registration_pulse ground 
extern const bpm_msg sim_model_lib0_rtZbpm_msg;// bpm_msg ground
extern const imu_msg sim_model_lib0_rtZimu_msg;// imu_msg ground

#ifdef __cplusplus

extern "C" {

#endif

  extern const char *RT_MEMORY_ALLOCATION_ERROR;

#ifdef __cplusplus

}
#endif

extern P_sim_model_lib0_T sim_model_lib0_P;// parameters

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern RT_MODEL_sim_model_lib0_T *sim_model_lib0(act_msg
    *sim_model_lib0_U_act_msg_l, cmc_msg *sim_model_lib0_U_cmc_msg_in,
    cvs_optical_flow_msg *sim_model_lib0_Y_cvs_optical_flow_msg_n,
    cvs_handrail_msg *sim_model_lib0_Y_cvs_handrail_msg_h, cmc_msg
    *sim_model_lib0_Y_cmc_msg_c, imu_msg *sim_model_lib0_Y_imu_msg_o, env_msg
    *sim_model_lib0_Y_env_msg_i, bpm_msg *sim_model_lib0_Y_bpm_msg_h,
    cvs_registration_pulse *sim_model_lib0_Y_cvs_registration_pulse_d,
    cvs_landmark_msg *sim_model_lib0_Y_cvs_landmark_msg_n, cvs_landmark_msg
    *sim_model_lib0_Y_cvs_ar_tag_msg, ex_time_msg
    *sim_model_lib0_Y_ex_time_msg_m);
  extern void sim_model_lib0_initialize(RT_MODEL_sim_model_lib0_T *const
    sim_model_lib0_M, act_msg *sim_model_lib0_U_act_msg_l, cmc_msg
    *sim_model_lib0_U_cmc_msg_in, cvs_optical_flow_msg
    *sim_model_lib0_Y_cvs_optical_flow_msg_n, cvs_handrail_msg
    *sim_model_lib0_Y_cvs_handrail_msg_h, cmc_msg *sim_model_lib0_Y_cmc_msg_c,
    imu_msg *sim_model_lib0_Y_imu_msg_o, env_msg *sim_model_lib0_Y_env_msg_i,
    bpm_msg *sim_model_lib0_Y_bpm_msg_h, cvs_registration_pulse
    *sim_model_lib0_Y_cvs_registration_pulse_d, cvs_landmark_msg
    *sim_model_lib0_Y_cvs_landmark_msg_n, cvs_landmark_msg
    *sim_model_lib0_Y_cvs_ar_tag_msg, ex_time_msg
    *sim_model_lib0_Y_ex_time_msg_m);
  extern void sim_model_lib0_step0(RT_MODEL_sim_model_lib0_T *const
    sim_model_lib0_M, act_msg *sim_model_lib0_U_act_msg_l, cmc_msg
    *sim_model_lib0_U_cmc_msg_in, cvs_optical_flow_msg
    *sim_model_lib0_Y_cvs_optical_flow_msg_n, cvs_handrail_msg
    *sim_model_lib0_Y_cvs_handrail_msg_h, cmc_msg *sim_model_lib0_Y_cmc_msg_c,
    imu_msg *sim_model_lib0_Y_imu_msg_o, env_msg *sim_model_lib0_Y_env_msg_i,
    bpm_msg *sim_model_lib0_Y_bpm_msg_h, cvs_registration_pulse
    *sim_model_lib0_Y_cvs_registration_pulse_d, cvs_landmark_msg
    *sim_model_lib0_Y_cvs_landmark_msg_n, cvs_landmark_msg
    *sim_model_lib0_Y_cvs_ar_tag_msg, ex_time_msg
    *sim_model_lib0_Y_ex_time_msg_m);
  extern void sim_model_lib0_step1(RT_MODEL_sim_model_lib0_T *const
    sim_model_lib0_M);
  extern void sim_model_lib0_step2(RT_MODEL_sim_model_lib0_T *const
    sim_model_lib0_M);
  extern void sim_model_lib0_step3(RT_MODEL_sim_model_lib0_T *const
    sim_model_lib0_M);
  extern void sim_model_lib0_step4(RT_MODEL_sim_model_lib0_T *const
    sim_model_lib0_M);
  extern void sim_model_lib0_terminate(RT_MODEL_sim_model_lib0_T
    * sim_model_lib0_M);

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
//  hilite_system('astrobee/sim_model_lib')    - opens subsystem astrobee/sim_model_lib
//  hilite_system('astrobee/sim_model_lib/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'astrobee'
//  '<S1>'   : 'astrobee/sim_model_lib'
//  '<S2>'   : 'astrobee/sim_model_lib/env_environment_model'
//  '<S3>'   : 'astrobee/sim_model_lib/mlp_mid_level_processor_model'
//  '<S4>'   : 'astrobee/sim_model_lib/veh_vehicle_model'
//  '<S5>'   : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics'
//  '<S6>'   : 'astrobee/sim_model_lib/env_environment_model/disturbance_model'
//  '<S7>'   : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics'
//  '<S8>'   : 'astrobee/sim_model_lib/env_environment_model/tsm_time_simulation_module'
//  '<S9>'   : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/cross_product1'
//  '<S10>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quat_normalize_and_enforce_positive_scalar'
//  '<S11>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quaternion_propogator'
//  '<S12>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/rotate_vec_A2B'
//  '<S13>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/cross_product1/skew_symetric_matrix_operator'
//  '<S14>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/cross_product1/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S15>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quat_normalize_and_enforce_positive_scalar/No-op'
//  '<S16>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quat_normalize_and_enforce_positive_scalar/Normalize'
//  '<S17>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quat_normalize_and_enforce_positive_scalar/vector_normalize'
//  '<S18>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quat_normalize_and_enforce_positive_scalar/Normalize/Data Type Conversion Inherited'
//  '<S19>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quat_normalize_and_enforce_positive_scalar/vector_normalize/No-op'
//  '<S20>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quat_normalize_and_enforce_positive_scalar/vector_normalize/Normalize'
//  '<S21>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quat_normalize_and_enforce_positive_scalar/vector_normalize/vector_magnitude'
//  '<S22>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quaternion_propogator/Compare To Constant'
//  '<S23>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quaternion_propogator/Compare To Constant1'
//  '<S24>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quaternion_propogator/Stopped'
//  '<S25>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quaternion_propogator/normal_speed'
//  '<S26>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quaternion_propogator/slow_speed_approx'
//  '<S27>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quaternion_propogator/vector_magnitude'
//  '<S28>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quaternion_propogator/vector_normalize'
//  '<S29>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quaternion_propogator/normal_speed/Data Type Conversion Inherited'
//  '<S30>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quaternion_propogator/normal_speed/create_omega_matrix'
//  '<S31>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quaternion_propogator/slow_speed_approx/create_omega_matrix'
//  '<S32>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quaternion_propogator/vector_normalize/No-op'
//  '<S33>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quaternion_propogator/vector_normalize/Normalize'
//  '<S34>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/quaternion_propogator/vector_normalize/vector_magnitude'
//  '<S35>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/rotate_vec_A2B/quaternion_to_DCM'
//  '<S36>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/rotate_vec_A2B/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S37>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/rotate_vec_A2B/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S38>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/rotate_vec_A2B/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S39>'  : 'astrobee/sim_model_lib/env_environment_model/attitude_dynamics/rotate_vec_A2B/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S40>'  : 'astrobee/sim_model_lib/env_environment_model/disturbance_model/saturate_vector_mag'
//  '<S41>'  : 'astrobee/sim_model_lib/env_environment_model/disturbance_model/saturate_vector_mag/vector_magnitude1'
//  '<S42>'  : 'astrobee/sim_model_lib/env_environment_model/disturbance_model/saturate_vector_mag/vector_normalize'
//  '<S43>'  : 'astrobee/sim_model_lib/env_environment_model/disturbance_model/saturate_vector_mag/vector_normalize/No-op'
//  '<S44>'  : 'astrobee/sim_model_lib/env_environment_model/disturbance_model/saturate_vector_mag/vector_normalize/Normalize'
//  '<S45>'  : 'astrobee/sim_model_lib/env_environment_model/disturbance_model/saturate_vector_mag/vector_normalize/vector_magnitude'
//  '<S46>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/Cross Product'
//  '<S47>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/Cross Product2'
//  '<S48>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/Cross Product3'
//  '<S49>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_A2B'
//  '<S50>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_A2B1'
//  '<S51>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_B2A'
//  '<S52>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_A2B/quaternion_to_DCM'
//  '<S53>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_A2B/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S54>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_A2B/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S55>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_A2B/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S56>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_A2B/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S57>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_A2B1/quaternion_to_DCM'
//  '<S58>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_A2B1/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S59>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_A2B1/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S60>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_A2B1/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S61>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_A2B1/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S62>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_B2A/quaternion_to_DCM'
//  '<S63>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_B2A/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S64>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_B2A/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S65>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_B2A/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S66>'  : 'astrobee/sim_model_lib/env_environment_model/translational_dynamics/rotate_vec_B2A/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S67>'  : 'astrobee/sim_model_lib/env_environment_model/tsm_time_simulation_module/Compare To Constant'
//  '<S68>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cmc_command_and_mode_control'
//  '<S69>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system'
//  '<S70>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cmc_command_and_mode_control/determine_cmd_idx'
//  '<S71>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cmc_command_and_mode_control/determine_cmd_idx1'
//  '<S72>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cmc_command_and_mode_control/determine_cmd_idx2'
//  '<S73>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cmc_command_and_mode_control/determine_cmd_idx3'
//  '<S74>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cmc_command_and_mode_control/select_cmd'
//  '<S75>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cmc_command_and_mode_control/determine_cmd_idx/Compare To Zero3'
//  '<S76>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cmc_command_and_mode_control/determine_cmd_idx1/Compare To Zero3'
//  '<S77>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cmc_command_and_mode_control/determine_cmd_idx2/Compare To Zero3'
//  '<S78>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cmc_command_and_mode_control/determine_cmd_idx3/Compare To Zero3'
//  '<S79>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing'
//  '<S80>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/generate_reg_pulse'
//  '<S81>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing'
//  '<S82>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing'
//  '<S83>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing'
//  '<S84>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/apply_valid_flag_mask'
//  '<S85>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model'
//  '<S86>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/generate_output'
//  '<S87>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/apply_valid_flag_mask/Compare To Zero3'
//  '<S88>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/convert_points_to_cam_frame'
//  '<S89>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/pinhole_projection_model'
//  '<S90>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication'
//  '<S91>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM'
//  '<S92>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1'
//  '<S93>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication/Quaternion Xi'
//  '<S94>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication/Quaternion Xi/Data Type Conversion Inherited'
//  '<S95>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator'
//  '<S96>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S97>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S98>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S99>'  : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S100>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S101>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1/Data Type Conversion Inherited'
//  '<S102>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1/Data Type Conversion Inherited1'
//  '<S103>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1/skew_symetric_matrix_operator'
//  '<S104>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S105>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/pinhole_projection_model/determine_points_in_FOV'
//  '<S106>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/pinhole_projection_model/pinhole_camera_projection'
//  '<S107>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/ARtag_image_processing/camera_model/pinhole_projection_model/determine_points_in_FOV/Compare To Zero'
//  '<S108>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/generate_reg_pulse/Compare To Constant'
//  '<S109>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/generate_reg_pulse/Compare To Constant1'
//  '<S110>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/generate_reg_pulse/Detect Change'
//  '<S111>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/generate_reg_pulse/Detect Change1'
//  '<S112>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/apply_valid_flag_mask'
//  '<S113>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model'
//  '<S114>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/generate_output'
//  '<S115>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/apply_valid_flag_mask/Compare To Zero3'
//  '<S116>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/convert_points_to_cam_frame'
//  '<S117>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/pinhole_projection_model'
//  '<S118>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication'
//  '<S119>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM'
//  '<S120>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1'
//  '<S121>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication/Quaternion Xi'
//  '<S122>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication/Quaternion Xi/Data Type Conversion Inherited'
//  '<S123>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator'
//  '<S124>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S125>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S126>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S127>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S128>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S129>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1/Data Type Conversion Inherited'
//  '<S130>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1/Data Type Conversion Inherited1'
//  '<S131>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1/skew_symetric_matrix_operator'
//  '<S132>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S133>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/pinhole_projection_model/determine_points_in_FOV'
//  '<S134>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/pinhole_projection_model/pinhole_camera_projection'
//  '<S135>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/handrail_image_processing/camera_model/pinhole_projection_model/determine_points_in_FOV/Compare To Zero'
//  '<S136>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/apply_valid_flag_mask'
//  '<S137>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model'
//  '<S138>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/generate_output'
//  '<S139>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/apply_valid_flag_mask/Compare To Zero3'
//  '<S140>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/convert_points_to_cam_frame'
//  '<S141>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/pinhole_projection_model'
//  '<S142>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication'
//  '<S143>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM'
//  '<S144>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1'
//  '<S145>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication/Quaternion Xi'
//  '<S146>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication/Quaternion Xi/Data Type Conversion Inherited'
//  '<S147>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator'
//  '<S148>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S149>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S150>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S151>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S152>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S153>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1/Data Type Conversion Inherited'
//  '<S154>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1/Data Type Conversion Inherited1'
//  '<S155>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1/skew_symetric_matrix_operator'
//  '<S156>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S157>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/pinhole_projection_model/determine_points_in_FOV'
//  '<S158>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/pinhole_projection_model/pinhole_camera_projection'
//  '<S159>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/landmark_image_processing/camera_model/pinhole_projection_model/determine_points_in_FOV/Compare To Zero'
//  '<S160>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/apply_valid_flag_mask'
//  '<S161>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model'
//  '<S162>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/generate_output'
//  '<S163>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/apply_valid_flag_mask/Compare To Zero3'
//  '<S164>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/convert_points_to_cam_frame'
//  '<S165>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/pinhole_projection_model'
//  '<S166>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication'
//  '<S167>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM'
//  '<S168>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1'
//  '<S169>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication/Quaternion Xi'
//  '<S170>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication/Quaternion Xi/Data Type Conversion Inherited'
//  '<S171>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator'
//  '<S172>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/convert_points_to_cam_frame/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S173>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S174>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S175>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S176>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S177>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1/Data Type Conversion Inherited'
//  '<S178>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1/Data Type Conversion Inherited1'
//  '<S179>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1/skew_symetric_matrix_operator'
//  '<S180>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/convert_points_to_cam_frame/quaternion_to_DCM1/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S181>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/pinhole_projection_model/determine_points_in_FOV'
//  '<S182>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/pinhole_projection_model/pinhole_camera_projection'
//  '<S183>' : 'astrobee/sim_model_lib/mlp_mid_level_processor_model/cvs_computer_vision_system/optical_flow_image_processing/camera_model/pinhole_projection_model/determine_points_in_FOV/Compare To Zero'
//  '<S184>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module'
//  '<S185>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module'
//  '<S186>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model'
//  '<S187>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_aerodynamics'
//  '<S188>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics'
//  '<S189>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/calc_nozzle_area'
//  '<S190>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/impeller_model'
//  '<S191>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/motor_rotational_dynamics'
//  '<S192>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/servo_model'
//  '<S193>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/speed_sensor'
//  '<S194>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/Detect Change'
//  '<S195>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/cross_product'
//  '<S196>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies'
//  '<S197>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/vector_normalize'
//  '<S198>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/cross_product/skew_symetric_matrix_operator'
//  '<S199>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/cross_product/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S200>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem'
//  '<S201>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/MATLAB Function'
//  '<S202>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B'
//  '<S203>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B/quaternion_to_DCM'
//  '<S204>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S205>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S206>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S207>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S208>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/vector_normalize/No-op'
//  '<S209>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/vector_normalize/Normalize'
//  '<S210>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/vector_normalize/vector_magnitude'
//  '<S211>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/impeller_model/dc_motor_model'
//  '<S212>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/impeller_model/speed_controller'
//  '<S213>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/impeller_model/speed_controller/Discrete PID Controller'
//  '<S214>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/impeller_model/speed_controller/Saturation Dynamic'
//  '<S215>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/impeller_model/speed_controller/Discrete PID Controller/Clamping circuit'
//  '<S216>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/servo_model/dc_motor_dynamics'
//  '<S217>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/servo_model/servo_controller'
//  '<S218>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/servo_model/servo_controller/Discrete PID Controller'
//  '<S219>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/servo_model/servo_controller/Discrete PID Controller/Clamping circuit'
//  '<S220>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_aerodynamics'
//  '<S221>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics'
//  '<S222>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/calc_nozzle_area'
//  '<S223>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/impeller_model'
//  '<S224>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/motor_rotational_dynamics'
//  '<S225>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/servo_model'
//  '<S226>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/speed_sensor'
//  '<S227>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/Detect Change'
//  '<S228>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/cross_product'
//  '<S229>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies'
//  '<S230>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/vector_normalize'
//  '<S231>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/cross_product/skew_symetric_matrix_operator'
//  '<S232>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/cross_product/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S233>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem'
//  '<S234>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/MATLAB Function'
//  '<S235>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B'
//  '<S236>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B/quaternion_to_DCM'
//  '<S237>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S238>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S239>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S240>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S241>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/vector_normalize/No-op'
//  '<S242>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/vector_normalize/Normalize'
//  '<S243>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/blower_body_dynamics/vector_normalize/vector_magnitude'
//  '<S244>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/impeller_model/dc_motor_model'
//  '<S245>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/impeller_model/speed_controller'
//  '<S246>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/impeller_model/speed_controller/Discrete PID Controller'
//  '<S247>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/impeller_model/speed_controller/Saturation Dynamic'
//  '<S248>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/impeller_model/speed_controller/Discrete PID Controller/Clamping circuit'
//  '<S249>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/servo_model/dc_motor_dynamics'
//  '<S250>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/servo_model/servo_controller'
//  '<S251>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/servo_model/servo_controller/Discrete PID Controller'
//  '<S252>' : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_2_propulsion_module/servo_model/servo_controller/Discrete PID Controller/Clamping circuit'
//  '<S253>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core'
//  '<S254>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/accelerometer_model'
//  '<S255>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/gyroscope_model'
//  '<S256>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_hardware'
//  '<S257>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics'
//  '<S258>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/Cross Product'
//  '<S259>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/Cross Product1'
//  '<S260>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/Cross Product2'
//  '<S261>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/Quaternion_Multiplication'
//  '<S262>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/Quaternion_Multiplication1'
//  '<S263>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B'
//  '<S264>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B1'
//  '<S265>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B2'
//  '<S266>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/Quaternion_Multiplication/Quaternion Xi'
//  '<S267>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/Quaternion_Multiplication/Quaternion Xi/Data Type Conversion Inherited'
//  '<S268>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator'
//  '<S269>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S270>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/Quaternion_Multiplication1/Quaternion Xi'
//  '<S271>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/Quaternion_Multiplication1/Quaternion Xi/Data Type Conversion Inherited'
//  '<S272>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/Quaternion_Multiplication1/Quaternion Xi/skew_symetric_matrix_operator'
//  '<S273>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/Quaternion_Multiplication1/Quaternion Xi/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S274>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B/quaternion_to_DCM'
//  '<S275>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S276>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S277>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S278>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S279>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B1/quaternion_to_DCM'
//  '<S280>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B1/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S281>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B1/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S282>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B1/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S283>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B1/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S284>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B2/quaternion_to_DCM'
//  '<S285>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B2/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S286>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B2/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S287>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B2/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S288>' : 'astrobee/sim_model_lib/veh_vehicle_model/epson_imu_model/imu_model_core/imu_kinematics/rotate_vec_A2B2/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'

#endif                                 // RTW_HEADER_sim_model_lib0_h_

//
// File trailer for generated code.
//
// [EOF]
//
