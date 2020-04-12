//
// File: bpm_blower_1_propulsion_module.h
//
// Code generated for Simulink model 'bpm_blower_1_propulsion_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Sep 23 17:46:28 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_bpm_blower_1_propulsion_module_h_
#define RTW_HEADER_bpm_blower_1_propulsion_module_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef bpm_blower_1_propulsion_module_COMMON_INCLUDES_
# define bpm_blower_1_propulsion_module_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#endif                                 // bpm_blower_1_propulsion_module_COMMON_INCLUDES_ 

#include "bpm_blower_1_propulsion_module_types.h"

// Child system includes
#include "blower_aerodynamics.h"
#include "rt_nonfinite.h"
#include "rt_nrand_Upu32_Yd_f_pw_snf.h"
#include "rt_roundf_snf.h"
#include "rtGetInf.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Block signals (auto storage)
typedef struct {
  real32_T OutportBufferForthrust2force_B[18];// '<S11>/MATLAB Function'
  real32_T OutportBufferForthrust2torque_B[18];// '<S11>/MATLAB Function'
  B_blower_aerodynamics_bpm_blo_T blower_aerodynamics;// '<S1>/blower_aerodynamics' 
} B_bpm_blower_1_propulsion_mod_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T NextOutput;                   // '<S8>/random_noise1'
  real_T NextOutput_e;                 // '<S8>/random_noise'
  real32_T DiscreteTimeIntegrator_DSTATE;// '<S6>/Discrete-Time Integrator'
  real32_T Delay2_DSTATE;              // '<S8>/Delay2'
  real32_T DiscreteTimeIntegrator1_DSTATE;// '<S8>/Discrete-Time Integrator1'
  real32_T DelayInput1_DSTATE[3];      // '<S9>/Delay Input1'
  real32_T DiscreteTimeIntegrator4_DSTATE[6];// '<S31>/Discrete-Time Integrator4' 
  real32_T Integrator_DSTATE[6];       // '<S33>/Integrator'
  real32_T Filter_DSTATE[6];           // '<S33>/Filter'
  real32_T DiscreteTimeIntegrator3_DSTATE[6];// '<S31>/Discrete-Time Integrator3' 
  real32_T Integrator_DSTATE_k;        // '<S28>/Integrator'
  real32_T Filter_DSTATE_o;            // '<S28>/Filter'
  real32_T PrevY[6];                   // '<S31>/Backlash1'
  real32_T PrevY_l;                    // '<S27>/Rate Limiter'
  uint32_T RandSeed;                   // '<S8>/random_noise1'
  uint32_T RandSeed_j;                 // '<S8>/random_noise'
  boolean_T UnitDelay_DSTATE;          // '<S3>/Unit Delay'
  DW_blower_aerodynamics_bpm_bl_T blower_aerodynamics;// '<S1>/blower_aerodynamics' 
} DW_bpm_blower_1_propulsion_mo_T;

// Parameters for system: '<S15>/CoreSubsys'
struct P_CoreSubsys_bpm_blower_1_pro_T_ {
  real_T Constant2_Value[9];           // Expression: zeros(9,1)
                                       //  Referenced by: '<S18>/Constant2'

  real_T Constant1_Value;              // Expression: 1
                                       //  Referenced by: '<S18>/Constant1'

  real_T Constant3_Value;              // Expression: 0
                                       //  Referenced by: '<S21>/Constant3'

  real32_T Gain_Gain;                  // Computed Parameter: Gain_Gain
                                       //  Referenced by: '<S18>/Gain'

  real32_T Gain1_Gain;                 // Computed Parameter: Gain1_Gain
                                       //  Referenced by: '<S18>/Gain1'

  real32_T Gain_Gain_a;                // Computed Parameter: Gain_Gain_a
                                       //  Referenced by: '<S21>/Gain'

  real32_T Gain1_Gain_m;               // Computed Parameter: Gain1_Gain_m
                                       //  Referenced by: '<S21>/Gain1'

  real32_T Gain2_Gain;                 // Computed Parameter: Gain2_Gain
                                       //  Referenced by: '<S21>/Gain2'

  real32_T Gain2_Gain_i;               // Computed Parameter: Gain2_Gain_i
                                       //  Referenced by: '<S18>/Gain2'

};

// Parameters (auto storage)
struct P_bpm_blower_1_propulsion_mod_T_ {
  real_T abp_P_CG_B_B_error[3];        // Variable: abp_P_CG_B_B_error
                                       //  Referenced by: '<S11>/Constant7'

  real_T astrobee_time_step_size;      // Variable: astrobee_time_step_size
                                       //  Referenced by:
                                       //    '<S8>/Gain1'
                                       //    '<S8>/Gain4'

  real_T bpm_PM1_nozzle_noise_feedback_gain[6];// Variable: bpm_PM1_nozzle_noise_feedback_gain
                                               //  Referenced by: '<S1>/blower_aerodynamics'

  real_T bpm_PM1_randn_noise_seed;     // Variable: bpm_PM1_randn_noise_seed
                                       //  Referenced by: '<S1>/blower_aerodynamics'

  real_T bpm_sensor_rand_seed;         // Variable: bpm_sensor_rand_seed
                                       //  Referenced by:
                                       //    '<S8>/random_noise'
                                       //    '<S8>/random_noise1'

  real_T bpm_servo_min_theta;          // Variable: bpm_servo_min_theta
                                       //  Referenced by: '<S31>/Discrete-Time Integrator4'

  real32_T abp_PM1_P_nozzle_B_B[18];   // Variable: abp_PM1_P_nozzle_B_B
                                       //  Referenced by: '<S11>/Constant1'

  real32_T abp_PM1_discharge_coeff[6]; // Variable: abp_PM1_discharge_coeff
                                       //  Referenced by: '<S1>/blower_aerodynamics'

  real32_T abp_PM1_nozzle_orientations[18];// Variable: abp_PM1_nozzle_orientations
                                           //  Referenced by: '<S11>/Constant3'

  real32_T abp_PM1_nozzle_widths[6];   // Variable: abp_PM1_nozzle_widths
                                       //  Referenced by: '<S4>/Constant6'

  real32_T abp_impeller_diameter;      // Variable: abp_impeller_diameter
                                       //  Referenced by: '<S1>/blower_aerodynamics'

  real32_T abp_nozzle_flap_count;      // Variable: abp_nozzle_flap_count
                                       //  Referenced by: '<S4>/Constant1'

  real32_T abp_nozzle_flap_length;     // Variable: abp_nozzle_flap_length
                                       //  Referenced by: '<S4>/Constant4'

  real32_T abp_nozzle_gear_ratio;      // Variable: abp_nozzle_gear_ratio
                                       //  Referenced by: '<S4>/Gain12'

  real32_T abp_nozzle_intake_height;   // Variable: abp_nozzle_intake_height
                                       //  Referenced by: '<S4>/Constant5'

  real32_T abp_nozzle_min_open_angle;  // Variable: abp_nozzle_min_open_angle
                                       //  Referenced by: '<S4>/Constant2'

  real32_T abp_pm1_impeller_orientation[3];// Variable: abp_pm1_impeller_orientation
                                           //  Referenced by: '<S3>/Constant1'

  real32_T abp_pm1_zero_thrust_area;   // Variable: abp_pm1_zero_thrust_area
                                       //  Referenced by: '<S1>/blower_aerodynamics'

  real32_T bmp_PM1_impeller_orientation_error[3];// Variable: bmp_PM1_impeller_orientation_error
                                                 //  Referenced by: '<S3>/Constant3'

  real32_T bpm_PM1_P_nozzle_B_B_error[18];// Variable: bpm_PM1_P_nozzle_B_B_error
                                          //  Referenced by: '<S11>/Constant4'

  real32_T bpm_PM1_Q_nozzle2misaligned[24];// Variable: bpm_PM1_Q_nozzle2misaligned
                                           //  Referenced by: '<S11>/Constant5'

  real32_T bpm_PM1_nozzle_discharge_coeff_error[6];// Variable: bpm_PM1_nozzle_discharge_coeff_error
                                                   //  Referenced by: '<S1>/blower_aerodynamics'

  real32_T bpm_PM1_zero_thrust_area_error;// Variable: bpm_PM1_zero_thrust_area_error
                                          //  Referenced by: '<S1>/blower_aerodynamics'

  real32_T bpm_imp_ctl_filt_n;         // Variable: bpm_imp_ctl_filt_n
                                       //  Referenced by: '<S28>/Filter Coefficient'

  real32_T bpm_imp_ctl_kd;             // Variable: bpm_imp_ctl_kd
                                       //  Referenced by: '<S28>/Derivative Gain'

  real32_T bpm_imp_ctl_ki;             // Variable: bpm_imp_ctl_ki
                                       //  Referenced by: '<S28>/Integral Gain'

  real32_T bpm_imp_ctl_kp;             // Variable: bpm_imp_ctl_kp
                                       //  Referenced by: '<S28>/Proportional Gain'

  real32_T bpm_imp_max_voltage;        // Variable: bpm_imp_max_voltage
                                       //  Referenced by:
                                       //    '<S28>/Saturate'
                                       //    '<S30>/DeadZone'

  real32_T bpm_imp_motor_friction_coeff;// Variable: bpm_imp_motor_friction_coeff
                                        //  Referenced by: '<S26>/Gain1'

  real32_T bpm_imp_motor_r;            // Variable: bpm_imp_motor_r
                                       //  Referenced by: '<S26>/Gain5'

  real32_T bpm_imp_motor_speed_k;      // Variable: bpm_imp_motor_speed_k
                                       //  Referenced by: '<S26>/Gain4'

  real32_T bpm_imp_motor_torque_k;     // Variable: bpm_imp_motor_torque_k
                                       //  Referenced by: '<S26>/Gain6'

  real32_T bpm_imp_speed_filt_den;     // Variable: bpm_imp_speed_filt_den
                                       //  Referenced by: '<S27>/Discrete Transfer Fcn1'

  real32_T bpm_imp_speed_filt_num;     // Variable: bpm_imp_speed_filt_num
                                       //  Referenced by: '<S27>/Discrete Transfer Fcn1'

  real32_T bpm_impeller_inertia;       // Variable: bpm_impeller_inertia
                                       //  Referenced by:
                                       //    '<S3>/Constant2'
                                       //    '<S6>/Gain'
                                       //    '<S6>/Gain2'

  real32_T bpm_impeller_inertia_error; // Variable: bpm_impeller_inertia_error
                                       //  Referenced by:
                                       //    '<S3>/Constant4'
                                       //    '<S6>/Gain2'

  real32_T bpm_impeller_init_speed;    // Variable: bpm_impeller_init_speed
                                       //  Referenced by: '<S6>/Discrete-Time Integrator'

  real32_T bpm_lookup_Cdp_data[334];   // Variable: bpm_lookup_Cdp_data
                                       //  Referenced by: '<S1>/blower_aerodynamics'

  real32_T bpm_lookup_totalarea_breakpoints[334];// Variable: bpm_lookup_totalarea_breakpoints
                                                 //  Referenced by: '<S1>/blower_aerodynamics'

  real32_T bpm_sensor_max;             // Variable: bpm_sensor_max
                                       //  Referenced by: '<S8>/Saturation'

  real32_T bpm_sensor_min;             // Variable: bpm_sensor_min
                                       //  Referenced by: '<S8>/Saturation'

  real32_T bpm_sensor_resolution;      // Variable: bpm_sensor_resolution
                                       //  Referenced by: '<S8>/Quantizer'

  real32_T bpm_sensor_sf;              // Variable: bpm_sensor_sf
                                       //  Referenced by: '<S8>/Gain3'

  real32_T bpm_servo_ctl_filt_n;       // Variable: bpm_servo_ctl_filt_n
                                       //  Referenced by: '<S33>/Filter Coefficient'

  real32_T bpm_servo_ctl_kd;           // Variable: bpm_servo_ctl_kd
                                       //  Referenced by: '<S33>/Derivative Gain'

  real32_T bpm_servo_ctl_ki;           // Variable: bpm_servo_ctl_ki
                                       //  Referenced by: '<S33>/Integral Gain'

  real32_T bpm_servo_ctl_kp;           // Variable: bpm_servo_ctl_kp
                                       //  Referenced by: '<S33>/Proportional Gain'

  real32_T bpm_servo_max_theta;        // Variable: bpm_servo_max_theta
                                       //  Referenced by: '<S31>/Discrete-Time Integrator4'

  real32_T bpm_servo_max_voltage;      // Variable: bpm_servo_max_voltage
                                       //  Referenced by:
                                       //    '<S33>/Saturate'
                                       //    '<S34>/DeadZone'

  real32_T bpm_servo_motor_backlash_deadband;// Variable: bpm_servo_motor_backlash_deadband
                                             //  Referenced by: '<S31>/Backlash1'

  real32_T bpm_servo_motor_friction_coeff;// Variable: bpm_servo_motor_friction_coeff
                                          //  Referenced by: '<S31>/Gain9'

  real32_T bpm_servo_motor_gear_box_inertia;// Variable: bpm_servo_motor_gear_box_inertia
                                            //  Referenced by: '<S31>/Gain6'

  real32_T bpm_servo_motor_gear_ratio; // Variable: bpm_servo_motor_gear_ratio
                                       //  Referenced by:
                                       //    '<S31>/Discrete-Time Integrator4'
                                       //    '<S31>/Gain1'

  real32_T bpm_servo_motor_k;          // Variable: bpm_servo_motor_k
                                       //  Referenced by:
                                       //    '<S31>/Gain11'
                                       //    '<S31>/Gain7'

  real32_T bpm_servo_motor_r;          // Variable: bpm_servo_motor_r
                                       //  Referenced by: '<S31>/Gain5'

  real32_T bpm_servo_pwm2angle;        // Variable: bpm_servo_pwm2angle
                                       //  Referenced by: '<S32>/Gain'

  real32_T bpm_servo_pwm2angle_bias;   // Variable: bpm_servo_pwm2angle_bias
                                       //  Referenced by: '<S32>/Constant'

  real32_T const_air_density;          // Variable: const_air_density
                                       //  Referenced by: '<S1>/blower_aerodynamics'

  real32_T tun_bpm_PM1_thrust_error_sf;// Variable: tun_bpm_PM1_thrust_error_sf
                                       //  Referenced by: '<S1>/blower_aerodynamics'

  real32_T tun_bpm_noise_on_flag;      // Variable: tun_bpm_noise_on_flag
                                       //  Referenced by:
                                       //    '<S1>/blower_aerodynamics'
                                       //    '<S3>/Gain'
                                       //    '<S3>/Gain1'
                                       //    '<S6>/Constant'
                                       //    '<S11>/Constant8'
                                       //    '<S11>/Gain'
                                       //    '<S11>/Gain2'

  real32_T DiscretePIDController_LowerSatu;// Mask Parameter: DiscretePIDController_LowerSatu
                                           //  Referenced by:
                                           //    '<S28>/Saturate'
                                           //    '<S30>/DeadZone'

  real32_T DiscretePIDController_LowerSa_b;// Mask Parameter: DiscretePIDController_LowerSa_b
                                           //  Referenced by:
                                           //    '<S33>/Saturate'
                                           //    '<S34>/DeadZone'

  real32_T bpm_blower_1_propulsion_module_;// Mask Parameter: bpm_blower_1_propulsion_module_
                                           //  Referenced by: '<S27>/Constant'

  real32_T DetectChange_vinit;         // Mask Parameter: DetectChange_vinit
                                       //  Referenced by: '<S9>/Delay Input1'

  real_T Constant3_Value;              // Expression: 0
                                       //  Referenced by: '<S13>/Constant3'

  real_T random_noise1_Mean;           // Expression: 0
                                       //  Referenced by: '<S8>/random_noise1'

  real_T random_noise1_StdDev;         // Computed Parameter: random_noise1_StdDev
                                       //  Referenced by: '<S8>/random_noise1'

  real_T random_noise_Mean;            // Expression: 0
                                       //  Referenced by: '<S8>/random_noise'

  real_T random_noise_StdDev;          // Computed Parameter: random_noise_StdDev
                                       //  Referenced by: '<S8>/random_noise'

  real32_T Constant2_Value[24];        // Expression: [zeros(6,3,'single'), ones(6,1,'single')]
                                       //  Referenced by: '<S11>/Constant2'

  real32_T thrust2torque_B_Y0;         // Computed Parameter: thrust2torque_B_Y0
                                       //  Referenced by: '<S11>/thrust2torque_B'

  real32_T thrust2force_B_Y0;          // Computed Parameter: thrust2force_B_Y0
                                       //  Referenced by: '<S11>/thrust2force_B'

  real32_T Switch_Threshold;           // Computed Parameter: Switch_Threshold
                                       //  Referenced by: '<S11>/Switch'

  real32_T Gain_Gain;                  // Computed Parameter: Gain_Gain
                                       //  Referenced by: '<S27>/Gain'

  real32_T DiscreteTransferFcn1_InitialSta;// Computed Parameter: DiscreteTransferFcn1_InitialSta
                                           //  Referenced by: '<S27>/Discrete Transfer Fcn1'

  real32_T RateLimiter_RisingLim;      // Computed Parameter: RateLimiter_RisingLim
                                       //  Referenced by: '<S27>/Rate Limiter'

  real32_T RateLimiter_FallingLim;     // Computed Parameter: RateLimiter_FallingLim
                                       //  Referenced by: '<S27>/Rate Limiter'

  real32_T RateLimiter_IC;             // Computed Parameter: RateLimiter_IC
                                       //  Referenced by: '<S27>/Rate Limiter'

  real32_T Integrator_gainval;         // Computed Parameter: Integrator_gainval
                                       //  Referenced by: '<S28>/Integrator'

  real32_T Integrator_IC;              // Computed Parameter: Integrator_IC
                                       //  Referenced by: '<S28>/Integrator'

  real32_T Filter_gainval;             // Computed Parameter: Filter_gainval
                                       //  Referenced by: '<S28>/Filter'

  real32_T Filter_IC;                  // Computed Parameter: Filter_IC
                                       //  Referenced by: '<S28>/Filter'

  real32_T ZeroGain_Gain;              // Computed Parameter: ZeroGain_Gain
                                       //  Referenced by: '<S30>/ZeroGain'

  real32_T Constant_Value;             // Computed Parameter: Constant_Value
                                       //  Referenced by: '<S28>/Constant'

  real32_T DiscreteTimeIntegrator4_gainval;// Computed Parameter: DiscreteTimeIntegrator4_gainval
                                           //  Referenced by: '<S31>/Discrete-Time Integrator4'

  real32_T Backlash1_InitialOutput;    // Computed Parameter: Backlash1_InitialOutput
                                       //  Referenced by: '<S31>/Backlash1'

  real32_T Integrator_gainval_f;       // Computed Parameter: Integrator_gainval_f
                                       //  Referenced by: '<S33>/Integrator'

  real32_T Integrator_IC_n;            // Computed Parameter: Integrator_IC_n
                                       //  Referenced by: '<S33>/Integrator'

  real32_T Filter_gainval_n;           // Computed Parameter: Filter_gainval_n
                                       //  Referenced by: '<S33>/Filter'

  real32_T Filter_IC_g;                // Computed Parameter: Filter_IC_g
                                       //  Referenced by: '<S33>/Filter'

  real32_T DiscreteTimeIntegrator3_gainval;// Computed Parameter: DiscreteTimeIntegrator3_gainval
                                           //  Referenced by: '<S31>/Discrete-Time Integrator3'

  real32_T DiscreteTimeIntegrator3_IC; // Expression: single(0)
                                       //  Referenced by: '<S31>/Discrete-Time Integrator3'

  real32_T ZeroGain_Gain_e;            // Computed Parameter: ZeroGain_Gain_e
                                       //  Referenced by: '<S34>/ZeroGain'

  real32_T Constant_Value_n;           // Computed Parameter: Constant_Value_n
                                       //  Referenced by: '<S33>/Constant'

  real32_T DiscreteTimeIntegrator_gainval;// Computed Parameter: DiscreteTimeIntegrator_gainval
                                          //  Referenced by: '<S6>/Discrete-Time Integrator'

  real32_T Gain1_Gain;                 // Computed Parameter: Gain1_Gain
                                       //  Referenced by: '<S6>/Gain1'

  real32_T Gain_Gain_h;                // Computed Parameter: Gain_Gain_h
                                       //  Referenced by: '<S13>/Gain'

  real32_T Gain1_Gain_f;               // Computed Parameter: Gain1_Gain_f
                                       //  Referenced by: '<S13>/Gain1'

  real32_T Gain2_Gain;                 // Computed Parameter: Gain2_Gain
                                       //  Referenced by: '<S13>/Gain2'

  real32_T Delay2_InitialCondition;    // Computed Parameter: Delay2_InitialCondition
                                       //  Referenced by: '<S8>/Delay2'

  real32_T DiscreteTimeIntegrator1_gainval;// Computed Parameter: DiscreteTimeIntegrator1_gainval
                                           //  Referenced by: '<S8>/Discrete-Time Integrator1'

  real32_T DiscreteTimeIntegrator1_IC; // Expression: single(0)
                                       //  Referenced by: '<S8>/Discrete-Time Integrator1'

  real32_T Switch_Threshold_j;         // Computed Parameter: Switch_Threshold_j
                                       //  Referenced by: '<S6>/Switch'

  uint32_T Delay2_DelayLength;         // Computed Parameter: Delay2_DelayLength
                                       //  Referenced by: '<S8>/Delay2'

  boolean_T UnitDelay_InitialCondition;// Computed Parameter: UnitDelay_InitialCondition
                                       //  Referenced by: '<S3>/Unit Delay'

  P_CoreSubsys_bpm_blower_1_pro_T CoreSubsys;// '<S15>/CoreSubsys'
  P_blower_aerodynamics_bpm_blo_T blower_aerodynamics;// '<S1>/blower_aerodynamics' 
};

// Real-time Model Data Structure
struct tag_RTM_bpm_blower_1_propulsi_T {
  const char_T * volatile errorStatus;
  B_bpm_blower_1_propulsion_mod_T *blockIO;
  P_bpm_blower_1_propulsion_mod_T *defaultParam;
  boolean_T paramIsMalloced;
  DW_bpm_blower_1_propulsion_mo_T *dwork;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// External data declarations for dependent source files
#ifdef __cplusplus

extern "C" {

#endif

  extern const char *RT_MEMORY_ALLOCATION_ERROR;

#ifdef __cplusplus

}
#endif

extern P_bpm_blower_1_propulsion_mod_T bpm_blower_1_propulsion_modul_P;// parameters 

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern RT_MODEL_bpm_blower_1_propuls_T *bpm_blower_1_propulsion_module
    (real32_T *bpm_blower_1_propulsion_modul_U_battery_voltage, real32_T
     bpm_blower_1_propulsion_modul_U_omega_B_ECI_B[3], uint8_T
     *bpm_blower_1_propulsion_modul_U_impeller_cmd, real32_T
     bpm_blower_1_propulsion_modul_U_servo_cmd[6], real32_T
     bpm_blower_1_propulsion_modul_U_veh_cm[3], real32_T
     *bpm_blower_1_propulsion_modul_Y_impeller_current, real32_T
     bpm_blower_1_propulsion_modul_Y_servo_current[6], real32_T
     bpm_blower_1_propulsion_modul_Y_torque_B[3], real32_T
     bpm_blower_1_propulsion_modul_Y_force_B[3], real32_T
     *bpm_blower_1_propulsion_modul_Y_motor_speed, real32_T
     bpm_blower_1_propulsion_modul_Y_nozzle_theta[6], real32_T
     *bpm_blower_1_propulsion_modul_Y_meas_motor_speed);
  extern void bpm_blower_1_propulsion_module_initialize
    (RT_MODEL_bpm_blower_1_propuls_T *const bpm_blower_1_propulsion_modu_M,
     real32_T *bpm_blower_1_propulsion_modul_U_battery_voltage, real32_T
     bpm_blower_1_propulsion_modul_U_omega_B_ECI_B[3], uint8_T
     *bpm_blower_1_propulsion_modul_U_impeller_cmd, real32_T
     bpm_blower_1_propulsion_modul_U_servo_cmd[6], real32_T
     bpm_blower_1_propulsion_modul_U_veh_cm[3], real32_T
     *bpm_blower_1_propulsion_modul_Y_impeller_current, real32_T
     bpm_blower_1_propulsion_modul_Y_servo_current[6], real32_T
     bpm_blower_1_propulsion_modul_Y_torque_B[3], real32_T
     bpm_blower_1_propulsion_modul_Y_force_B[3], real32_T
     *bpm_blower_1_propulsion_modul_Y_motor_speed, real32_T
     bpm_blower_1_propulsion_modul_Y_nozzle_theta[6], real32_T
     *bpm_blower_1_propulsion_modul_Y_meas_motor_speed);
  extern void bpm_blower_1_propulsion_module_step
    (RT_MODEL_bpm_blower_1_propuls_T *const bpm_blower_1_propulsion_modu_M,
     real32_T bpm_blower_1_propulsion_modul_U_battery_voltage, real32_T
     bpm_blower_1_propulsion_modul_U_omega_B_ECI_B[3], uint8_T
     bpm_blower_1_propulsion_modul_U_impeller_cmd, real32_T
     bpm_blower_1_propulsion_modul_U_servo_cmd[6], real32_T
     bpm_blower_1_propulsion_modul_U_veh_cm[3], real32_T
     *bpm_blower_1_propulsion_modul_Y_impeller_current, real32_T
     bpm_blower_1_propulsion_modul_Y_servo_current[6], real32_T
     bpm_blower_1_propulsion_modul_Y_torque_B[3], real32_T
     bpm_blower_1_propulsion_modul_Y_force_B[3], real32_T
     *bpm_blower_1_propulsion_modul_Y_motor_speed, real32_T
     bpm_blower_1_propulsion_modul_Y_nozzle_theta[6], real32_T
     *bpm_blower_1_propulsion_modul_Y_meas_motor_speed);
  extern void bpm_blower_1_propulsion_module_terminate
    (RT_MODEL_bpm_blower_1_propuls_T * bpm_blower_1_propulsion_modu_M);

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
//  hilite_system('astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module')    - opens subsystem astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module
//  hilite_system('astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'astrobee/sim_model_lib/veh_vehicle_model'
//  '<S1>'   : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module'
//  '<S2>'   : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_aerodynamics'
//  '<S3>'   : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics'
//  '<S4>'   : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/calc_nozzle_area'
//  '<S5>'   : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/impeller_model'
//  '<S6>'   : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/motor_rotational_dynamics'
//  '<S7>'   : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/servo_model'
//  '<S8>'   : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/speed_sensor'
//  '<S9>'   : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/Detect Change'
//  '<S10>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/cross_product'
//  '<S11>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies'
//  '<S12>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/vector_normalize'
//  '<S13>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/cross_product/skew_symetric_matrix_operator'
//  '<S14>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/cross_product/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S15>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem'
//  '<S16>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/MATLAB Function'
//  '<S17>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B'
//  '<S18>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B/quaternion_to_DCM'
//  '<S19>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S20>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S21>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S22>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/For Each Subsystem/rotate_vec_A2B/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S23>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/vector_normalize/No-op'
//  '<S24>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/vector_normalize/Normalize'
//  '<S25>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/blower_body_dynamics/vector_normalize/vector_magnitude'
//  '<S26>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/impeller_model/dc_motor_model'
//  '<S27>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/impeller_model/speed_controller'
//  '<S28>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/impeller_model/speed_controller/Discrete PID Controller'
//  '<S29>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/impeller_model/speed_controller/Saturation Dynamic'
//  '<S30>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/impeller_model/speed_controller/Discrete PID Controller/Clamping circuit'
//  '<S31>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/servo_model/dc_motor_dynamics'
//  '<S32>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/servo_model/servo_controller'
//  '<S33>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/servo_model/servo_controller/Discrete PID Controller'
//  '<S34>'  : 'astrobee/sim_model_lib/veh_vehicle_model/bpm_blower_1_propulsion_module/servo_model/servo_controller/Discrete PID Controller/Clamping circuit'

#endif                                 // RTW_HEADER_bpm_blower_1_propulsion_module_h_ 

//
// File trailer for generated code.
//
// [EOF]
//
