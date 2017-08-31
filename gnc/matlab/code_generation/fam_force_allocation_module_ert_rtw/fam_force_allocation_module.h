//
// File: fam_force_allocation_module.h
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Thu Aug 31 10:21:33 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_fam_force_allocation_module_h_
#define RTW_HEADER_fam_force_allocation_module_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef fam_force_allocation_module_COMMON_INCLUDES_
# define fam_force_allocation_module_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#endif                                 // fam_force_allocation_module_COMMON_INCLUDES_ 

#include "fam_force_allocation_module_types.h"
#include "look1_iflf_pbinlcapw.h"
#include "plook_u32f_binckpan.h"
#include "pphlopppohlnphdb_pinv.h"
#include "rt_nonfinite.h"
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
  real32_T OutportBufferForforce2thrust_B[36];// '<S9>/MATLAB Function'
  real32_T OutportBufferForthrust2force_B[36];// '<S9>/MATLAB Function'
  real32_T OutportBufferForthrust2torque_B[36];// '<S9>/MATLAB Function'
  real32_T OutportBufferFortorque2thrust_B[36];// '<S9>/MATLAB Function'
} B_fam_force_allocation_module_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T UnitDelay_DSTATE;             // '<S3>/Unit Delay'
  uint32_T m_bpIndex;                  // '<S11>/impeller_speed_lookup'
  uint32_T m_bpIndex_e;                // '<S14>/impeller_speed_lookup'
  uint32_T m_bpIndex_n;                // '<S13>/fam_Cdp_lookup_pm1'
  uint32_T m_bpIndex_a;                // '<S16>/fam_Cdp_lookup_pm1'
} DW_fam_force_allocation_modul_T;

// Parameters (auto storage)
struct P_fam_force_allocation_module_T_ {
  real32_T abp_PM1_discharge_coeff[6]; // Variable: abp_PM1_discharge_coeff
                                       //  Referenced by:
                                       //    '<S12>/Constant5'
                                       //    '<S13>/Constant5'

  real32_T abp_PM1_nozzle_widths[6];   // Variable: abp_PM1_nozzle_widths
                                       //  Referenced by: '<S12>/Constant7'

  real32_T abp_PM2_discharge_coeff[6]; // Variable: abp_PM2_discharge_coeff
                                       //  Referenced by:
                                       //    '<S15>/Constant5'
                                       //    '<S16>/Constant5'

  real32_T abp_PM2_nozzle_widths[6];   // Variable: abp_PM2_nozzle_widths
                                       //  Referenced by: '<S15>/Constant7'

  real32_T abp_impeller_diameter;      // Variable: abp_impeller_diameter
                                       //  Referenced by:
                                       //    '<S13>/Constant1'
                                       //    '<S16>/Constant1'

  real32_T abp_impeller_speed2pwm;     // Variable: abp_impeller_speed2pwm
                                       //  Referenced by:
                                       //    '<S11>/Constant'
                                       //    '<S14>/Constant'

  real32_T abp_nozzle_flap_count;      // Variable: abp_nozzle_flap_count
                                       //  Referenced by:
                                       //    '<S12>/Constant3'
                                       //    '<S15>/Constant3'

  real32_T abp_nozzle_flap_length;     // Variable: abp_nozzle_flap_length
                                       //  Referenced by:
                                       //    '<S12>/Constant6'
                                       //    '<S15>/Constant6'

  real32_T abp_nozzle_intake_height;   // Variable: abp_nozzle_intake_height
                                       //  Referenced by:
                                       //    '<S12>/Constant9'
                                       //    '<S15>/Constant9'

  real32_T abp_nozzle_max_open_angle;  // Variable: abp_nozzle_max_open_angle
                                       //  Referenced by:
                                       //    '<S12>/Saturation'
                                       //    '<S15>/Saturation'

  real32_T abp_nozzle_min_open_angle;  // Variable: abp_nozzle_min_open_angle
                                       //  Referenced by:
                                       //    '<S12>/Constant8'
                                       //    '<S12>/Saturation'
                                       //    '<S15>/Constant8'
                                       //    '<S15>/Saturation'

  real32_T abp_servo_min_PWM;          // Variable: abp_servo_min_PWM
                                       //  Referenced by:
                                       //    '<S12>/Constant2'
                                       //    '<S15>/Constant2'

  real32_T const_air_density;          // Variable: const_air_density
                                       //  Referenced by:
                                       //    '<S13>/Constant2'
                                       //    '<S16>/Constant2'

  real32_T fam_PM1_lookup_Cdp_data[316];// Variable: fam_PM1_lookup_Cdp_data
                                        //  Referenced by: '<S13>/fam_Cdp_lookup_pm1'

  real32_T fam_PM1_lookup_thrust_breakpoints[316];// Variable: fam_PM1_lookup_thrust_breakpoints
                                                  //  Referenced by: '<S13>/fam_Cdp_lookup_pm1'

  real32_T fam_PM2_lookup_Cdp_data[316];// Variable: fam_PM2_lookup_Cdp_data
                                        //  Referenced by: '<S16>/fam_Cdp_lookup_pm1'

  real32_T fam_PM2_lookup_thrust_breakpoints[316];// Variable: fam_PM2_lookup_thrust_breakpoints
                                                  //  Referenced by: '<S16>/fam_Cdp_lookup_pm1'

  real32_T fam_P_nozzle_B_B[36];       // Variable: fam_P_nozzle_B_B
                                       //  Referenced by: '<S9>/Constant1'

  real32_T fam_impeller_speeds[3];     // Variable: fam_impeller_speeds
                                       //  Referenced by:
                                       //    '<S11>/impeller_speed_lookup'
                                       //    '<S14>/impeller_speed_lookup'

  real32_T fam_nozzle_angle2pwm;       // Variable: fam_nozzle_angle2pwm
                                       //  Referenced by:
                                       //    '<S12>/Constant1'
                                       //    '<S15>/Constant1'

  real32_T fam_nozzle_orientations[36];// Variable: fam_nozzle_orientations
                                       //  Referenced by: '<S9>/Constant3'

  real_T UnitDelay_InitialCondition;   // Expression: 1
                                       //  Referenced by: '<S3>/Unit Delay'

  real_T Constant_Value;               // Expression: 0
                                       //  Referenced by: '<S3>/Constant'

  real32_T force2thrust_B_Y0;          // Computed Parameter: force2thrust_B_Y0
                                       //  Referenced by: '<S9>/force2thrust_B'

  real32_T torque2thrust_B_Y0;         // Computed Parameter: torque2thrust_B_Y0
                                       //  Referenced by: '<S9>/torque2thrust_B'

  real32_T thrust2force_B_Y0;          // Computed Parameter: thrust2force_B_Y0
                                       //  Referenced by: '<S9>/thrust2force_B'

  real32_T thrust2torque_B_Y0;         // Computed Parameter: thrust2torque_B_Y0
                                       //  Referenced by: '<S9>/thrust2torque_B'

  real32_T impeller_speed_lookup_bp01Data[3];// Computed Parameter: impeller_speed_lookup_bp01Data
                                             //  Referenced by: '<S11>/impeller_speed_lookup'

  real32_T impeller_speed_lookup_bp01Dat_l[3];// Computed Parameter: impeller_speed_lookup_bp01Dat_l
                                              //  Referenced by: '<S14>/impeller_speed_lookup'

  real32_T Constant4_Value[12];        // Expression: zeros(12,1,'single')
                                       //  Referenced by: '<S3>/Constant4'

  real32_T Constant_Value_k;           // Computed Parameter: Constant_Value_k
                                       //  Referenced by: '<S6>/Constant'

  real32_T Constant_Value_i;           // Computed Parameter: Constant_Value_i
                                       //  Referenced by: '<S7>/Constant'

  real32_T Constant_Value_ii;          // Computed Parameter: Constant_Value_ii
                                       //  Referenced by: '<S8>/Constant'

  real32_T Constant4_Value_o;          // Expression: single(2)
                                       //  Referenced by: '<S12>/Constant4'

  real32_T Constant4_Value_m;          // Expression: single(2)
                                       //  Referenced by: '<S15>/Constant4'

};

// Real-time Model Data Structure
struct tag_RTM_fam_force_allocation__T {
  const char_T * volatile errorStatus;
  B_fam_force_allocation_module_T *blockIO;
  P_fam_force_allocation_module_T *defaultParam;
  boolean_T paramIsMalloced;
  DW_fam_force_allocation_modul_T *dwork;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// External data declarations for dependent source files
extern const ex_time_msg fam_force_allocation_module_rtZex_time_msg;// ex_time_msg ground 
extern const cmd_msg fam_force_allocation_module_rtZcmd_msg;// cmd_msg ground
extern const ctl_msg fam_force_allocation_module_rtZctl_msg;// ctl_msg ground
extern const cmc_msg fam_force_allocation_module_rtZcmc_msg;// cmc_msg ground
extern const act_msg fam_force_allocation_module_rtZact_msg;// act_msg ground

#ifdef __cplusplus

extern "C" {

#endif

  extern const char *RT_MEMORY_ALLOCATION_ERROR;

#ifdef __cplusplus

}
#endif

extern P_fam_force_allocation_module_T fam_force_allocation_module_P;// parameters 

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern RT_MODEL_fam_force_allocation_T *fam_force_allocation_module
    (ex_time_msg *fam_force_allocation_module_U_current_time, cmd_msg
     *fam_force_allocation_module_U_cmd_msg_f, ctl_msg
     *fam_force_allocation_module_U_ctl_msg_n, cmc_msg
     *fam_force_allocation_module_U_cmc_msg_h, act_msg
     *fam_force_allocation_module_Y_act_msg_c);
  extern void fam_force_allocation_module_initialize
    (RT_MODEL_fam_force_allocation_T *const fam_force_allocation_module_M,
     ex_time_msg *fam_force_allocation_module_U_current_time, cmd_msg
     *fam_force_allocation_module_U_cmd_msg_f, ctl_msg
     *fam_force_allocation_module_U_ctl_msg_n, cmc_msg
     *fam_force_allocation_module_U_cmc_msg_h, act_msg
     *fam_force_allocation_module_Y_act_msg_c);
  extern void fam_force_allocation_module_step(RT_MODEL_fam_force_allocation_T *
    const fam_force_allocation_module_M, ex_time_msg
    *fam_force_allocation_module_U_current_time, cmd_msg
    *fam_force_allocation_module_U_cmd_msg_f, ctl_msg
    *fam_force_allocation_module_U_ctl_msg_n, cmc_msg
    *fam_force_allocation_module_U_cmc_msg_h, act_msg
    *fam_force_allocation_module_Y_act_msg_c);
  extern void fam_force_allocation_module_terminate
    (RT_MODEL_fam_force_allocation_T * fam_force_allocation_module_M);

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
//  hilite_system('astrobee/fsw_lib/fam_force_allocation_module')    - opens subsystem astrobee/fsw_lib/fam_force_allocation_module
//  hilite_system('astrobee/fsw_lib/fam_force_allocation_module/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'astrobee/fsw_lib'
//  '<S1>'   : 'astrobee/fsw_lib/fam_force_allocation_module'
//  '<S2>'   : 'astrobee/fsw_lib/fam_force_allocation_module/back_predict_force_and_torque'
//  '<S3>'   : 'astrobee/fsw_lib/fam_force_allocation_module/calc_nozzle_thrust'
//  '<S4>'   : 'astrobee/fsw_lib/fam_force_allocation_module/calculate_PM1_servo_command'
//  '<S5>'   : 'astrobee/fsw_lib/fam_force_allocation_module/calculate_PM2_servo_command'
//  '<S6>'   : 'astrobee/fsw_lib/fam_force_allocation_module/calc_nozzle_thrust/Compare To Zero6'
//  '<S7>'   : 'astrobee/fsw_lib/fam_force_allocation_module/calc_nozzle_thrust/Compare To Zero7'
//  '<S8>'   : 'astrobee/fsw_lib/fam_force_allocation_module/calc_nozzle_thrust/Compare To Zero8'
//  '<S9>'   : 'astrobee/fsw_lib/fam_force_allocation_module/calc_nozzle_thrust/latch_nozzle_thrust_matricies'
//  '<S10>'  : 'astrobee/fsw_lib/fam_force_allocation_module/calc_nozzle_thrust/latch_nozzle_thrust_matricies/MATLAB Function'
//  '<S11>'  : 'astrobee/fsw_lib/fam_force_allocation_module/calculate_PM1_servo_command/calc_impeller_speed'
//  '<S12>'  : 'astrobee/fsw_lib/fam_force_allocation_module/calculate_PM1_servo_command/calc_servo_cmd'
//  '<S13>'  : 'astrobee/fsw_lib/fam_force_allocation_module/calculate_PM1_servo_command/lookup_data_tables'
//  '<S14>'  : 'astrobee/fsw_lib/fam_force_allocation_module/calculate_PM2_servo_command/calc_impeller_speed'
//  '<S15>'  : 'astrobee/fsw_lib/fam_force_allocation_module/calculate_PM2_servo_command/calc_servo_cmd'
//  '<S16>'  : 'astrobee/fsw_lib/fam_force_allocation_module/calculate_PM2_servo_command/lookup_data_tables'

#endif                                 // RTW_HEADER_fam_force_allocation_module_h_ 

//
// File trailer for generated code.
//
// [EOF]
//
