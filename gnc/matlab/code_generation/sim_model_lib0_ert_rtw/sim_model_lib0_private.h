//
// File: sim_model_lib0_private.h
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
#ifndef RTW_HEADER_sim_model_lib0_private_h_
#define RTW_HEADER_sim_model_lib0_private_h_
#include "rtwtypes.h"
#include "sim_model_lib0.h"
#if !defined(rt_VALIDATE_MEMORY)
#define rt_VALIDATE_MEMORY(S, ptr)     if(!(ptr)) {\
 rtmSetErrorStatus(sim_model_lib0_M, RT_MEMORY_ALLOCATION_ERROR);\
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

#define rtCP_pooled1                   rtCP_pooled_15LYgUQHWYtd  // Pooled Parameter (Expression: noz_randn_seed +[1:6])
                                                                 //  Referenced by:
                                                                 //    '<S187>/random_noise'
                                                                 //    '<S220>/random_noise'


void sim_model_lib0_Normalize(const real32_T rtu_Vec[4], real32_T rtu_Magnitude,
  real32_T rty_Normalized_Vec[4]);
void sim_model_lib0_Normalize_e(const real32_T rtu_Vec[3], real32_T
  rtu_Magnitude, real32_T rty_Normalized_Vec[3]);
void pinhole_projection_mod_Init(int32_T NumIters,
  DW_pinhole_projection_model_s_T localDW[], P_pinhole_projection_model_si_T
  *localP, real_T rtp_pixel_noise_var, real_T rtp_pixel_noise_seed);
void pinhole_projection_mo_Start(int32_T NumIters,
  DW_pinhole_projection_model_s_T localDW[]);
void si_pinhole_projection_model(int32_T NumIters, const real32_T
  rtu_P_points_cam_cam[], real32_T rty_P_points_2D_cam[], boolean_T
  rty_points_in_FOV_flag[], DW_pinhole_projection_model_s_T localDW[],
  P_pinhole_projection_model_si_T *localP, uint8_T rtp_noise_on_flag, real32_T
  rtp_cam_focal_length_Y, real32_T rtp_cam_focal_length_X, real_T
  rtp_pixel_noise_var, real32_T rtp_cam_num_X_pixels, real32_T
  rtp_cam_num_Y_pixels, real32_T rtp_cam_min_dist, real32_T rtp_cam_max_dist,
  const real32_T rtp_cam_pointing[3]);
void latch_nozzle_thrust_ma_Init(B_latch_nozzle_thrust_matrici_T *localB,
  P_latch_nozzle_thrust_matrici_T *localP);
void latch_nozzle_thrust_matrici(boolean_T rtu_Enable, const real32_T
  rtu_veh_cm[3], B_latch_nozzle_thrust_matrici_T *localB,
  P_latch_nozzle_thrust_matrici_T *localP, const real32_T
  rtp_nozzle_orientation_offset[24], const real32_T rtp_P_nozzle_B_B[18], const
  real32_T rtp_nozzle_orientation_B[18], const real32_T rtp_P_nozzle_B_B_offset
  [18], const real_T rtp_P_CG_B_B_error[3], real32_T rtp_noise_on_flag);
void sim_model__calc_nozzle_area(const real32_T rtu_servo_theta[6],
  B_calc_nozzle_area_sim_model__T *localB, real32_T rtp_noz_flap_count, real32_T
  rtp_noz_min_theta, real32_T rtp_noz_flap_length, real32_T
  rtp_noz_intake_height, const real32_T rtp_noz_width[6], real32_T
  rtp_noz_gear_ratio);
void sim_model_li_dc_motor_model(real32_T rtu_ctl_voltage, real32_T
  rtu_curr_rotor_speed, B_dc_motor_model_sim_model_li_T *localB, real32_T
  rtp_motor_kn, real32_T rtp_motor_r, real32_T rtp_motor_kt, real32_T
  rtp_motor_friction_coeff);
void sim_m_speed_controller_Init(DW_speed_controller_sim_model_T *localDW,
  P_speed_controller_sim_model__T *localP);
void sim_model__speed_controller(real32_T rtu_battery_voltage, uint8_T
  rtu_speed_cmd, real32_T rtu_speed_curr, B_speed_controller_sim_model__T
  *localB, DW_speed_controller_sim_model_T *localDW,
  P_speed_controller_sim_model__T *localP, real32_T rtp_ctl_pwm2speed, real32_T
  rtp_ctl_speed_filt_num, real32_T rtp_ctl_speed_cilt_den, real32_T rtp_ctl_kp,
  real32_T rtp_ctl_kd, real32_T rtp_ctl_filt_n, real32_T rtp_ctl_max_voltage,
  real32_T rtp_ctl_ki);
void sim_model__servo_model_Init(DW_servo_model_sim_model_lib0_T *localDW,
  P_servo_model_sim_model_lib0_T *localP, real32_T rtp_servo_min_theta, real32_T
  rtp_motor_gear_ratio);
void sim_model_lib0_servo_model(const real32_T rtu_command_PWM[6],
  B_servo_model_sim_model_lib0_T *localB, DW_servo_model_sim_model_lib0_T
  *localDW, P_servo_model_sim_model_lib0_T *localP, real32_T
  rtp_servo_pwm2angle_bias, real32_T rtp_servo_pwm2angle, real32_T
  rtp_servo_min_theta, real32_T rtp_motor_gear_ratio, real32_T
  rtp_servo_max_theta, real32_T rtp_motor_backlash_deadband, real32_T
  rtp_servo_ctl_kp, real32_T rtp_servo_ctl_kd, real32_T rtp_servo_ctl_filt_n,
  real32_T rtp_servo_max_voltage, real32_T rtp_motor_k, real32_T rtp_motor_r,
  real32_T rtp_motor_friction_coeff, real32_T rtp_motor_gear_box_inertia,
  real32_T rtp_servo_ctl_ki);

#endif                                 // RTW_HEADER_sim_model_lib0_private_h_

//
// File trailer for generated code.
//
// [EOF]
//
