//
// File: sim_model_lib0_types.h
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Sep 23 17:45:47 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_sim_model_lib0_types_h_
#define RTW_HEADER_sim_model_lib0_types_h_
#include "rtwtypes.h"
#ifndef DEFINED_TYPEDEF_FOR_act_msg_
#define DEFINED_TYPEDEF_FOR_act_msg_

typedef struct {
  uint32_T act_timestamp_sec;
  uint32_T act_timestamp_nsec;
  uint8_T act_impeller_speed_cmd[2];
  real32_T act_servo_pwm_cmd[12];
  real32_T act_nozzle_theta[12];
  real32_T act_predicted_force_B[3];
  real32_T act_predicted_torque_B[3];
} act_msg;

#endif

#ifndef DEFINED_TYPEDEF_FOR_cmc_state_cmd_
#define DEFINED_TYPEDEF_FOR_cmc_state_cmd_

typedef struct {
  uint32_T timestamp_sec;
  uint32_T timestamp_nsec;
  real32_T P_B_ISS_ISS[3];
  real32_T V_B_ISS_ISS[3];
  real32_T A_B_ISS_ISS[3];
  real32_T quat_ISS2B[4];
  real32_T omega_B_ISS_B[3];
  real32_T alpha_B_ISS_B[3];
} cmc_state_cmd;

#endif

#ifndef DEFINED_TYPEDEF_FOR_cmc_msg_
#define DEFINED_TYPEDEF_FOR_cmc_msg_

typedef struct {
  cmc_state_cmd cmc_state_cmd_a;
  cmc_state_cmd cmc_state_cmd_b;
  uint8_T cmc_mode_cmd;
  uint8_T speed_gain_cmd;
  uint8_T localization_mode_cmd;
  real32_T att_kp[3];
  real32_T att_ki[3];
  real32_T omega_kd[3];
  real32_T pos_kp[3];
  real32_T pos_ki[3];
  real32_T vel_kd[3];
  real32_T center_of_mass[3];
  real32_T inertia_matrix[9];
  real32_T mass;
} cmc_msg;

#endif

#ifndef DEFINED_TYPEDEF_FOR_env_msg_
#define DEFINED_TYPEDEF_FOR_env_msg_

typedef struct {
  real32_T P_B_ISS_ISS[3];
  real32_T V_B_ISS_ISS[3];
  real32_T A_B_ISS_ISS[3];
  real32_T A_B_ISS_B[3];
  real32_T A_B_ECI_B[3];
  real32_T Q_ISS2B[4];
  real32_T omega_B_ISS_B[3];
  real32_T omega_B_ECI_B[3];
  real32_T alpha_B_ISS_B[3];
  real32_T fan_torques_B[3];
  real32_T fan_forces_B[3];
} env_msg;

#endif

#ifndef DEFINED_TYPEDEF_FOR_ex_time_msg_
#define DEFINED_TYPEDEF_FOR_ex_time_msg_

typedef struct {
  uint32_T timestamp_sec;
  uint32_T timestamp_nsec;
} ex_time_msg;

#endif

#ifndef DEFINED_TYPEDEF_FOR_cvs_landmark_msg_
#define DEFINED_TYPEDEF_FOR_cvs_landmark_msg_

typedef struct {
  uint32_T cvs_timestamp_sec;
  uint32_T cvs_timestamp_nsec;
  real32_T cvs_landmarks[150];
  real32_T cvs_observations[100];
  uint8_T cvs_valid_flag[50];
} cvs_landmark_msg;

#endif

#ifndef DEFINED_TYPEDEF_FOR_cvs_optical_flow_msg_
#define DEFINED_TYPEDEF_FOR_cvs_optical_flow_msg_

typedef struct {
  uint32_T cvs_timestamp_sec;
  uint32_T cvs_timestamp_nsec;
  real32_T cvs_observations[1600];
  uint8_T cvs_valid_flag[800];
  real32_T cvs_id_tag[50];
} cvs_optical_flow_msg;

#endif

#ifndef DEFINED_TYPEDEF_FOR_cvs_handrail_msg_
#define DEFINED_TYPEDEF_FOR_cvs_handrail_msg_

typedef struct {
  uint32_T cvs_timestamp_sec;
  uint32_T cvs_timestamp_nsec;
  real32_T cvs_landmarks[150];
  real32_T cvs_observations[150];
  uint8_T cvs_valid_flag[50];
  uint8_T cvs_3d_knowledge_flag;
  real32_T cvs_handrail_local_pos[3];
  real32_T cvs_handrail_local_quat[4];
  uint8_T cvs_handrail_update_global_pose_flag;
} cvs_handrail_msg;

#endif

#ifndef DEFINED_TYPEDEF_FOR_cvs_registration_pulse_
#define DEFINED_TYPEDEF_FOR_cvs_registration_pulse_

typedef struct {
  uint8_T cvs_ar_tag_pulse;
  uint8_T cvs_landmark_pulse;
  uint8_T cvs_optical_flow_pulse;
  uint8_T cvs_handrail_pulse;
} cvs_registration_pulse;

#endif

#ifndef DEFINED_TYPEDEF_FOR_bpm_msg_
#define DEFINED_TYPEDEF_FOR_bpm_msg_

typedef struct {
  uint32_T bpm_timestamp_sec;
  uint32_T bpm_timestamp_nsec;
  real32_T bpm_motor_curr[2];
  real32_T bpm_servo_curr[12];
  real32_T bpm_torque_B[3];
  real32_T bpm_force_B[3];
  real32_T bpm_motor_speed[2];
  real32_T bpm_nozzle_theta[12];
} bpm_msg;

#endif

#ifndef DEFINED_TYPEDEF_FOR_imu_msg_
#define DEFINED_TYPEDEF_FOR_imu_msg_

typedef struct {
  uint32_T imu_timestamp_sec;
  uint32_T imu_timestamp_nsec;
  real32_T imu_A_B_ECI_sensor[3];
  real32_T imu_accel_bias[3];
  real32_T imu_omega_B_ECI_sensor[3];
  real32_T imu_gyro_bias[3];
  uint8_T imu_validity_flag;
  uint8_T imu_sat_flag;
} imu_msg;

#endif

// Parameters for system: '<S89>/CoreSubsys'
typedef struct P_CoreSubsys_sim_model_lib0_T_ P_CoreSubsys_sim_model_lib0_T;

// Parameters for system: '<S85>/pinhole_projection_model'
typedef struct P_pinhole_projection_model_si_T_ P_pinhole_projection_model_si_T;

// Parameters for system: '<S184>/blower_aerodynamics'
typedef struct P_blower_aerodynamics_sim_mod_T_ P_blower_aerodynamics_sim_mod_T;

// Parameters for system: '<S200>/CoreSubsys'
typedef struct P_CoreSubsys_sim_model_lib0_a_T_ P_CoreSubsys_sim_model_lib0_a_T;

// Parameters for system: '<S188>/latch_nozzle_thrust_matricies'
typedef struct P_latch_nozzle_thrust_matrici_T_ P_latch_nozzle_thrust_matrici_T;

// Parameters for system: '<S190>/speed_controller'
typedef struct P_speed_controller_sim_model__T_ P_speed_controller_sim_model__T;

// Parameters for system: '<S184>/servo_model'
typedef struct P_servo_model_sim_model_lib0_T_ P_servo_model_sim_model_lib0_T;

// Parameters for system: '<S185>/blower_aerodynamics'
typedef struct P_blower_aerodynamics_sim_m_m_T_ P_blower_aerodynamics_sim_m_m_T;

// Parameters (auto storage)
typedef struct P_sim_model_lib0_T_ P_sim_model_lib0_T;

// Forward declaration for rtModel
typedef struct tag_RTM_sim_model_lib0_T RT_MODEL_sim_model_lib0_T;

#endif                                 // RTW_HEADER_sim_model_lib0_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
