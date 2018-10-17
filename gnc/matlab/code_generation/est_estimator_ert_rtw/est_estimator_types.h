//
// File: est_estimator_types.h
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed Aug 22 07:24:56 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_est_estimator_types_h_
#define RTW_HEADER_est_estimator_types_h_
#include "rtwtypes.h"
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

#ifndef DEFINED_TYPEDEF_FOR_cvs_registration_pulse_
#define DEFINED_TYPEDEF_FOR_cvs_registration_pulse_

typedef struct {
  uint8_T cvs_ar_tag_pulse;
  uint8_T cvs_landmark_pulse;
  uint8_T cvs_optical_flow_pulse;
  uint8_T cvs_handrail_pulse;
} cvs_registration_pulse;

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

#ifndef DEFINED_TYPEDEF_FOR_ase_cov_datatype_
#define DEFINED_TYPEDEF_FOR_ase_cov_datatype_

typedef real32_T ase_cov_datatype;
typedef creal32_T case_cov_datatype;

#endif

#ifndef DEFINED_TYPEDEF_FOR_kfl_msg_
#define DEFINED_TYPEDEF_FOR_kfl_msg_

typedef struct {
  real32_T quat_ISS2B[4];
  real32_T omega_B_ISS_B[3];
  real32_T gyro_bias[3];
  real32_T V_B_ISS_ISS[3];
  real32_T A_B_ISS_ISS[3];
  real32_T accel_bias[3];
  real32_T P_B_ISS_ISS[3];
  uint8_T confidence;
  uint32_T aug_state_enum;
  real32_T ml_quat_ISS2cam[4];
  real32_T ml_P_cam_ISS_ISS[3];
  real32_T of_quat_ISS2cam[64];
  real32_T of_P_cam_ISS_ISS[48];
  ase_cov_datatype cov_diag[117];
  uint16_T kfl_status;
  uint8_T update_OF_tracks_cnt;
  uint8_T update_ML_features_cnt;
  real_T of_mahal_distance[50];
  real_T ml_mahal_distance[50];
  real32_T hr_P_hr_ISS_ISS[3];
  real32_T hr_quat_ISS2hr[4];
  real32_T P_EST_ISS_ISS[3];
} kfl_msg;

#endif

// Parameters for system: '<S45>/CoreSubsys'
typedef struct P_CoreSubsys_est_estimator_T_ P_CoreSubsys_est_estimator_T;

// Parameters for system: '<S46>/CoreSubsys'
typedef struct P_CoreSubsys_est_estimator_g_T_ P_CoreSubsys_est_estimator_g_T;

// Parameters for system: '<S116>/Normalize'
typedef struct P_Normalize_est_estimator_T_ P_Normalize_est_estimator_T;

// Parameters (auto storage)
typedef struct P_est_estimator_T_ P_est_estimator_T;

// Forward declaration for rtModel
typedef struct tag_RTM_est_estimator_T RT_MODEL_est_estimator_T;

#endif                                 // RTW_HEADER_est_estimator_types_h_

//
// File trailer for generated code.
//
// [EOF]
//
