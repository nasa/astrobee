//
// File: est_estimator.h
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Sep 23 17:43:06 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_est_estimator_h_
#define RTW_HEADER_est_estimator_h_
#include <math.h>
#include <string.h>
#include <stddef.h>
#ifndef est_estimator_COMMON_INCLUDES_
# define est_estimator_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#include "compute_delta_state_and_cov.h"
#include "apply_delta_state.h"
#include "of_residual_and_h.h"
#include "matrix_multiply.h"
#endif                                 // est_estimator_COMMON_INCLUDES_

#include "est_estimator_types.h"
#include "rtGetNaN.h"
#include "rt_nonfinite.h"
#include "aaiepphlecjelnoh_permute.h"
#include "aimgbiekknohhdjm_norm.h"
#include "baaanophjmgddbie_sum.h"
#include "baieimopcbaiaaai_eulers_to_quat.h"
#include "biecdbieglngohlf_pinv.h"
#include "biekcjmgdbaadbim_eye.h"
#include "div_nzp_s32_floor.h"
#include "djmgjecbcbiengln_power.h"
#include "ecjedbaiaiekohln_quaternion_to_rotation.h"
#include "fkfcbaiengdjgdje_quaternion_to_rotation.h"
#include "fkngdjekgdjepphl_sum.h"
#include "gdjmmglnmgdjlfkf_quat_rotation_vec.h"
#include "glfcngdjgdjmmglf_pinv.h"
#include "hdbaohdbkngdbimo_PadeApproximantOfDegree.h"
#include "hdbihlngknopkfcj_repmat.h"
#include "iecjbieccbailfcb_abs.h"
#include "iecjopppiecjmgln_quatmult.h"
#include "iekfiecjknopophd_pinv.h"
#include "imohcjmoimopimoh_nullAssignment.h"
#include "imohknglphlfpphd_repmat.h"
#include "jekfopppngdbhlng_diag.h"
#include "jmglopphppphkfkf_qr.h"
#include "jmohiecblfcjnohl_qr.h"
#include "kngldbimhdbaimgd_quat_propagate_step.h"
#include "mgdbbiekfknonglf_nullAssignment.h"
#include "mglfbimobiechdbi_bitget.h"
#include "mglnkfkfmglfjekn_PadeApproximantOfDegree.h"
#include "moppbaaafkfkimgd_diag.h"
#include "ngdjjecbgdbaglfc_eye.h"
#include "nohdcbaibiecnohl_power.h"
#include "nohlcjekmohddjmg_abs.h"
#include "ophlcjmgkfcbmohl_nullAssignment.h"
#include "rt_powf_snf.h"
#include "rt_roundd_snf.h"
#include "rtGetInf.h"

// Macros for accessing real-time model data structure
#ifndef rtmGetErrorStatus
# define rtmGetErrorStatus(rtm)        ((rtm)->errorStatus)
#endif

#ifndef rtmSetErrorStatus
# define rtmSetErrorStatus(rtm, val)   ((rtm)->errorStatus = (val))
#endif

// Exported data define

// Definition for custom storage class: Define
#define ASE_ML_NUM_FEATURES            50
#define ASE_OF_NUM_AUG                 16
#define ASE_OF_NUM_FEATURES            50

// Block signals for system '<S46>/MATLAB Function1'
typedef struct {
  real32_T num_out[3];                 // '<S46>/MATLAB Function1'
  real32_T den_out[3];                 // '<S46>/MATLAB Function1'
} B_MATLABFunction1_est_estimat_T;

// Block states (auto storage) for system '<S46>/MATLAB Function1'
typedef struct {
  real32_T y;                          // '<S46>/MATLAB Function1'
  real32_T num[3];                     // '<S46>/MATLAB Function1'
  real32_T den[3];                     // '<S46>/MATLAB Function1'
  real32_T prev_impeller_speed;        // '<S46>/MATLAB Function1'
} DW_MATLABFunction1_est_estima_T;

// Block signals for system '<S46>/CoreSubsys'
typedef struct {
  B_MATLABFunction1_est_estimat_T sf_MATLABFunction1;// '<S46>/MATLAB Function1' 
} B_CoreSubsys_est_estimator_T;

// Block states (auto storage) for system '<S46>/CoreSubsys'
typedef struct {
  real32_T uHzLowPass_states;          // '<S46>/3 Hz Low Pass'
  real32_T DiscreteTransferFcn_states[2];// '<S46>/Discrete Transfer Fcn'
  DW_MATLABFunction1_est_estima_T sf_MATLABFunction1;// '<S46>/MATLAB Function1' 
} DW_CoreSubsys_est_estimator_T;

// Block signals for system '<S47>/CoreSubsys'
typedef struct {
  B_MATLABFunction1_est_estimat_T sf_MATLABFunction1;// '<S47>/MATLAB Function1' 
} B_CoreSubsys_est_estimator_k_T;

// Block states (auto storage) for system '<S47>/CoreSubsys'
typedef struct {
  real32_T uHzLowPass_states;          // '<S47>/3 Hz Low Pass'
  real32_T DiscreteTransferFcn_states[2];// '<S47>/Discrete Transfer Fcn'
  real32_T HighPassFilter_states;      // '<S47>/High Pass Filter'
  DW_MATLABFunction1_est_estima_T sf_MATLABFunction1;// '<S47>/MATLAB Function1' 
} DW_CoreSubsys_est_estimator_a_T;

// Block signals for system '<S113>/MATLAB Function'
typedef struct {
  real32_T y[16];                      // '<S113>/MATLAB Function'
} B_MATLABFunction_est_estimato_T;

// Block signals (auto storage)
typedef struct {
  real_T tmp_data[22500];
  real32_T q1_data[22500];
  real_T tmp_data_m[10000];
  real32_T ex_compute_delta_state_an_e[13689];// '<S24>/ex_compute_delta_state_and_cov' 
  real32_T Assignment[13689];          // '<S2>/Assignment'
  real32_T Switch1[13689];
  real32_T MatrixConcatenate2[11934];  // '<S95>/Matrix Concatenate2'
  real32_T ex_of_residual_and_h_o3[11232];// '<S24>/ex_of_residual_and_h'
  real32_T q1_data_c[10000];
  real32_T ex_of_residual_and_h_o7[9216];// '<S24>/ex_of_residual_and_h'
  ase_cov_datatype Switch1_k[8100];
  real32_T x_data[2500];
  ase_cov_datatype Switch1_c[1890];
  real32_T MatrixConcatenate[1755];    // '<S95>/Matrix Concatenate'
  real32_T of_measured_in[1600];
  real32_T of_measured[1600];          // '<S24>/compute_of_global_points'
  real32_T b_x_data[1600];
  real32_T of_measured_in_data[1600];
  ase_cov_datatype Selector1[1530];    // '<S95>/Selector1'
  ase_cov_datatype P_IC[1530];         // '<S95>/MATLAB Function2'
  real32_T r_out[6];                   // '<S13>/Merge'
  real32_T H_out[702];                 // '<S13>/Merge'
  real32_T R_mat[36];                  // '<S13>/Merge'
  int32_T error_out;                   // '<S13>/Merge'
  uint8_T SumofElements3[50];          // '<S40>/Sum of Elements3'
  B_MATLABFunction_est_estimato_T sf_MATLABFunction_l0;// '<S164>/MATLAB Function' 
  B_MATLABFunction_est_estimato_T sf_MATLABFunction_o;// '<S144>/MATLAB Function' 
  B_MATLABFunction_est_estimato_T sf_MATLABFunction;// '<S113>/MATLAB Function'
  B_CoreSubsys_est_estimator_k_T CoreSubsys_l[3];// '<S47>/CoreSubsys'
  B_CoreSubsys_est_estimator_T CoreSubsys[3];// '<S46>/CoreSubsys'
} B_est_estimator_T;

// Block states (auto storage) for system '<Root>'
typedef struct {
  real_T UnitDelay20_DSTATE[3];        // '<S2>/Unit Delay20'
  real_T UnitDelay21_DSTATE[3];        // '<S2>/Unit Delay21'
  real_T UnitDelay22_DSTATE[48];       // '<S2>/Unit Delay22'
  real_T UnitDelay23_DSTATE[48];       // '<S2>/Unit Delay23'
  real_T UnitDelay18_DSTATE[50];       // '<S2>/Unit Delay18'
  real_T UnitDelay19_DSTATE[50];       // '<S2>/Unit Delay19'
  real_T UnitDelay_DSTATE_j;           // '<S5>/Unit Delay'
  real_T num_consecutive_rejected_meas;// '<S17>/Compute Residual and H'
  ase_cov_datatype Delay_DSTATE[13689];// '<S2>/Delay'
  real32_T UnitDelay1_DSTATE[4];       // '<S2>/Unit Delay1'
  real32_T UnitDelay3_DSTATE[3];       // '<S2>/Unit Delay3'
  real32_T UnitDelay4_DSTATE[3];       // '<S2>/Unit Delay4'
  real32_T UnitDelay6_DSTATE[3];       // '<S2>/Unit Delay6'
  real32_T UnitDelay7_DSTATE[3];       // '<S2>/Unit Delay7'
  real32_T UnitDelay10_DSTATE[4];      // '<S2>/Unit Delay10'
  real32_T UnitDelay11_DSTATE[3];      // '<S2>/Unit Delay11'
  real32_T UnitDelay12_DSTATE[64];     // '<S2>/Unit Delay12'
  real32_T UnitDelay13_DSTATE[48];     // '<S2>/Unit Delay13'
  ase_cov_datatype UnitDelay14_DSTATE[117];// '<S2>/Unit Delay14'
  real32_T UnitDelay24_DSTATE[3];      // '<S2>/Unit Delay24'
  real32_T UnitDelay25_DSTATE[4];      // '<S2>/Unit Delay25'
  real32_T UnitDelay26_DSTATE[3];      // '<S2>/Unit Delay26'
  uint32_T UnitDelay9_DSTATE;          // '<S2>/Unit Delay9'
  uint32_T DelayInput1_DSTATE;         // '<S75>/Delay Input1'
  uint32_T DelayInput1_DSTATE_h;       // '<S76>/Delay Input1'
  uint32_T DelayInput1_DSTATE_o;       // '<S77>/Delay Input1'
  uint32_T DelayInput1_DSTATE_n;       // '<S78>/Delay Input1'
  uint32_T DelayInput1_DSTATE_b;       // '<S79>/Delay Input1'
  uint32_T DelayInput1_DSTATE_d;       // '<S80>/Delay Input1'
  real32_T aug_velocity[48];           // '<S129>/MATLAB Function'
  real32_T aug_velocity_mag[16];       // '<S129>/MATLAB Function'
  real32_T aug_omega[48];              // '<S129>/MATLAB Function'
  real32_T aug_omega_mag[16];          // '<S129>/MATLAB Function'
  real32_T hr_P_hr_ISS_ISS_pers[3];    // '<S16>/Compute Global positions of Handrail Features' 
  real32_T hr_quat_ISS2hr_pers[4];     // '<S16>/Compute Global positions of Handrail Features' 
  uint16_T UnitDelay15_DSTATE;         // '<S2>/Unit Delay15'
  uint8_T UnitDelay8_DSTATE;           // '<S2>/Unit Delay8'
  uint8_T UnitDelay16_DSTATE;          // '<S2>/Unit Delay16'
  uint8_T UnitDelay17_DSTATE;          // '<S2>/Unit Delay17'
  uint8_T DelayInput1_DSTATE_k;        // '<S69>/Delay Input1'
  uint8_T icLoad;                      // '<S2>/Delay'
  boolean_T aug_velocity_not_empty;    // '<S129>/MATLAB Function'
  boolean_T aug_velocity_mag_not_empty;// '<S129>/MATLAB Function'
  boolean_T aug_omega_not_empty;       // '<S129>/MATLAB Function'
  boolean_T aug_omega_mag_not_empty;   // '<S129>/MATLAB Function'
  boolean_T hr_P_hr_ISS_ISS_pers_not_empty;// '<S16>/Compute Global positions of Handrail Features' 
  boolean_T hr_quat_ISS2hr_pers_not_empty;// '<S16>/Compute Global positions of Handrail Features' 
  DW_CoreSubsys_est_estimator_a_T CoreSubsys_l[3];// '<S47>/CoreSubsys'
  DW_CoreSubsys_est_estimator_T CoreSubsys[3];// '<S46>/CoreSubsys'
} DW_est_estimator_T;

// Parameters for system: '<S46>/CoreSubsys'
struct P_CoreSubsys_est_estimator_T_ {
  real32_T uHzLowPass_NumCoef;         // Expression: single([0.2604])
                                       //  Referenced by: '<S46>/3 Hz Low Pass'

  real32_T uHzLowPass_DenCoef[2];      // Expression: single([1 -0.7396])
                                       //  Referenced by: '<S46>/3 Hz Low Pass'

  real32_T uHzLowPass_InitialStates;   // Computed Parameter: uHzLowPass_InitialStates
                                       //  Referenced by: '<S46>/3 Hz Low Pass'

  real32_T DiscreteTransferFcn_InitialStat;// Computed Parameter: DiscreteTransferFcn_InitialStat
                                           //  Referenced by: '<S46>/Discrete Transfer Fcn'

};

// Parameters for system: '<S47>/CoreSubsys'
struct P_CoreSubsys_est_estimator_g_T_ {
  real32_T uHzLowPass_NumCoef;         // Expression: single([0.2604])
                                       //  Referenced by: '<S47>/3 Hz Low Pass'

  real32_T uHzLowPass_DenCoef[2];      // Expression: single([1 -0.7396])
                                       //  Referenced by: '<S47>/3 Hz Low Pass'

  real32_T uHzLowPass_InitialStates;   // Computed Parameter: uHzLowPass_InitialStates
                                       //  Referenced by: '<S47>/3 Hz Low Pass'

  real32_T DiscreteTransferFcn_InitialStat;// Computed Parameter: DiscreteTransferFcn_InitialStat
                                           //  Referenced by: '<S47>/Discrete Transfer Fcn'

  real32_T HighPassFilter_InitialStates;// Computed Parameter: HighPassFilter_InitialStates
                                        //  Referenced by: '<S47>/High Pass Filter'

};

// Parameters for system: '<S117>/Normalize'
struct P_Normalize_est_estimator_T_ {
  real_T Constant1_Value;              // Expression: -1
                                       //  Referenced by: '<S119>/Constant1'

};

// Parameters (auto storage)
struct P_est_estimator_T_ {
  real_T ase_minumum_resid_thresh;     // Variable: ase_minumum_resid_thresh
                                       //  Referenced by:
                                       //    '<S24>/Constant1'
                                       //    '<S15>/Constant1'

  real_T ase_ts;                       // Variable: ase_ts
                                       //  Referenced by:
                                       //    '<S6>/Constant'
                                       //    '<S6>/Gain'
                                       //    '<S6>/Gain1'
                                       //    '<S95>/MATLAB Function'
                                       //    '<S95>/Gain3'

  real_T astrobee_fsw_step_size;       // Variable: astrobee_fsw_step_size
                                       //  Referenced by:
                                       //    '<S46>/Constant2'
                                       //    '<S47>/Constant2'

  real32_T ase_accel_fixed_bias[3];    // Variable: ase_accel_fixed_bias
                                       //  Referenced by: '<S41>/Constant5'

  real32_T ase_gyro_fixed_bias[3];     // Variable: ase_gyro_fixed_bias
                                       //  Referenced by: '<S41>/Constant1'

  real32_T ase_state_ic_A_B_ISS_ISS[3];// Variable: ase_state_ic_A_B_ISS_ISS
                                       //  Referenced by: '<S2>/Unit Delay5'

  real32_T ase_state_ic_accel_bias[3]; // Variable: ase_state_ic_accel_bias
                                       //  Referenced by: '<S2>/Unit Delay6'

  real32_T ase_state_ic_cov_diag[117]; // Variable: ase_state_ic_cov_diag
                                       //  Referenced by: '<S2>/Unit Delay14'

  real32_T ase_state_ic_gyro_bias[3];  // Variable: ase_state_ic_gyro_bias
                                       //  Referenced by: '<S2>/Unit Delay3'

  real32_T ase_state_ic_ml_P_cam_ISS_ISS[3];// Variable: ase_state_ic_ml_P_cam_ISS_ISS
                                            //  Referenced by: '<S2>/Unit Delay11'

  real32_T ase_state_ic_ml_quat_ISS2cam[4];// Variable: ase_state_ic_ml_quat_ISS2cam
                                           //  Referenced by: '<S2>/Unit Delay10'

  real32_T ase_state_ic_of_P_cam_ISS_ISS[48];// Variable: ase_state_ic_of_P_cam_ISS_ISS
                                             //  Referenced by: '<S2>/Unit Delay13'

  real32_T ase_state_ic_of_quat_ISS2cam[64];// Variable: ase_state_ic_of_quat_ISS2cam
                                            //  Referenced by: '<S2>/Unit Delay12'

  real32_T fam_impeller_speeds[3];     // Variable: fam_impeller_speeds
                                       //  Referenced by:
                                       //    '<S46>/Constant'
                                       //    '<S47>/Constant'

  real32_T tun_abp_p_dockcam_imu_est[3];// Variable: tun_abp_p_dockcam_imu_est
                                        //  Referenced by: '<S161>/Constant2'

  real32_T tun_abp_p_imu_body_body[3]; // Variable: tun_abp_p_imu_body_body
                                       //  Referenced by: '<S5>/Constant5'

  real32_T tun_abp_p_navcam_imu_est[3];// Variable: tun_abp_p_navcam_imu_est
                                       //  Referenced by:
                                       //    '<S129>/MATLAB Function'
                                       //    '<S129>/Constant1'
                                       //    '<S161>/Constant1'

  real32_T tun_abp_p_perchcam_imu_est[3];// Variable: tun_abp_p_perchcam_imu_est
                                         //  Referenced by: '<S161>/Constant4'

  real32_T tun_abp_q_body2dockcam[4];  // Variable: tun_abp_q_body2dockcam
                                       //  Referenced by:
                                       //    '<S161>/Constant7'
                                       //    '<S17>/Constant1'

  real32_T tun_abp_q_body2navcam[4];   // Variable: tun_abp_q_body2navcam
                                       //  Referenced by:
                                       //    '<S129>/MATLAB Function'
                                       //    '<S129>/Constant2'
                                       //    '<S161>/Constant6'
                                       //    '<S17>/Constant'

  real32_T tun_abp_q_body2perchcam[4]; // Variable: tun_abp_q_body2perchcam
                                       //  Referenced by:
                                       //    '<S161>/Constant8'
                                       //    '<S16>/Compute Residual and H'

  real32_T tun_abp_quat_body2imu[4];   // Variable: tun_abp_quat_body2imu
                                       //  Referenced by:
                                       //    '<S41>/Constant'
                                       //    '<S41>/Constant2'

  real32_T tun_ase_Q_imu[12];          // Variable: tun_ase_Q_imu
                                       //  Referenced by: '<S95>/Constant9'

  real32_T tun_ase_acquired_ticks;     // Variable: tun_ase_acquired_ticks
                                       //  Referenced by:
                                       //    '<S5>/Saturation'
                                       //    '<S86>/Constant'

  real32_T tun_ase_converged_thresh;   // Variable: tun_ase_converged_thresh
                                       //  Referenced by: '<S83>/Constant'

  real32_T tun_ase_diverged_thresh;    // Variable: tun_ase_diverged_thresh
                                       //  Referenced by: '<S84>/Constant'

  real32_T tun_ase_dock_r_mag;         // Variable: tun_ase_dock_r_mag
                                       //  Referenced by: '<S17>/Compute Residual and H'

  real32_T tun_ase_dockcam_distortion; // Variable: tun_ase_dockcam_distortion
                                       //  Referenced by: '<S17>/Constant5'

  real32_T tun_ase_dockcam_inv_focal_length;// Variable: tun_ase_dockcam_inv_focal_length
                                            //  Referenced by: '<S17>/Constant3'

  real32_T tun_ase_gravity_accel[3];   // Variable: tun_ase_gravity_accel
                                       //  Referenced by: '<S41>/Constant3'

  real32_T tun_ase_hr_distance_r;      // Variable: tun_ase_hr_distance_r
                                       //  Referenced by: '<S16>/Compute Residual and H'

  real32_T tun_ase_hr_r_mag;           // Variable: tun_ase_hr_r_mag
                                       //  Referenced by: '<S16>/Compute Residual and H'

  real32_T tun_ase_mahal_distance_max; // Variable: tun_ase_mahal_distance_max
                                       //  Referenced by:
                                       //    '<S24>/ex_of_residual_and_h'
                                       //    '<S17>/Compute Residual and H'

  real32_T tun_ase_map_error;          // Variable: tun_ase_map_error
                                       //  Referenced by: '<S17>/Compute Residual and H'

  real32_T tun_ase_max_accel;          // Variable: tun_ase_max_accel
                                       //  Referenced by: '<S95>/diag'

  real32_T tun_ase_max_gyro;           // Variable: tun_ase_max_gyro
                                       //  Referenced by: '<S95>/diag'

  real32_T tun_ase_min_ar_meas;        // Variable: tun_ase_min_ar_meas
                                       //  Referenced by: '<S17>/Compute Residual and H'

  real32_T tun_ase_min_ml_meas;        // Variable: tun_ase_min_ml_meas
                                       //  Referenced by: '<S17>/Compute Residual and H'

  real32_T tun_ase_ml_forward_projection_time;// Variable: tun_ase_ml_forward_projection_time
                                              //  Referenced by:
                                              //    '<S129>/Constant'
                                              //    '<S129>/Gain'
                                              //    '<S161>/Constant'
                                              //    '<S161>/Gain'
                                              //    '<S16>/Compute Residual and H'
                                              //    '<S17>/Compute Residual and H'

  real32_T tun_ase_navcam_distortion;  // Variable: tun_ase_navcam_distortion
                                       //  Referenced by:
                                       //    '<S24>/ex_of_residual_and_h'
                                       //    '<S17>/Constant4'

  real32_T tun_ase_navcam_inv_focal_length;// Variable: tun_ase_navcam_inv_focal_length
                                           //  Referenced by:
                                           //    '<S24>/ex_of_residual_and_h'
                                           //    '<S17>/Constant2'

  real32_T tun_ase_of_r_mag;           // Variable: tun_ase_of_r_mag
                                       //  Referenced by: '<S24>/ex_of_residual_and_h'

  real32_T tun_ase_q_saturated_accel;  // Variable: tun_ase_q_saturated_accel
                                       //  Referenced by: '<S95>/diag'

  real32_T tun_ase_q_saturated_gyro;   // Variable: tun_ase_q_saturated_gyro
                                       //  Referenced by: '<S95>/diag'

  real32_T tun_ase_state_ic_P_B_ISS_ISS[3];// Variable: tun_ase_state_ic_P_B_ISS_ISS
                                           //  Referenced by: '<S2>/Unit Delay7'

  real32_T tun_ase_state_ic_P_EST_ISS_ISS[3];// Variable: tun_ase_state_ic_P_EST_ISS_ISS
                                             //  Referenced by: '<S2>/Unit Delay26'

  real32_T tun_ase_state_ic_V_B_ISS_ISS[3];// Variable: tun_ase_state_ic_V_B_ISS_ISS
                                           //  Referenced by: '<S2>/Unit Delay4'

  real32_T tun_ase_state_ic_cov_diag[15];// Variable: tun_ase_state_ic_cov_diag
                                         //  Referenced by: '<S2>/Constant4'

  real32_T tun_ase_state_ic_omega_B_ISS_B[3];// Variable: tun_ase_state_ic_omega_B_ISS_B
                                             //  Referenced by: '<S2>/Unit Delay2'

  real32_T tun_ase_state_ic_quat_ISS2B[4];// Variable: tun_ase_state_ic_quat_ISS2B
                                          //  Referenced by: '<S2>/Unit Delay1'

  real32_T tun_ase_vis_r_mag;          // Variable: tun_ase_vis_r_mag
                                       //  Referenced by: '<S17>/Compute Residual and H'

  real32_T tun_ase_vocam_inv_focal_length;// Variable: tun_ase_vocam_inv_focal_length
                                          //  Referenced by: '<S24>/compute_of_global_points'

  real32_T tun_aug_ic_cov;             // Variable: tun_aug_ic_cov
                                       //  Referenced by: '<S2>/Gain'

  real32_T tun_grav_hp_den[2];         // Variable: tun_grav_hp_den
                                       //  Referenced by: '<S47>/High Pass Filter'

  real32_T tun_grav_hp_num[2];         // Variable: tun_grav_hp_num
                                       //  Referenced by: '<S47>/High Pass Filter'

  real32_T tun_ic_cov_pos;             // Variable: tun_ic_cov_pos
                                       //  Referenced by: '<S42>/Constant3'

  real32_T tun_ic_cov_quat;            // Variable: tun_ic_cov_quat
                                       //  Referenced by: '<S42>/Constant1'

  real32_T tun_max_mahal_reject_frames;// Variable: tun_max_mahal_reject_frames
                                       //  Referenced by: '<S17>/Compute Residual and H'

  uint32_T ase_aug_state_bitmask;      // Variable: ase_aug_state_bitmask
                                       //  Referenced by: '<S31>/Constant'

  uint8_T ase_local_mode_docking;      // Variable: ase_local_mode_docking
                                       //  Referenced by:
                                       //    '<S37>/Constant'
                                       //    '<S156>/Constant'

  uint8_T ase_local_mode_map;          // Variable: ase_local_mode_map
                                       //  Referenced by:
                                       //    '<S36>/Constant'
                                       //    '<S158>/Constant'
                                       //    '<S20>/Constant'

  uint8_T ase_local_mode_perching;     // Variable: ase_local_mode_perching
                                       //  Referenced by:
                                       //    '<S34>/Constant'
                                       //    '<S157>/Constant'

  uint8_T ase_state_ic_aug_state_enum; // Variable: ase_state_ic_aug_state_enum
                                       //  Referenced by: '<S2>/Unit Delay9'

  uint8_T ase_state_ic_confidence;     // Variable: ase_state_ic_confidence
                                       //  Referenced by: '<S2>/Unit Delay8'

  uint8_T ase_state_ic_status;         // Variable: ase_state_ic_status
                                       //  Referenced by: '<S2>/Unit Delay15'

  uint8_T ase_status_acquiring;        // Variable: ase_status_acquiring
                                       //  Referenced by:
                                       //    '<S5>/Constant4'
                                       //    '<S87>/Constant'
                                       //    '<S88>/Constant'

  uint8_T ase_status_converged;        // Variable: ase_status_converged
                                       //  Referenced by:
                                       //    '<S5>/Constant1'
                                       //    '<S82>/Constant'
                                       //    '<S85>/Constant'
                                       //    '<S21>/Constant'

  uint8_T ase_status_diverged;         // Variable: ase_status_diverged
                                       //  Referenced by:
                                       //    '<S5>/Constant'
                                       //    '<S81>/Constant'

  uint8_T fam_impeller_speeds_cnt;     // Variable: fam_impeller_speeds_cnt
                                       //  Referenced by: '<S41>/Saturation'

  uint8_T tun_ase_enable_of;           // Variable: tun_ase_enable_of
                                       //  Referenced by:
                                       //    '<S11>/Constant'
                                       //    '<S125>/Constant'

  uint8_T tun_ase_gravity_removal;     // Variable: tun_ase_gravity_removal
                                       //  Referenced by: '<S41>/Constant4'

  uint8_T tun_grav_hp_enable_f;        // Variable: tun_grav_hp_enable_f
                                       //  Referenced by: '<S47>/Constant1'

  uint32_T FixPtBitwiseOperator3_BitMask;// Mask Parameter: FixPtBitwiseOperator3_BitMask
                                         //  Referenced by: '<S68>/FixPt Bitwise Operator3'

  uint32_T BitwiseOperator2_BitMask;   // Mask Parameter: BitwiseOperator2_BitMask
                                       //  Referenced by: '<S161>/Bitwise Operator2'

  uint32_T BitwiseOperator_BitMask;    // Mask Parameter: BitwiseOperator_BitMask
                                       //  Referenced by: '<S11>/Bitwise Operator'

  uint32_T DetectChange6_vinit;        // Mask Parameter: DetectChange6_vinit
                                       //  Referenced by: '<S75>/Delay Input1'

  uint32_T DetectChange7_vinit;        // Mask Parameter: DetectChange7_vinit
                                       //  Referenced by: '<S76>/Delay Input1'

  uint32_T DetectChange_vinit;         // Mask Parameter: DetectChange_vinit
                                       //  Referenced by: '<S77>/Delay Input1'

  uint32_T DetectChange1_vinit;        // Mask Parameter: DetectChange1_vinit
                                       //  Referenced by: '<S78>/Delay Input1'

  uint32_T DetectChange4_vinit;        // Mask Parameter: DetectChange4_vinit
                                       //  Referenced by: '<S79>/Delay Input1'

  uint32_T DetectChange5_vinit;        // Mask Parameter: DetectChange5_vinit
                                       //  Referenced by: '<S80>/Delay Input1'

  uint16_T BitwiseOperator_BitMask_c;  // Mask Parameter: BitwiseOperator_BitMask_c
                                       //  Referenced by: '<S9>/Bitwise Operator'

  uint16_T BitwiseOperator1_BitMask;   // Mask Parameter: BitwiseOperator1_BitMask
                                       //  Referenced by: '<S126>/Bitwise Operator1'

  uint16_T BitwiseOperator_BitMask_l;  // Mask Parameter: BitwiseOperator_BitMask_l
                                       //  Referenced by: '<S5>/Bitwise Operator'

  uint8_T CompareToConstant7_const;    // Mask Parameter: CompareToConstant7_const
                                       //  Referenced by: '<S38>/Constant'

  uint8_T CompareToConstant_const;     // Mask Parameter: CompareToConstant_const
                                       //  Referenced by: '<S127>/Constant'

  uint8_T DetectChange_vinit_i;        // Mask Parameter: DetectChange_vinit_i
                                       //  Referenced by: '<S69>/Delay Input1'

  kfl_msg State_out_Y0;                // Computed Parameter: State_out_Y0
                                       //  Referenced by: '<S12>/State_out'

  kfl_msg UnitDelay_InitialCondition;  // Computed Parameter: UnitDelay_InitialCondition
                                       //  Referenced by: '<S12>/Unit Delay'

  kfl_msg State_out_Y0_h;              // Computed Parameter: State_out_Y0_h
                                       //  Referenced by: '<S24>/State_out'

  kfl_msg UnitDelay_InitialCondition_p;// Computed Parameter: UnitDelay_InitialCondition_p
                                       //  Referenced by: '<S24>/Unit Delay'

  real_T Constant_Value[50];           // Expression: zeros(ase_ml_num_features, 1)
                                       //  Referenced by: '<S16>/Constant'

  real_T Merge_6_InitialOutput;        // Computed Parameter: Merge_6_InitialOutput
                                       //  Referenced by: '<S13>/Merge'

  real_T Constant3_Value[50];          // Expression: zeros(ase_of_num_features,1)
                                       //  Referenced by: '<S14>/Constant3'

  real_T Constant2_Value[50];          // Expression: zeros(ase_ml_num_features,1)
                                       //  Referenced by: '<S24>/Constant2'

  real_T Constant2_Value_j[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S138>/Constant2'

  real_T Constant_Value_p[3];          // Expression: [0 0 0]
                                       //  Referenced by: '<S133>/Constant'

  real_T Constant1_Value;              // Expression: 1
                                       //  Referenced by: '<S138>/Constant1'

  real_T Constant3_Value_l;            // Expression: 0
                                       //  Referenced by: '<S141>/Constant3'

  real_T Constant2_Value_l[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S134>/Constant2'

  real_T Constant3_Value_p;            // Expression: 0
                                       //  Referenced by: '<S136>/Constant3'

  real_T Constant2_Value_m[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S187>/Constant2'

  real_T Constant1_Value_m;            // Expression: 1
                                       //  Referenced by: '<S187>/Constant1'

  real_T Constant3_Value_m;            // Expression: 0
                                       //  Referenced by: '<S190>/Constant3'

  real_T Constant2_Value_b[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S165>/Constant2'

  real_T Constant1_Value_n;            // Expression: 1
                                       //  Referenced by: '<S165>/Constant1'

  real_T Constant3_Value_d;            // Expression: 0
                                       //  Referenced by: '<S185>/Constant3'

  real_T Constant3_Value_pv;           // Expression: 0
                                       //  Referenced by: '<S167>/Constant3'

  real_T Constant2_Value_jy[9];        // Expression: zeros(9,1)
                                       //  Referenced by: '<S168>/Constant2'

  real_T Constant3_Value_pi;           // Expression: 0
                                       //  Referenced by: '<S170>/Constant3'

  real_T UnitDelay20_InitialCondition; // Expression: 0
                                       //  Referenced by: '<S2>/Unit Delay20'

  real_T UnitDelay21_InitialCondition; // Expression: 0
                                       //  Referenced by: '<S2>/Unit Delay21'

  real_T UnitDelay22_InitialCondition; // Expression: 0
                                       //  Referenced by: '<S2>/Unit Delay22'

  real_T UnitDelay23_InitialCondition; // Expression: 0
                                       //  Referenced by: '<S2>/Unit Delay23'

  real_T UnitDelay18_InitialCondition; // Expression: 0
                                       //  Referenced by: '<S2>/Unit Delay18'

  real_T UnitDelay19_InitialCondition; // Expression: 0
                                       //  Referenced by: '<S2>/Unit Delay19'

  real_T Constant_Value_l[3];          // Expression: [0 0 0]
                                       //  Referenced by: '<S98>/Constant'

  real_T Constant2_Value_a[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S58>/Constant2'

  real_T Constant1_Value_mb;           // Expression: 1
                                       //  Referenced by: '<S58>/Constant1'

  real_T Constant3_Value_k;            // Expression: 0
                                       //  Referenced by: '<S61>/Constant3'

  real_T Constant2_Value_ja[9];        // Expression: zeros(9,1)
                                       //  Referenced by: '<S107>/Constant2'

  real_T Constant1_Value_c;            // Expression: 1
                                       //  Referenced by: '<S107>/Constant1'

  real_T Constant3_Value_c;            // Expression: 0
                                       //  Referenced by: '<S110>/Constant3'

  real_T Constant2_Value_bl[9];        // Expression: zeros(9,1)
                                       //  Referenced by: '<S63>/Constant2'

  real_T Constant1_Value_l;            // Expression: 1
                                       //  Referenced by: '<S63>/Constant1'

  real_T Constant3_Value_o;            // Expression: 0
                                       //  Referenced by: '<S66>/Constant3'

  real_T Constant2_Value_p[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S53>/Constant2'

  real_T Constant1_Value_k;            // Expression: 1
                                       //  Referenced by: '<S53>/Constant1'

  real_T Constant3_Value_i;            // Expression: 0
                                       //  Referenced by: '<S56>/Constant3'

  real_T Constant2_Value_bd[9];        // Expression: zeros(9,1)
                                       //  Referenced by: '<S102>/Constant2'

  real_T Constant1_Value_e;            // Expression: 1
                                       //  Referenced by: '<S102>/Constant1'

  real_T Constant3_Value_kc;           // Expression: 0
                                       //  Referenced by: '<S105>/Constant3'

  real_T Merge2_1_InitialOutput;       // Computed Parameter: Merge2_1_InitialOutput
                                       //  Referenced by: '<S126>/Merge2'

  real_T Merge2_2_InitialOutput;       // Computed Parameter: Merge2_2_InitialOutput
                                       //  Referenced by: '<S126>/Merge2'

  real_T Merge2_3_InitialOutput;       // Computed Parameter: Merge2_3_InitialOutput
                                       //  Referenced by: '<S126>/Merge2'

  real_T Merge2_4_InitialOutput;       // Computed Parameter: Merge2_4_InitialOutput
                                       //  Referenced by: '<S126>/Merge2'

  real_T Constant2_Value_h;            // Expression: 0
                                       //  Referenced by: '<S5>/Constant2'

  real_T UnitDelay_InitialCondition_h; // Expression: 0
                                       //  Referenced by: '<S5>/Unit Delay'

  real_T Constant3_Value_p3;           // Expression: 1
                                       //  Referenced by: '<S5>/Constant3'

  real_T Saturation_LowerSat;          // Expression: 0
                                       //  Referenced by: '<S5>/Saturation'

  real_T Constant2_Value_d[9];         // Expression: zeros(9,1)
                                       //  Referenced by: '<S90>/Constant2'

  real_T Constant1_Value_ec;           // Expression: 1
                                       //  Referenced by: '<S90>/Constant1'

  real_T Constant3_Value_ol;           // Expression: 0
                                       //  Referenced by: '<S93>/Constant3'

  real_T Merge2_1_InitialOutput_c;     // Computed Parameter: Merge2_1_InitialOutput_c
                                       //  Referenced by: '<S125>/Merge2'

  real_T Merge2_2_InitialOutput_c;     // Computed Parameter: Merge2_2_InitialOutput_c
                                       //  Referenced by: '<S125>/Merge2'

  real_T Merge2_3_InitialOutput_c;     // Computed Parameter: Merge2_3_InitialOutput_c
                                       //  Referenced by: '<S125>/Merge2'

  real_T Merge2_4_InitialOutput_c;     // Computed Parameter: Merge2_4_InitialOutput_c
                                       //  Referenced by: '<S125>/Merge2'

  real32_T P_out_Y0;                   // Computed Parameter: P_out_Y0
                                       //  Referenced by: '<S12>/P_out'

  real32_T UnitDelay1_InitialCondition;// Computed Parameter: UnitDelay1_InitialCondition
                                       //  Referenced by: '<S12>/Unit Delay1'

  real32_T Merge_1_InitialOutput;      // Computed Parameter: Merge_1_InitialOutput
                                       //  Referenced by: '<S13>/Merge'

  real32_T Merge_3_InitialOutput;      // Computed Parameter: Merge_3_InitialOutput
                                       //  Referenced by: '<S13>/Merge'

  real32_T Merge_4_InitialOutput;      // Computed Parameter: Merge_4_InitialOutput
                                       //  Referenced by: '<S13>/Merge'

  real32_T Merge_7_InitialOutput;      // Computed Parameter: Merge_7_InitialOutput
                                       //  Referenced by: '<S13>/Merge'

  real32_T Merge_8_InitialOutput;      // Computed Parameter: Merge_8_InitialOutput
                                       //  Referenced by: '<S13>/Merge'

  real32_T UnitDelay2_InitialCondition;// Computed Parameter: UnitDelay2_InitialCondition
                                       //  Referenced by: '<S15>/Unit Delay2'

  real32_T P_out_Y0_e;                 // Computed Parameter: P_out_Y0_e
                                       //  Referenced by: '<S24>/P_out'

  real32_T UnitDelay2_InitialCondition_f;// Computed Parameter: UnitDelay2_InitialCondition_f
                                         //  Referenced by: '<S24>/Unit Delay2'

  real32_T UnitDelay1_InitialCondition_m;// Computed Parameter: UnitDelay1_InitialCondition_m
                                         //  Referenced by: '<S24>/Unit Delay1'

  real32_T Gain2_Gain;                 // Computed Parameter: Gain2_Gain
                                       //  Referenced by: '<S53>/Gain2'

  real32_T Gain2_Gain_e;               // Computed Parameter: Gain2_Gain_e
                                       //  Referenced by: '<S56>/Gain2'

  real32_T Gain1_Gain;                 // Computed Parameter: Gain1_Gain
                                       //  Referenced by: '<S56>/Gain1'

  real32_T Gain_Gain;                  // Computed Parameter: Gain_Gain
                                       //  Referenced by: '<S56>/Gain'

  real32_T Gain1_Gain_d;               // Computed Parameter: Gain1_Gain_d
                                       //  Referenced by: '<S53>/Gain1'

  real32_T Gain_Gain_l;                // Computed Parameter: Gain_Gain_l
                                       //  Referenced by: '<S53>/Gain'

  real32_T Constant1_Value_cn;         // Computed Parameter: Constant1_Value_cn
                                       //  Referenced by: '<S144>/Constant1'

  real32_T Constant3_Value_f;          // Computed Parameter: Constant3_Value_f
                                       //  Referenced by: '<S144>/Constant3'

  real32_T Constant3_Value_cc;         // Computed Parameter: Constant3_Value_cc
                                       //  Referenced by: '<S146>/Constant3'

  real32_T Gain_Gain_h;                // Computed Parameter: Gain_Gain_h
                                       //  Referenced by: '<S146>/Gain'

  real32_T Gain1_Gain_b;               // Computed Parameter: Gain1_Gain_b
                                       //  Referenced by: '<S146>/Gain1'

  real32_T Constant2_Value_pd;         // Computed Parameter: Constant2_Value_pd
                                       //  Referenced by: '<S146>/Constant2'

  real32_T Gain2_Gain_h;               // Computed Parameter: Gain2_Gain_h
                                       //  Referenced by: '<S146>/Gain2'

  real32_T Gain3_Gain;                 // Computed Parameter: Gain3_Gain
                                       //  Referenced by: '<S146>/Gain3'

  real32_T Gain4_Gain;                 // Computed Parameter: Gain4_Gain
                                       //  Referenced by: '<S146>/Gain4'

  real32_T Constant1_Value_j;          // Computed Parameter: Constant1_Value_j
                                       //  Referenced by: '<S146>/Constant1'

  real32_T Gain5_Gain;                 // Computed Parameter: Gain5_Gain
                                       //  Referenced by: '<S146>/Gain5'

  real32_T Constant_Value_f;           // Computed Parameter: Constant_Value_f
                                       //  Referenced by: '<S146>/Constant'

  real32_T Constant3_Value_iq;         // Computed Parameter: Constant3_Value_iq
                                       //  Referenced by: '<S147>/Constant3'

  real32_T Gain1_Gain_j;               // Computed Parameter: Gain1_Gain_j
                                       //  Referenced by: '<S129>/Gain1'

  real32_T Gain_Gain_n;                // Computed Parameter: Gain_Gain_n
                                       //  Referenced by: '<S147>/Gain'

  real32_T Gain1_Gain_f;               // Computed Parameter: Gain1_Gain_f
                                       //  Referenced by: '<S147>/Gain1'

  real32_T Constant2_Value_n;          // Computed Parameter: Constant2_Value_n
                                       //  Referenced by: '<S147>/Constant2'

  real32_T Gain2_Gain_a;               // Computed Parameter: Gain2_Gain_a
                                       //  Referenced by: '<S147>/Gain2'

  real32_T Gain3_Gain_l;               // Computed Parameter: Gain3_Gain_l
                                       //  Referenced by: '<S147>/Gain3'

  real32_T Gain4_Gain_o;               // Computed Parameter: Gain4_Gain_o
                                       //  Referenced by: '<S147>/Gain4'

  real32_T Constant1_Value_nk;         // Computed Parameter: Constant1_Value_nk
                                       //  Referenced by: '<S147>/Constant1'

  real32_T Gain5_Gain_j;               // Computed Parameter: Gain5_Gain_j
                                       //  Referenced by: '<S147>/Gain5'

  real32_T Constant_Value_j;           // Computed Parameter: Constant_Value_j
                                       //  Referenced by: '<S147>/Constant'

  real32_T Constant2_Value_p3;         // Computed Parameter: Constant2_Value_p3
                                       //  Referenced by: '<S144>/Constant2'

  real32_T Gain_Gain_a;                // Computed Parameter: Gain_Gain_a
                                       //  Referenced by: '<S138>/Gain'

  real32_T Gain1_Gain_e;               // Computed Parameter: Gain1_Gain_e
                                       //  Referenced by: '<S138>/Gain1'

  real32_T Gain_Gain_f;                // Computed Parameter: Gain_Gain_f
                                       //  Referenced by: '<S141>/Gain'

  real32_T Gain1_Gain_k;               // Computed Parameter: Gain1_Gain_k
                                       //  Referenced by: '<S141>/Gain1'

  real32_T Gain2_Gain_i;               // Computed Parameter: Gain2_Gain_i
                                       //  Referenced by: '<S141>/Gain2'

  real32_T Gain2_Gain_p;               // Computed Parameter: Gain2_Gain_p
                                       //  Referenced by: '<S138>/Gain2'

  real32_T Gain_Gain_g;                // Computed Parameter: Gain_Gain_g
                                       //  Referenced by: '<S136>/Gain'

  real32_T Gain1_Gain_jc;              // Computed Parameter: Gain1_Gain_jc
                                       //  Referenced by: '<S136>/Gain1'

  real32_T Gain2_Gain_o;               // Computed Parameter: Gain2_Gain_o
                                       //  Referenced by: '<S136>/Gain2'

  real32_T Gain1_Gain_i;               // Computed Parameter: Gain1_Gain_i
                                       //  Referenced by: '<S134>/Gain1'

  real32_T Constant1_Value_a;          // Computed Parameter: Constant1_Value_a
                                       //  Referenced by: '<S164>/Constant1'

  real32_T Constant3_Value_da;         // Computed Parameter: Constant3_Value_da
                                       //  Referenced by: '<S164>/Constant3'

  real32_T Constant3_Value_oe;         // Computed Parameter: Constant3_Value_oe
                                       //  Referenced by: '<S173>/Constant3'

  real32_T Constant5_Value[3];         // Expression: single([0 0 0])
                                       //  Referenced by: '<S161>/Constant5'

  real32_T Gain_Gain_d;                // Computed Parameter: Gain_Gain_d
                                       //  Referenced by: '<S173>/Gain'

  real32_T Gain1_Gain_n;               // Computed Parameter: Gain1_Gain_n
                                       //  Referenced by: '<S173>/Gain1'

  real32_T Constant2_Value_m2;         // Computed Parameter: Constant2_Value_m2
                                       //  Referenced by: '<S173>/Constant2'

  real32_T Gain2_Gain_k;               // Computed Parameter: Gain2_Gain_k
                                       //  Referenced by: '<S173>/Gain2'

  real32_T Gain3_Gain_f;               // Computed Parameter: Gain3_Gain_f
                                       //  Referenced by: '<S173>/Gain3'

  real32_T Gain4_Gain_b;               // Computed Parameter: Gain4_Gain_b
                                       //  Referenced by: '<S173>/Gain4'

  real32_T Constant1_Value_i;          // Computed Parameter: Constant1_Value_i
                                       //  Referenced by: '<S173>/Constant1'

  real32_T Gain5_Gain_f;               // Computed Parameter: Gain5_Gain_f
                                       //  Referenced by: '<S173>/Gain5'

  real32_T Constant_Value_m;           // Computed Parameter: Constant_Value_m
                                       //  Referenced by: '<S173>/Constant'

  real32_T Constant3_Value_op;         // Computed Parameter: Constant3_Value_op
                                       //  Referenced by: '<S174>/Constant3'

  real32_T Gain1_Gain_o;               // Computed Parameter: Gain1_Gain_o
                                       //  Referenced by: '<S161>/Gain1'

  real32_T Gain_Gain_ng;               // Computed Parameter: Gain_Gain_ng
                                       //  Referenced by: '<S174>/Gain'

  real32_T Gain1_Gain_a;               // Computed Parameter: Gain1_Gain_a
                                       //  Referenced by: '<S174>/Gain1'

  real32_T Constant2_Value_o;          // Computed Parameter: Constant2_Value_o
                                       //  Referenced by: '<S174>/Constant2'

  real32_T Gain2_Gain_b;               // Computed Parameter: Gain2_Gain_b
                                       //  Referenced by: '<S174>/Gain2'

  real32_T Gain3_Gain_o;               // Computed Parameter: Gain3_Gain_o
                                       //  Referenced by: '<S174>/Gain3'

  real32_T Gain4_Gain_i;               // Computed Parameter: Gain4_Gain_i
                                       //  Referenced by: '<S174>/Gain4'

  real32_T Constant1_Value_b;          // Computed Parameter: Constant1_Value_b
                                       //  Referenced by: '<S174>/Constant1'

  real32_T Gain5_Gain_c;               // Computed Parameter: Gain5_Gain_c
                                       //  Referenced by: '<S174>/Gain5'

  real32_T Constant_Value_px;          // Computed Parameter: Constant_Value_px
                                       //  Referenced by: '<S174>/Constant'

  real32_T Constant2_Value_mp;         // Computed Parameter: Constant2_Value_mp
                                       //  Referenced by: '<S164>/Constant2'

  real32_T Gain_Gain_c;                // Computed Parameter: Gain_Gain_c
                                       //  Referenced by: '<S187>/Gain'

  real32_T Gain1_Gain_nq;              // Computed Parameter: Gain1_Gain_nq
                                       //  Referenced by: '<S187>/Gain1'

  real32_T Gain_Gain_fr;               // Computed Parameter: Gain_Gain_fr
                                       //  Referenced by: '<S190>/Gain'

  real32_T Gain1_Gain_c;               // Computed Parameter: Gain1_Gain_c
                                       //  Referenced by: '<S190>/Gain1'

  real32_T Gain2_Gain_hc;              // Computed Parameter: Gain2_Gain_hc
                                       //  Referenced by: '<S190>/Gain2'

  real32_T Gain2_Gain_g;               // Computed Parameter: Gain2_Gain_g
                                       //  Referenced by: '<S187>/Gain2'

  ase_cov_datatype Constant3_Value_l2[90];// Computed Parameter: Constant3_Value_l2
                                          //  Referenced by: '<S161>/Constant3'

  real32_T Gain_Gain_cu;               // Computed Parameter: Gain_Gain_cu
                                       //  Referenced by: '<S165>/Gain'

  real32_T Gain1_Gain_l;               // Computed Parameter: Gain1_Gain_l
                                       //  Referenced by: '<S165>/Gain1'

  real32_T Gain_Gain_ne;               // Computed Parameter: Gain_Gain_ne
                                       //  Referenced by: '<S185>/Gain'

  real32_T Gain1_Gain_j4;              // Computed Parameter: Gain1_Gain_j4
                                       //  Referenced by: '<S185>/Gain1'

  real32_T Gain2_Gain_n;               // Computed Parameter: Gain2_Gain_n
                                       //  Referenced by: '<S185>/Gain2'

  real32_T Gain2_Gain_p0;              // Computed Parameter: Gain2_Gain_p0
                                       //  Referenced by: '<S165>/Gain2'

  real32_T Gain_Gain_fx;               // Computed Parameter: Gain_Gain_fx
                                       //  Referenced by: '<S167>/Gain'

  real32_T Gain1_Gain_ik;              // Computed Parameter: Gain1_Gain_ik
                                       //  Referenced by: '<S167>/Gain1'

  real32_T Gain2_Gain_m;               // Computed Parameter: Gain2_Gain_m
                                       //  Referenced by: '<S167>/Gain2'

  real32_T Gain_Gain_j;                // Computed Parameter: Gain_Gain_j
                                       //  Referenced by: '<S170>/Gain'

  real32_T Gain1_Gain_g;               // Computed Parameter: Gain1_Gain_g
                                       //  Referenced by: '<S170>/Gain1'

  real32_T Gain2_Gain_eo;              // Computed Parameter: Gain2_Gain_eo
                                       //  Referenced by: '<S170>/Gain2'

  real32_T Gain1_Gain_ei;              // Computed Parameter: Gain1_Gain_ei
                                       //  Referenced by: '<S168>/Gain1'

  real32_T Constant2_Value_ad[13689];  // Expression: zeros(ase_total_num_states^2,1, 'single')
                                       //  Referenced by: '<S2>/Constant2'

  real32_T Constant5_Value_a[102];     // Expression:  ones(1, ase_total_num_states - size(tun_ase_state_ic_cov_diag, 2), 'single')
                                       //  Referenced by: '<S2>/Constant5'

  real32_T UnitDelay24_InitialCondition;// Computed Parameter: UnitDelay24_InitialCondition
                                        //  Referenced by: '<S2>/Unit Delay24'

  real32_T UnitDelay25_InitialCondition;// Computed Parameter: UnitDelay25_InitialCondition
                                        //  Referenced by: '<S2>/Unit Delay25'

  real32_T Constant1_Value_i3;         // Computed Parameter: Constant1_Value_i3
                                       //  Referenced by: '<S113>/Constant1'

  real32_T Constant3_Value_dc;         // Computed Parameter: Constant3_Value_dc
                                       //  Referenced by: '<S113>/Constant3'

  real32_T Constant3_Value_c4;         // Computed Parameter: Constant3_Value_c4
                                       //  Referenced by: '<S115>/Constant3'

  real32_T Gain_Gain_n1;               // Computed Parameter: Gain_Gain_n1
                                       //  Referenced by: '<S115>/Gain'

  real32_T Gain1_Gain_au;              // Computed Parameter: Gain1_Gain_au
                                       //  Referenced by: '<S115>/Gain1'

  real32_T Constant2_Value_am;         // Computed Parameter: Constant2_Value_am
                                       //  Referenced by: '<S115>/Constant2'

  real32_T Gain2_Gain_a5;              // Computed Parameter: Gain2_Gain_a5
                                       //  Referenced by: '<S115>/Gain2'

  real32_T Gain3_Gain_m;               // Computed Parameter: Gain3_Gain_m
                                       //  Referenced by: '<S115>/Gain3'

  real32_T Gain4_Gain_h;               // Computed Parameter: Gain4_Gain_h
                                       //  Referenced by: '<S115>/Gain4'

  real32_T Constant1_Value_p;          // Computed Parameter: Constant1_Value_p
                                       //  Referenced by: '<S115>/Constant1'

  real32_T Gain5_Gain_a;               // Computed Parameter: Gain5_Gain_a
                                       //  Referenced by: '<S115>/Gain5'

  real32_T Constant_Value_d;           // Computed Parameter: Constant_Value_d
                                       //  Referenced by: '<S115>/Constant'

  real32_T Constant3_Value_dg;         // Computed Parameter: Constant3_Value_dg
                                       //  Referenced by: '<S116>/Constant3'

  real32_T Gain_Gain_g0;               // Computed Parameter: Gain_Gain_g0
                                       //  Referenced by: '<S58>/Gain'

  real32_T Gain1_Gain_jx;              // Computed Parameter: Gain1_Gain_jx
                                       //  Referenced by: '<S58>/Gain1'

  real32_T Gain_Gain_e;                // Computed Parameter: Gain_Gain_e
                                       //  Referenced by: '<S61>/Gain'

  real32_T Gain1_Gain_nf;              // Computed Parameter: Gain1_Gain_nf
                                       //  Referenced by: '<S61>/Gain1'

  real32_T Gain2_Gain_h5;              // Computed Parameter: Gain2_Gain_h5
                                       //  Referenced by: '<S61>/Gain2'

  real32_T Gain2_Gain_gg;              // Computed Parameter: Gain2_Gain_gg
                                       //  Referenced by: '<S58>/Gain2'

  real32_T Gain_Gain_co;               // Computed Parameter: Gain_Gain_co
                                       //  Referenced by: '<S116>/Gain'

  real32_T Gain1_Gain_cz;              // Computed Parameter: Gain1_Gain_cz
                                       //  Referenced by: '<S116>/Gain1'

  real32_T Constant2_Value_c;          // Computed Parameter: Constant2_Value_c
                                       //  Referenced by: '<S116>/Constant2'

  real32_T Gain2_Gain_g4;              // Computed Parameter: Gain2_Gain_g4
                                       //  Referenced by: '<S116>/Gain2'

  real32_T Gain3_Gain_fu;              // Computed Parameter: Gain3_Gain_fu
                                       //  Referenced by: '<S116>/Gain3'

  real32_T Gain4_Gain_g;               // Computed Parameter: Gain4_Gain_g
                                       //  Referenced by: '<S116>/Gain4'

  real32_T Constant1_Value_o;          // Computed Parameter: Constant1_Value_o
                                       //  Referenced by: '<S116>/Constant1'

  real32_T Gain5_Gain_jr;              // Computed Parameter: Gain5_Gain_jr
                                       //  Referenced by: '<S116>/Gain5'

  real32_T Constant_Value_lw;          // Computed Parameter: Constant_Value_lw
                                       //  Referenced by: '<S116>/Constant'

  real32_T Constant2_Value_j0;         // Computed Parameter: Constant2_Value_j0
                                       //  Referenced by: '<S113>/Constant2'

  real32_T Gain_Gain_c0;               // Computed Parameter: Gain_Gain_c0
                                       //  Referenced by: '<S107>/Gain'

  real32_T Gain1_Gain_il;              // Computed Parameter: Gain1_Gain_il
                                       //  Referenced by: '<S107>/Gain1'

  real32_T Gain_Gain_lj;               // Computed Parameter: Gain_Gain_lj
                                       //  Referenced by: '<S110>/Gain'

  real32_T Gain1_Gain_lb;              // Computed Parameter: Gain1_Gain_lb
                                       //  Referenced by: '<S110>/Gain1'

  real32_T Gain2_Gain_m4;              // Computed Parameter: Gain2_Gain_m4
                                       //  Referenced by: '<S110>/Gain2'

  real32_T Gain2_Gain_j;               // Computed Parameter: Gain2_Gain_j
                                       //  Referenced by: '<S107>/Gain2'

  real32_T Gain_Gain_p;                // Computed Parameter: Gain_Gain_p
                                       //  Referenced by: '<S63>/Gain'

  real32_T Gain1_Gain_fe;              // Computed Parameter: Gain1_Gain_fe
                                       //  Referenced by: '<S63>/Gain1'

  real32_T Gain_Gain_az;               // Computed Parameter: Gain_Gain_az
                                       //  Referenced by: '<S66>/Gain'

  real32_T Gain1_Gain_ln;              // Computed Parameter: Gain1_Gain_ln
                                       //  Referenced by: '<S66>/Gain1'

  real32_T Gain2_Gain_c;               // Computed Parameter: Gain2_Gain_c
                                       //  Referenced by: '<S66>/Gain2'

  real32_T Gain2_Gain_i5;              // Computed Parameter: Gain2_Gain_i5
                                       //  Referenced by: '<S63>/Gain2'

  real32_T Constant6_Value[3];         // Expression: single([0 0 0])
                                       //  Referenced by: '<S41>/Constant6'

  ase_cov_datatype Constant5_Value_p[180];// Computed Parameter: Constant5_Value_p
                                          //  Referenced by: '<S95>/Constant5'

  real32_T Gain_Gain_fm;               // Computed Parameter: Gain_Gain_fm
                                       //  Referenced by: '<S102>/Gain'

  real32_T Gain1_Gain_l4;              // Computed Parameter: Gain1_Gain_l4
                                       //  Referenced by: '<S102>/Gain1'

  real32_T Gain_Gain_k;                // Computed Parameter: Gain_Gain_k
                                       //  Referenced by: '<S105>/Gain'

  real32_T Gain1_Gain_j5;              // Computed Parameter: Gain1_Gain_j5
                                       //  Referenced by: '<S105>/Gain1'

  real32_T Gain2_Gain_i5w;             // Computed Parameter: Gain2_Gain_i5w
                                       //  Referenced by: '<S105>/Gain2'

  real32_T Gain2_Gain_kj;              // Computed Parameter: Gain2_Gain_kj
                                       //  Referenced by: '<S102>/Gain2'

  real32_T Gain2_Gain_cx;              // Computed Parameter: Gain2_Gain_cx
                                       //  Referenced by: '<S95>/Gain2'

  real32_T Gain_Gain_h3;               // Computed Parameter: Gain_Gain_h3
                                       //  Referenced by: '<S90>/Gain'

  real32_T Gain1_Gain_jb;              // Computed Parameter: Gain1_Gain_jb
                                       //  Referenced by: '<S90>/Gain1'

  real32_T Gain_Gain_j2;               // Computed Parameter: Gain_Gain_j2
                                       //  Referenced by: '<S93>/Gain'

  real32_T Gain1_Gain_jp;              // Computed Parameter: Gain1_Gain_jp
                                       //  Referenced by: '<S93>/Gain1'

  real32_T Gain2_Gain_ko;              // Computed Parameter: Gain2_Gain_ko
                                       //  Referenced by: '<S93>/Gain2'

  real32_T Gain2_Gain_a2;              // Computed Parameter: Gain2_Gain_a2
                                       //  Referenced by: '<S90>/Gain2'

  int32_T Switch3_Threshold;           // Computed Parameter: Switch3_Threshold
                                       //  Referenced by: '<S12>/Switch3'

  int32_T Switch2_Threshold;           // Computed Parameter: Switch2_Threshold
                                       //  Referenced by: '<S12>/Switch2'

  int32_T Merge_2_InitialOutput;       // Computed Parameter: Merge_2_InitialOutput
                                       //  Referenced by: '<S13>/Merge'

  int32_T Switch2_Threshold_b;         // Computed Parameter: Switch2_Threshold_b
                                       //  Referenced by: '<S24>/Switch2'

  int32_T Switch3_Threshold_d;         // Computed Parameter: Switch3_Threshold_d
                                       //  Referenced by: '<S24>/Switch3'

  uint32_T Constant_Value_h;           // Expression: uint32(1)
                                       //  Referenced by: '<S12>/Constant'

  uint32_T Delay_DelayLength;          // Computed Parameter: Delay_DelayLength
                                       //  Referenced by: '<S2>/Delay'

  uint32_T BitwiseOperator1_BitMask_l; // Expression: BitMask
                                       //  Referenced by: '<S11>/Bitwise Operator1'

  uint16_T Constant_Value_ln;          // Expression: uint16(1)
                                       //  Referenced by: '<S23>/Constant'

  uint16_T Constant_Value_c;           // Expression: uint16(2)
                                       //  Referenced by: '<S27>/Constant'

  uint16_T Constant_Value_o;           // Computed Parameter: Constant_Value_o
                                       //  Referenced by: '<S159>/Constant'

  uint8_T Merge_5_InitialOutput;       // Computed Parameter: Merge_5_InitialOutput
                                       //  Referenced by: '<S13>/Merge'

  uint8_T Constant2_Value_n4;          // Expression: uint8(0)
                                       //  Referenced by: '<S14>/Constant2'

  uint8_T Constant_Value_dd;           // Expression: uint8(0)
                                       //  Referenced by: '<S24>/Constant'

  uint8_T Constant_Value_oa;           // Expression: uint8(0)
                                       //  Referenced by: '<S9>/Constant'

  uint8_T Constant1_Value_f;           // Expression: uint8(0)
                                       //  Referenced by: '<S9>/Constant1'

  uint8_T Out1_Y0;                     // Computed Parameter: Out1_Y0
                                       //  Referenced by: '<S40>/Out1'

  uint8_T UnitDelay16_InitialCondition;// Computed Parameter: UnitDelay16_InitialCondition
                                       //  Referenced by: '<S2>/Unit Delay16'

  uint8_T UnitDelay17_InitialCondition;// Computed Parameter: UnitDelay17_InitialCondition
                                       //  Referenced by: '<S2>/Unit Delay17'

  uint8_T Constant_Value_oq;           // Expression: const
                                       //  Referenced by: '<S33>/Constant'

  uint8_T Saturation_LowerSat_n;       // Computed Parameter: Saturation_LowerSat_n
                                       //  Referenced by: '<S41>/Saturation'

  uint8_T Switch_Threshold;            // Computed Parameter: Switch_Threshold
                                       //  Referenced by: '<S41>/Switch'

  uint8_T Constant_Value_lf;           // Expression: const
                                       //  Referenced by: '<S35>/Constant'

  uint8_T Constant_Value_js;           // Expression: const
                                       //  Referenced by: '<S39>/Constant'

  uint8_T Constant_Value_e;            // Expression: const
                                       //  Referenced by: '<S32>/Constant'

  P_Normalize_est_estimator_T Normalize_i;// '<S175>/Normalize'
  P_Normalize_est_estimator_T Normalize_h;// '<S148>/Normalize'
  P_Normalize_est_estimator_T Normalize;// '<S117>/Normalize'
  P_CoreSubsys_est_estimator_g_T CoreSubsys_l;// '<S47>/CoreSubsys'
  P_CoreSubsys_est_estimator_T CoreSubsys;// '<S46>/CoreSubsys'
};

// Real-time Model Data Structure
struct tag_RTM_est_estimator_T {
  const char_T * volatile errorStatus;
  B_est_estimator_T *blockIO;
  P_est_estimator_T *defaultParam;
  boolean_T paramIsMalloced;
  DW_est_estimator_T *dwork;
};

#ifdef __cplusplus

extern "C" {

#endif

#ifdef __cplusplus

}
#endif

// External data declarations for dependent source files
extern const cvs_landmark_msg est_estimator_rtZcvs_landmark_msg;// cvs_landmark_msg ground 
extern const cvs_registration_pulse est_estimator_rtZcvs_registration_pulse;// cvs_registration_pulse ground 
extern const cvs_optical_flow_msg est_estimator_rtZcvs_optical_flow_msg;// cvs_optical_flow_msg ground 
extern const cvs_handrail_msg est_estimator_rtZcvs_handrail_msg;// cvs_handrail_msg ground 
extern const imu_msg est_estimator_rtZimu_msg;// imu_msg ground
extern const cmc_msg est_estimator_rtZcmc_msg;// cmc_msg ground
extern const kfl_msg est_estimator_rtZkfl_msg;// kfl_msg ground

#ifdef __cplusplus

extern "C" {

#endif

  extern const char *RT_MEMORY_ALLOCATION_ERROR;

#ifdef __cplusplus

}
#endif

extern P_est_estimator_T est_estimator_P;// parameters

#ifdef __cplusplus

extern "C" {

#endif

  // Model entry point functions
  extern RT_MODEL_est_estimator_T *est_estimator(cvs_landmark_msg
    *est_estimator_U_landmark_msg, cvs_registration_pulse
    *est_estimator_U_VisionRegistration, cvs_optical_flow_msg
    *est_estimator_U_cvs_optical_flow_msg_n, cvs_handrail_msg
    *est_estimator_U_handrail_msg, imu_msg *est_estimator_U_imu_msg_c, cmc_msg
    *est_estimator_U_cmc_msg_o, real32_T est_estimator_U_Q_ISS2B[4], kfl_msg
    *est_estimator_Y_kfl_msg_h, ase_cov_datatype est_estimator_Y_P_out[13689]);
  extern void est_estimator_initialize(RT_MODEL_est_estimator_T *const
    est_estimator_M, cvs_landmark_msg *est_estimator_U_landmark_msg,
    cvs_registration_pulse *est_estimator_U_VisionRegistration,
    cvs_optical_flow_msg *est_estimator_U_cvs_optical_flow_msg_n,
    cvs_handrail_msg *est_estimator_U_handrail_msg, imu_msg
    *est_estimator_U_imu_msg_c, cmc_msg *est_estimator_U_cmc_msg_o, real32_T
    est_estimator_U_Q_ISS2B[4], kfl_msg *est_estimator_Y_kfl_msg_h,
    ase_cov_datatype est_estimator_Y_P_out[13689]);
  extern void est_estimator_step(RT_MODEL_est_estimator_T *const est_estimator_M,
    cvs_landmark_msg *est_estimator_U_landmark_msg, cvs_registration_pulse
    *est_estimator_U_VisionRegistration, cvs_optical_flow_msg
    *est_estimator_U_cvs_optical_flow_msg_n, cvs_handrail_msg
    *est_estimator_U_handrail_msg, imu_msg *est_estimator_U_imu_msg_c, cmc_msg
    *est_estimator_U_cmc_msg_o, real32_T est_estimator_U_Q_ISS2B[4], kfl_msg
    *est_estimator_Y_kfl_msg_h, ase_cov_datatype est_estimator_Y_P_out[13689]);
  extern void est_estimator_terminate(RT_MODEL_est_estimator_T * est_estimator_M);

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
//  hilite_system('astrobee/fsw_lib/est_estimator')    - opens subsystem astrobee/fsw_lib/est_estimator
//  hilite_system('astrobee/fsw_lib/est_estimator/Kp') - opens and selects block Kp
//
//  Here is the system hierarchy for this model
//
//  '<Root>' : 'astrobee/fsw_lib'
//  '<S1>'   : 'astrobee/fsw_lib/est_estimator'
//  '<S2>'   : 'astrobee/fsw_lib/est_estimator/Delay - Initial Conditions'
//  '<S3>'   : 'astrobee/fsw_lib/est_estimator/camera_update'
//  '<S4>'   : 'astrobee/fsw_lib/est_estimator/filter_prep'
//  '<S5>'   : 'astrobee/fsw_lib/est_estimator/output_prep'
//  '<S6>'   : 'astrobee/fsw_lib/est_estimator/predictor'
//  '<S7>'   : 'astrobee/fsw_lib/est_estimator/state_manager'
//  '<S8>'   : 'astrobee/fsw_lib/est_estimator/camera_update/Absolute_Update'
//  '<S9>'   : 'astrobee/fsw_lib/est_estimator/camera_update/If Action Subsystem1'
//  '<S10>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Optical_Flow_Update'
//  '<S11>'  : 'astrobee/fsw_lib/est_estimator/camera_update/vision_preprocessing'
//  '<S12>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Absolute_Update/ML Update'
//  '<S13>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Absolute_Update/ML Update/Compute Residual and H matrix'
//  '<S14>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Absolute_Update/ML Update/ML_correector'
//  '<S15>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Absolute_Update/ML Update/update_validity_check'
//  '<S16>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Absolute_Update/ML Update/Compute Residual and H matrix/HR_Compute_Residual_and_H1'
//  '<S17>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Absolute_Update/ML Update/Compute Residual and H matrix/ML_Compute_Residual_and_H'
//  '<S18>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Absolute_Update/ML Update/Compute Residual and H matrix/HR_Compute_Residual_and_H1/Compute Global positions of Handrail Features'
//  '<S19>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Absolute_Update/ML Update/Compute Residual and H matrix/HR_Compute_Residual_and_H1/Compute Residual and H'
//  '<S20>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Absolute_Update/ML Update/Compute Residual and H matrix/ML_Compute_Residual_and_H/Compare To Constant'
//  '<S21>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Absolute_Update/ML Update/Compute Residual and H matrix/ML_Compute_Residual_and_H/Compare To Constant2'
//  '<S22>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Absolute_Update/ML Update/Compute Residual and H matrix/ML_Compute_Residual_and_H/Compute Residual and H'
//  '<S23>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Absolute_Update/ML Update/ML_correector/apply_delta_state'
//  '<S24>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Optical_Flow_Update/OF Update'
//  '<S25>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Optical_Flow_Update/OF Update/Compute_DeltaState_and_Covariance'
//  '<S26>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Optical_Flow_Update/OF Update/Simulink Compute H and R'
//  '<S27>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Optical_Flow_Update/OF Update/apply_delta_state'
//  '<S28>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Optical_Flow_Update/OF Update/compute_of_global_points'
//  '<S29>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Optical_Flow_Update/OF Update/Simulink Compute H and R/compress_of_residual_and_h'
//  '<S30>'  : 'astrobee/fsw_lib/est_estimator/camera_update/Optical_Flow_Update/OF Update/Simulink Compute H and R/compute_of_residual_and_h'
//  '<S31>'  : 'astrobee/fsw_lib/est_estimator/camera_update/vision_preprocessing/Compare To Constant'
//  '<S32>'  : 'astrobee/fsw_lib/est_estimator/camera_update/vision_preprocessing/Compare To Constant1'
//  '<S33>'  : 'astrobee/fsw_lib/est_estimator/camera_update/vision_preprocessing/Compare To Constant2'
//  '<S34>'  : 'astrobee/fsw_lib/est_estimator/camera_update/vision_preprocessing/Compare To Constant3'
//  '<S35>'  : 'astrobee/fsw_lib/est_estimator/camera_update/vision_preprocessing/Compare To Constant4'
//  '<S36>'  : 'astrobee/fsw_lib/est_estimator/camera_update/vision_preprocessing/Compare To Constant5'
//  '<S37>'  : 'astrobee/fsw_lib/est_estimator/camera_update/vision_preprocessing/Compare To Constant6'
//  '<S38>'  : 'astrobee/fsw_lib/est_estimator/camera_update/vision_preprocessing/Compare To Constant7'
//  '<S39>'  : 'astrobee/fsw_lib/est_estimator/camera_update/vision_preprocessing/Compare To Constant8'
//  '<S40>'  : 'astrobee/fsw_lib/est_estimator/camera_update/vision_preprocessing/Enabled Row-Wise SUM'
//  '<S41>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep'
//  '<S42>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/kfl_system_prep'
//  '<S43>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/vision_system_prep'
//  '<S44>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/Detect Change2'
//  '<S45>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/Detect Change3'
//  '<S46>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/filter'
//  '<S47>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/filter_with_HP_filter'
//  '<S48>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_A2B'
//  '<S49>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_B2A'
//  '<S50>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_B2A1'
//  '<S51>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/filter/MATLAB Function1'
//  '<S52>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/filter_with_HP_filter/MATLAB Function1'
//  '<S53>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_A2B/quaternion_to_DCM'
//  '<S54>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_A2B/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S55>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_A2B/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S56>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_A2B/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S57>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_A2B/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S58>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_B2A/quaternion_to_DCM'
//  '<S59>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_B2A/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S60>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_B2A/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S61>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_B2A/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S62>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_B2A/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S63>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_B2A1/quaternion_to_DCM'
//  '<S64>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_B2A1/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S65>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_B2A1/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S66>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_B2A1/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S67>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/imu_prep/rotate_vec_B2A1/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S68>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/kfl_system_prep/Bit Clear'
//  '<S69>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/kfl_system_prep/Detect Change'
//  '<S70>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/kfl_system_prep/MATLAB Function'
//  '<S71>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/kfl_system_prep/dummy_subsystem'
//  '<S72>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/vision_system_prep/handrail_prep'
//  '<S73>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/vision_system_prep/landmark_prep'
//  '<S74>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/vision_system_prep/optical_flow_prep'
//  '<S75>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/vision_system_prep/handrail_prep/Detect Change6'
//  '<S76>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/vision_system_prep/handrail_prep/Detect Change7'
//  '<S77>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/vision_system_prep/landmark_prep/Detect Change'
//  '<S78>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/vision_system_prep/landmark_prep/Detect Change1'
//  '<S79>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/vision_system_prep/optical_flow_prep/Detect Change4'
//  '<S80>'  : 'astrobee/fsw_lib/est_estimator/filter_prep/vision_system_prep/optical_flow_prep/Detect Change5'
//  '<S81>'  : 'astrobee/fsw_lib/est_estimator/output_prep/Compare To Constant'
//  '<S82>'  : 'astrobee/fsw_lib/est_estimator/output_prep/Compare To Constant1'
//  '<S83>'  : 'astrobee/fsw_lib/est_estimator/output_prep/Compare To Constant2'
//  '<S84>'  : 'astrobee/fsw_lib/est_estimator/output_prep/Compare To Constant3'
//  '<S85>'  : 'astrobee/fsw_lib/est_estimator/output_prep/Compare To Constant4'
//  '<S86>'  : 'astrobee/fsw_lib/est_estimator/output_prep/Compare To Constant5'
//  '<S87>'  : 'astrobee/fsw_lib/est_estimator/output_prep/Compare To Constant6'
//  '<S88>'  : 'astrobee/fsw_lib/est_estimator/output_prep/Compare To Constant7'
//  '<S89>'  : 'astrobee/fsw_lib/est_estimator/output_prep/rotate_vec_B2A'
//  '<S90>'  : 'astrobee/fsw_lib/est_estimator/output_prep/rotate_vec_B2A/quaternion_to_DCM'
//  '<S91>'  : 'astrobee/fsw_lib/est_estimator/output_prep/rotate_vec_B2A/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S92>'  : 'astrobee/fsw_lib/est_estimator/output_prep/rotate_vec_B2A/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S93>'  : 'astrobee/fsw_lib/est_estimator/output_prep/rotate_vec_B2A/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S94>'  : 'astrobee/fsw_lib/est_estimator/output_prep/rotate_vec_B2A/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S95>'  : 'astrobee/fsw_lib/est_estimator/predictor/Covariance Propogation'
//  '<S96>'  : 'astrobee/fsw_lib/est_estimator/predictor/Filter_10hz_1order'
//  '<S97>'  : 'astrobee/fsw_lib/est_estimator/predictor/rotate_vec_B2A'
//  '<S98>'  : 'astrobee/fsw_lib/est_estimator/predictor/single_step_zero_order_propogator'
//  '<S99>'  : 'astrobee/fsw_lib/est_estimator/predictor/Covariance Propogation/MATLAB Function'
//  '<S100>' : 'astrobee/fsw_lib/est_estimator/predictor/Covariance Propogation/MATLAB Function2'
//  '<S101>' : 'astrobee/fsw_lib/est_estimator/predictor/Covariance Propogation/diag'
//  '<S102>' : 'astrobee/fsw_lib/est_estimator/predictor/Covariance Propogation/quaternion_to_DCM'
//  '<S103>' : 'astrobee/fsw_lib/est_estimator/predictor/Covariance Propogation/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S104>' : 'astrobee/fsw_lib/est_estimator/predictor/Covariance Propogation/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S105>' : 'astrobee/fsw_lib/est_estimator/predictor/Covariance Propogation/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S106>' : 'astrobee/fsw_lib/est_estimator/predictor/Covariance Propogation/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S107>' : 'astrobee/fsw_lib/est_estimator/predictor/rotate_vec_B2A/quaternion_to_DCM'
//  '<S108>' : 'astrobee/fsw_lib/est_estimator/predictor/rotate_vec_B2A/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S109>' : 'astrobee/fsw_lib/est_estimator/predictor/rotate_vec_B2A/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S110>' : 'astrobee/fsw_lib/est_estimator/predictor/rotate_vec_B2A/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S111>' : 'astrobee/fsw_lib/est_estimator/predictor/rotate_vec_B2A/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S112>' : 'astrobee/fsw_lib/est_estimator/predictor/single_step_zero_order_propogator/Data Type Conversion Inherited'
//  '<S113>' : 'astrobee/fsw_lib/est_estimator/predictor/single_step_zero_order_propogator/first_order_quaternion_propogation'
//  '<S114>' : 'astrobee/fsw_lib/est_estimator/predictor/single_step_zero_order_propogator/first_order_quaternion_propogation/MATLAB Function'
//  '<S115>' : 'astrobee/fsw_lib/est_estimator/predictor/single_step_zero_order_propogator/first_order_quaternion_propogation/create_omega_matrix'
//  '<S116>' : 'astrobee/fsw_lib/est_estimator/predictor/single_step_zero_order_propogator/first_order_quaternion_propogation/create_omega_matrix1'
//  '<S117>' : 'astrobee/fsw_lib/est_estimator/predictor/single_step_zero_order_propogator/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar'
//  '<S118>' : 'astrobee/fsw_lib/est_estimator/predictor/single_step_zero_order_propogator/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/No-op'
//  '<S119>' : 'astrobee/fsw_lib/est_estimator/predictor/single_step_zero_order_propogator/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/Normalize'
//  '<S120>' : 'astrobee/fsw_lib/est_estimator/predictor/single_step_zero_order_propogator/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/vector_normalize'
//  '<S121>' : 'astrobee/fsw_lib/est_estimator/predictor/single_step_zero_order_propogator/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/Normalize/Data Type Conversion Inherited'
//  '<S122>' : 'astrobee/fsw_lib/est_estimator/predictor/single_step_zero_order_propogator/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/vector_normalize/No-op'
//  '<S123>' : 'astrobee/fsw_lib/est_estimator/predictor/single_step_zero_order_propogator/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/vector_normalize/Normalize'
//  '<S124>' : 'astrobee/fsw_lib/est_estimator/predictor/single_step_zero_order_propogator/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/vector_normalize/vector_magnitude'
//  '<S125>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager'
//  '<S126>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager'
//  '<S127>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/Compare To Constant'
//  '<S128>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem1'
//  '<S129>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2'
//  '<S130>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/MATLAB Function'
//  '<S131>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/Quaternion_Multiplication'
//  '<S132>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/rotate_vec_B2A'
//  '<S133>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/single_step_zero_order_propogator'
//  '<S134>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/Quaternion_Multiplication/Quaternion Xi'
//  '<S135>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/Quaternion_Multiplication/Quaternion Xi/Data Type Conversion Inherited'
//  '<S136>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator'
//  '<S137>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S138>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/rotate_vec_B2A/quaternion_to_DCM'
//  '<S139>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/rotate_vec_B2A/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S140>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/rotate_vec_B2A/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S141>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/rotate_vec_B2A/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S142>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/rotate_vec_B2A/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S143>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/single_step_zero_order_propogator/Data Type Conversion Inherited'
//  '<S144>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/single_step_zero_order_propogator/first_order_quaternion_propogation'
//  '<S145>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/single_step_zero_order_propogator/first_order_quaternion_propogation/MATLAB Function'
//  '<S146>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/single_step_zero_order_propogator/first_order_quaternion_propogation/create_omega_matrix'
//  '<S147>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/single_step_zero_order_propogator/first_order_quaternion_propogation/create_omega_matrix1'
//  '<S148>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/single_step_zero_order_propogator/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar'
//  '<S149>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/single_step_zero_order_propogator/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/No-op'
//  '<S150>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/single_step_zero_order_propogator/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/Normalize'
//  '<S151>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/single_step_zero_order_propogator/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/vector_normalize'
//  '<S152>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/single_step_zero_order_propogator/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/Normalize/Data Type Conversion Inherited'
//  '<S153>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/single_step_zero_order_propogator/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/vector_normalize/No-op'
//  '<S154>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/single_step_zero_order_propogator/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/vector_normalize/Normalize'
//  '<S155>' : 'astrobee/fsw_lib/est_estimator/state_manager/Optical Flow Registration Manager/If Action Subsystem2/single_step_zero_order_propogator/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/vector_normalize/vector_magnitude'
//  '<S156>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/Compare To Constant1'
//  '<S157>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/Compare To Constant3'
//  '<S158>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/Compare To Constant5'
//  '<S159>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/Compare To Zero'
//  '<S160>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem1'
//  '<S161>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2'
//  '<S162>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem4'
//  '<S163>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/Quaternion_Multiplication'
//  '<S164>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/first_order_quaternion_propogation'
//  '<S165>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/quaternion_to_DCM'
//  '<S166>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/rotate_vec_B2A'
//  '<S167>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/skew_symetric_matrix_operator'
//  '<S168>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/Quaternion_Multiplication/Quaternion Xi'
//  '<S169>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/Quaternion_Multiplication/Quaternion Xi/Data Type Conversion Inherited'
//  '<S170>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator'
//  '<S171>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/Quaternion_Multiplication/Quaternion Xi/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S172>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/first_order_quaternion_propogation/MATLAB Function'
//  '<S173>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/first_order_quaternion_propogation/create_omega_matrix'
//  '<S174>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/first_order_quaternion_propogation/create_omega_matrix1'
//  '<S175>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar'
//  '<S176>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/No-op'
//  '<S177>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/Normalize'
//  '<S178>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/vector_normalize'
//  '<S179>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/Normalize/Data Type Conversion Inherited'
//  '<S180>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/vector_normalize/No-op'
//  '<S181>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/vector_normalize/Normalize'
//  '<S182>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/first_order_quaternion_propogation/quat_normalize_and_enforce_positive_scalar/vector_normalize/vector_magnitude'
//  '<S183>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S184>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S185>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S186>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S187>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/rotate_vec_B2A/quaternion_to_DCM'
//  '<S188>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/rotate_vec_B2A/quaternion_to_DCM/Data Type Conversion Inherited'
//  '<S189>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/rotate_vec_B2A/quaternion_to_DCM/Data Type Conversion Inherited1'
//  '<S190>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/rotate_vec_B2A/quaternion_to_DCM/skew_symetric_matrix_operator'
//  '<S191>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/rotate_vec_B2A/quaternion_to_DCM/skew_symetric_matrix_operator/Data Type Conversion Inherited'
//  '<S192>' : 'astrobee/fsw_lib/est_estimator/state_manager/absolute_localizaton_registration_manager/If Action Subsystem2/skew_symetric_matrix_operator/Data Type Conversion Inherited'

#endif                                 // RTW_HEADER_est_estimator_h_

//
// File trailer for generated code.
//
// [EOF]
//
