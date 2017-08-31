//
// File: ctl_controller0.cpp
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

const cmd_msg ctl_controller0_rtZcmd_msg = {
  0U,                                  // cmd_timestamp_sec
  0U,                                  // cmd_timestamp_nsec
  0U,                                  // cmd_mode
  0U,                                  // speed_gain_cmd
  0U,                                  // cmd_B_inuse

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // traj_pos

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // traj_vel

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // traj_accel

  {
    0.0F, 0.0F, 0.0F, 0.0F }
  ,                                    // traj_quat

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // traj_omega

  {
    0.0F, 0.0F, 0.0F }
  // traj_alpha
} ;                                    // cmd_msg ground

const ctl_msg ctl_controller0_rtZctl_msg = {
  {
    0.0F, 0.0F, 0.0F }
  ,                                    // body_force_cmd

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // body_accel_cmd

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // pos_err

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // pos_err_int

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // body_torque_cmd

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // body_alpha_cmd

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // att_err
  0.0F,                                // att_err_mag

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // att_err_int
  0U,                                  // ctl_status
  0.0F,                                // traj_error_pos
  0.0F,                                // traj_error_att
  0.0F,                                // traj_error_vel
  0.0F                                 // traj_error_omega
} ;                                    // ctl_msg ground

const cmc_msg ctl_controller0_rtZcmc_msg = { { 0U,// timestamp_sec
    0U,                                // timestamp_nsec
    { 0.0F, 0.0F, 0.0F },              // P_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F },              // V_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F },              // A_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F, 0.0F },        // quat_ISS2B
    { 0.0F, 0.0F, 0.0F },              // omega_B_ISS_B
    { 0.0F, 0.0F, 0.0F }               // alpha_B_ISS_B
  },                                   // cmc_state_cmd_a
  { 0U,                                // timestamp_sec
    0U,                                // timestamp_nsec
    { 0.0F, 0.0F, 0.0F },              // P_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F },              // V_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F },              // A_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F, 0.0F },        // quat_ISS2B
    { 0.0F, 0.0F, 0.0F },              // omega_B_ISS_B
    { 0.0F, 0.0F, 0.0F }               // alpha_B_ISS_B
  },                                   // cmc_state_cmd_b
  0U,                                  // cmc_mode_cmd
  0U,                                  // speed_gain_cmd
  0U,                                  // localization_mode_cmd
  { 0.0F, 0.0F, 0.0F },                // att_kp
  { 0.0F, 0.0F, 0.0F },                // att_ki
  { 0.0F, 0.0F, 0.0F },                // omega_kd
  { 0.0F, 0.0F, 0.0F },                // pos_kp
  { 0.0F, 0.0F, 0.0F },                // pos_ki
  { 0.0F, 0.0F, 0.0F },                // vel_kd
  { 0.0F, 0.0F, 0.0F },                // center_of_mass
  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },// inertia_matrix
  0.0F                                 // mass
};

const env_msg ctl_controller0_rtZenv_msg = { { 0.0F, 0.0F, 0.0F },// P_B_ISS_ISS 
  { 0.0F, 0.0F, 0.0F },                // V_B_ISS_ISS
  { 0.0F, 0.0F, 0.0F },                // A_B_ISS_ISS
  { 0.0F, 0.0F, 0.0F },                // A_B_ISS_B
  { 0.0F, 0.0F, 0.0F },                // A_B_ECI_B
  { 0.0F, 0.0F, 0.0F, 0.0F },          // Q_ISS2B
  { 0.0F, 0.0F, 0.0F },                // omega_B_ISS_B
  { 0.0F, 0.0F, 0.0F },                // omega_B_ECI_B
  { 0.0F, 0.0F, 0.0F },                // alpha_B_ISS_B
  { 0.0F, 0.0F, 0.0F },                // fan_torques_B
  { 0.0F, 0.0F, 0.0F }                 // fan_forces_B
};

const ex_time_msg ctl_controller0_rtZex_time_msg = { 0U,// timestamp_sec
  0U                                   // timestamp_nsec
};

const kfl_msg ctl_controller0_rtZkfl_msg = { { 0.0F, 0.0F, 0.0F, 0.0F },// quat_ISS2B 
  { 0.0F, 0.0F, 0.0F },                // omega_B_ISS_B
  { 0.0F, 0.0F, 0.0F },                // gyro_bias
  { 0.0F, 0.0F, 0.0F },                // V_B_ISS_ISS
  { 0.0F, 0.0F, 0.0F },                // A_B_ISS_ISS
  { 0.0F, 0.0F, 0.0F },                // accel_bias
  { 0.0F, 0.0F, 0.0F },                // P_B_ISS_ISS
  0U,                                  // confidence
  0U,                                  // aug_state_enum
  { 0.0F, 0.0F, 0.0F, 0.0F },          // ml_quat_ISS2cam
  { 0.0F, 0.0F, 0.0F },                // ml_P_cam_ISS_ISS
  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },// of_quat_ISS2cam 
  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },// of_P_cam_ISS_ISS
  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F,
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F
  },                                   // cov_diag
  0U,                                  // kfl_status
  0U,                                  // update_OF_tracks_cnt
  0U,                                  // update_ML_features_cnt
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0 },         // of_mahal_distance
  { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0 },         // ml_mahal_distance
  { 0.0F, 0.0F, 0.0F },                // hr_P_hr_ISS_ISS
  { 0.0F, 0.0F, 0.0F, 0.0F },          // hr_quat_ISS2hr
  { 0.0F, 0.0F, 0.0F }                 // P_EST_ISS_ISS
};

//
// Output and update for action system:
//    '<S44>/Normalize'
//    '<S63>/Normalize'
//    '<S77>/Normalize'
//
void ctl_controller0_Normalize(const real32_T rtu_q_in[4], real32_T
  rty_positive_scalar_q[4], P_Normalize_ctl_controller0_T *localP)
{
  // Product: '<S50>/Product' incorporates:
  //   Constant: '<S50>/Constant1'
  //   DataTypeConversion: '<S52>/Conversion'

  rty_positive_scalar_q[0] = rtu_q_in[0] * (real32_T)localP->Constant1_Value;
  rty_positive_scalar_q[1] = rtu_q_in[1] * (real32_T)localP->Constant1_Value;
  rty_positive_scalar_q[2] = rtu_q_in[2] * (real32_T)localP->Constant1_Value;
  rty_positive_scalar_q[3] = rtu_q_in[3] * (real32_T)localP->Constant1_Value;
}

//
// Termination for action system:
//    '<S44>/Normalize'
//    '<S63>/Normalize'
//    '<S77>/Normalize'
//
void ctl_controller0_Normalize_Term(void)
{
}

//
// Output and update for action system:
//    '<S51>/Normalize'
//    '<S66>/Normalize'
//    '<S84>/Normalize'
//
void ctl_controller0_Normalize_e(const real32_T rtu_Vec[4], real32_T
  rtu_Magnitude, real32_T rty_Normalized_Vec[4])
{
  // Product: '<S54>/Divide'
  rty_Normalized_Vec[0] = rtu_Vec[0] / rtu_Magnitude;
  rty_Normalized_Vec[1] = rtu_Vec[1] / rtu_Magnitude;
  rty_Normalized_Vec[2] = rtu_Vec[2] / rtu_Magnitude;
  rty_Normalized_Vec[3] = rtu_Vec[3] / rtu_Magnitude;
}

//
// Termination for action system:
//    '<S51>/Normalize'
//    '<S66>/Normalize'
//    '<S84>/Normalize'
//
void ctl_controller_Normalize_p_Term(void)
{
}

//
// System initialize for iterator system:
//    '<S5>/For Each Subsystem'
//    '<S5>/For Each Subsystem1'
//
void ctl_contr_ForEachSubsystem_Init(int32_T NumIters,
  DW_ForEachSubsystem_ctl_contr_T localDW[3], P_ForEachSubsystem_ctl_contro_T
  *localP)
{
  // local scratch DWork variables
  int32_T ForEach_itr;
  for (ForEach_itr = 0; ForEach_itr < NumIters; ForEach_itr++) {
    // InitializeConditions for DiscreteTransferFcn: '<S91>/3 Hz Low Pass'
    localDW[ForEach_itr].CoreSubsys.uHzLowPass_states =
      localP->CoreSubsys.uHzLowPass_InitialStates;
  }
}

//
// Start for iterator system:
//    '<S5>/For Each Subsystem'
//    '<S5>/For Each Subsystem1'
//
void ctl_cont_ForEachSubsystem_Start(int32_T NumIters,
  DW_ForEachSubsystem_ctl_contr_T localDW[3])
{
  // local scratch DWork variables
  int32_T ForEach_itr;
  for (ForEach_itr = 0; ForEach_itr < NumIters; ForEach_itr++) {
    localDW[ForEach_itr].CoreSubsys.uHzLowPass_states = 0.0F;
  }
}

//
// Output and update for iterator system:
//    '<S5>/For Each Subsystem'
//    '<S5>/For Each Subsystem1'
//
void ctl_controller_ForEachSubsystem(int32_T NumIters, const real32_T rtu_X[3],
  real32_T rty_Y[3], DW_ForEachSubsystem_ctl_contr_T localDW[3],
  P_ForEachSubsystem_ctl_contro_T *localP, real32_T rtp_filt_enable)
{
  // local scratch DWork variables
  int32_T ForEach_itr;
  real32_T rtb_uHzLowPass;
  real32_T uHzLowPass_tmp;

  // Outputs for Iterator SubSystem: '<S5>/For Each Subsystem' incorporates:
  //   ForEach: '<S91>/For Each'

  for (ForEach_itr = 0; ForEach_itr < NumIters; ForEach_itr++) {
    // DiscreteTransferFcn: '<S91>/3 Hz Low Pass' incorporates:
    //   ForEachSliceSelector: '<S91>/ImpSel_InsertedFor_X_at_outport_0'

    uHzLowPass_tmp = (rtu_X[ForEach_itr] - localP->
                      CoreSubsys.uHzLowPass_DenCoef[1] * localDW[ForEach_itr].
                      CoreSubsys.uHzLowPass_states) /
      localP->CoreSubsys.uHzLowPass_DenCoef[0];
    rtb_uHzLowPass = localP->CoreSubsys.uHzLowPass_NumCoef[0] * uHzLowPass_tmp +
      localP->CoreSubsys.uHzLowPass_NumCoef[1] * localDW[ForEach_itr].
      CoreSubsys.uHzLowPass_states;

    // Update for DiscreteTransferFcn: '<S91>/3 Hz Low Pass'
    localDW[ForEach_itr].CoreSubsys.uHzLowPass_states = uHzLowPass_tmp;

    // Switch: '<S91>/Switch' incorporates:
    //   Constant: '<S91>/Constant'

    if (rtp_filt_enable != 0.0F) {
      // ForEachSliceAssignment: '<S91>/ImpAsg_InsertedFor_Y_at_inport_0'
      rty_Y[ForEach_itr] = rtb_uHzLowPass;
    } else {
      // ForEachSliceAssignment: '<S91>/ImpAsg_InsertedFor_Y_at_inport_0' incorporates:
      //   ForEachSliceSelector: '<S91>/ImpSel_InsertedFor_X_at_outport_0'

      rty_Y[ForEach_itr] = rtu_X[ForEach_itr];
    }

    // End of Switch: '<S91>/Switch'
  }

  // End of Outputs for SubSystem: '<S5>/For Each Subsystem'
}

//
// Termination for iterator system:
//    '<S5>/For Each Subsystem'
//    '<S5>/For Each Subsystem1'
//
void ctl_contr_ForEachSubsystem_Term(void)
{
}

// Model step function
void ctl_controller0_step(RT_MODEL_ctl_controller0_T *const ctl_controller0_M,
  kfl_msg *ctl_controller0_U_kfl_msg_l, cmc_msg *ctl_controller0_U_cmc_msg_f,
  ex_time_msg *ctl_controller0_U_ex_time, env_msg *ctl_controller0_U_env_msg_h,
  cmd_msg *ctl_controller0_Y_cmd_msg_c, ctl_msg *ctl_controller0_Y_ctl_msg_o)
{
  P_ctl_controller0_T *ctl_controller0_P = ((P_ctl_controller0_T *)
    ctl_controller0_M->defaultParam);
  DW_ctl_controller0_T *ctl_controller0_DW = ((DW_ctl_controller0_T *)
    ctl_controller0_M->dwork);

  // local block i/o variables
  real32_T rtb_ImpAsg_InsertedFor_Y_at_inp[3];
  real32_T rtb_ImpAsg_InsertedFor_Y_at_i_d[3];
  real32_T normA;
  real32_T b_s;
  int32_T eint;
  static const real32_T theta[3] = { 0.425873F, 1.8801527F, 3.92572474F };

  real32_T rtb_Assignment_h[9];
  real32_T rtb_Sqrt_bt;
  real32_T rtb_TSamp[4];
  real32_T rtb_Product[3];
  real32_T rtb_SumofElements;
  real32_T rtb_SumofElements1;
  boolean_T rtb_LogicalOperator2_c;
  uint8_T rtb_Switch2;
  real32_T rtb_VectorConcatenate[16];
  real32_T rtb_VectorConcatenate_m[16];
  real32_T rtb_Product1_m[4];
  boolean_T rtb_LogicalOperator2;
  real32_T rtb_Gain_ci[3];
  real32_T rtb_Assignment_l[9];
  real32_T rtb_Divide_f[3];
  boolean_T rtb_Compare_j;
  real32_T rtb_Product_f[4];
  boolean_T rtb_Compare_a;
  real32_T rtb_Product3_n[3];
  uint32_T rtb_Switch_h_timestamp_sec;
  uint32_T rtb_Switch_h_timestamp_nsec;
  real32_T rtb_Switch_h_A_B_ISS_ISS[3];
  real32_T rtb_Merge[4];
  real32_T rtb_Merge_k[4];
  real32_T rtb_Merge_az[4];
  real32_T rtb_y[16];
  real32_T rtb_Diff[3];
  real32_T rtb_Product3[16];
  int32_T i;
  real32_T rtb_Product3_0[16];
  real32_T rtb_Assignment_b[12];
  real32_T tmp[12];
  int32_T i_0;
  real32_T rtb_VectorConcatenate_p[16];
  real32_T tmp_0[9];
  real32_T rtb_Merge_j[9];
  real32_T rtb_Assignment_b_0[9];
  real32_T rtb_Assignment_b_1[3];
  real32_T tmp_1[9];
  real32_T tmp_2[9];
  real32_T tmp_3[9];
  real32_T tmp_4[9];
  real32_T rtb_Product_md;
  real32_T rtb_Diff_e;
  real32_T rtb_SumA21_f;
  real32_T rtb_Switch_h_alpha_B_ISS_B_idx_;
  real32_T rtb_Switch_h_alpha_B_ISS_B_id_0;
  real32_T rtb_Switch_h_alpha_B_ISS_B_id_1;
  real32_T rtb_Sum4_o_idx_2;
  real32_T rtb_Sum4_o_idx_1;
  real32_T rtb_Sum4_o_idx_0;
  real32_T rtb_Switch_omega_B_ISS_B_idx_0;
  real32_T rtb_Switch_omega_B_ISS_B_idx_1;
  real32_T rtb_Switch_omega_B_ISS_B_idx_2;
  real32_T rtb_Sum3_k_idx_0;
  real32_T rtb_Switch_V_B_ISS_ISS_idx_0;
  real32_T rtb_Switch_P_B_ISS_ISS_idx_0;
  real32_T rtb_Switch_V_B_ISS_ISS_idx_1;
  real32_T rtb_Switch_P_B_ISS_ISS_idx_1;
  real32_T rtb_Switch_V_B_ISS_ISS_idx_2;
  real32_T rtb_Switch_P_B_ISS_ISS_idx_2;
  real32_T rtb_Sum3_k_idx_1;
  real32_T rtb_Sum3_k_idx_2;
  real32_T rtb_SumA21_d_idx_0;
  real32_T rtb_SumA21_d_idx_1;
  real32_T rtb_Gain1_j_idx_0;
  real32_T rtb_Gain1_j_idx_1;
  real32_T rtb_Gain1_j_idx_2;
  real32_T rtb_BusCreator1_traj_omega_idx_;
  real32_T rtb_Switch8_idx_0;
  real32_T rtb_BusCreator1_traj_pos_idx_1;
  real32_T rtb_BusCreator1_traj_vel_idx_1;
  real32_T rtb_BusCreator1_traj_omega_id_0;
  real32_T rtb_Switch8_idx_1;
  real32_T rtb_Switch_c_idx_0;
  real32_T rtb_Switch_c_idx_1;
  real32_T rtb_Switch_c_idx_2;
  real32_T rtb_Sum2_idx_0;
  real32_T rtb_Sum2_idx_1;
  real32_T rtb_Sum2_idx_2;
  real32_T rtb_LogicalOperator2_idx_2;
  real32_T rtb_LogicalOperator2_idx_3;
  real32_T rtb_LogicalOperator2_0;
  boolean_T exitg1;
  boolean_T exitg2;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  // DataTypeConversion: '<S95>/Conversion' incorporates:
  //   Constant: '<S94>/Constant2'

  for (i = 0; i < 9; i++) {
    rtb_Assignment_h[i] = (real32_T)ctl_controller0_P->Constant2_Value[i];
  }

  // End of DataTypeConversion: '<S95>/Conversion'

  // Assignment: '<S94>/Assignment' incorporates:
  //   Inport: '<Root>/env_msg'

  rtb_Assignment_h[0] = ctl_controller0_U_env_msg_h->Q_ISS2B[3];
  rtb_Assignment_h[4] = ctl_controller0_U_env_msg_h->Q_ISS2B[3];
  rtb_Assignment_h[8] = ctl_controller0_U_env_msg_h->Q_ISS2B[3];

  // SampleTimeMath: '<S93>/TSamp' incorporates:
  //   Inport: '<Root>/env_msg'
  //
  //  About '<S93>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp[0] = ctl_controller0_U_env_msg_h->Q_ISS2B[0] *
    ctl_controller0_P->TSamp_WtEt;
  rtb_TSamp[1] = ctl_controller0_U_env_msg_h->Q_ISS2B[1] *
    ctl_controller0_P->TSamp_WtEt;
  rtb_TSamp[2] = ctl_controller0_U_env_msg_h->Q_ISS2B[2] *
    ctl_controller0_P->TSamp_WtEt;
  rtb_TSamp[3] = ctl_controller0_U_env_msg_h->Q_ISS2B[3] *
    ctl_controller0_P->TSamp_WtEt;

  // Sum: '<S94>/Sum2' incorporates:
  //   Constant: '<S96>/Constant3'
  //   DataTypeConversion: '<S97>/Conversion'
  //   Gain: '<S96>/Gain'
  //   Gain: '<S96>/Gain1'
  //   Gain: '<S96>/Gain2'
  //   Inport: '<Root>/env_msg'

  rtb_Assignment_l[0] = (real32_T)ctl_controller0_P->Constant3_Value;
  rtb_Assignment_l[1] = ctl_controller0_U_env_msg_h->Q_ISS2B[2];
  rtb_Assignment_l[2] = ctl_controller0_P->Gain_Gain_p *
    ctl_controller0_U_env_msg_h->Q_ISS2B[1];
  rtb_Assignment_l[3] = ctl_controller0_P->Gain1_Gain_e *
    ctl_controller0_U_env_msg_h->Q_ISS2B[2];
  rtb_Assignment_l[4] = (real32_T)ctl_controller0_P->Constant3_Value;
  rtb_Assignment_l[5] = ctl_controller0_U_env_msg_h->Q_ISS2B[0];
  rtb_Assignment_l[6] = ctl_controller0_U_env_msg_h->Q_ISS2B[1];
  rtb_Assignment_l[7] = ctl_controller0_P->Gain2_Gain_k *
    ctl_controller0_U_env_msg_h->Q_ISS2B[0];
  rtb_Assignment_l[8] = (real32_T)ctl_controller0_P->Constant3_Value;

  // Math: '<S89>/Math Function' incorporates:
  //   Concatenate: '<S94>/Matrix Concatenate'
  //   Gain: '<S89>/Gain'
  //   Gain: '<S94>/Gain1'
  //   Inport: '<Root>/env_msg'
  //   Sum: '<S94>/Sum2'

  for (i = 0; i < 3; i++) {
    rtb_Assignment_b[(int32_T)(3 * i)] = rtb_Assignment_h[i] +
      rtb_Assignment_l[i];
    rtb_Assignment_b[(int32_T)(1 + (int32_T)(3 * i))] = rtb_Assignment_h
      [(int32_T)(i + 3)] + rtb_Assignment_l[(int32_T)(i + 3)];
    rtb_Assignment_b[(int32_T)(2 + (int32_T)(3 * i))] = rtb_Assignment_h
      [(int32_T)(i + 6)] + rtb_Assignment_l[(int32_T)(i + 6)];
    rtb_Assignment_b[(int32_T)(9 + i)] = ctl_controller0_P->Gain1_Gain_o *
      ctl_controller0_U_env_msg_h->Q_ISS2B[i];
  }

  // End of Math: '<S89>/Math Function'
  for (i = 0; i < 4; i++) {
    // Gain: '<S89>/Gain' incorporates:
    //   Product: '<S89>/Product'

    tmp[(int32_T)(3 * i)] = rtb_Assignment_b[(int32_T)(3 * i)] *
      ctl_controller0_P->Gain_Gain_m;
    tmp[(int32_T)(1 + (int32_T)(3 * i))] = rtb_Assignment_b[(int32_T)((int32_T)
      (3 * i) + 1)] * ctl_controller0_P->Gain_Gain_m;
    tmp[(int32_T)(2 + (int32_T)(3 * i))] = rtb_Assignment_b[(int32_T)((int32_T)
      (3 * i) + 2)] * ctl_controller0_P->Gain_Gain_m;

    // Sum: '<S93>/Diff' incorporates:
    //   Product: '<S89>/Product'
    //   UnitDelay: '<S93>/UD'

    rtb_Product1_m[i] = rtb_TSamp[i] - ctl_controller0_DW->UD_DSTATE[i];
  }

  // Product: '<S89>/Product'
  for (i = 0; i < 3; i++) {
    rtb_Product_md = tmp[(int32_T)(i + 9)] * rtb_Product1_m[3] + (tmp[(int32_T)
      (i + 6)] * rtb_Product1_m[2] + (tmp[(int32_T)(i + 3)] * rtb_Product1_m[1]
      + tmp[i] * rtb_Product1_m[0]));
    rtb_Product[i] = rtb_Product_md;
  }

  // Outputs for Iterator SubSystem: '<S5>/For Each Subsystem'
  ctl_controller_ForEachSubsystem(3, rtb_Product,
    rtb_ImpAsg_InsertedFor_Y_at_i_d, ctl_controller0_DW->ForEachSubsystem,
    (P_ForEachSubsystem_ctl_contro_T *)&ctl_controller0_P->ForEachSubsystem,
    ctl_controller0_P->tun_truth_q_omega_filt_enable);

  // End of Outputs for SubSystem: '<S5>/For Each Subsystem'

  // SampleTimeMath: '<S90>/TSamp' incorporates:
  //   Inport: '<Root>/env_msg'
  //
  //  About '<S90>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_Product_md = ctl_controller0_U_env_msg_h->P_B_ISS_ISS[0] *
    ctl_controller0_P->TSamp_WtEt_k;

  // Sum: '<S90>/Diff' incorporates:
  //   UnitDelay: '<S90>/UD'

  rtb_Diff[0] = rtb_Product_md - ctl_controller0_DW->UD_DSTATE_e[0];

  // SampleTimeMath: '<S90>/TSamp' incorporates:
  //   Inport: '<Root>/env_msg'
  //
  //  About '<S90>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_Product[0] = rtb_Product_md;
  rtb_Product_md = ctl_controller0_U_env_msg_h->P_B_ISS_ISS[1] *
    ctl_controller0_P->TSamp_WtEt_k;

  // Sum: '<S90>/Diff' incorporates:
  //   UnitDelay: '<S90>/UD'

  rtb_Diff[1] = rtb_Product_md - ctl_controller0_DW->UD_DSTATE_e[1];

  // SampleTimeMath: '<S90>/TSamp' incorporates:
  //   Inport: '<Root>/env_msg'
  //
  //  About '<S90>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_Product[1] = rtb_Product_md;
  rtb_Product_md = ctl_controller0_U_env_msg_h->P_B_ISS_ISS[2] *
    ctl_controller0_P->TSamp_WtEt_k;

  // Sum: '<S90>/Diff' incorporates:
  //   UnitDelay: '<S90>/UD'

  rtb_Diff[2] = rtb_Product_md - ctl_controller0_DW->UD_DSTATE_e[2];

  // Outputs for Iterator SubSystem: '<S5>/For Each Subsystem1'
  ctl_controller_ForEachSubsystem(3, rtb_Diff, rtb_ImpAsg_InsertedFor_Y_at_inp,
    ctl_controller0_DW->ForEachSubsystem1, (P_ForEachSubsystem_ctl_contro_T *)
    &ctl_controller0_P->ForEachSubsystem1,
    ctl_controller0_P->tun_truth_velocity_filt_enable);

  // End of Outputs for SubSystem: '<S5>/For Each Subsystem1'

  // Switch: '<S1>/Switch' incorporates:
  //   BusCreator: '<S5>/bus_creator1'
  //   Constant: '<S1>/Constant'
  //   Constant: '<S5>/constant39'
  //   Inport: '<Root>/env_msg'
  //   Inport: '<Root>/kfl_msg'

  if ((int32_T)ctl_controller0_P->tun_debug_ctl_use_truth != 0) {
    rtb_Merge_az[0] = ctl_controller0_U_env_msg_h->Q_ISS2B[0];
    rtb_Merge_az[1] = ctl_controller0_U_env_msg_h->Q_ISS2B[1];
    rtb_Merge_az[2] = ctl_controller0_U_env_msg_h->Q_ISS2B[2];
    rtb_Merge_az[3] = ctl_controller0_U_env_msg_h->Q_ISS2B[3];
    rtb_Switch_omega_B_ISS_B_idx_0 = rtb_ImpAsg_InsertedFor_Y_at_i_d[0];
    rtb_Switch_V_B_ISS_ISS_idx_0 = rtb_ImpAsg_InsertedFor_Y_at_inp[0];
    rtb_Switch_P_B_ISS_ISS_idx_0 = ctl_controller0_U_env_msg_h->P_B_ISS_ISS[0];
    rtb_Switch_omega_B_ISS_B_idx_1 = rtb_ImpAsg_InsertedFor_Y_at_i_d[1];
    rtb_Switch_V_B_ISS_ISS_idx_1 = rtb_ImpAsg_InsertedFor_Y_at_inp[1];
    rtb_Switch_P_B_ISS_ISS_idx_1 = ctl_controller0_U_env_msg_h->P_B_ISS_ISS[1];
    rtb_Switch_omega_B_ISS_B_idx_2 = rtb_ImpAsg_InsertedFor_Y_at_i_d[2];
    rtb_Switch_V_B_ISS_ISS_idx_2 = rtb_ImpAsg_InsertedFor_Y_at_inp[2];
    rtb_Switch_P_B_ISS_ISS_idx_2 = ctl_controller0_U_env_msg_h->P_B_ISS_ISS[2];
    rtb_Switch2 = ctl_controller0_P->constant39_Value;
  } else {
    rtb_Merge_az[0] = ctl_controller0_U_kfl_msg_l->quat_ISS2B[0];
    rtb_Merge_az[1] = ctl_controller0_U_kfl_msg_l->quat_ISS2B[1];
    rtb_Merge_az[2] = ctl_controller0_U_kfl_msg_l->quat_ISS2B[2];
    rtb_Merge_az[3] = ctl_controller0_U_kfl_msg_l->quat_ISS2B[3];
    rtb_Switch_omega_B_ISS_B_idx_0 = ctl_controller0_U_kfl_msg_l->omega_B_ISS_B
      [0];
    rtb_Switch_V_B_ISS_ISS_idx_0 = ctl_controller0_U_kfl_msg_l->V_B_ISS_ISS[0];
    rtb_Switch_P_B_ISS_ISS_idx_0 = ctl_controller0_U_kfl_msg_l->P_B_ISS_ISS[0];
    rtb_Switch_omega_B_ISS_B_idx_1 = ctl_controller0_U_kfl_msg_l->omega_B_ISS_B
      [1];
    rtb_Switch_V_B_ISS_ISS_idx_1 = ctl_controller0_U_kfl_msg_l->V_B_ISS_ISS[1];
    rtb_Switch_P_B_ISS_ISS_idx_1 = ctl_controller0_U_kfl_msg_l->P_B_ISS_ISS[1];
    rtb_Switch_omega_B_ISS_B_idx_2 = ctl_controller0_U_kfl_msg_l->omega_B_ISS_B
      [2];
    rtb_Switch_V_B_ISS_ISS_idx_2 = ctl_controller0_U_kfl_msg_l->V_B_ISS_ISS[2];
    rtb_Switch_P_B_ISS_ISS_idx_2 = ctl_controller0_U_kfl_msg_l->P_B_ISS_ISS[2];
    rtb_Switch2 = ctl_controller0_U_kfl_msg_l->confidence;
  }

  // End of Switch: '<S1>/Switch'

  // Sum: '<S10>/SumA21' incorporates:
  //   Delay: '<S10>/Delay11'
  //   Gain: '<S10>/a(2)(1)'
  //   Gain: '<S10>/s(1)'

  rtb_Diff_e = (real32_T)(ctl_controller0_P->s1_Gain * (real_T)
    rtb_Switch_V_B_ISS_ISS_idx_0) - (real32_T)(ctl_controller0_P->a21_Gain *
    (real_T)ctl_controller0_DW->Delay11_DSTATE[0]);

  // Sum: '<S10>/SumB21' incorporates:
  //   Delay: '<S10>/Delay11'

  rtb_Sum3_k_idx_2 = rtb_Diff_e + ctl_controller0_DW->Delay11_DSTATE[0];

  // Math: '<S2>/Math Function'
  rtb_Sum3_k_idx_2 *= rtb_Sum3_k_idx_2;

  // Sum: '<S10>/SumA21'
  rtb_Diff[0] = rtb_Diff_e;

  // Sum: '<S10>/SumB21'
  rtb_Sum3_k_idx_0 = rtb_Sum3_k_idx_2;

  // Sum: '<S10>/SumA21' incorporates:
  //   Delay: '<S10>/Delay11'
  //   Gain: '<S10>/a(2)(1)'
  //   Gain: '<S10>/s(1)'

  rtb_Diff_e = (real32_T)(ctl_controller0_P->s1_Gain * (real_T)
    rtb_Switch_V_B_ISS_ISS_idx_1) - (real32_T)(ctl_controller0_P->a21_Gain *
    (real_T)ctl_controller0_DW->Delay11_DSTATE[1]);

  // Sum: '<S10>/SumB21' incorporates:
  //   Delay: '<S10>/Delay11'

  rtb_Sum3_k_idx_2 = rtb_Diff_e + ctl_controller0_DW->Delay11_DSTATE[1];

  // Math: '<S2>/Math Function'
  rtb_Sum3_k_idx_2 *= rtb_Sum3_k_idx_2;

  // Sum: '<S10>/SumA21'
  rtb_Diff[1] = rtb_Diff_e;

  // Sum: '<S10>/SumB21'
  rtb_Sum3_k_idx_1 = rtb_Sum3_k_idx_2;

  // Sum: '<S10>/SumA21' incorporates:
  //   Delay: '<S10>/Delay11'
  //   Gain: '<S10>/a(2)(1)'
  //   Gain: '<S10>/s(1)'

  rtb_Diff_e = (real32_T)(ctl_controller0_P->s1_Gain * (real_T)
    rtb_Switch_V_B_ISS_ISS_idx_2) - (real32_T)(ctl_controller0_P->a21_Gain *
    (real_T)ctl_controller0_DW->Delay11_DSTATE[2]);

  // Sum: '<S10>/SumB21' incorporates:
  //   Delay: '<S10>/Delay11'

  rtb_Sum3_k_idx_2 = rtb_Diff_e + ctl_controller0_DW->Delay11_DSTATE[2];

  // Math: '<S2>/Math Function'
  rtb_Sum3_k_idx_2 *= rtb_Sum3_k_idx_2;

  // Sum: '<S2>/Sum of Elements'
  rtb_SumofElements = (rtb_Sum3_k_idx_0 + rtb_Sum3_k_idx_1) + rtb_Sum3_k_idx_2;

  // Sum: '<S11>/SumA21' incorporates:
  //   Delay: '<S11>/Delay11'
  //   Gain: '<S11>/a(2)(1)'
  //   Gain: '<S11>/s(1)'

  rtb_SumA21_f = (real32_T)(ctl_controller0_P->s1_Gain_b * (real_T)
    rtb_Switch_omega_B_ISS_B_idx_0) - (real32_T)(ctl_controller0_P->a21_Gain_l *
    (real_T)ctl_controller0_DW->Delay11_DSTATE_i[0]);

  // Sum: '<S11>/SumB21' incorporates:
  //   Delay: '<S11>/Delay11'

  rtb_Sum3_k_idx_2 = rtb_SumA21_f + ctl_controller0_DW->Delay11_DSTATE_i[0];

  // Math: '<S2>/Math Function1'
  rtb_Sum3_k_idx_2 *= rtb_Sum3_k_idx_2;

  // Sum: '<S11>/SumA21'
  rtb_SumA21_d_idx_0 = rtb_SumA21_f;

  // Sum: '<S11>/SumB21'
  rtb_Sum3_k_idx_0 = rtb_Sum3_k_idx_2;

  // Sum: '<S11>/SumA21' incorporates:
  //   Delay: '<S11>/Delay11'
  //   Gain: '<S11>/a(2)(1)'
  //   Gain: '<S11>/s(1)'

  rtb_SumA21_f = (real32_T)(ctl_controller0_P->s1_Gain_b * (real_T)
    rtb_Switch_omega_B_ISS_B_idx_1) - (real32_T)(ctl_controller0_P->a21_Gain_l *
    (real_T)ctl_controller0_DW->Delay11_DSTATE_i[1]);

  // Sum: '<S11>/SumB21' incorporates:
  //   Delay: '<S11>/Delay11'

  rtb_Sum3_k_idx_2 = rtb_SumA21_f + ctl_controller0_DW->Delay11_DSTATE_i[1];

  // Math: '<S2>/Math Function1'
  rtb_Sum3_k_idx_2 *= rtb_Sum3_k_idx_2;

  // Sum: '<S11>/SumA21'
  rtb_SumA21_d_idx_1 = rtb_SumA21_f;

  // Sum: '<S11>/SumB21'
  rtb_Sum3_k_idx_1 = rtb_Sum3_k_idx_2;

  // Sum: '<S11>/SumA21' incorporates:
  //   Delay: '<S11>/Delay11'
  //   Gain: '<S11>/a(2)(1)'
  //   Gain: '<S11>/s(1)'

  rtb_SumA21_f = (real32_T)(ctl_controller0_P->s1_Gain_b * (real_T)
    rtb_Switch_omega_B_ISS_B_idx_2) - (real32_T)(ctl_controller0_P->a21_Gain_l *
    (real_T)ctl_controller0_DW->Delay11_DSTATE_i[2]);

  // Sum: '<S11>/SumB21' incorporates:
  //   Delay: '<S11>/Delay11'

  rtb_Sum3_k_idx_2 = rtb_SumA21_f + ctl_controller0_DW->Delay11_DSTATE_i[2];

  // Math: '<S2>/Math Function1'
  rtb_Sum3_k_idx_2 *= rtb_Sum3_k_idx_2;

  // Sum: '<S2>/Sum of Elements1'
  rtb_SumofElements1 = (rtb_Sum3_k_idx_0 + rtb_Sum3_k_idx_1) + rtb_Sum3_k_idx_2;

  // Logic: '<S56>/Logical Operator1' incorporates:
  //   Inport: '<Root>/cmc_msg'
  //   Inport: '<Root>/ex_time'
  //   Logic: '<S56>/Logical Operator'
  //   RelationalOperator: '<S56>/Relational Operator'
  //   RelationalOperator: '<S56>/Relational Operator1'
  //   RelationalOperator: '<S56>/Relational Operator2'
  //   SignalConversion: '<S1>/Signal Conversion'

  rtb_LogicalOperator2_c =
    ((ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.timestamp_sec >
      ctl_controller0_U_ex_time->timestamp_sec) ||
     ((ctl_controller0_U_ex_time->timestamp_sec ==
       ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.timestamp_sec) &&
      (ctl_controller0_U_ex_time->timestamp_nsec <
       ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.timestamp_nsec)));

  // Switch: '<S56>/Switch' incorporates:
  //   Inport: '<Root>/cmc_msg'
  //   SignalConversion: '<S1>/Signal Conversion'

  if (rtb_LogicalOperator2_c) {
    rtb_Switch_h_timestamp_sec =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.timestamp_sec;
    rtb_Switch_h_timestamp_nsec =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.timestamp_nsec;
    rtb_Gain1_j_idx_0 = ctl_controller0_U_cmc_msg_f->
      cmc_state_cmd_a.P_B_ISS_ISS[0];
    rtb_Sum3_k_idx_0 = ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.V_B_ISS_ISS
      [0];
    rtb_Switch_h_A_B_ISS_ISS[0] =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.A_B_ISS_ISS[0];
    rtb_Gain1_j_idx_1 = ctl_controller0_U_cmc_msg_f->
      cmc_state_cmd_a.P_B_ISS_ISS[1];
    rtb_Sum3_k_idx_1 = ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.V_B_ISS_ISS
      [1];
    rtb_Switch_h_A_B_ISS_ISS[1] =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.A_B_ISS_ISS[1];
    rtb_Gain1_j_idx_2 = ctl_controller0_U_cmc_msg_f->
      cmc_state_cmd_a.P_B_ISS_ISS[2];
    rtb_Sum3_k_idx_2 = ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.V_B_ISS_ISS
      [2];
    rtb_Switch_h_A_B_ISS_ISS[2] =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.A_B_ISS_ISS[2];
    rtb_Merge[0] = ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.quat_ISS2B[0];
    rtb_Merge[1] = ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.quat_ISS2B[1];
    rtb_Merge[2] = ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.quat_ISS2B[2];
    rtb_Merge[3] = ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.quat_ISS2B[3];
    rtb_Sum4_o_idx_0 =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.omega_B_ISS_B[0];
    rtb_Switch_h_alpha_B_ISS_B_id_1 =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.alpha_B_ISS_B[0];
    rtb_Sum4_o_idx_1 =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.omega_B_ISS_B[1];
    rtb_Switch_h_alpha_B_ISS_B_id_0 =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.alpha_B_ISS_B[1];
    rtb_Sum4_o_idx_2 =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.omega_B_ISS_B[2];
    rtb_Switch_h_alpha_B_ISS_B_idx_ =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_a.alpha_B_ISS_B[2];
  } else {
    rtb_Switch_h_timestamp_sec =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.timestamp_sec;
    rtb_Switch_h_timestamp_nsec =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.timestamp_nsec;
    rtb_Gain1_j_idx_0 = ctl_controller0_U_cmc_msg_f->
      cmc_state_cmd_b.P_B_ISS_ISS[0];
    rtb_Sum3_k_idx_0 = ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.V_B_ISS_ISS
      [0];
    rtb_Switch_h_A_B_ISS_ISS[0] =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.A_B_ISS_ISS[0];
    rtb_Gain1_j_idx_1 = ctl_controller0_U_cmc_msg_f->
      cmc_state_cmd_b.P_B_ISS_ISS[1];
    rtb_Sum3_k_idx_1 = ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.V_B_ISS_ISS
      [1];
    rtb_Switch_h_A_B_ISS_ISS[1] =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.A_B_ISS_ISS[1];
    rtb_Gain1_j_idx_2 = ctl_controller0_U_cmc_msg_f->
      cmc_state_cmd_b.P_B_ISS_ISS[2];
    rtb_Sum3_k_idx_2 = ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.V_B_ISS_ISS
      [2];
    rtb_Switch_h_A_B_ISS_ISS[2] =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.A_B_ISS_ISS[2];
    rtb_Merge[0] = ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.quat_ISS2B[0];
    rtb_Merge[1] = ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.quat_ISS2B[1];
    rtb_Merge[2] = ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.quat_ISS2B[2];
    rtb_Merge[3] = ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.quat_ISS2B[3];
    rtb_Sum4_o_idx_0 =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.omega_B_ISS_B[0];
    rtb_Switch_h_alpha_B_ISS_B_id_1 =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.alpha_B_ISS_B[0];
    rtb_Sum4_o_idx_1 =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.omega_B_ISS_B[1];
    rtb_Switch_h_alpha_B_ISS_B_id_0 =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.alpha_B_ISS_B[1];
    rtb_Sum4_o_idx_2 =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.omega_B_ISS_B[2];
    rtb_Switch_h_alpha_B_ISS_B_idx_ =
      ctl_controller0_U_cmc_msg_f->cmc_state_cmd_b.alpha_B_ISS_B[2];
  }

  // End of Switch: '<S56>/Switch'

  // Logic: '<S56>/Logical Operator2'
  rtb_LogicalOperator2_c = !rtb_LogicalOperator2_c;

  // Sum: '<S56>/Sum' incorporates:
  //   DataTypeConversion: '<S56>/Data Type Conversion4'
  //   Gain: '<S56>/Gain'
  //   Inport: '<Root>/ex_time'
  //   Sum: '<S56>/Subtract3'
  //   Sum: '<S56>/Subtract4'

  rtb_Sqrt_bt = (real32_T)(int32_T)((int32_T)
    ctl_controller0_U_ex_time->timestamp_nsec - (int32_T)
    rtb_Switch_h_timestamp_nsec) * ctl_controller0_P->Gain_Gain_h + (real32_T)
    (int32_T)((int32_T)ctl_controller0_U_ex_time->timestamp_sec - (int32_T)
              rtb_Switch_h_timestamp_sec);

  // Sum: '<S58>/Sum3' incorporates:
  //   Constant: '<S58>/Constant'
  //   Product: '<S58>/Product'
  //   Product: '<S58>/Product2'

  rtb_Gain1_j_idx_0 = (ctl_controller0_P->Constant_Value *
                       rtb_Switch_h_A_B_ISS_ISS[0] * rtb_Sqrt_bt * rtb_Sqrt_bt +
                       rtb_Gain1_j_idx_0) + rtb_Sum3_k_idx_0 * rtb_Sqrt_bt;

  // Sum: '<S58>/Sum1' incorporates:
  //   Product: '<S58>/Product1'
  //   Sum: '<S58>/Sum3'

  rtb_Sum3_k_idx_0 += rtb_Switch_h_A_B_ISS_ISS[0] * rtb_Sqrt_bt;

  // Sum: '<S58>/Sum3' incorporates:
  //   Constant: '<S58>/Constant'
  //   Product: '<S58>/Product'
  //   Product: '<S58>/Product2'

  rtb_Gain1_j_idx_1 = (ctl_controller0_P->Constant_Value *
                       rtb_Switch_h_A_B_ISS_ISS[1] * rtb_Sqrt_bt * rtb_Sqrt_bt +
                       rtb_Gain1_j_idx_1) + rtb_Sum3_k_idx_1 * rtb_Sqrt_bt;

  // Sum: '<S58>/Sum1' incorporates:
  //   Product: '<S58>/Product1'
  //   Sum: '<S58>/Sum3'

  rtb_Sum3_k_idx_1 += rtb_Switch_h_A_B_ISS_ISS[1] * rtb_Sqrt_bt;

  // Sum: '<S58>/Sum3' incorporates:
  //   Constant: '<S58>/Constant'
  //   Product: '<S58>/Product'
  //   Product: '<S58>/Product2'

  rtb_Gain1_j_idx_2 = (ctl_controller0_P->Constant_Value *
                       rtb_Switch_h_A_B_ISS_ISS[2] * rtb_Sqrt_bt * rtb_Sqrt_bt +
                       rtb_Gain1_j_idx_2) + rtb_Sum3_k_idx_2 * rtb_Sqrt_bt;

  // Sum: '<S58>/Sum1' incorporates:
  //   Product: '<S58>/Product1'
  //   Sum: '<S58>/Sum3'

  rtb_Sum3_k_idx_2 += rtb_Switch_h_A_B_ISS_ISS[2] * rtb_Sqrt_bt;

  // Constant: '<S62>/Constant3'
  rtb_VectorConcatenate[0] = ctl_controller0_P->Constant3_Value_dz;

  // Gain: '<S62>/Gain'
  rtb_VectorConcatenate[1] = ctl_controller0_P->Gain_Gain_n *
    rtb_Switch_h_alpha_B_ISS_B_idx_;

  // SignalConversion: '<S62>/ConcatBufferAtVector ConcatenateIn3'
  rtb_VectorConcatenate[2] = rtb_Switch_h_alpha_B_ISS_B_id_0;

  // Gain: '<S62>/Gain1'
  rtb_VectorConcatenate[3] = ctl_controller0_P->Gain1_Gain_b *
    rtb_Switch_h_alpha_B_ISS_B_id_1;

  // SignalConversion: '<S62>/ConcatBufferAtVector ConcatenateIn5'
  rtb_VectorConcatenate[4] = rtb_Switch_h_alpha_B_ISS_B_idx_;

  // Constant: '<S62>/Constant2'
  rtb_VectorConcatenate[5] = ctl_controller0_P->Constant2_Value_d;

  // Gain: '<S62>/Gain2'
  rtb_VectorConcatenate[6] = ctl_controller0_P->Gain2_Gain_kx *
    rtb_Switch_h_alpha_B_ISS_B_id_1;

  // Gain: '<S62>/Gain3'
  rtb_VectorConcatenate[7] = ctl_controller0_P->Gain3_Gain *
    rtb_Switch_h_alpha_B_ISS_B_id_0;

  // Gain: '<S62>/Gain4'
  rtb_VectorConcatenate[8] = ctl_controller0_P->Gain4_Gain *
    rtb_Switch_h_alpha_B_ISS_B_id_0;

  // SignalConversion: '<S62>/ConcatBufferAtVector ConcatenateIn10'
  rtb_VectorConcatenate[9] = rtb_Switch_h_alpha_B_ISS_B_id_1;

  // Constant: '<S62>/Constant1'
  rtb_VectorConcatenate[10] = ctl_controller0_P->Constant1_Value_l;

  // Gain: '<S62>/Gain5'
  rtb_VectorConcatenate[11] = ctl_controller0_P->Gain5_Gain *
    rtb_Switch_h_alpha_B_ISS_B_idx_;

  // SignalConversion: '<S62>/ConcatBufferAtVector ConcatenateIn13'
  rtb_VectorConcatenate[12] = rtb_Switch_h_alpha_B_ISS_B_id_1;

  // SignalConversion: '<S62>/ConcatBufferAtVector ConcatenateIn14'
  rtb_VectorConcatenate[13] = rtb_Switch_h_alpha_B_ISS_B_id_0;

  // SignalConversion: '<S62>/ConcatBufferAtVector ConcatenateIn15'
  rtb_VectorConcatenate[14] = rtb_Switch_h_alpha_B_ISS_B_idx_;

  // Constant: '<S62>/Constant'
  rtb_VectorConcatenate[15] = ctl_controller0_P->Constant_Value_g;

  // Constant: '<S61>/Constant3'
  rtb_VectorConcatenate_m[0] = ctl_controller0_P->Constant3_Value_f;

  // Gain: '<S61>/Gain'
  rtb_VectorConcatenate_m[1] = ctl_controller0_P->Gain_Gain_g * rtb_Sum4_o_idx_2;

  // SignalConversion: '<S61>/ConcatBufferAtVector ConcatenateIn3'
  rtb_VectorConcatenate_m[2] = rtb_Sum4_o_idx_1;

  // Gain: '<S61>/Gain1'
  rtb_VectorConcatenate_m[3] = ctl_controller0_P->Gain1_Gain_j *
    rtb_Sum4_o_idx_0;

  // SignalConversion: '<S61>/ConcatBufferAtVector ConcatenateIn5'
  rtb_VectorConcatenate_m[4] = rtb_Sum4_o_idx_2;

  // Constant: '<S61>/Constant2'
  rtb_VectorConcatenate_m[5] = ctl_controller0_P->Constant2_Value_gq;

  // Gain: '<S61>/Gain2'
  rtb_VectorConcatenate_m[6] = ctl_controller0_P->Gain2_Gain_c *
    rtb_Sum4_o_idx_0;

  // Gain: '<S61>/Gain3'
  rtb_VectorConcatenate_m[7] = ctl_controller0_P->Gain3_Gain_o *
    rtb_Sum4_o_idx_1;

  // Gain: '<S61>/Gain4'
  rtb_VectorConcatenate_m[8] = ctl_controller0_P->Gain4_Gain_c *
    rtb_Sum4_o_idx_1;

  // SignalConversion: '<S61>/ConcatBufferAtVector ConcatenateIn10'
  rtb_VectorConcatenate_m[9] = rtb_Sum4_o_idx_0;

  // Constant: '<S61>/Constant1'
  rtb_VectorConcatenate_m[10] = ctl_controller0_P->Constant1_Value_gk;

  // Gain: '<S61>/Gain5'
  rtb_VectorConcatenate_m[11] = ctl_controller0_P->Gain5_Gain_a *
    rtb_Sum4_o_idx_2;

  // SignalConversion: '<S61>/ConcatBufferAtVector ConcatenateIn13'
  rtb_VectorConcatenate_m[12] = rtb_Sum4_o_idx_0;

  // SignalConversion: '<S61>/ConcatBufferAtVector ConcatenateIn14'
  rtb_VectorConcatenate_m[13] = rtb_Sum4_o_idx_1;

  // SignalConversion: '<S61>/ConcatBufferAtVector ConcatenateIn15'
  rtb_VectorConcatenate_m[14] = rtb_Sum4_o_idx_2;

  // Constant: '<S61>/Constant'
  rtb_VectorConcatenate_m[15] = ctl_controller0_P->Constant_Value_m;

  // Product: '<S57>/Product3' incorporates:
  //   Constant: '<S57>/Constant1'
  //   Constant: '<S57>/Constant3'
  //   Product: '<S57>/Product'
  //   Sum: '<S57>/Add'

  for (i = 0; i < 16; i++) {
    rtb_Product3[i] = (ctl_controller0_P->Constant3_Value_d *
                       rtb_VectorConcatenate[i] * rtb_Sqrt_bt +
                       rtb_VectorConcatenate_m[i]) *
      ctl_controller0_P->Constant1_Value_g * rtb_Sqrt_bt;
  }

  // End of Product: '<S57>/Product3'

  // MATLAB Function: '<S57>/MATLAB Function'
  // MATLAB Function 'cmd_command_shaper/generate_cmd_attitude/MATLAB Function': '<S60>:1' 
  // '<S60>:1:4'
  normA = 0.0F;
  i = 0;
  exitg2 = false;
  while ((!exitg2) && (i < 4)) {
    b_s = (((real32_T)fabs((real_T)rtb_Product3[(int32_T)((int32_T)(i << 2) + 1)])
            + (real32_T)fabs((real_T)rtb_Product3[(int32_T)(i << 2)])) +
           (real32_T)fabs((real_T)rtb_Product3[(int32_T)((int32_T)(i << 2) + 2)]))
      + (real32_T)fabs((real_T)rtb_Product3[(int32_T)((int32_T)(i << 2) + 3)]);
    if (rtIsNaNF(b_s)) {
      normA = (rtNaNF);
      exitg2 = true;
    } else {
      if (b_s > normA) {
        normA = b_s;
      }

      i++;
    }
  }

  if (normA <= 3.92572474F) {
    eint = 0;
    exitg1 = false;
    while ((!exitg1) && (eint < 3)) {
      if (normA <= theta[eint]) {
        mglnkfkfmglfjekn_PadeApproximantOfDegree(rtb_Product3, (uint8_T)(int32_T)
          ((int32_T)(eint << 1) + 3), rtb_y);
        exitg1 = true;
      } else {
        eint++;
      }
    }
  } else {
    b_s = normA / 3.92572474F;
    if ((!rtIsInfF(b_s)) && (!rtIsNaNF(b_s))) {
      b_s = (real32_T)frexp((real_T)b_s, &eint);
      normA = (real32_T)eint;
    } else {
      normA = 0.0F;
    }

    if (b_s == 0.5F) {
      normA--;
    }

    b_s = rt_powf_snf(2.0F, normA);
    for (i = 0; i < 16; i++) {
      rtb_Product3_0[i] = rtb_Product3[i] / b_s;
    }

    mglnkfkfmglfjekn_PadeApproximantOfDegree(rtb_Product3_0, 7U, rtb_y);
    for (eint = 0; eint <= (int32_T)((int32_T)normA - 1); eint++) {
      for (i = 0; i < 4; i++) {
        for (i_0 = 0; i_0 < 4; i_0++) {
          rtb_Product3[(int32_T)(i + (int32_T)(i_0 << 2))] = 0.0F;
          rtb_Product3[(int32_T)(i + (int32_T)(i_0 << 2))] += rtb_y[(int32_T)
            (i_0 << 2)] * rtb_y[i];
          rtb_Product3[(int32_T)(i + (int32_T)(i_0 << 2))] += rtb_y[(int32_T)
            ((int32_T)(i_0 << 2) + 1)] * rtb_y[(int32_T)(i + 4)];
          rtb_Product3[(int32_T)(i + (int32_T)(i_0 << 2))] += rtb_y[(int32_T)
            ((int32_T)(i_0 << 2) + 2)] * rtb_y[(int32_T)(i + 8)];
          rtb_Product3[(int32_T)(i + (int32_T)(i_0 << 2))] += rtb_y[(int32_T)
            ((int32_T)(i_0 << 2) + 3)] * rtb_y[(int32_T)(i + 12)];
        }
      }

      for (i = 0; i < 4; i++) {
        rtb_y[(int32_T)(i << 2)] = rtb_Product3[(int32_T)(i << 2)];
        rtb_y[(int32_T)(1 + (int32_T)(i << 2))] = rtb_Product3[(int32_T)
          ((int32_T)(i << 2) + 1)];
        rtb_y[(int32_T)(2 + (int32_T)(i << 2))] = rtb_Product3[(int32_T)
          ((int32_T)(i << 2) + 2)];
        rtb_y[(int32_T)(3 + (int32_T)(i << 2))] = rtb_Product3[(int32_T)
          ((int32_T)(i << 2) + 3)];
      }
    }
  }

  // End of MATLAB Function: '<S57>/MATLAB Function'

  // Product: '<S57>/Product6'
  normA = rtb_Sqrt_bt * rtb_Sqrt_bt * rtb_Sqrt_bt;
  for (i = 0; i < 4; i++) {
    for (i_0 = 0; i_0 < 4; i_0++) {
      // Product: '<S57>/Product5' incorporates:
      //   Sum: '<S57>/Add2'

      rtb_Product3_0[(int32_T)(i_0 + (int32_T)(i << 2))] = 0.0F;

      // Product: '<S57>/Product4' incorporates:
      //   Sum: '<S57>/Add2'

      rtb_VectorConcatenate_p[(int32_T)(i_0 + (int32_T)(i << 2))] = 0.0F;

      // Product: '<S57>/Product5' incorporates:
      //   Sum: '<S57>/Add2'

      rtb_Product3_0[(int32_T)(i_0 + (int32_T)(i << 2))] +=
        rtb_VectorConcatenate_m[(int32_T)(i << 2)] * rtb_VectorConcatenate[i_0];

      // Product: '<S57>/Product4' incorporates:
      //   Sum: '<S57>/Add2'

      rtb_VectorConcatenate_p[(int32_T)(i_0 + (int32_T)(i << 2))] +=
        rtb_VectorConcatenate[(int32_T)(i << 2)] * rtb_VectorConcatenate_m[i_0];

      // Product: '<S57>/Product5' incorporates:
      //   Sum: '<S57>/Add2'

      rtb_Product3_0[(int32_T)(i_0 + (int32_T)(i << 2))] +=
        rtb_VectorConcatenate_m[(int32_T)((int32_T)(i << 2) + 1)] *
        rtb_VectorConcatenate[(int32_T)(i_0 + 4)];

      // Product: '<S57>/Product4' incorporates:
      //   Sum: '<S57>/Add2'

      rtb_VectorConcatenate_p[(int32_T)(i_0 + (int32_T)(i << 2))] +=
        rtb_VectorConcatenate[(int32_T)((int32_T)(i << 2) + 1)] *
        rtb_VectorConcatenate_m[(int32_T)(i_0 + 4)];

      // Product: '<S57>/Product5' incorporates:
      //   Sum: '<S57>/Add2'

      rtb_Product3_0[(int32_T)(i_0 + (int32_T)(i << 2))] +=
        rtb_VectorConcatenate_m[(int32_T)((int32_T)(i << 2) + 2)] *
        rtb_VectorConcatenate[(int32_T)(i_0 + 8)];

      // Product: '<S57>/Product4' incorporates:
      //   Sum: '<S57>/Add2'

      rtb_VectorConcatenate_p[(int32_T)(i_0 + (int32_T)(i << 2))] +=
        rtb_VectorConcatenate[(int32_T)((int32_T)(i << 2) + 2)] *
        rtb_VectorConcatenate_m[(int32_T)(i_0 + 8)];

      // Product: '<S57>/Product5' incorporates:
      //   Sum: '<S57>/Add2'

      rtb_Product3_0[(int32_T)(i_0 + (int32_T)(i << 2))] +=
        rtb_VectorConcatenate_m[(int32_T)((int32_T)(i << 2) + 3)] *
        rtb_VectorConcatenate[(int32_T)(i_0 + 12)];

      // Product: '<S57>/Product4' incorporates:
      //   Sum: '<S57>/Add2'

      rtb_VectorConcatenate_p[(int32_T)(i_0 + (int32_T)(i << 2))] +=
        rtb_VectorConcatenate[(int32_T)((int32_T)(i << 2) + 3)] *
        rtb_VectorConcatenate_m[(int32_T)(i_0 + 12)];
    }
  }

  // Sum: '<S57>/Add1' incorporates:
  //   Constant: '<S57>/Constant2'
  //   Product: '<S57>/Product1'
  //   Product: '<S57>/Product6'
  //   Sum: '<S57>/Add2'

  for (i = 0; i < 4; i++) {
    rtb_Product3[(int32_T)(i << 2)] = (rtb_Product3_0[(int32_T)(i << 2)] -
      rtb_VectorConcatenate_p[(int32_T)(i << 2)]) * normA /
      ctl_controller0_P->Constant2_Value_it + rtb_y[(int32_T)(i << 2)];
    rtb_Product3[(int32_T)(1 + (int32_T)(i << 2))] = (rtb_Product3_0[(int32_T)
      ((int32_T)(i << 2) + 1)] - rtb_VectorConcatenate_p[(int32_T)((int32_T)(i <<
      2) + 1)]) * normA / ctl_controller0_P->Constant2_Value_it + rtb_y[(int32_T)
      ((int32_T)(i << 2) + 1)];
    rtb_Product3[(int32_T)(2 + (int32_T)(i << 2))] = (rtb_Product3_0[(int32_T)
      ((int32_T)(i << 2) + 2)] - rtb_VectorConcatenate_p[(int32_T)((int32_T)(i <<
      2) + 2)]) * normA / ctl_controller0_P->Constant2_Value_it + rtb_y[(int32_T)
      ((int32_T)(i << 2) + 2)];
    rtb_Product3[(int32_T)(3 + (int32_T)(i << 2))] = (rtb_Product3_0[(int32_T)
      ((int32_T)(i << 2) + 3)] - rtb_VectorConcatenate_p[(int32_T)((int32_T)(i <<
      2) + 3)]) * normA / ctl_controller0_P->Constant2_Value_it + rtb_y[(int32_T)
      ((int32_T)(i << 2) + 3)];
  }

  // End of Sum: '<S57>/Add1'

  // Product: '<S57>/Product1'
  for (i = 0; i < 4; i++) {
    normA = rtb_Product3[(int32_T)(i + 12)] * rtb_Merge[3] + (rtb_Product3
      [(int32_T)(i + 8)] * rtb_Merge[2] + (rtb_Product3[(int32_T)(i + 4)] *
      rtb_Merge[1] + rtb_Product3[i] * rtb_Merge[0]));
    rtb_Product1_m[i] = normA;
  }

  // If: '<S63>/If' incorporates:
  //   Inport: '<S64>/In1'

  if (rtb_Product1_m[3] < 0.0F) {
    // Outputs for IfAction SubSystem: '<S63>/Normalize' incorporates:
    //   ActionPort: '<S65>/Action Port'

    ctl_controller0_Normalize(rtb_Product1_m, rtb_Merge,
      (P_Normalize_ctl_controller0_T *)&ctl_controller0_P->Normalize);

    // End of Outputs for SubSystem: '<S63>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S63>/No-op' incorporates:
    //   ActionPort: '<S64>/Action Port'

    rtb_Merge[0] = rtb_Product1_m[0];
    rtb_Merge[1] = rtb_Product1_m[1];
    rtb_Merge[2] = rtb_Product1_m[2];
    rtb_Merge[3] = rtb_Product1_m[3];

    // End of Outputs for SubSystem: '<S63>/No-op'
  }

  // End of If: '<S63>/If'

  // Sqrt: '<S70>/Sqrt' incorporates:
  //   DotProduct: '<S70>/Dot Product'

  normA = (real32_T)sqrt((real_T)(((rtb_Merge[0] * rtb_Merge[0] + rtb_Merge[1] *
    rtb_Merge[1]) + rtb_Merge[2] * rtb_Merge[2]) + rtb_Merge[3] * rtb_Merge[3]));

  // If: '<S66>/If' incorporates:
  //   DataTypeConversion: '<S66>/Data Type Conversion'
  //   Inport: '<S68>/In1'

  if ((real_T)normA > 1.0E-7) {
    // Outputs for IfAction SubSystem: '<S66>/Normalize' incorporates:
    //   ActionPort: '<S69>/Action Port'

    ctl_controller0_Normalize_e(rtb_Merge, normA, rtb_Product1_m);

    // End of Outputs for SubSystem: '<S66>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S66>/No-op' incorporates:
    //   ActionPort: '<S68>/Action Port'

    rtb_Product1_m[0] = rtb_Merge[0];
    rtb_Product1_m[1] = rtb_Merge[1];
    rtb_Product1_m[2] = rtb_Merge[2];
    rtb_Product1_m[3] = rtb_Merge[3];

    // End of Outputs for SubSystem: '<S66>/No-op'
  }

  // End of If: '<S66>/If'

  // Switch: '<S2>/Switch12' incorporates:
  //   Constant: '<S2>/Constant4'
  //   Constant: '<S9>/Constant'
  //   Inport: '<Root>/cmc_msg'
  //   RelationalOperator: '<S9>/Compare'
  //   SignalConversion: '<S1>/Signal Conversion'

  if (rtb_Switch2 != ctl_controller0_P->ase_status_converged) {
    rtb_Switch2 = ctl_controller0_P->ctl_idle_mode;
  } else {
    rtb_Switch2 = ctl_controller0_U_cmc_msg_f->cmc_mode_cmd;
  }

  // End of Switch: '<S2>/Switch12'

  // Logic: '<S2>/Logical Operator2' incorporates:
  //   Constant: '<S6>/Constant'
  //   Constant: '<S7>/Constant'
  //   Constant: '<S8>/Constant'
  //   Logic: '<S2>/Logical Operator'
  //   RelationalOperator: '<S6>/Compare'
  //   RelationalOperator: '<S7>/Compare'
  //   RelationalOperator: '<S8>/Compare'

  rtb_LogicalOperator2 = ((rtb_Switch2 == ctl_controller0_P->ctl_stopping_mode) &&
    ((rtb_SumofElements < ctl_controller0_P->ctl_stopping_vel_thresh) &&
     (rtb_SumofElements1 < ctl_controller0_P->ctl_stopping_omega_thresh)));

  // Switch: '<S2>/Switch2' incorporates:
  //   Constant: '<S2>/Constant'
  //   Switch: '<S2>/Switch8'
  //   UnitDelay: '<S2>/Unit Delay1'

  if (rtb_LogicalOperator2) {
    rtb_Switch2 = ctl_controller0_P->ctl_stopped_mode;
    rtb_Switch8_idx_0 = ctl_controller0_DW->UnitDelay1_DSTATE[0];
  } else {
    rtb_Switch8_idx_0 = rtb_Switch_P_B_ISS_ISS_idx_0;
  }

  // End of Switch: '<S2>/Switch2'

  // Sum: '<S57>/Sum1' incorporates:
  //   Product: '<S57>/Product2'

  rtb_SumofElements1 = rtb_Switch_h_alpha_B_ISS_B_id_1 * rtb_Sqrt_bt +
    rtb_Sum4_o_idx_0;

  // BusCreator: '<S4>/Bus Creator1'
  normA = rtb_Gain1_j_idx_0;
  b_s = rtb_Sum3_k_idx_0;
  rtb_BusCreator1_traj_omega_idx_ = rtb_SumofElements1;

  // Sum: '<S57>/Sum1' incorporates:
  //   Product: '<S57>/Product2'

  rtb_Sum4_o_idx_0 = rtb_SumofElements1;
  rtb_SumofElements1 = rtb_Switch_h_alpha_B_ISS_B_id_0 * rtb_Sqrt_bt +
    rtb_Sum4_o_idx_1;

  // BusCreator: '<S4>/Bus Creator1'
  rtb_BusCreator1_traj_pos_idx_1 = rtb_Gain1_j_idx_1;
  rtb_BusCreator1_traj_vel_idx_1 = rtb_Sum3_k_idx_1;
  rtb_BusCreator1_traj_omega_id_0 = rtb_SumofElements1;

  // Sum: '<S57>/Sum1' incorporates:
  //   Product: '<S57>/Product2'

  rtb_Sum4_o_idx_1 = rtb_SumofElements1;
  rtb_SumofElements1 = rtb_Switch_h_alpha_B_ISS_B_idx_ * rtb_Sqrt_bt +
    rtb_Sum4_o_idx_2;

  // Switch: '<S2>/Switch8' incorporates:
  //   Switch: '<S2>/Switch9'
  //   UnitDelay: '<S2>/Unit Delay1'
  //   UnitDelay: '<S2>/Unit Delay2'

  if (rtb_LogicalOperator2) {
    rtb_Switch8_idx_1 = ctl_controller0_DW->UnitDelay1_DSTATE[1];
    rtb_Sum4_o_idx_2 = ctl_controller0_DW->UnitDelay1_DSTATE[2];
    rtb_Merge[0] = ctl_controller0_DW->UnitDelay2_DSTATE[0];
    rtb_Merge[1] = ctl_controller0_DW->UnitDelay2_DSTATE[1];
    rtb_Merge[2] = ctl_controller0_DW->UnitDelay2_DSTATE[2];
    rtb_Merge[3] = ctl_controller0_DW->UnitDelay2_DSTATE[3];
  } else {
    rtb_Switch8_idx_1 = rtb_Switch_P_B_ISS_ISS_idx_1;
    rtb_Sum4_o_idx_2 = rtb_Switch_P_B_ISS_ISS_idx_2;
    rtb_Merge[0] = rtb_Merge_az[0];
    rtb_Merge[1] = rtb_Merge_az[1];
    rtb_Merge[2] = rtb_Merge_az[2];
    rtb_Merge[3] = rtb_Merge_az[3];
  }

  // Sum: '<S25>/Sum' incorporates:
  //   Constant: '<S25>/Constant1'
  //   DataTypeConversion: '<S27>/Conversion'
  //   Gain: '<S25>/Gain'
  //   Math: '<S25>/Math Function'

  rtb_Sqrt_bt = rtb_Merge_az[3] * rtb_Merge_az[3] *
    ctl_controller0_P->Gain_Gain_l - (real32_T)
    ctl_controller0_P->Constant1_Value;
  for (i = 0; i < 9; i++) {
    // Assignment: '<S25>/Assignment' incorporates:
    //   Constant: '<S25>/Constant2'
    //   DataTypeConversion: '<S26>/Conversion'

    rtb_Assignment_h[i] = (real32_T)ctl_controller0_P->Constant2_Value_p[i];

    // Assignment: '<S20>/Assignment' incorporates:
    //   Constant: '<S20>/Constant2'
    //   DataTypeConversion: '<S21>/Conversion'

    rtb_Assignment_l[i] = (real32_T)ctl_controller0_P->Constant2_Value_k[i];
  }

  // Assignment: '<S25>/Assignment'
  rtb_Assignment_h[0] = rtb_Sqrt_bt;
  rtb_Assignment_h[4] = rtb_Sqrt_bt;
  rtb_Assignment_h[8] = rtb_Sqrt_bt;

  // Gain: '<S25>/Gain1'
  rtb_Sqrt_bt = ctl_controller0_P->Gain1_Gain_ej * rtb_Merge_az[3];

  // Sum: '<S20>/Sum' incorporates:
  //   Constant: '<S20>/Constant1'
  //   DataTypeConversion: '<S22>/Conversion'
  //   Gain: '<S20>/Gain'
  //   Math: '<S20>/Math Function'

  rtb_SumofElements = rtb_Merge_az[3] * rtb_Merge_az[3] *
    ctl_controller0_P->Gain_Gain_b - (real32_T)
    ctl_controller0_P->Constant1_Value_f;

  // Assignment: '<S20>/Assignment'
  rtb_Assignment_l[0] = rtb_SumofElements;
  rtb_Assignment_l[4] = rtb_SumofElements;
  rtb_Assignment_l[8] = rtb_SumofElements;

  // Gain: '<S20>/Gain1'
  rtb_SumofElements = ctl_controller0_P->Gain1_Gain_g * rtb_Merge_az[3];

  // Switch: '<S12>/Switch2' incorporates:
  //   BusCreator: '<S4>/Bus Creator1'
  //   Constant: '<S12>/Constant4'
  //   Constant: '<S16>/Constant'
  //   RelationalOperator: '<S16>/Compare'
  //   Switch: '<S2>/Switch10'

  if (rtb_Switch2 <= ctl_controller0_P->CompareToConstant2_const) {
    rtb_Switch_c_idx_0 = ctl_controller0_P->Constant4_Value[0];
    rtb_Switch_c_idx_1 = ctl_controller0_P->Constant4_Value[1];
    rtb_Switch_c_idx_2 = ctl_controller0_P->Constant4_Value[2];
  } else if (rtb_LogicalOperator2) {
    // Switch: '<S2>/Switch10' incorporates:
    //   Constant: '<S2>/Constant2'

    rtb_Switch_c_idx_0 = ctl_controller0_P->Constant2_Value_n[0];
    rtb_Switch_c_idx_1 = ctl_controller0_P->Constant2_Value_n[1];
    rtb_Switch_c_idx_2 = ctl_controller0_P->Constant2_Value_n[2];
  } else {
    rtb_Switch_c_idx_0 = rtb_Sum3_k_idx_0;
    rtb_Switch_c_idx_1 = rtb_Sum3_k_idx_1;
    rtb_Switch_c_idx_2 = rtb_Sum3_k_idx_2;
  }

  // End of Switch: '<S12>/Switch2'

  // Switch: '<S2>/Switch6' incorporates:
  //   BusCreator: '<S4>/Bus Creator1'

  if (rtb_LogicalOperator2) {
    rtb_LogicalOperator2_0 = rtb_Switch8_idx_0;
  } else {
    rtb_LogicalOperator2_0 = rtb_Gain1_j_idx_0;
  }

  // Sum: '<S12>/Sum2'
  rtb_Sum2_idx_0 = rtb_LogicalOperator2_0 - rtb_Switch_P_B_ISS_ISS_idx_0;

  // Switch: '<S2>/Switch6' incorporates:
  //   BusCreator: '<S4>/Bus Creator1'

  if (rtb_LogicalOperator2) {
    rtb_LogicalOperator2_0 = rtb_Switch8_idx_1;
  } else {
    rtb_LogicalOperator2_0 = rtb_Gain1_j_idx_1;
  }

  // Sum: '<S12>/Sum2'
  rtb_Sum2_idx_1 = rtb_LogicalOperator2_0 - rtb_Switch_P_B_ISS_ISS_idx_1;

  // Switch: '<S2>/Switch6'
  if (rtb_LogicalOperator2) {
    rtb_LogicalOperator2_0 = rtb_Sum4_o_idx_2;
  } else {
    rtb_LogicalOperator2_0 = rtb_Gain1_j_idx_2;
  }

  // Sum: '<S12>/Sum2'
  rtb_Sum2_idx_2 = rtb_LogicalOperator2_0 - rtb_Switch_P_B_ISS_ISS_idx_2;

  // RelationalOperator: '<S14>/Compare' incorporates:
  //   Constant: '<S14>/Constant'

  rtb_Compare_j = (rtb_Switch2 <= ctl_controller0_P->CompareToConstant_const);

  // DiscreteIntegrator: '<S12>/Discrete-Time Integrator1'
  if (rtb_Compare_j || ((int32_T)
                        ctl_controller0_DW->DiscreteTimeIntegrator1_PrevRes != 0))
  {
    ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0] =
      ctl_controller0_P->DiscreteTimeIntegrator1_IC[0];
    ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1] =
      ctl_controller0_P->DiscreteTimeIntegrator1_IC[1];
    ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2] =
      ctl_controller0_P->DiscreteTimeIntegrator1_IC[2];
  }

  if (ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0] >=
      ctl_controller0_P->tun_ctl_pos_sat_upper) {
    ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0] =
      ctl_controller0_P->tun_ctl_pos_sat_upper;
  } else {
    if (ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0] <=
        ctl_controller0_P->tun_ctl_pos_sat_lower) {
      ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0] =
        ctl_controller0_P->tun_ctl_pos_sat_lower;
    }
  }

  if (ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1] >=
      ctl_controller0_P->tun_ctl_pos_sat_upper) {
    ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1] =
      ctl_controller0_P->tun_ctl_pos_sat_upper;
  } else {
    if (ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1] <=
        ctl_controller0_P->tun_ctl_pos_sat_lower) {
      ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1] =
        ctl_controller0_P->tun_ctl_pos_sat_lower;
    }
  }

  if (ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2] >=
      ctl_controller0_P->tun_ctl_pos_sat_upper) {
    ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2] =
      ctl_controller0_P->tun_ctl_pos_sat_upper;
  } else {
    if (ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2] <=
        ctl_controller0_P->tun_ctl_pos_sat_lower) {
      ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2] =
        ctl_controller0_P->tun_ctl_pos_sat_lower;
    }
  }

  // Switch: '<S12>/Switch1' incorporates:
  //   Constant: '<S12>/Constant1'
  //   Constant: '<S15>/Constant'
  //   DiscreteIntegrator: '<S12>/Discrete-Time Integrator1'
  //   Inport: '<Root>/cmc_msg'
  //   Product: '<S12>/Divide1'
  //   Product: '<S12>/Product'
  //   RelationalOperator: '<S15>/Compare'
  //   SignalConversion: '<S1>/Signal Conversion'
  //   Sum: '<S12>/Sum4'

  if (rtb_Switch2 <= ctl_controller0_P->CompareToConstant1_const) {
    rtb_Divide_f[0] = ctl_controller0_P->Constant1_Value_k[0];
    rtb_Divide_f[1] = ctl_controller0_P->Constant1_Value_k[1];
    rtb_Divide_f[2] = ctl_controller0_P->Constant1_Value_k[2];
  } else {
    rtb_Divide_f[0] = ctl_controller0_U_cmc_msg_f->pos_kp[0] /
      ctl_controller0_U_cmc_msg_f->vel_kd[0] * rtb_Sum2_idx_0 +
      ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0];
    rtb_Divide_f[1] = ctl_controller0_U_cmc_msg_f->pos_kp[1] /
      ctl_controller0_U_cmc_msg_f->vel_kd[1] * rtb_Sum2_idx_1 +
      ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1];
    rtb_Divide_f[2] = ctl_controller0_U_cmc_msg_f->pos_kp[2] /
      ctl_controller0_U_cmc_msg_f->vel_kd[2] * rtb_Sum2_idx_2 +
      ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2];
  }

  // End of Switch: '<S12>/Switch1'

  // Sum: '<S12>/Sum1'
  rtb_Switch_c_idx_0 = (rtb_Switch_c_idx_0 + rtb_Divide_f[0]) -
    rtb_Switch_V_B_ISS_ISS_idx_0;
  rtb_Switch_c_idx_1 = (rtb_Switch_c_idx_1 + rtb_Divide_f[1]) -
    rtb_Switch_V_B_ISS_ISS_idx_1;
  rtb_Switch_c_idx_2 = (rtb_Switch_c_idx_2 + rtb_Divide_f[2]) -
    rtb_Switch_V_B_ISS_ISS_idx_2;

  // Product: '<S25>/Product' incorporates:
  //   Constant: '<S28>/Constant3'
  //   DataTypeConversion: '<S29>/Conversion'
  //   Gain: '<S28>/Gain'
  //   Gain: '<S28>/Gain1'
  //   Gain: '<S28>/Gain2'

  tmp_0[0] = (real32_T)ctl_controller0_P->Constant3_Value_o;
  tmp_0[1] = rtb_Merge_az[2];
  tmp_0[2] = ctl_controller0_P->Gain_Gain_i * rtb_Merge_az[1];
  tmp_0[3] = ctl_controller0_P->Gain1_Gain_k * rtb_Merge_az[2];
  tmp_0[4] = (real32_T)ctl_controller0_P->Constant3_Value_o;
  tmp_0[5] = rtb_Merge_az[0];
  tmp_0[6] = rtb_Merge_az[1];
  tmp_0[7] = ctl_controller0_P->Gain2_Gain_b * rtb_Merge_az[0];
  tmp_0[8] = (real32_T)ctl_controller0_P->Constant3_Value_o;

  // Product: '<S25>/Product1' incorporates:
  //   Gain: '<S25>/Gain2'

  for (i = 0; i < 3; i++) {
    rtb_Merge_j[i] = rtb_Merge_az[i] * rtb_Merge_az[0];
    rtb_Merge_j[(int32_T)(i + 3)] = rtb_Merge_az[i] * rtb_Merge_az[1];
    rtb_Merge_j[(int32_T)(i + 6)] = rtb_Merge_az[i] * rtb_Merge_az[2];
  }

  // End of Product: '<S25>/Product1'
  for (i = 0; i < 3; i++) {
    // Sum: '<S25>/Sum1' incorporates:
    //   Gain: '<S25>/Gain2'
    //   Product: '<S18>/Product'
    //   Product: '<S25>/Product'

    rtb_Assignment_b_0[(int32_T)(3 * i)] = (rtb_Assignment_h[(int32_T)(3 * i)] -
      tmp_0[(int32_T)(3 * i)] * rtb_Sqrt_bt) + rtb_Merge_j[(int32_T)(3 * i)] *
      ctl_controller0_P->Gain2_Gain_k1;
    rtb_Assignment_b_0[(int32_T)(1 + (int32_T)(3 * i))] = (rtb_Assignment_h
      [(int32_T)((int32_T)(3 * i) + 1)] - tmp_0[(int32_T)((int32_T)(3 * i) + 1)]
      * rtb_Sqrt_bt) + rtb_Merge_j[(int32_T)((int32_T)(3 * i) + 1)] *
      ctl_controller0_P->Gain2_Gain_k1;
    rtb_Assignment_b_0[(int32_T)(2 + (int32_T)(3 * i))] = (rtb_Assignment_h
      [(int32_T)((int32_T)(3 * i) + 2)] - tmp_0[(int32_T)((int32_T)(3 * i) + 2)]
      * rtb_Sqrt_bt) + rtb_Merge_j[(int32_T)((int32_T)(3 * i) + 2)] *
      ctl_controller0_P->Gain2_Gain_k1;

    // Gain: '<S12>/Gain' incorporates:
    //   Constant: '<S2>/Constant5'
    //   Product: '<S18>/Product'
    //   Switch: '<S2>/Switch14'

    if (rtb_LogicalOperator2) {
      rtb_LogicalOperator2_0 = ctl_controller0_P->Constant5_Value[i];
    } else {
      rtb_LogicalOperator2_0 = rtb_Switch_h_A_B_ISS_ISS[i];
    }

    rtb_Divide_f[i] = ctl_controller0_P->tun_accel_gain[i] *
      rtb_LogicalOperator2_0;

    // End of Gain: '<S12>/Gain'
  }

  // Product: '<S20>/Product' incorporates:
  //   Constant: '<S23>/Constant3'
  //   DataTypeConversion: '<S24>/Conversion'
  //   Gain: '<S23>/Gain'
  //   Gain: '<S23>/Gain1'
  //   Gain: '<S23>/Gain2'

  tmp_1[0] = (real32_T)ctl_controller0_P->Constant3_Value_b;
  tmp_1[1] = rtb_Merge_az[2];
  tmp_1[2] = ctl_controller0_P->Gain_Gain_d * rtb_Merge_az[1];
  tmp_1[3] = ctl_controller0_P->Gain1_Gain_ji * rtb_Merge_az[2];
  tmp_1[4] = (real32_T)ctl_controller0_P->Constant3_Value_b;
  tmp_1[5] = rtb_Merge_az[0];
  tmp_1[6] = rtb_Merge_az[1];
  tmp_1[7] = ctl_controller0_P->Gain2_Gain_bs * rtb_Merge_az[0];
  tmp_1[8] = (real32_T)ctl_controller0_P->Constant3_Value_b;
  for (i = 0; i < 3; i++) {
    // Product: '<S20>/Product1' incorporates:
    //   Gain: '<S20>/Gain2'

    rtb_Merge_j[i] = rtb_Merge_az[i] * rtb_Merge_az[0];
    rtb_Merge_j[(int32_T)(i + 3)] = rtb_Merge_az[i] * rtb_Merge_az[1];
    rtb_Merge_j[(int32_T)(i + 6)] = rtb_Merge_az[i] * rtb_Merge_az[2];

    // Product: '<S18>/Product' incorporates:
    //   Product: '<S12>/Divide3'

    rtb_Assignment_b_1[i] = rtb_Assignment_b_0[(int32_T)(i + 6)] * rtb_Divide_f
      [2] + (rtb_Assignment_b_0[(int32_T)(i + 3)] * rtb_Divide_f[1] +
             rtb_Assignment_b_0[i] * rtb_Divide_f[0]);
  }

  // Sum: '<S20>/Sum1' incorporates:
  //   Gain: '<S20>/Gain2'
  //   Product: '<S17>/Product'
  //   Product: '<S20>/Product'

  for (i = 0; i < 3; i++) {
    rtb_Assignment_h[(int32_T)(3 * i)] = (rtb_Assignment_l[(int32_T)(3 * i)] -
      tmp_1[(int32_T)(3 * i)] * rtb_SumofElements) + rtb_Merge_j[(int32_T)(3 * i)]
      * ctl_controller0_P->Gain2_Gain_kh;
    rtb_Assignment_h[(int32_T)(1 + (int32_T)(3 * i))] = (rtb_Assignment_l
      [(int32_T)((int32_T)(3 * i) + 1)] - tmp_1[(int32_T)((int32_T)(3 * i) + 1)]
      * rtb_SumofElements) + rtb_Merge_j[(int32_T)((int32_T)(3 * i) + 1)] *
      ctl_controller0_P->Gain2_Gain_kh;
    rtb_Assignment_h[(int32_T)(2 + (int32_T)(3 * i))] = (rtb_Assignment_l
      [(int32_T)((int32_T)(3 * i) + 2)] - tmp_1[(int32_T)((int32_T)(3 * i) + 2)]
      * rtb_SumofElements) + rtb_Merge_j[(int32_T)((int32_T)(3 * i) + 2)] *
      ctl_controller0_P->Gain2_Gain_kh;
  }

  // End of Sum: '<S20>/Sum1'

  // DotProduct: '<S34>/Dot Product'
  rtb_Sqrt_bt = 0.0F;
  for (i = 0; i < 3; i++) {
    // Sum: '<S12>/Sum' incorporates:
    //   Inport: '<Root>/cmc_msg'
    //   Product: '<S12>/Divide3'
    //   Product: '<S12>/Product2'
    //   Product: '<S12>/Product3'
    //   Product: '<S17>/Product'
    //   SignalConversion: '<S1>/Signal Conversion'

    rtb_SumofElements = ctl_controller0_U_cmc_msg_f->vel_kd[i] *
      ctl_controller0_U_cmc_msg_f->mass * (rtb_Assignment_h[(int32_T)(i + 6)] *
      rtb_Switch_c_idx_2 + (rtb_Assignment_h[(int32_T)(i + 3)] *
      rtb_Switch_c_idx_1 + rtb_Assignment_h[i] * rtb_Switch_c_idx_0)) +
      ctl_controller0_U_cmc_msg_f->mass * rtb_Assignment_b_1[i];

    // DotProduct: '<S34>/Dot Product'
    rtb_Sqrt_bt += rtb_SumofElements * rtb_SumofElements;

    // Sum: '<S12>/Sum' incorporates:
    //   Product: '<S12>/Divide3'

    rtb_Divide_f[i] = rtb_SumofElements;
  }

  // Sqrt: '<S34>/Sqrt' incorporates:
  //   DotProduct: '<S34>/Dot Product'

  rtb_Sqrt_bt = (real32_T)sqrt((real_T)rtb_Sqrt_bt);

  // If: '<S31>/If' incorporates:
  //   DataTypeConversion: '<S31>/Data Type Conversion'
  //   Inport: '<S32>/In1'

  if ((real_T)rtb_Sqrt_bt > 1.0E-7) {
    // Outputs for IfAction SubSystem: '<S31>/Normalize' incorporates:
    //   ActionPort: '<S33>/Action Port'

    // Product: '<S33>/Divide'
    rtb_Switch_c_idx_0 = rtb_Divide_f[0] / rtb_Sqrt_bt;
    rtb_Switch_c_idx_1 = rtb_Divide_f[1] / rtb_Sqrt_bt;
    rtb_Switch_c_idx_2 = rtb_Divide_f[2] / rtb_Sqrt_bt;

    // End of Outputs for SubSystem: '<S31>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S31>/No-op' incorporates:
    //   ActionPort: '<S32>/Action Port'

    rtb_Switch_c_idx_0 = rtb_Divide_f[0];
    rtb_Switch_c_idx_1 = rtb_Divide_f[1];
    rtb_Switch_c_idx_2 = rtb_Divide_f[2];

    // End of Outputs for SubSystem: '<S31>/No-op'
  }

  // End of If: '<S31>/If'

  // Switch: '<S12>/Switch' incorporates:
  //   Constant: '<S12>/Constant3'

  if ((int32_T)rtb_Switch2 != 0) {
    // Switch: '<S19>/Switch' incorporates:
    //   Constant: '<S12>/Constant2'
    //   DotProduct: '<S30>/Dot Product'
    //   Product: '<S19>/Product'
    //   RelationalOperator: '<S19>/Relational Operator'
    //   Sqrt: '<S30>/Sqrt'

    if (!((real32_T)sqrt((real_T)((rtb_Divide_f[0] * rtb_Divide_f[0] +
            rtb_Divide_f[1] * rtb_Divide_f[1]) + rtb_Divide_f[2] * rtb_Divide_f
           [2])) < ctl_controller0_P->tun_ctl_linear_force_limit)) {
      rtb_Divide_f[0] = ctl_controller0_P->tun_ctl_linear_force_limit *
        rtb_Switch_c_idx_0;
      rtb_Divide_f[1] = ctl_controller0_P->tun_ctl_linear_force_limit *
        rtb_Switch_c_idx_1;
      rtb_Divide_f[2] = ctl_controller0_P->tun_ctl_linear_force_limit *
        rtb_Switch_c_idx_2;
    }

    // End of Switch: '<S19>/Switch'
    rtb_Switch_c_idx_0 = rtb_Divide_f[0];
    rtb_Switch_c_idx_1 = rtb_Divide_f[1];
    rtb_Switch_c_idx_2 = rtb_Divide_f[2];
  } else {
    rtb_Switch_c_idx_0 = ctl_controller0_P->Constant3_Value_e[0];
    rtb_Switch_c_idx_1 = ctl_controller0_P->Constant3_Value_e[1];
    rtb_Switch_c_idx_2 = ctl_controller0_P->Constant3_Value_e[2];
  }

  // End of Switch: '<S12>/Switch'

  // Switch: '<S13>/Switch3' incorporates:
  //   BusCreator: '<S4>/Bus Creator1'
  //   Constant: '<S13>/Constant2'
  //   Constant: '<S37>/Constant'
  //   RelationalOperator: '<S37>/Compare'
  //   Switch: '<S2>/Switch11'

  if (rtb_Switch2 <= ctl_controller0_P->CompareToConstant2_const_o) {
    rtb_Gain_ci[0] = ctl_controller0_P->Constant2_Value_o[0];
    rtb_Gain_ci[1] = ctl_controller0_P->Constant2_Value_o[1];
    rtb_Gain_ci[2] = ctl_controller0_P->Constant2_Value_o[2];
  } else if (rtb_LogicalOperator2) {
    // Switch: '<S2>/Switch11' incorporates:
    //   Constant: '<S2>/Constant3'

    rtb_Gain_ci[0] = ctl_controller0_P->Constant3_Value_j[0];
    rtb_Gain_ci[1] = ctl_controller0_P->Constant3_Value_j[1];
    rtb_Gain_ci[2] = ctl_controller0_P->Constant3_Value_j[2];
  } else {
    rtb_Gain_ci[0] = rtb_Sum4_o_idx_0;
    rtb_Gain_ci[1] = rtb_Sum4_o_idx_1;
    rtb_Gain_ci[2] = rtb_SumofElements1;
  }

  // End of Switch: '<S13>/Switch3'

  // DataTypeConversion: '<S46>/Conversion' incorporates:
  //   Constant: '<S45>/Constant2'

  for (i = 0; i < 9; i++) {
    rtb_Assignment_h[i] = (real32_T)ctl_controller0_P->Constant2_Value_g[i];
  }

  // End of DataTypeConversion: '<S46>/Conversion'

  // Assignment: '<S45>/Assignment'
  rtb_Assignment_h[0] = rtb_Merge_az[3];

  // Gain: '<S42>/Gain'
  rtb_Divide_f[0] = ctl_controller0_P->Gain_Gain_pf * rtb_Merge_az[0];

  // Assignment: '<S45>/Assignment'
  rtb_Assignment_h[4] = rtb_Merge_az[3];

  // Gain: '<S42>/Gain'
  rtb_Divide_f[1] = ctl_controller0_P->Gain_Gain_pf * rtb_Merge_az[1];

  // Assignment: '<S45>/Assignment'
  rtb_Assignment_h[8] = rtb_Merge_az[3];

  // Gain: '<S42>/Gain'
  rtb_Divide_f[2] = ctl_controller0_P->Gain_Gain_pf * rtb_Merge_az[2];

  // Sum: '<S45>/Sum2' incorporates:
  //   Constant: '<S47>/Constant3'
  //   DataTypeConversion: '<S48>/Conversion'
  //   Gain: '<S47>/Gain'
  //   Gain: '<S47>/Gain1'
  //   Gain: '<S47>/Gain2'

  tmp_2[0] = (real32_T)ctl_controller0_P->Constant3_Value_a;
  tmp_2[1] = rtb_Divide_f[2];
  tmp_2[2] = ctl_controller0_P->Gain_Gain_k * rtb_Divide_f[1];
  tmp_2[3] = ctl_controller0_P->Gain1_Gain_jz * rtb_Divide_f[2];
  tmp_2[4] = (real32_T)ctl_controller0_P->Constant3_Value_a;
  tmp_2[5] = rtb_Divide_f[0];
  tmp_2[6] = rtb_Divide_f[1];
  tmp_2[7] = ctl_controller0_P->Gain2_Gain_n * rtb_Divide_f[0];
  tmp_2[8] = (real32_T)ctl_controller0_P->Constant3_Value_a;

  // Concatenate: '<S45>/Matrix Concatenate' incorporates:
  //   Gain: '<S45>/Gain1'
  //   Sum: '<S45>/Sum2'

  for (i = 0; i < 3; i++) {
    rtb_VectorConcatenate[(int32_T)(i << 2)] = rtb_Assignment_h[(int32_T)(3 * i)]
      + tmp_2[(int32_T)(3 * i)];
    rtb_VectorConcatenate[(int32_T)(1 + (int32_T)(i << 2))] = rtb_Assignment_h
      [(int32_T)((int32_T)(3 * i) + 1)] + tmp_2[(int32_T)((int32_T)(3 * i) + 1)];
    rtb_VectorConcatenate[(int32_T)(2 + (int32_T)(i << 2))] = rtb_Assignment_h
      [(int32_T)((int32_T)(3 * i) + 2)] + tmp_2[(int32_T)((int32_T)(3 * i) + 2)];
  }

  rtb_VectorConcatenate[3] = ctl_controller0_P->Gain1_Gain_od * rtb_Divide_f[0];
  rtb_VectorConcatenate[7] = ctl_controller0_P->Gain1_Gain_od * rtb_Divide_f[1];
  rtb_VectorConcatenate[11] = ctl_controller0_P->Gain1_Gain_od * rtb_Divide_f[2];

  // End of Concatenate: '<S45>/Matrix Concatenate'

  // Reshape: '<S43>/Reshape1'
  rtb_VectorConcatenate[12] = rtb_Divide_f[0];
  rtb_VectorConcatenate[13] = rtb_Divide_f[1];
  rtb_VectorConcatenate[14] = rtb_Divide_f[2];
  rtb_VectorConcatenate[15] = rtb_Merge_az[3];

  // Product: '<S43>/Product' incorporates:
  //   BusCreator: '<S4>/Bus Creator1'
  //   Switch: '<S2>/Switch7'

  if (rtb_LogicalOperator2) {
    rtb_SumofElements = rtb_Merge[0];
    rtb_LogicalOperator2_0 = rtb_Merge[1];
    rtb_LogicalOperator2_idx_2 = rtb_Merge[2];
    rtb_LogicalOperator2_idx_3 = rtb_Merge[3];
  } else {
    rtb_SumofElements = rtb_Product1_m[0];
    rtb_LogicalOperator2_0 = rtb_Product1_m[1];
    rtb_LogicalOperator2_idx_2 = rtb_Product1_m[2];
    rtb_LogicalOperator2_idx_3 = rtb_Product1_m[3];
  }

  for (i = 0; i < 4; i++) {
    rtb_Sqrt_bt = rtb_VectorConcatenate[(int32_T)(i + 12)] *
      rtb_LogicalOperator2_idx_3 + (rtb_VectorConcatenate[(int32_T)(i + 8)] *
      rtb_LogicalOperator2_idx_2 + (rtb_VectorConcatenate[(int32_T)(i + 4)] *
      rtb_LogicalOperator2_0 + rtb_VectorConcatenate[i] * rtb_SumofElements));
    rtb_Product_f[i] = rtb_Sqrt_bt;
  }

  // End of Product: '<S43>/Product'

  // If: '<S44>/If' incorporates:
  //   Inport: '<S49>/In1'

  if (rtb_Product_f[3] < 0.0F) {
    // Outputs for IfAction SubSystem: '<S44>/Normalize' incorporates:
    //   ActionPort: '<S50>/Action Port'

    ctl_controller0_Normalize(rtb_Product_f, rtb_Merge_k,
      (P_Normalize_ctl_controller0_T *)&ctl_controller0_P->Normalize_l);

    // End of Outputs for SubSystem: '<S44>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S44>/No-op' incorporates:
    //   ActionPort: '<S49>/Action Port'

    rtb_Merge_k[0] = rtb_Product_f[0];
    rtb_Merge_k[1] = rtb_Product_f[1];
    rtb_Merge_k[2] = rtb_Product_f[2];
    rtb_Merge_k[3] = rtb_Product_f[3];

    // End of Outputs for SubSystem: '<S44>/No-op'
  }

  // End of If: '<S44>/If'

  // Sqrt: '<S55>/Sqrt' incorporates:
  //   DotProduct: '<S55>/Dot Product'

  rtb_Sqrt_bt = (real32_T)sqrt((real_T)(((rtb_Merge_k[0] * rtb_Merge_k[0] +
    rtb_Merge_k[1] * rtb_Merge_k[1]) + rtb_Merge_k[2] * rtb_Merge_k[2]) +
    rtb_Merge_k[3] * rtb_Merge_k[3]));

  // If: '<S51>/If' incorporates:
  //   DataTypeConversion: '<S51>/Data Type Conversion'
  //   Inport: '<S53>/In1'

  if ((real_T)rtb_Sqrt_bt > 1.0E-7) {
    // Outputs for IfAction SubSystem: '<S51>/Normalize' incorporates:
    //   ActionPort: '<S54>/Action Port'

    ctl_controller0_Normalize_e(rtb_Merge_k, rtb_Sqrt_bt, rtb_Product_f);

    // End of Outputs for SubSystem: '<S51>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S51>/No-op' incorporates:
    //   ActionPort: '<S53>/Action Port'

    rtb_Product_f[0] = rtb_Merge_k[0];
    rtb_Product_f[1] = rtb_Merge_k[1];
    rtb_Product_f[2] = rtb_Merge_k[2];
    rtb_Product_f[3] = rtb_Merge_k[3];

    // End of Outputs for SubSystem: '<S51>/No-op'
  }

  // End of If: '<S51>/If'

  // RelationalOperator: '<S35>/Compare' incorporates:
  //   Constant: '<S35>/Constant'

  rtb_Compare_a = (rtb_Switch2 <= ctl_controller0_P->CompareToConstant_const_f);

  // DiscreteIntegrator: '<S13>/Discrete-Time Integrator'
  if (rtb_Compare_a || ((int32_T)
                        ctl_controller0_DW->DiscreteTimeIntegrator_PrevRese != 0))
  {
    ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0] =
      ctl_controller0_P->DiscreteTimeIntegrator_IC[0];
    ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1] =
      ctl_controller0_P->DiscreteTimeIntegrator_IC[1];
    ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[2] =
      ctl_controller0_P->DiscreteTimeIntegrator_IC[2];
  }

  if (ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0] >=
      ctl_controller0_P->tun_ctl_att_sat_upper) {
    ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0] =
      ctl_controller0_P->tun_ctl_att_sat_upper;
  } else {
    if (ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0] <=
        ctl_controller0_P->tun_ctl_att_sat_lower) {
      ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0] =
        ctl_controller0_P->tun_ctl_att_sat_lower;
    }
  }

  if (ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1] >=
      ctl_controller0_P->tun_ctl_att_sat_upper) {
    ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1] =
      ctl_controller0_P->tun_ctl_att_sat_upper;
  } else {
    if (ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1] <=
        ctl_controller0_P->tun_ctl_att_sat_lower) {
      ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1] =
        ctl_controller0_P->tun_ctl_att_sat_lower;
    }
  }

  if (ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[2] >=
      ctl_controller0_P->tun_ctl_att_sat_upper) {
    ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[2] =
      ctl_controller0_P->tun_ctl_att_sat_upper;
  } else {
    if (ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[2] <=
        ctl_controller0_P->tun_ctl_att_sat_lower) {
      ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[2] =
        ctl_controller0_P->tun_ctl_att_sat_lower;
    }
  }

  // Switch: '<S13>/Switch2' incorporates:
  //   Constant: '<S13>/Constant1'
  //   Constant: '<S36>/Constant'
  //   DiscreteIntegrator: '<S13>/Discrete-Time Integrator'
  //   Inport: '<Root>/cmc_msg'
  //   Product: '<S13>/Divide1'
  //   Product: '<S13>/Product'
  //   RelationalOperator: '<S36>/Compare'
  //   SignalConversion: '<S1>/Signal Conversion'
  //   Sum: '<S13>/Sum3'

  if (rtb_Switch2 <= ctl_controller0_P->CompareToConstant1_const_m) {
    rtb_Divide_f[0] = ctl_controller0_P->Constant1_Value_o[0];
    rtb_Divide_f[1] = ctl_controller0_P->Constant1_Value_o[1];
    rtb_Divide_f[2] = ctl_controller0_P->Constant1_Value_o[2];
  } else {
    rtb_Divide_f[0] = ctl_controller0_U_cmc_msg_f->att_kp[0] /
      ctl_controller0_U_cmc_msg_f->omega_kd[0] * rtb_Product_f[0] +
      ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0];
    rtb_Divide_f[1] = ctl_controller0_U_cmc_msg_f->att_kp[1] /
      ctl_controller0_U_cmc_msg_f->omega_kd[1] * rtb_Product_f[1] +
      ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1];
    rtb_Divide_f[2] = ctl_controller0_U_cmc_msg_f->att_kp[2] /
      ctl_controller0_U_cmc_msg_f->omega_kd[2] * rtb_Product_f[2] +
      ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[2];
  }

  // End of Switch: '<S13>/Switch2'

  // Product: '<S13>/Product3' incorporates:
  //   Inport: '<Root>/cmc_msg'
  //   Product: '<S13>/Product4'
  //   SignalConversion: '<S1>/Signal Conversion'
  //   Sum: '<S13>/Sum'

  rtb_Product3_n[0] = ((rtb_Gain_ci[0] + rtb_Divide_f[0]) -
                       rtb_Switch_omega_B_ISS_B_idx_0) *
    (ctl_controller0_U_cmc_msg_f->omega_kd[0] *
     ctl_controller0_U_cmc_msg_f->inertia_matrix[0]);
  rtb_Product3_n[1] = ((rtb_Gain_ci[1] + rtb_Divide_f[1]) -
                       rtb_Switch_omega_B_ISS_B_idx_1) *
    (ctl_controller0_U_cmc_msg_f->omega_kd[1] *
     ctl_controller0_U_cmc_msg_f->inertia_matrix[4]);
  rtb_Product3_n[2] = ((rtb_Gain_ci[2] + rtb_Divide_f[2]) -
                       rtb_Switch_omega_B_ISS_B_idx_2) *
    (ctl_controller0_U_cmc_msg_f->omega_kd[2] *
     ctl_controller0_U_cmc_msg_f->inertia_matrix[8]);

  // Product: '<S13>/Product2' incorporates:
  //   Inport: '<Root>/cmc_msg'
  //   SignalConversion: '<S1>/Signal Conversion'

  for (i = 0; i < 3; i++) {
    rtb_Gain_ci[i] = ctl_controller0_U_cmc_msg_f->inertia_matrix[(int32_T)(i + 6)]
      * rtb_Switch_omega_B_ISS_B_idx_2 +
      (ctl_controller0_U_cmc_msg_f->inertia_matrix[(int32_T)(i + 3)] *
       rtb_Switch_omega_B_ISS_B_idx_1 +
       ctl_controller0_U_cmc_msg_f->inertia_matrix[i] *
       rtb_Switch_omega_B_ISS_B_idx_0);
  }

  // End of Product: '<S13>/Product2'

  // Switch: '<S13>/Switch1' incorporates:
  //   Constant: '<S13>/Constant4'

  if ((int32_T)rtb_Switch2 != 0) {
    // Product: '<S13>/Divide3' incorporates:
    //   Constant: '<S2>/Constant1'
    //   Gain: '<S13>/Gain'
    //   Switch: '<S2>/Switch1'

    if (rtb_LogicalOperator2) {
      rtb_LogicalOperator2_0 = ctl_controller0_P->Constant1_Value_a[0];
    } else {
      rtb_LogicalOperator2_0 = rtb_Switch_h_alpha_B_ISS_B_id_1;
    }

    rtb_Sqrt_bt = ctl_controller0_P->tun_alpha_gain[0] * rtb_LogicalOperator2_0;
    if (rtb_LogicalOperator2) {
      rtb_LogicalOperator2_0 = ctl_controller0_P->Constant1_Value_a[1];
    } else {
      rtb_LogicalOperator2_0 = rtb_Switch_h_alpha_B_ISS_B_id_0;
    }

    rtb_SumofElements = ctl_controller0_P->tun_alpha_gain[1] *
      rtb_LogicalOperator2_0;
    if (rtb_LogicalOperator2) {
      rtb_LogicalOperator2_0 = ctl_controller0_P->Constant1_Value_a[2];
    } else {
      rtb_LogicalOperator2_0 = rtb_Switch_h_alpha_B_ISS_B_idx_;
    }

    rtb_LogicalOperator2_0 *= ctl_controller0_P->tun_alpha_gain[2];

    // SignalConversion: '<S38>/TmpSignal ConversionAtProductInport1' incorporates:
    //   Constant: '<S40>/Constant3'
    //   DataTypeConversion: '<S41>/Conversion'
    //   Gain: '<S40>/Gain'
    //   Gain: '<S40>/Gain1'
    //   Gain: '<S40>/Gain2'
    //   Product: '<S38>/Product'

    tmp_3[0] = (real32_T)ctl_controller0_P->Constant3_Value_c;
    tmp_3[1] = rtb_Gain_ci[2];
    tmp_3[2] = ctl_controller0_P->Gain_Gain * rtb_Gain_ci[1];
    tmp_3[3] = ctl_controller0_P->Gain1_Gain * rtb_Gain_ci[2];
    tmp_3[4] = (real32_T)ctl_controller0_P->Constant3_Value_c;
    tmp_3[5] = rtb_Gain_ci[0];
    tmp_3[6] = rtb_Gain_ci[1];
    tmp_3[7] = ctl_controller0_P->Gain2_Gain * rtb_Gain_ci[0];
    tmp_3[8] = (real32_T)ctl_controller0_P->Constant3_Value_c;

    // Product: '<S38>/Product' incorporates:
    //   Inport: '<Root>/cmc_msg'
    //   Product: '<S13>/Divide3'
    //   SignalConversion: '<S1>/Signal Conversion'
    //   Sum: '<S13>/Sum1'

    for (i = 0; i < 3; i++) {
      rtb_Divide_f[i] = (((ctl_controller0_U_cmc_msg_f->inertia_matrix[(int32_T)
                           (i + 3)] * rtb_SumofElements +
                           ctl_controller0_U_cmc_msg_f->inertia_matrix[i] *
                           rtb_Sqrt_bt) +
                          ctl_controller0_U_cmc_msg_f->inertia_matrix[(int32_T)
                          (i + 6)] * rtb_LogicalOperator2_0) + rtb_Product3_n[i])
        - (tmp_3[(int32_T)(i + 6)] * rtb_Switch_omega_B_ISS_B_idx_2 + (tmp_3
            [(int32_T)(i + 3)] * rtb_Switch_omega_B_ISS_B_idx_1 + tmp_3[i] *
            rtb_Switch_omega_B_ISS_B_idx_0));
    }
  } else {
    rtb_Divide_f[0] = ctl_controller0_P->Constant4_Value_f[0];
    rtb_Divide_f[1] = ctl_controller0_P->Constant4_Value_f[1];
    rtb_Divide_f[2] = ctl_controller0_P->Constant4_Value_f[2];
  }

  // End of Switch: '<S13>/Switch1'

  // Sum: '<S59>/Sum1'
  rtb_Gain1_j_idx_0 -= rtb_Switch_P_B_ISS_ISS_idx_0;
  rtb_Gain1_j_idx_1 -= rtb_Switch_P_B_ISS_ISS_idx_1;
  rtb_Switch_P_B_ISS_ISS_idx_0 = rtb_Gain1_j_idx_2 -
    rtb_Switch_P_B_ISS_ISS_idx_2;

  // DataTypeConversion: '<S79>/Conversion' incorporates:
  //   Constant: '<S78>/Constant2'

  for (i = 0; i < 9; i++) {
    rtb_Assignment_h[i] = (real32_T)ctl_controller0_P->Constant2_Value_i[i];
  }

  // End of DataTypeConversion: '<S79>/Conversion'

  // Assignment: '<S78>/Assignment'
  rtb_Assignment_h[0] = rtb_Merge_az[3];

  // Gain: '<S75>/Gain'
  rtb_Gain_ci[0] = ctl_controller0_P->Gain_Gain_is * rtb_Merge_az[0];

  // Assignment: '<S78>/Assignment'
  rtb_Assignment_h[4] = rtb_Merge_az[3];

  // Gain: '<S75>/Gain'
  rtb_Gain_ci[1] = ctl_controller0_P->Gain_Gain_is * rtb_Merge_az[1];

  // Assignment: '<S78>/Assignment'
  rtb_Assignment_h[8] = rtb_Merge_az[3];

  // Gain: '<S75>/Gain'
  rtb_Gain_ci[2] = ctl_controller0_P->Gain_Gain_is * rtb_Merge_az[2];

  // Sum: '<S78>/Sum2' incorporates:
  //   Constant: '<S80>/Constant3'
  //   DataTypeConversion: '<S81>/Conversion'
  //   Gain: '<S80>/Gain'
  //   Gain: '<S80>/Gain1'
  //   Gain: '<S80>/Gain2'

  tmp_4[0] = (real32_T)ctl_controller0_P->Constant3_Value_n;
  tmp_4[1] = rtb_Gain_ci[2];
  tmp_4[2] = ctl_controller0_P->Gain_Gain_m2 * rtb_Gain_ci[1];
  tmp_4[3] = ctl_controller0_P->Gain1_Gain_bs * rtb_Gain_ci[2];
  tmp_4[4] = (real32_T)ctl_controller0_P->Constant3_Value_n;
  tmp_4[5] = rtb_Gain_ci[0];
  tmp_4[6] = rtb_Gain_ci[1];
  tmp_4[7] = ctl_controller0_P->Gain2_Gain_o * rtb_Gain_ci[0];
  tmp_4[8] = (real32_T)ctl_controller0_P->Constant3_Value_n;

  // Concatenate: '<S78>/Matrix Concatenate' incorporates:
  //   Gain: '<S78>/Gain1'
  //   Sum: '<S78>/Sum2'

  for (i = 0; i < 3; i++) {
    rtb_VectorConcatenate[(int32_T)(i << 2)] = rtb_Assignment_h[(int32_T)(3 * i)]
      + tmp_4[(int32_T)(3 * i)];
    rtb_VectorConcatenate[(int32_T)(1 + (int32_T)(i << 2))] = rtb_Assignment_h
      [(int32_T)((int32_T)(3 * i) + 1)] + tmp_4[(int32_T)((int32_T)(3 * i) + 1)];
    rtb_VectorConcatenate[(int32_T)(2 + (int32_T)(i << 2))] = rtb_Assignment_h
      [(int32_T)((int32_T)(3 * i) + 2)] + tmp_4[(int32_T)((int32_T)(3 * i) + 2)];
  }

  rtb_VectorConcatenate[3] = ctl_controller0_P->Gain1_Gain_ed * rtb_Gain_ci[0];
  rtb_VectorConcatenate[7] = ctl_controller0_P->Gain1_Gain_ed * rtb_Gain_ci[1];
  rtb_VectorConcatenate[11] = ctl_controller0_P->Gain1_Gain_ed * rtb_Gain_ci[2];

  // End of Concatenate: '<S78>/Matrix Concatenate'

  // Reshape: '<S76>/Reshape1'
  rtb_VectorConcatenate[12] = rtb_Gain_ci[0];
  rtb_VectorConcatenate[13] = rtb_Gain_ci[1];
  rtb_VectorConcatenate[14] = rtb_Gain_ci[2];
  rtb_VectorConcatenate[15] = rtb_Merge_az[3];

  // Product: '<S76>/Product'
  for (i = 0; i < 4; i++) {
    rtb_Switch_P_B_ISS_ISS_idx_1 = rtb_VectorConcatenate[(int32_T)(i + 12)] *
      rtb_Product1_m[3] + (rtb_VectorConcatenate[(int32_T)(i + 8)] *
      rtb_Product1_m[2] + (rtb_VectorConcatenate[(int32_T)(i + 4)] *
      rtb_Product1_m[1] + rtb_VectorConcatenate[i] * rtb_Product1_m[0]));
    rtb_Merge_k[i] = rtb_Switch_P_B_ISS_ISS_idx_1;
  }

  // End of Product: '<S76>/Product'

  // If: '<S77>/If' incorporates:
  //   Inport: '<S82>/In1'

  if (rtb_Merge_k[3] < 0.0F) {
    // Outputs for IfAction SubSystem: '<S77>/Normalize' incorporates:
    //   ActionPort: '<S83>/Action Port'

    ctl_controller0_Normalize(rtb_Merge_k, rtb_Merge_az,
      (P_Normalize_ctl_controller0_T *)&ctl_controller0_P->Normalize_k);

    // End of Outputs for SubSystem: '<S77>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S77>/No-op' incorporates:
    //   ActionPort: '<S82>/Action Port'

    rtb_Merge_az[0] = rtb_Merge_k[0];
    rtb_Merge_az[1] = rtb_Merge_k[1];
    rtb_Merge_az[2] = rtb_Merge_k[2];
    rtb_Merge_az[3] = rtb_Merge_k[3];

    // End of Outputs for SubSystem: '<S77>/No-op'
  }

  // End of If: '<S77>/If'

  // Sqrt: '<S88>/Sqrt' incorporates:
  //   DotProduct: '<S88>/Dot Product'

  rtb_Sqrt_bt = (real32_T)sqrt((real_T)(((rtb_Merge_az[0] * rtb_Merge_az[0] +
    rtb_Merge_az[1] * rtb_Merge_az[1]) + rtb_Merge_az[2] * rtb_Merge_az[2]) +
    rtb_Merge_az[3] * rtb_Merge_az[3]));

  // If: '<S84>/If' incorporates:
  //   DataTypeConversion: '<S84>/Data Type Conversion'
  //   Inport: '<S86>/In1'

  if ((real_T)rtb_Sqrt_bt > 1.0E-7) {
    // Outputs for IfAction SubSystem: '<S84>/Normalize' incorporates:
    //   ActionPort: '<S87>/Action Port'

    ctl_controller0_Normalize_e(rtb_Merge_az, rtb_Sqrt_bt, rtb_Merge_k);

    // End of Outputs for SubSystem: '<S84>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S84>/No-op' incorporates:
    //   ActionPort: '<S86>/Action Port'

    rtb_Merge_k[3] = rtb_Merge_az[3];

    // End of Outputs for SubSystem: '<S84>/No-op'
  }

  // End of If: '<S84>/If'

  // Sum: '<S59>/Sum3'
  rtb_Sum3_k_idx_0 -= rtb_Switch_V_B_ISS_ISS_idx_0;

  // DiscreteIntegrator: '<S13>/Discrete-Time Integrator'
  rtb_Switch_V_B_ISS_ISS_idx_0 =
    ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0];

  // Sum: '<S59>/Sum4'
  rtb_Sum4_o_idx_0 -= rtb_Switch_omega_B_ISS_B_idx_0;

  // DiscreteIntegrator: '<S12>/Discrete-Time Integrator1'
  rtb_Switch_omega_B_ISS_B_idx_0 =
    ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0];

  // Sum: '<S59>/Sum3'
  rtb_Sum3_k_idx_1 -= rtb_Switch_V_B_ISS_ISS_idx_1;

  // DiscreteIntegrator: '<S13>/Discrete-Time Integrator'
  rtb_Switch_V_B_ISS_ISS_idx_1 =
    ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1];

  // Sum: '<S59>/Sum4'
  rtb_Sum4_o_idx_1 -= rtb_Switch_omega_B_ISS_B_idx_1;

  // DiscreteIntegrator: '<S12>/Discrete-Time Integrator1'
  rtb_Switch_omega_B_ISS_B_idx_1 =
    ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1];

  // Sum: '<S59>/Sum3'
  rtb_Switch_V_B_ISS_ISS_idx_2 = rtb_Sum3_k_idx_2 - rtb_Switch_V_B_ISS_ISS_idx_2;

  // Sum: '<S59>/Sum4'
  rtb_Switch_omega_B_ISS_B_idx_2 = rtb_SumofElements1 -
    rtb_Switch_omega_B_ISS_B_idx_2;

  // BusCreator: '<S3>/bus_creator' incorporates:
  //   DiscreteIntegrator: '<S12>/Discrete-Time Integrator1'
  //   DiscreteIntegrator: '<S13>/Discrete-Time Integrator'

  rtb_Switch_P_B_ISS_ISS_idx_1 =
    ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2];
  rtb_Switch_P_B_ISS_ISS_idx_2 =
    ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[2];

  // Update for UnitDelay: '<S93>/UD'
  ctl_controller0_DW->UD_DSTATE[0] = rtb_TSamp[0];
  ctl_controller0_DW->UD_DSTATE[1] = rtb_TSamp[1];
  ctl_controller0_DW->UD_DSTATE[2] = rtb_TSamp[2];
  ctl_controller0_DW->UD_DSTATE[3] = rtb_TSamp[3];

  // Update for UnitDelay: '<S90>/UD'
  ctl_controller0_DW->UD_DSTATE_e[0] = rtb_Product[0];

  // Update for Delay: '<S10>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE[0] = rtb_Diff[0];

  // Update for Delay: '<S11>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE_i[0] = rtb_SumA21_d_idx_0;

  // Update for UnitDelay: '<S2>/Unit Delay1'
  ctl_controller0_DW->UnitDelay1_DSTATE[0] = rtb_Switch8_idx_0;

  // Update for UnitDelay: '<S90>/UD'
  ctl_controller0_DW->UD_DSTATE_e[1] = rtb_Product[1];

  // Update for Delay: '<S10>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE[1] = rtb_Diff[1];

  // Update for Delay: '<S11>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE_i[1] = rtb_SumA21_d_idx_1;

  // Update for UnitDelay: '<S2>/Unit Delay1'
  ctl_controller0_DW->UnitDelay1_DSTATE[1] = rtb_Switch8_idx_1;

  // Update for UnitDelay: '<S90>/UD'
  ctl_controller0_DW->UD_DSTATE_e[2] = rtb_Product_md;

  // Update for Delay: '<S10>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE[2] = rtb_Diff_e;

  // Update for Delay: '<S11>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE_i[2] = rtb_SumA21_f;

  // Update for UnitDelay: '<S2>/Unit Delay1'
  ctl_controller0_DW->UnitDelay1_DSTATE[2] = rtb_Sum4_o_idx_2;

  // Update for UnitDelay: '<S2>/Unit Delay2'
  ctl_controller0_DW->UnitDelay2_DSTATE[0] = rtb_Merge[0];
  ctl_controller0_DW->UnitDelay2_DSTATE[1] = rtb_Merge[1];
  ctl_controller0_DW->UnitDelay2_DSTATE[2] = rtb_Merge[2];
  ctl_controller0_DW->UnitDelay2_DSTATE[3] = rtb_Merge[3];

  // Update for DiscreteIntegrator: '<S12>/Discrete-Time Integrator1'
  ctl_controller0_DW->DiscreteTimeIntegrator1_PrevRes = (int8_T)rtb_Compare_j;

  // Update for DiscreteIntegrator: '<S13>/Discrete-Time Integrator'
  ctl_controller0_DW->DiscreteTimeIntegrator_PrevRese = (int8_T)rtb_Compare_a;

  // End of Outputs for SubSystem: '<Root>/ctl_controller'

  // Outport: '<Root>/cmd_msg' incorporates:
  //   DataTypeConversion: '<S56>/Data Type Conversion'
  //   Inport: '<Root>/cmc_msg'
  //   SignalConversion: '<S1>/Signal Conversion'

  ctl_controller0_Y_cmd_msg_c->cmd_timestamp_sec = rtb_Switch_h_timestamp_sec;
  ctl_controller0_Y_cmd_msg_c->cmd_timestamp_nsec = rtb_Switch_h_timestamp_nsec;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_cmd_msg_c->cmd_mode =
    ctl_controller0_U_cmc_msg_f->cmc_mode_cmd;
  ctl_controller0_Y_cmd_msg_c->speed_gain_cmd =
    ctl_controller0_U_cmc_msg_f->speed_gain_cmd;
  ctl_controller0_Y_cmd_msg_c->cmd_B_inuse = (uint8_T)rtb_LogicalOperator2_c;

  // Update for DiscreteIntegrator: '<S12>/Discrete-Time Integrator1' incorporates:
  //   Inport: '<Root>/cmc_msg'
  //   Product: '<S12>/Divide2'
  //   Product: '<S12>/Product1'
  //   SignalConversion: '<S1>/Signal Conversion'

  ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0] +=
    ctl_controller0_U_cmc_msg_f->pos_ki[0] / ctl_controller0_U_cmc_msg_f->
    vel_kd[0] * rtb_Sum2_idx_0 *
    ctl_controller0_P->DiscreteTimeIntegrator1_gainval;
  if (ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0] >=
      ctl_controller0_P->tun_ctl_pos_sat_upper) {
    ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0] =
      ctl_controller0_P->tun_ctl_pos_sat_upper;
  } else {
    if (ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0] <=
        ctl_controller0_P->tun_ctl_pos_sat_lower) {
      ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0] =
        ctl_controller0_P->tun_ctl_pos_sat_lower;
    }
  }

  // Update for DiscreteIntegrator: '<S13>/Discrete-Time Integrator' incorporates:
  //   Inport: '<Root>/cmc_msg'
  //   Product: '<S13>/Divide2'
  //   Product: '<S13>/Product1'
  //   SignalConversion: '<S1>/Signal Conversion'

  ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0] +=
    ctl_controller0_U_cmc_msg_f->att_ki[0] /
    ctl_controller0_U_cmc_msg_f->omega_kd[0] * rtb_Product_f[0] *
    ctl_controller0_P->DiscreteTimeIntegrator_gainval;
  if (ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0] >=
      ctl_controller0_P->tun_ctl_att_sat_upper) {
    ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0] =
      ctl_controller0_P->tun_ctl_att_sat_upper;
  } else {
    if (ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0] <=
        ctl_controller0_P->tun_ctl_att_sat_lower) {
      ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0] =
        ctl_controller0_P->tun_ctl_att_sat_lower;
    }
  }

  // End of Outputs for SubSystem: '<Root>/ctl_controller'

  // Outport: '<Root>/cmd_msg'
  ctl_controller0_Y_cmd_msg_c->traj_pos[0] = normA;
  ctl_controller0_Y_cmd_msg_c->traj_vel[0] = b_s;
  ctl_controller0_Y_cmd_msg_c->traj_accel[0] = rtb_Switch_h_A_B_ISS_ISS[0];

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  // Update for DiscreteIntegrator: '<S12>/Discrete-Time Integrator1' incorporates:
  //   Inport: '<Root>/cmc_msg'
  //   Product: '<S12>/Divide2'
  //   Product: '<S12>/Product1'
  //   SignalConversion: '<S1>/Signal Conversion'

  ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1] +=
    ctl_controller0_U_cmc_msg_f->pos_ki[1] / ctl_controller0_U_cmc_msg_f->
    vel_kd[1] * rtb_Sum2_idx_1 *
    ctl_controller0_P->DiscreteTimeIntegrator1_gainval;
  if (ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1] >=
      ctl_controller0_P->tun_ctl_pos_sat_upper) {
    ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1] =
      ctl_controller0_P->tun_ctl_pos_sat_upper;
  } else {
    if (ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1] <=
        ctl_controller0_P->tun_ctl_pos_sat_lower) {
      ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1] =
        ctl_controller0_P->tun_ctl_pos_sat_lower;
    }
  }

  // Update for DiscreteIntegrator: '<S13>/Discrete-Time Integrator' incorporates:
  //   Inport: '<Root>/cmc_msg'
  //   Product: '<S13>/Divide2'
  //   Product: '<S13>/Product1'
  //   SignalConversion: '<S1>/Signal Conversion'

  ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1] +=
    ctl_controller0_U_cmc_msg_f->att_ki[1] /
    ctl_controller0_U_cmc_msg_f->omega_kd[1] * rtb_Product_f[1] *
    ctl_controller0_P->DiscreteTimeIntegrator_gainval;
  if (ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1] >=
      ctl_controller0_P->tun_ctl_att_sat_upper) {
    ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1] =
      ctl_controller0_P->tun_ctl_att_sat_upper;
  } else {
    if (ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1] <=
        ctl_controller0_P->tun_ctl_att_sat_lower) {
      ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1] =
        ctl_controller0_P->tun_ctl_att_sat_lower;
    }
  }

  // End of Outputs for SubSystem: '<Root>/ctl_controller'

  // Outport: '<Root>/cmd_msg'
  ctl_controller0_Y_cmd_msg_c->traj_pos[1] = rtb_BusCreator1_traj_pos_idx_1;
  ctl_controller0_Y_cmd_msg_c->traj_vel[1] = rtb_BusCreator1_traj_vel_idx_1;
  ctl_controller0_Y_cmd_msg_c->traj_accel[1] = rtb_Switch_h_A_B_ISS_ISS[1];

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  // Update for DiscreteIntegrator: '<S12>/Discrete-Time Integrator1' incorporates:
  //   Inport: '<Root>/cmc_msg'
  //   Product: '<S12>/Divide2'
  //   Product: '<S12>/Product1'
  //   SignalConversion: '<S1>/Signal Conversion'

  ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2] +=
    ctl_controller0_U_cmc_msg_f->pos_ki[2] / ctl_controller0_U_cmc_msg_f->
    vel_kd[2] * rtb_Sum2_idx_2 *
    ctl_controller0_P->DiscreteTimeIntegrator1_gainval;
  if (ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2] >=
      ctl_controller0_P->tun_ctl_pos_sat_upper) {
    ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2] =
      ctl_controller0_P->tun_ctl_pos_sat_upper;
  } else {
    if (ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2] <=
        ctl_controller0_P->tun_ctl_pos_sat_lower) {
      ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2] =
        ctl_controller0_P->tun_ctl_pos_sat_lower;
    }
  }

  // Update for DiscreteIntegrator: '<S13>/Discrete-Time Integrator' incorporates:
  //   Inport: '<Root>/cmc_msg'
  //   Product: '<S13>/Divide2'
  //   Product: '<S13>/Product1'
  //   SignalConversion: '<S1>/Signal Conversion'

  ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[2] +=
    ctl_controller0_U_cmc_msg_f->att_ki[2] /
    ctl_controller0_U_cmc_msg_f->omega_kd[2] * rtb_Product_f[2] *
    ctl_controller0_P->DiscreteTimeIntegrator_gainval;
  if (ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[2] >=
      ctl_controller0_P->tun_ctl_att_sat_upper) {
    ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[2] =
      ctl_controller0_P->tun_ctl_att_sat_upper;
  } else {
    if (ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[2] <=
        ctl_controller0_P->tun_ctl_att_sat_lower) {
      ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[2] =
        ctl_controller0_P->tun_ctl_att_sat_lower;
    }
  }

  // End of Outputs for SubSystem: '<Root>/ctl_controller'

  // Outport: '<Root>/cmd_msg' incorporates:
  //   BusCreator: '<S4>/Bus Creator1'

  ctl_controller0_Y_cmd_msg_c->traj_pos[2] = rtb_Gain1_j_idx_2;
  ctl_controller0_Y_cmd_msg_c->traj_vel[2] = rtb_Sum3_k_idx_2;
  ctl_controller0_Y_cmd_msg_c->traj_accel[2] = rtb_Switch_h_A_B_ISS_ISS[2];

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_cmd_msg_c->traj_quat[0] = rtb_Product1_m[0];
  ctl_controller0_Y_cmd_msg_c->traj_quat[1] = rtb_Product1_m[1];
  ctl_controller0_Y_cmd_msg_c->traj_quat[2] = rtb_Product1_m[2];
  ctl_controller0_Y_cmd_msg_c->traj_quat[3] = rtb_Product1_m[3];

  // Product: '<S13>/Divide' incorporates:
  //   Inport: '<Root>/cmc_msg'
  //   Outport: '<Root>/ctl_msg'
  //   SignalConversion: '<S1>/Signal Conversion'

  rt_mldivide_U1f3x3_U2f_XeZWzB4d(ctl_controller0_U_cmc_msg_f->inertia_matrix,
    rtb_Product3_n, ctl_controller0_Y_ctl_msg_o->body_alpha_cmd);

  // Trigonometry: '<S13>/Trigonometric Function'
  if (rtb_Product_f[3] > 1.0F) {
    rtb_Sqrt_bt = 1.0F;
  } else if (rtb_Product_f[3] < -1.0F) {
    rtb_Sqrt_bt = -1.0F;
  } else {
    rtb_Sqrt_bt = rtb_Product_f[3];
  }

  // Outport: '<Root>/ctl_msg' incorporates:
  //   BusCreator: '<S3>/bus_creator'
  //   Gain: '<S13>/Gain2'
  //   Trigonometry: '<S13>/Trigonometric Function'

  ctl_controller0_Y_ctl_msg_o->att_err_mag = ctl_controller0_P->Gain2_Gain_j *
    (real32_T)acos((real_T)rtb_Sqrt_bt);
  ctl_controller0_Y_ctl_msg_o->ctl_status = rtb_Switch2;

  // End of Outputs for SubSystem: '<Root>/ctl_controller'

  // Outport: '<Root>/cmd_msg'
  ctl_controller0_Y_cmd_msg_c->traj_omega[0] = rtb_BusCreator1_traj_omega_idx_;
  ctl_controller0_Y_cmd_msg_c->traj_alpha[0] = rtb_Switch_h_alpha_B_ISS_B_id_1;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  // Outport: '<Root>/ctl_msg' incorporates:
  //   BusCreator: '<S3>/bus_creator'
  //   Inport: '<Root>/cmc_msg'
  //   Product: '<S12>/Divide'
  //   SignalConversion: '<S1>/Signal Conversion'

  ctl_controller0_Y_ctl_msg_o->body_force_cmd[0] = rtb_Switch_c_idx_0;
  ctl_controller0_Y_ctl_msg_o->body_accel_cmd[0] = 1.0F /
    ctl_controller0_U_cmc_msg_f->mass * rtb_Switch_c_idx_0;
  ctl_controller0_Y_ctl_msg_o->pos_err[0] = rtb_Sum2_idx_0;

  // End of Outputs for SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_o->pos_err_int[0] = rtb_Switch_omega_B_ISS_B_idx_0;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_o->body_torque_cmd[0] = rtb_Divide_f[0];
  ctl_controller0_Y_ctl_msg_o->att_err[0] = rtb_Product_f[0];

  // End of Outputs for SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_o->att_err_int[0] = rtb_Switch_V_B_ISS_ISS_idx_0;

  // Outport: '<Root>/cmd_msg'
  ctl_controller0_Y_cmd_msg_c->traj_omega[1] = rtb_BusCreator1_traj_omega_id_0;
  ctl_controller0_Y_cmd_msg_c->traj_alpha[1] = rtb_Switch_h_alpha_B_ISS_B_id_0;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  // Outport: '<Root>/ctl_msg' incorporates:
  //   BusCreator: '<S3>/bus_creator'
  //   Inport: '<Root>/cmc_msg'
  //   Product: '<S12>/Divide'
  //   SignalConversion: '<S1>/Signal Conversion'

  ctl_controller0_Y_ctl_msg_o->body_force_cmd[1] = rtb_Switch_c_idx_1;
  ctl_controller0_Y_ctl_msg_o->body_accel_cmd[1] = 1.0F /
    ctl_controller0_U_cmc_msg_f->mass * rtb_Switch_c_idx_1;
  ctl_controller0_Y_ctl_msg_o->pos_err[1] = rtb_Sum2_idx_1;

  // End of Outputs for SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_o->pos_err_int[1] = rtb_Switch_omega_B_ISS_B_idx_1;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_o->body_torque_cmd[1] = rtb_Divide_f[1];
  ctl_controller0_Y_ctl_msg_o->att_err[1] = rtb_Product_f[1];

  // End of Outputs for SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_o->att_err_int[1] = rtb_Switch_V_B_ISS_ISS_idx_1;

  // Outport: '<Root>/cmd_msg'
  ctl_controller0_Y_cmd_msg_c->traj_omega[2] = rtb_SumofElements1;
  ctl_controller0_Y_cmd_msg_c->traj_alpha[2] = rtb_Switch_h_alpha_B_ISS_B_idx_;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  // Outport: '<Root>/ctl_msg' incorporates:
  //   BusCreator: '<S3>/bus_creator'
  //   DotProduct: '<S73>/Dot Product'
  //   Inport: '<Root>/cmc_msg'
  //   Product: '<S12>/Divide'
  //   SignalConversion: '<S1>/Signal Conversion'
  //   Sqrt: '<S73>/Sqrt'

  ctl_controller0_Y_ctl_msg_o->body_force_cmd[2] = rtb_Switch_c_idx_2;
  ctl_controller0_Y_ctl_msg_o->body_accel_cmd[2] = 1.0F /
    ctl_controller0_U_cmc_msg_f->mass * rtb_Switch_c_idx_2;
  ctl_controller0_Y_ctl_msg_o->pos_err[2] = rtb_Sum2_idx_2;

  // End of Outputs for SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_o->pos_err_int[2] = rtb_Switch_P_B_ISS_ISS_idx_1;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_o->body_torque_cmd[2] = rtb_Divide_f[2];
  ctl_controller0_Y_ctl_msg_o->att_err[2] = rtb_Product_f[2];

  // End of Outputs for SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_o->att_err_int[2] = rtb_Switch_P_B_ISS_ISS_idx_2;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_o->traj_error_pos = (real32_T)sqrt((real_T)
    ((rtb_Gain1_j_idx_0 * rtb_Gain1_j_idx_0 + rtb_Gain1_j_idx_1 *
      rtb_Gain1_j_idx_1) + rtb_Switch_P_B_ISS_ISS_idx_0 *
     rtb_Switch_P_B_ISS_ISS_idx_0));

  // Trigonometry: '<S59>/Trigonometric Function'
  if (rtb_Merge_k[3] > 1.0F) {
    rtb_Switch_P_B_ISS_ISS_idx_1 = 1.0F;
  } else if (rtb_Merge_k[3] < -1.0F) {
    rtb_Switch_P_B_ISS_ISS_idx_1 = -1.0F;
  } else {
    rtb_Switch_P_B_ISS_ISS_idx_1 = rtb_Merge_k[3];
  }

  // Outport: '<Root>/ctl_msg' incorporates:
  //   Abs: '<S59>/Abs'
  //   DotProduct: '<S72>/Dot Product'
  //   DotProduct: '<S74>/Dot Product'
  //   Gain: '<S59>/Gain'
  //   Sqrt: '<S72>/Sqrt'
  //   Sqrt: '<S74>/Sqrt'
  //   Trigonometry: '<S59>/Trigonometric Function'

  ctl_controller0_Y_ctl_msg_o->traj_error_att = ctl_controller0_P->Gain_Gain_hg *
    (real32_T)fabs((real_T)(real32_T)acos((real_T)rtb_Switch_P_B_ISS_ISS_idx_1));
  ctl_controller0_Y_ctl_msg_o->traj_error_vel = (real32_T)sqrt((real_T)
    ((rtb_Sum3_k_idx_0 * rtb_Sum3_k_idx_0 + rtb_Sum3_k_idx_1 * rtb_Sum3_k_idx_1)
     + rtb_Switch_V_B_ISS_ISS_idx_2 * rtb_Switch_V_B_ISS_ISS_idx_2));
  ctl_controller0_Y_ctl_msg_o->traj_error_omega = (real32_T)sqrt((real_T)
    ((rtb_Sum4_o_idx_0 * rtb_Sum4_o_idx_0 + rtb_Sum4_o_idx_1 * rtb_Sum4_o_idx_1)
     + rtb_Switch_omega_B_ISS_B_idx_2 * rtb_Switch_omega_B_ISS_B_idx_2));

  // End of Outputs for SubSystem: '<Root>/ctl_controller'
}

// Model initialize function
void ctl_controller0_initialize(RT_MODEL_ctl_controller0_T *const
  ctl_controller0_M, kfl_msg *ctl_controller0_U_kfl_msg_l, cmc_msg
  *ctl_controller0_U_cmc_msg_f, ex_time_msg *ctl_controller0_U_ex_time, env_msg *
  ctl_controller0_U_env_msg_h, cmd_msg *ctl_controller0_Y_cmd_msg_c, ctl_msg
  *ctl_controller0_Y_ctl_msg_o)
{
  P_ctl_controller0_T *ctl_controller0_P = ((P_ctl_controller0_T *)
    ctl_controller0_M->defaultParam);
  DW_ctl_controller0_T *ctl_controller0_DW = ((DW_ctl_controller0_T *)
    ctl_controller0_M->dwork);

  // Start for Atomic SubSystem: '<Root>/ctl_controller'

  // Start for Iterator SubSystem: '<S5>/For Each Subsystem'
  ctl_cont_ForEachSubsystem_Start(3, ctl_controller0_DW->ForEachSubsystem);

  // End of Start for SubSystem: '<S5>/For Each Subsystem'

  // Start for Iterator SubSystem: '<S5>/For Each Subsystem1'
  ctl_cont_ForEachSubsystem_Start(3, ctl_controller0_DW->ForEachSubsystem1);

  // End of Start for SubSystem: '<S5>/For Each Subsystem1'

  // End of Start for SubSystem: '<Root>/ctl_controller'

  // SystemInitialize for Atomic SubSystem: '<Root>/ctl_controller'
  // InitializeConditions for UnitDelay: '<S93>/UD'
  ctl_controller0_DW->UD_DSTATE[0] =
    ctl_controller0_P->DiscreteDerivative_ICPrevScaled;
  ctl_controller0_DW->UD_DSTATE[1] =
    ctl_controller0_P->DiscreteDerivative_ICPrevScaled;
  ctl_controller0_DW->UD_DSTATE[2] =
    ctl_controller0_P->DiscreteDerivative_ICPrevScaled;
  ctl_controller0_DW->UD_DSTATE[3] =
    ctl_controller0_P->DiscreteDerivative_ICPrevScaled;

  // InitializeConditions for UnitDelay: '<S90>/UD'
  ctl_controller0_DW->UD_DSTATE_e[0] =
    ctl_controller0_P->DiscreteDerivative_ICPrevScal_k;

  // InitializeConditions for Delay: '<S10>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE[0] =
    ctl_controller0_P->Delay11_InitialCondition;

  // InitializeConditions for Delay: '<S11>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE_i[0] =
    ctl_controller0_P->Delay11_InitialCondition_k;

  // InitializeConditions for UnitDelay: '<S2>/Unit Delay1'
  ctl_controller0_DW->UnitDelay1_DSTATE[0] =
    ctl_controller0_P->UnitDelay1_InitialCondition;

  // InitializeConditions for UnitDelay: '<S90>/UD'
  ctl_controller0_DW->UD_DSTATE_e[1] =
    ctl_controller0_P->DiscreteDerivative_ICPrevScal_k;

  // InitializeConditions for Delay: '<S10>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE[1] =
    ctl_controller0_P->Delay11_InitialCondition;

  // InitializeConditions for Delay: '<S11>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE_i[1] =
    ctl_controller0_P->Delay11_InitialCondition_k;

  // InitializeConditions for UnitDelay: '<S2>/Unit Delay1'
  ctl_controller0_DW->UnitDelay1_DSTATE[1] =
    ctl_controller0_P->UnitDelay1_InitialCondition;

  // InitializeConditions for UnitDelay: '<S90>/UD'
  ctl_controller0_DW->UD_DSTATE_e[2] =
    ctl_controller0_P->DiscreteDerivative_ICPrevScal_k;

  // InitializeConditions for Delay: '<S10>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE[2] =
    ctl_controller0_P->Delay11_InitialCondition;

  // InitializeConditions for Delay: '<S11>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE_i[2] =
    ctl_controller0_P->Delay11_InitialCondition_k;

  // InitializeConditions for UnitDelay: '<S2>/Unit Delay1'
  ctl_controller0_DW->UnitDelay1_DSTATE[2] =
    ctl_controller0_P->UnitDelay1_InitialCondition;

  // InitializeConditions for UnitDelay: '<S2>/Unit Delay2'
  ctl_controller0_DW->UnitDelay2_DSTATE[0] =
    ctl_controller0_P->UnitDelay2_InitialCondition;
  ctl_controller0_DW->UnitDelay2_DSTATE[1] =
    ctl_controller0_P->UnitDelay2_InitialCondition;
  ctl_controller0_DW->UnitDelay2_DSTATE[2] =
    ctl_controller0_P->UnitDelay2_InitialCondition;
  ctl_controller0_DW->UnitDelay2_DSTATE[3] =
    ctl_controller0_P->UnitDelay2_InitialCondition;

  // InitializeConditions for DiscreteIntegrator: '<S12>/Discrete-Time Integrator1' 
  ctl_controller0_DW->DiscreteTimeIntegrator1_PrevRes = 0;
  ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0] =
    ctl_controller0_P->DiscreteTimeIntegrator1_IC[0];

  // InitializeConditions for DiscreteIntegrator: '<S13>/Discrete-Time Integrator' 
  ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0] =
    ctl_controller0_P->DiscreteTimeIntegrator_IC[0];

  // InitializeConditions for DiscreteIntegrator: '<S12>/Discrete-Time Integrator1' 
  ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1] =
    ctl_controller0_P->DiscreteTimeIntegrator1_IC[1];

  // InitializeConditions for DiscreteIntegrator: '<S13>/Discrete-Time Integrator' 
  ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1] =
    ctl_controller0_P->DiscreteTimeIntegrator_IC[1];

  // InitializeConditions for DiscreteIntegrator: '<S12>/Discrete-Time Integrator1' 
  ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2] =
    ctl_controller0_P->DiscreteTimeIntegrator1_IC[2];

  // InitializeConditions for DiscreteIntegrator: '<S13>/Discrete-Time Integrator' 
  ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[2] =
    ctl_controller0_P->DiscreteTimeIntegrator_IC[2];
  ctl_controller0_DW->DiscreteTimeIntegrator_PrevRese = 0;

  // SystemInitialize for Iterator SubSystem: '<S5>/For Each Subsystem'
  ctl_contr_ForEachSubsystem_Init(3, ctl_controller0_DW->ForEachSubsystem,
    (P_ForEachSubsystem_ctl_contro_T *)&ctl_controller0_P->ForEachSubsystem);

  // End of SystemInitialize for SubSystem: '<S5>/For Each Subsystem'

  // SystemInitialize for Iterator SubSystem: '<S5>/For Each Subsystem1'
  ctl_contr_ForEachSubsystem_Init(3, ctl_controller0_DW->ForEachSubsystem1,
    (P_ForEachSubsystem_ctl_contro_T *)&ctl_controller0_P->ForEachSubsystem1);

  // End of SystemInitialize for SubSystem: '<S5>/For Each Subsystem1'

  // End of SystemInitialize for SubSystem: '<Root>/ctl_controller'
}

// Model terminate function
void ctl_controller0_terminate(RT_MODEL_ctl_controller0_T * ctl_controller0_M)
{
  // model code
  if (ctl_controller0_M->paramIsMalloced) {
    rt_FREE(ctl_controller0_M->defaultParam);
  }

  rt_FREE(ctl_controller0_M->dwork);
  rt_FREE(ctl_controller0_M);
}

// Model data allocation function
RT_MODEL_ctl_controller0_T *ctl_controller0(kfl_msg *ctl_controller0_U_kfl_msg_l,
  cmc_msg *ctl_controller0_U_cmc_msg_f, ex_time_msg *ctl_controller0_U_ex_time,
  env_msg *ctl_controller0_U_env_msg_h, cmd_msg *ctl_controller0_Y_cmd_msg_c,
  ctl_msg *ctl_controller0_Y_ctl_msg_o)
{
  RT_MODEL_ctl_controller0_T *ctl_controller0_M;
  ctl_controller0_M = (RT_MODEL_ctl_controller0_T *) malloc(sizeof
    (RT_MODEL_ctl_controller0_T));
  if (ctl_controller0_M == NULL) {
    return NULL;
  }

  (void) memset((char *)ctl_controller0_M, 0,
                sizeof(RT_MODEL_ctl_controller0_T));

  // parameters
  {
    P_ctl_controller0_T *p;
    static int_T pSeen = 0;

    // only malloc on multiple model instantiation
    if (pSeen == 1 ) {
      p = (P_ctl_controller0_T *) malloc(sizeof(P_ctl_controller0_T));
      rt_VALIDATE_MEMORY(ctl_controller0_M,p);
      (void) memcpy(p, &ctl_controller0_P,
                    sizeof(P_ctl_controller0_T));
      ctl_controller0_M->paramIsMalloced = (true);
    } else {
      p = &ctl_controller0_P;
      ctl_controller0_M->paramIsMalloced = (false);
      pSeen = 1;
    }

    ctl_controller0_M->defaultParam = (p);
  }

  // states (dwork)
  {
    DW_ctl_controller0_T *dwork = (DW_ctl_controller0_T *) malloc(sizeof
      (DW_ctl_controller0_T));
    rt_VALIDATE_MEMORY(ctl_controller0_M,dwork);
    ctl_controller0_M->dwork = (dwork);
  }

  {
    P_ctl_controller0_T *ctl_controller0_P = ((P_ctl_controller0_T *)
      ctl_controller0_M->defaultParam);
    DW_ctl_controller0_T *ctl_controller0_DW = ((DW_ctl_controller0_T *)
      ctl_controller0_M->dwork);

    // initialize non-finites
    rt_InitInfAndNaN(sizeof(real_T));

    // states (dwork)
    (void) memset((void *)ctl_controller0_DW, 0,
                  sizeof(DW_ctl_controller0_T));

    // external inputs
    *ctl_controller0_U_kfl_msg_l = ctl_controller0_rtZkfl_msg;
    *ctl_controller0_U_cmc_msg_f = ctl_controller0_rtZcmc_msg;
    *ctl_controller0_U_ex_time = ctl_controller0_rtZex_time_msg;
    *ctl_controller0_U_env_msg_h = ctl_controller0_rtZenv_msg;

    // external outputs
    (*ctl_controller0_Y_cmd_msg_c) = ctl_controller0_rtZcmd_msg;
    (*ctl_controller0_Y_ctl_msg_o) = ctl_controller0_rtZctl_msg;
  }

  return ctl_controller0_M;
}

//
// File trailer for generated code.
//
// [EOF]
//
