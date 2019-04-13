//
// File: ctl_controller0.cpp
//
// Code generated for Simulink model 'ctl_controller0'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Thu Mar  7 13:22:34 2019
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

const ctl_input_msg ctl_controller0_rtZctl_input_ms = { { 0.0F, 0.0F, 0.0F, 0.0F
  },                                   // est_quat_ISS2B
  { 0.0F, 0.0F, 0.0F },                // est_omega_B_ISS_B
  { 0.0F, 0.0F, 0.0F },                // est_V_B_ISS_ISS
  { 0.0F, 0.0F, 0.0F },                // est_P_B_ISS_ISS
  0U,                                  // est_confidence
  { 0U,                                // timestamp_sec
    0U,                                // timestamp_nsec
    { 0.0F, 0.0F, 0.0F },              // P_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F },              // V_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F },              // A_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F, 0.0F },        // quat_ISS2B
    { 0.0F, 0.0F, 0.0F },              // omega_B_ISS_B
    { 0.0F, 0.0F, 0.0F }               // alpha_B_ISS_B
  },                                   // cmd_state_a
  { 0U,                                // timestamp_sec
    0U,                                // timestamp_nsec
    { 0.0F, 0.0F, 0.0F },              // P_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F },              // V_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F },              // A_B_ISS_ISS
    { 0.0F, 0.0F, 0.0F, 0.0F },        // quat_ISS2B
    { 0.0F, 0.0F, 0.0F },              // omega_B_ISS_B
    { 0.0F, 0.0F, 0.0F }               // alpha_B_ISS_B
  },                                   // cmd_state_b
  0U,                                  // ctl_mode_cmd
  0U,                                  // current_time_sec
  0U,                                  // current_time_nsec
  0U,                                  // speed_gain_cmd
  { 0.0F, 0.0F, 0.0F },                // att_kp
  { 0.0F, 0.0F, 0.0F },                // att_ki
  { 0.0F, 0.0F, 0.0F },                // omega_kd
  { 0.0F, 0.0F, 0.0F },                // pos_kp
  { 0.0F, 0.0F, 0.0F },                // pos_ki
  { 0.0F, 0.0F, 0.0F },                // vel_kd
  { 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F },// inertia_matrix
  0.0F                                 // mass
};

//
// Output and update for action system:
//    '<S18>/Normalize'
//    '<S66>/Normalize'
//    '<S85>/Normalize'
//    '<S99>/Normalize'
//
void ctl_controller0_Normalize(const real32_T rtu_q_in[4], real32_T
  rty_positive_scalar_q[4], P_Normalize_ctl_controller0_T *localP)
{
  // Product: '<S24>/Product' incorporates:
  //   Constant: '<S24>/Constant1'
  //   DataTypeConversion: '<S26>/Conversion'

  rty_positive_scalar_q[0] = rtu_q_in[0] * (real32_T)localP->Constant1_Value;
  rty_positive_scalar_q[1] = rtu_q_in[1] * (real32_T)localP->Constant1_Value;
  rty_positive_scalar_q[2] = rtu_q_in[2] * (real32_T)localP->Constant1_Value;
  rty_positive_scalar_q[3] = rtu_q_in[3] * (real32_T)localP->Constant1_Value;
}

//
// Termination for action system:
//    '<S18>/Normalize'
//    '<S66>/Normalize'
//    '<S85>/Normalize'
//    '<S99>/Normalize'
//
void ctl_controller0_Normalize_Term(void)
{
}

//
// Output and update for action system:
//    '<S25>/Normalize'
//    '<S73>/Normalize'
//    '<S88>/Normalize'
//    '<S106>/Normalize'
//
void ctl_controller0_Normalize_e(const real32_T rtu_Vec[4], real32_T
  rtu_Magnitude, real32_T rty_Normalized_Vec[4])
{
  // Product: '<S28>/Divide'
  rty_Normalized_Vec[0] = rtu_Vec[0] / rtu_Magnitude;
  rty_Normalized_Vec[1] = rtu_Vec[1] / rtu_Magnitude;
  rty_Normalized_Vec[2] = rtu_Vec[2] / rtu_Magnitude;
  rty_Normalized_Vec[3] = rtu_Vec[3] / rtu_Magnitude;
}

//
// Termination for action system:
//    '<S25>/Normalize'
//    '<S73>/Normalize'
//    '<S88>/Normalize'
//    '<S106>/Normalize'
//
void ctl_controller_Normalize_j_Term(void)
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
    // InitializeConditions for DiscreteTransferFcn: '<S113>/3 Hz Low Pass'
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
  //   ForEach: '<S113>/For Each'

  for (ForEach_itr = 0; ForEach_itr < NumIters; ForEach_itr++) {
    // DiscreteTransferFcn: '<S113>/3 Hz Low Pass' incorporates:
    //   ForEachSliceSelector: '<S113>/ImpSel_InsertedFor_X_at_outport_0'

    uHzLowPass_tmp = (rtu_X[ForEach_itr] - localP->
                      CoreSubsys.uHzLowPass_DenCoef[1] * localDW[ForEach_itr].
                      CoreSubsys.uHzLowPass_states) /
      localP->CoreSubsys.uHzLowPass_DenCoef[0];
    rtb_uHzLowPass = localP->CoreSubsys.uHzLowPass_NumCoef[0] * uHzLowPass_tmp +
      localP->CoreSubsys.uHzLowPass_NumCoef[1] * localDW[ForEach_itr].
      CoreSubsys.uHzLowPass_states;

    // Update for DiscreteTransferFcn: '<S113>/3 Hz Low Pass'
    localDW[ForEach_itr].CoreSubsys.uHzLowPass_states = uHzLowPass_tmp;

    // Switch: '<S113>/Switch' incorporates:
    //   Constant: '<S113>/Constant'

    if (rtp_filt_enable != 0.0F) {
      // ForEachSliceAssignment: '<S113>/ImpAsg_InsertedFor_Y_at_inport_0'
      rty_Y[ForEach_itr] = rtb_uHzLowPass;
    } else {
      // ForEachSliceAssignment: '<S113>/ImpAsg_InsertedFor_Y_at_inport_0' incorporates:
      //   ForEachSliceSelector: '<S113>/ImpSel_InsertedFor_X_at_outport_0'

      rty_Y[ForEach_itr] = rtu_X[ForEach_itr];
    }

    // End of Switch: '<S113>/Switch'
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
  ctl_input_msg *ctl_controller0_U_ctl_input_msg_l, cmd_msg
  *ctl_controller0_Y_cmd_msg_f, ctl_msg *ctl_controller0_Y_ctl_msg_n)
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
  real32_T rtb_TSamp[4];
  real32_T rtb_Product[3];
  real32_T rtb_Product5[16];
  real32_T rtb_Product_b[4];
  real32_T rtb_Sqrt;
  real32_T rtb_SumofElements;
  real32_T rtb_SumofElements1;
  boolean_T rtb_LogicalOperator2_c;
  uint8_T rtb_Switch3;
  real32_T rtb_VectorConcatenate[16];
  real32_T rtb_VectorConcatenate_m[16];
  real32_T rtb_Product1_m[4];
  real32_T rtb_Merge_pa[4];
  real32_T rtb_Divide_f[3];
  uint8_T rtb_Switch12;
  boolean_T rtb_LogicalOperator2;
  real32_T rtb_Assignment_l[9];
  boolean_T rtb_Compare_j;
  real32_T rtb_Switch_b[3];
  boolean_T rtb_Compare_fc;
  real32_T rtb_Merge[4];
  real32_T rtb_Sum_f[3];
  real32_T rtb_Merge_j[4];
  real32_T rtb_y[16];
  real32_T rtb_Diff[3];
  uint32_T rtb_Switch_cmd_state_a_timestam;
  uint32_T rtb_Switch_cmd_state_a_timest_0;
  uint32_T rtb_Switch_cmd_state_b_timestam;
  uint32_T rtb_Switch_cmd_state_b_timest_0;
  uint8_T rtb_Switch_ctl_mode_cmd;
  uint32_T rtb_Switch_current_time_sec;
  uint32_T rtb_Switch_current_time_nsec;
  uint8_T rtb_Switch_speed_gain_cmd;
  real32_T rtb_Switch_vel_kd[3];
  real32_T rtb_Switch_inertia_matrix[9];
  real32_T rtb_Switch_mass;
  int32_T i;
  real32_T rtb_Product5_0[16];
  real32_T rtb_Assignment_b[12];
  real32_T tmp[12];
  int32_T i_0;
  real32_T rtb_VectorConcatenate_p[16];
  real32_T tmp_0[9];
  real32_T rtb_Merge_j_0[9];
  real32_T rtb_Assignment_b_0[9];
  real32_T rtb_Assignment_b_1[3];
  real32_T tmp_1[9];
  real32_T tmp_2[9];
  real32_T tmp_3[9];
  real32_T tmp_4[9];
  real32_T rtb_Product_f;
  real32_T rtb_Sum4_p;
  real32_T rtb_SumA21_f;
  real32_T rtb_Sum4_o_idx_2;
  real32_T rtb_Sum4_o_idx_1;
  real32_T rtb_Sum4_o_idx_0;
  real32_T rtb_Switch_cmd_state_b_alpha_B_;
  real32_T rtb_Switch_cmd_state_b_alpha__0;
  real32_T rtb_Switch_cmd_state_b_alpha__1;
  real32_T rtb_Switch_est_omega_B_ISS_B_id;
  real32_T rtb_Switch_omega_kd_idx_0;
  real32_T rtb_Switch_est_omega_B_ISS_B__0;
  real32_T rtb_Switch_omega_kd_idx_1;
  real32_T rtb_Switch_est_omega_B_ISS_B__1;
  real32_T rtb_Switch_omega_kd_idx_2;
  real32_T rtb_Sum3_k_idx_0;
  real32_T rtb_Switch_est_V_B_ISS_ISS_idx_;
  real32_T rtb_Switch_est_P_B_ISS_ISS_idx_;
  real32_T rtb_Switch_cmd_state_a_A_B_ISS_;
  real32_T rtb_Switch_est_V_B_ISS_ISS_id_0;
  real32_T rtb_Switch_est_P_B_ISS_ISS_id_0;
  real32_T rtb_Switch_cmd_state_a_P_B_ISS_;
  real32_T rtb_Switch_cmd_state_a_V_B_ISS_;
  real32_T rtb_Switch_cmd_state_a_A_B_IS_0;
  real32_T rtb_Switch_est_V_B_ISS_ISS_id_1;
  real32_T rtb_Switch_est_P_B_ISS_ISS_id_1;
  real32_T rtb_Switch_cmd_state_a_P_B_IS_0;
  real32_T rtb_Switch_cmd_state_a_A_B_IS_1;
  real32_T rtb_Switch8_idx_0;
  real32_T rtb_Sum2_idx_0;
  real32_T rtb_Gain1_j_idx_0;
  real32_T rtb_Switch_c_idx_0;
  real32_T rtb_Switch8_idx_1;
  real32_T rtb_Sum2_idx_1;
  real32_T rtb_Gain1_j_idx_1;
  real32_T rtb_Switch_c_idx_1;
  real32_T rtb_Switch8_idx_2;
  real32_T rtb_Sum2_idx_2;
  real32_T rtb_Gain1_j_idx_2;
  real32_T rtb_Switch_c_idx_2;
  real32_T rtb_Switch_att_ki_idx_0;
  real32_T rtb_Switch_pos_ki_idx_0;
  real32_T rtb_Switch_att_ki_idx_1;
  real32_T rtb_Switch_pos_ki_idx_1;
  real32_T rtb_Switch_att_ki_idx_2;
  real32_T rtb_Switch_pos_ki_idx_2;
  real32_T rtb_SumA21_idx_0;
  real32_T rtb_SumA21_idx_1;
  real32_T rtb_SumA21_d_idx_0;
  real32_T rtb_SumA21_d_idx_1;
  real32_T rtb_Sum3_k_idx_1;
  real32_T rtb_Sum3_k_idx_2;
  boolean_T exitg1;
  boolean_T exitg2;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  // DataTypeConversion: '<S117>/Conversion' incorporates:
  //   Constant: '<S116>/Constant2'

  for (i = 0; i < 9; i++) {
    rtb_Assignment_h[i] = (real32_T)ctl_controller0_P->Constant2_Value[i];
  }

  // End of DataTypeConversion: '<S117>/Conversion'

  // Assignment: '<S116>/Assignment' incorporates:
  //   Inport: '<Root>/ctl_input_msg'

  rtb_Assignment_h[0] = ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[3];
  rtb_Assignment_h[4] = ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[3];
  rtb_Assignment_h[8] = ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[3];

  // SampleTimeMath: '<S115>/TSamp' incorporates:
  //   Inport: '<Root>/ctl_input_msg'
  //
  //  About '<S115>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_TSamp[0] = ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[0] *
    ctl_controller0_P->TSamp_WtEt;
  rtb_TSamp[1] = ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[1] *
    ctl_controller0_P->TSamp_WtEt;
  rtb_TSamp[2] = ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[2] *
    ctl_controller0_P->TSamp_WtEt;
  rtb_TSamp[3] = ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[3] *
    ctl_controller0_P->TSamp_WtEt;

  // Sum: '<S116>/Sum2' incorporates:
  //   Constant: '<S118>/Constant3'
  //   DataTypeConversion: '<S119>/Conversion'
  //   Gain: '<S118>/Gain'
  //   Gain: '<S118>/Gain1'
  //   Gain: '<S118>/Gain2'
  //   Inport: '<Root>/ctl_input_msg'

  rtb_Switch_inertia_matrix[0] = (real32_T)ctl_controller0_P->Constant3_Value;
  rtb_Switch_inertia_matrix[1] =
    ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[2];
  rtb_Switch_inertia_matrix[2] = ctl_controller0_P->Gain_Gain_p *
    ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[1];
  rtb_Switch_inertia_matrix[3] = ctl_controller0_P->Gain1_Gain_e *
    ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[2];
  rtb_Switch_inertia_matrix[4] = (real32_T)ctl_controller0_P->Constant3_Value;
  rtb_Switch_inertia_matrix[5] =
    ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[0];
  rtb_Switch_inertia_matrix[6] =
    ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[1];
  rtb_Switch_inertia_matrix[7] = ctl_controller0_P->Gain2_Gain_k *
    ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[0];
  rtb_Switch_inertia_matrix[8] = (real32_T)ctl_controller0_P->Constant3_Value;

  // Math: '<S111>/Math Function' incorporates:
  //   Concatenate: '<S116>/Matrix Concatenate'
  //   Gain: '<S111>/Gain'
  //   Gain: '<S116>/Gain1'
  //   Inport: '<Root>/ctl_input_msg'
  //   Sum: '<S116>/Sum2'

  for (i = 0; i < 3; i++) {
    rtb_Assignment_b[(int32_T)(3 * i)] = rtb_Assignment_h[i] +
      rtb_Switch_inertia_matrix[i];
    rtb_Assignment_b[(int32_T)(1 + (int32_T)(3 * i))] = rtb_Assignment_h
      [(int32_T)(i + 3)] + rtb_Switch_inertia_matrix[(int32_T)(i + 3)];
    rtb_Assignment_b[(int32_T)(2 + (int32_T)(3 * i))] = rtb_Assignment_h
      [(int32_T)(i + 6)] + rtb_Switch_inertia_matrix[(int32_T)(i + 6)];
    rtb_Assignment_b[(int32_T)(9 + i)] = ctl_controller0_P->Gain1_Gain_o *
      ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[i];
  }

  // End of Math: '<S111>/Math Function'
  for (i = 0; i < 4; i++) {
    // Gain: '<S111>/Gain' incorporates:
    //   Product: '<S111>/Product'

    tmp[(int32_T)(3 * i)] = rtb_Assignment_b[(int32_T)(3 * i)] *
      ctl_controller0_P->Gain_Gain_m;
    tmp[(int32_T)(1 + (int32_T)(3 * i))] = rtb_Assignment_b[(int32_T)((int32_T)
      (3 * i) + 1)] * ctl_controller0_P->Gain_Gain_m;
    tmp[(int32_T)(2 + (int32_T)(3 * i))] = rtb_Assignment_b[(int32_T)((int32_T)
      (3 * i) + 2)] * ctl_controller0_P->Gain_Gain_m;

    // Sum: '<S115>/Diff' incorporates:
    //   Product: '<S111>/Product'
    //   UnitDelay: '<S115>/UD'

    rtb_Product_b[i] = rtb_TSamp[i] - ctl_controller0_DW->UD_DSTATE[i];
  }

  // Product: '<S111>/Product'
  for (i = 0; i < 3; i++) {
    rtb_Product_f = tmp[(int32_T)(i + 9)] * rtb_Product_b[3] + (tmp[(int32_T)(i
      + 6)] * rtb_Product_b[2] + (tmp[(int32_T)(i + 3)] * rtb_Product_b[1] +
      tmp[i] * rtb_Product_b[0]));
    rtb_Product[i] = rtb_Product_f;
  }

  // Outputs for Iterator SubSystem: '<S5>/For Each Subsystem'
  ctl_controller_ForEachSubsystem(3, rtb_Product,
    rtb_ImpAsg_InsertedFor_Y_at_i_d, ctl_controller0_DW->ForEachSubsystem,
    (P_ForEachSubsystem_ctl_contro_T *)&ctl_controller0_P->ForEachSubsystem,
    ctl_controller0_P->tun_truth_q_omega_filt_enable);

  // End of Outputs for SubSystem: '<S5>/For Each Subsystem'

  // SampleTimeMath: '<S112>/TSamp' incorporates:
  //   Inport: '<Root>/ctl_input_msg'
  //
  //  About '<S112>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_Product_f = ctl_controller0_U_ctl_input_msg_l->est_P_B_ISS_ISS[0] *
    ctl_controller0_P->TSamp_WtEt_k;

  // Sum: '<S112>/Diff' incorporates:
  //   UnitDelay: '<S112>/UD'

  rtb_Diff[0] = rtb_Product_f - ctl_controller0_DW->UD_DSTATE_e[0];

  // SampleTimeMath: '<S112>/TSamp' incorporates:
  //   Inport: '<Root>/ctl_input_msg'
  //
  //  About '<S112>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_Product[0] = rtb_Product_f;
  rtb_Product_f = ctl_controller0_U_ctl_input_msg_l->est_P_B_ISS_ISS[1] *
    ctl_controller0_P->TSamp_WtEt_k;

  // Sum: '<S112>/Diff' incorporates:
  //   UnitDelay: '<S112>/UD'

  rtb_Diff[1] = rtb_Product_f - ctl_controller0_DW->UD_DSTATE_e[1];

  // SampleTimeMath: '<S112>/TSamp' incorporates:
  //   Inport: '<Root>/ctl_input_msg'
  //
  //  About '<S112>/TSamp':
  //   y = u * K where K = 1 / ( w * Ts )

  rtb_Product[1] = rtb_Product_f;
  rtb_Product_f = ctl_controller0_U_ctl_input_msg_l->est_P_B_ISS_ISS[2] *
    ctl_controller0_P->TSamp_WtEt_k;

  // Sum: '<S112>/Diff' incorporates:
  //   UnitDelay: '<S112>/UD'

  rtb_Diff[2] = rtb_Product_f - ctl_controller0_DW->UD_DSTATE_e[2];

  // Outputs for Iterator SubSystem: '<S5>/For Each Subsystem1'
  ctl_controller_ForEachSubsystem(3, rtb_Diff, rtb_ImpAsg_InsertedFor_Y_at_inp,
    ctl_controller0_DW->ForEachSubsystem1, (P_ForEachSubsystem_ctl_contro_T *)
    &ctl_controller0_P->ForEachSubsystem1,
    ctl_controller0_P->tun_truth_velocity_filt_enable);

  // End of Outputs for SubSystem: '<S5>/For Each Subsystem1'

  // Switch: '<S1>/Switch' incorporates:
  //   BusAssignment: '<S1>/Bus Assignment'
  //   Constant: '<S1>/Constant'
  //   Inport: '<Root>/ctl_input_msg'

  if ((int32_T)ctl_controller0_P->tun_debug_ctl_use_truth != 0) {
    rtb_Merge_j[0] = ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[0];
    rtb_Merge_j[1] = ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[1];
    rtb_Merge_j[2] = ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[2];
    rtb_Merge_j[3] = ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[3];
    rtb_Switch12 = ctl_controller0_U_ctl_input_msg_l->est_confidence;
    rtb_Switch_cmd_state_a_timestam =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.timestamp_sec;
    rtb_Switch_cmd_state_a_timest_0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.timestamp_nsec;
    rtb_Switch_est_omega_B_ISS_B_id = rtb_ImpAsg_InsertedFor_Y_at_i_d[0];
    rtb_Switch_est_V_B_ISS_ISS_idx_ = rtb_ImpAsg_InsertedFor_Y_at_inp[0];
    rtb_Switch_est_P_B_ISS_ISS_idx_ =
      ctl_controller0_U_ctl_input_msg_l->est_P_B_ISS_ISS[0];
    normA = ctl_controller0_U_ctl_input_msg_l->cmd_state_a.P_B_ISS_ISS[0];
    b_s = ctl_controller0_U_ctl_input_msg_l->cmd_state_a.V_B_ISS_ISS[0];
    rtb_Switch_cmd_state_a_A_B_ISS_ =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.A_B_ISS_ISS[0];
    rtb_Switch_est_omega_B_ISS_B__0 = rtb_ImpAsg_InsertedFor_Y_at_i_d[1];
    rtb_Switch_est_V_B_ISS_ISS_id_0 = rtb_ImpAsg_InsertedFor_Y_at_inp[1];
    rtb_Switch_est_P_B_ISS_ISS_id_0 =
      ctl_controller0_U_ctl_input_msg_l->est_P_B_ISS_ISS[1];
    rtb_Switch_cmd_state_a_P_B_ISS_ =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.P_B_ISS_ISS[1];
    rtb_Switch_cmd_state_a_V_B_ISS_ =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.V_B_ISS_ISS[1];
    rtb_Switch_cmd_state_a_A_B_IS_0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.A_B_ISS_ISS[1];
    rtb_Switch_est_omega_B_ISS_B__1 = rtb_ImpAsg_InsertedFor_Y_at_i_d[2];
    rtb_Switch_est_V_B_ISS_ISS_id_1 = rtb_ImpAsg_InsertedFor_Y_at_inp[2];
    rtb_Switch_est_P_B_ISS_ISS_id_1 =
      ctl_controller0_U_ctl_input_msg_l->est_P_B_ISS_ISS[2];
    rtb_Switch_cmd_state_a_P_B_IS_0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.P_B_ISS_ISS[2];
    rtb_Sum3_k_idx_2 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.V_B_ISS_ISS[2];
    rtb_Switch_cmd_state_a_A_B_IS_1 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.A_B_ISS_ISS[2];
    rtb_Product1_m[0] =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.quat_ISS2B[0];
    rtb_Product1_m[1] =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.quat_ISS2B[1];
    rtb_Product1_m[2] =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.quat_ISS2B[2];
    rtb_Product1_m[3] =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.quat_ISS2B[3];
    rtb_Switch_cmd_state_b_timestam =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.timestamp_sec;
    rtb_Switch_cmd_state_b_timest_0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.timestamp_nsec;
    rtb_Switch8_idx_0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.omega_B_ISS_B[0];
    rtb_Sum2_idx_0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.alpha_B_ISS_B[0];
    rtb_Gain1_j_idx_0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.P_B_ISS_ISS[0];
    rtb_Switch_c_idx_0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.V_B_ISS_ISS[0];
    rtb_Diff[0] = ctl_controller0_U_ctl_input_msg_l->cmd_state_b.A_B_ISS_ISS[0];
    rtb_Switch8_idx_1 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.omega_B_ISS_B[1];
    rtb_Sum2_idx_1 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.alpha_B_ISS_B[1];
    rtb_Gain1_j_idx_1 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.P_B_ISS_ISS[1];
    rtb_Switch_c_idx_1 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.V_B_ISS_ISS[1];
    rtb_Diff[1] = ctl_controller0_U_ctl_input_msg_l->cmd_state_b.A_B_ISS_ISS[1];
    rtb_Switch8_idx_2 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.omega_B_ISS_B[2];
    rtb_Sum2_idx_2 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.alpha_B_ISS_B[2];
    rtb_Gain1_j_idx_2 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.P_B_ISS_ISS[2];
    rtb_Switch_c_idx_2 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.V_B_ISS_ISS[2];
    rtb_Diff[2] = ctl_controller0_U_ctl_input_msg_l->cmd_state_b.A_B_ISS_ISS[2];
    rtb_Merge_pa[0] = ctl_controller0_U_ctl_input_msg_l->cmd_state_b.quat_ISS2B
      [0];
    rtb_Merge_pa[1] = ctl_controller0_U_ctl_input_msg_l->cmd_state_b.quat_ISS2B
      [1];
    rtb_Merge_pa[2] = ctl_controller0_U_ctl_input_msg_l->cmd_state_b.quat_ISS2B
      [2];
    rtb_Merge_pa[3] = ctl_controller0_U_ctl_input_msg_l->cmd_state_b.quat_ISS2B
      [3];
    rtb_Switch_ctl_mode_cmd = ctl_controller0_U_ctl_input_msg_l->ctl_mode_cmd;
    rtb_Switch_current_time_sec =
      ctl_controller0_U_ctl_input_msg_l->current_time_sec;
    rtb_Switch_current_time_nsec =
      ctl_controller0_U_ctl_input_msg_l->current_time_nsec;
    rtb_Switch_speed_gain_cmd =
      ctl_controller0_U_ctl_input_msg_l->speed_gain_cmd;
    rtb_Divide_f[0] =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.omega_B_ISS_B[0];
    rtb_Switch_cmd_state_b_alpha__1 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.alpha_B_ISS_B[0];
    rtb_Switch_b[0] = ctl_controller0_U_ctl_input_msg_l->att_kp[0];
    rtb_Switch_att_ki_idx_0 = ctl_controller0_U_ctl_input_msg_l->att_ki[0];
    rtb_Switch_omega_kd_idx_0 = ctl_controller0_U_ctl_input_msg_l->omega_kd[0];
    rtb_Sum_f[0] = ctl_controller0_U_ctl_input_msg_l->pos_kp[0];
    rtb_Switch_pos_ki_idx_0 = ctl_controller0_U_ctl_input_msg_l->pos_ki[0];
    rtb_Switch_vel_kd[0] = ctl_controller0_U_ctl_input_msg_l->vel_kd[0];
    rtb_Divide_f[1] =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.omega_B_ISS_B[1];
    rtb_Switch_cmd_state_b_alpha__0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.alpha_B_ISS_B[1];
    rtb_Switch_b[1] = ctl_controller0_U_ctl_input_msg_l->att_kp[1];
    rtb_Switch_att_ki_idx_1 = ctl_controller0_U_ctl_input_msg_l->att_ki[1];
    rtb_Switch_omega_kd_idx_1 = ctl_controller0_U_ctl_input_msg_l->omega_kd[1];
    rtb_Sum_f[1] = ctl_controller0_U_ctl_input_msg_l->pos_kp[1];
    rtb_Switch_pos_ki_idx_1 = ctl_controller0_U_ctl_input_msg_l->pos_ki[1];
    rtb_Switch_vel_kd[1] = ctl_controller0_U_ctl_input_msg_l->vel_kd[1];
    rtb_Divide_f[2] =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.omega_B_ISS_B[2];
    rtb_Switch_cmd_state_b_alpha_B_ =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.alpha_B_ISS_B[2];
    rtb_Switch_b[2] = ctl_controller0_U_ctl_input_msg_l->att_kp[2];
    rtb_Switch_att_ki_idx_2 = ctl_controller0_U_ctl_input_msg_l->att_ki[2];
    rtb_Switch_omega_kd_idx_2 = ctl_controller0_U_ctl_input_msg_l->omega_kd[2];
    rtb_Sum_f[2] = ctl_controller0_U_ctl_input_msg_l->pos_kp[2];
    rtb_Switch_pos_ki_idx_2 = ctl_controller0_U_ctl_input_msg_l->pos_ki[2];
    rtb_Switch_vel_kd[2] = ctl_controller0_U_ctl_input_msg_l->vel_kd[2];
    for (i = 0; i < 9; i++) {
      rtb_Switch_inertia_matrix[i] =
        ctl_controller0_U_ctl_input_msg_l->inertia_matrix[i];
    }

    rtb_Switch_mass = ctl_controller0_U_ctl_input_msg_l->mass;
  } else {
    rtb_Merge_j[0] = ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[0];
    rtb_Merge_j[1] = ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[1];
    rtb_Merge_j[2] = ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[2];
    rtb_Merge_j[3] = ctl_controller0_U_ctl_input_msg_l->est_quat_ISS2B[3];
    rtb_Switch12 = ctl_controller0_U_ctl_input_msg_l->est_confidence;
    rtb_Switch_cmd_state_a_timestam =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.timestamp_sec;
    rtb_Switch_cmd_state_a_timest_0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.timestamp_nsec;
    rtb_Switch_est_omega_B_ISS_B_id =
      ctl_controller0_U_ctl_input_msg_l->est_omega_B_ISS_B[0];
    rtb_Switch_est_V_B_ISS_ISS_idx_ =
      ctl_controller0_U_ctl_input_msg_l->est_V_B_ISS_ISS[0];
    rtb_Switch_est_P_B_ISS_ISS_idx_ =
      ctl_controller0_U_ctl_input_msg_l->est_P_B_ISS_ISS[0];
    normA = ctl_controller0_U_ctl_input_msg_l->cmd_state_a.P_B_ISS_ISS[0];
    b_s = ctl_controller0_U_ctl_input_msg_l->cmd_state_a.V_B_ISS_ISS[0];
    rtb_Switch_cmd_state_a_A_B_ISS_ =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.A_B_ISS_ISS[0];
    rtb_Switch_est_omega_B_ISS_B__0 =
      ctl_controller0_U_ctl_input_msg_l->est_omega_B_ISS_B[1];
    rtb_Switch_est_V_B_ISS_ISS_id_0 =
      ctl_controller0_U_ctl_input_msg_l->est_V_B_ISS_ISS[1];
    rtb_Switch_est_P_B_ISS_ISS_id_0 =
      ctl_controller0_U_ctl_input_msg_l->est_P_B_ISS_ISS[1];
    rtb_Switch_cmd_state_a_P_B_ISS_ =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.P_B_ISS_ISS[1];
    rtb_Switch_cmd_state_a_V_B_ISS_ =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.V_B_ISS_ISS[1];
    rtb_Switch_cmd_state_a_A_B_IS_0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.A_B_ISS_ISS[1];
    rtb_Switch_est_omega_B_ISS_B__1 =
      ctl_controller0_U_ctl_input_msg_l->est_omega_B_ISS_B[2];
    rtb_Switch_est_V_B_ISS_ISS_id_1 =
      ctl_controller0_U_ctl_input_msg_l->est_V_B_ISS_ISS[2];
    rtb_Switch_est_P_B_ISS_ISS_id_1 =
      ctl_controller0_U_ctl_input_msg_l->est_P_B_ISS_ISS[2];
    rtb_Switch_cmd_state_a_P_B_IS_0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.P_B_ISS_ISS[2];
    rtb_Sum3_k_idx_2 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.V_B_ISS_ISS[2];
    rtb_Switch_cmd_state_a_A_B_IS_1 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.A_B_ISS_ISS[2];
    rtb_Product1_m[0] =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.quat_ISS2B[0];
    rtb_Product1_m[1] =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.quat_ISS2B[1];
    rtb_Product1_m[2] =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.quat_ISS2B[2];
    rtb_Product1_m[3] =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.quat_ISS2B[3];
    rtb_Switch_cmd_state_b_timestam =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.timestamp_sec;
    rtb_Switch_cmd_state_b_timest_0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.timestamp_nsec;
    rtb_Switch8_idx_0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.omega_B_ISS_B[0];
    rtb_Sum2_idx_0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.alpha_B_ISS_B[0];
    rtb_Gain1_j_idx_0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.P_B_ISS_ISS[0];
    rtb_Switch_c_idx_0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.V_B_ISS_ISS[0];
    rtb_Diff[0] = ctl_controller0_U_ctl_input_msg_l->cmd_state_b.A_B_ISS_ISS[0];
    rtb_Switch8_idx_1 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.omega_B_ISS_B[1];
    rtb_Sum2_idx_1 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.alpha_B_ISS_B[1];
    rtb_Gain1_j_idx_1 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.P_B_ISS_ISS[1];
    rtb_Switch_c_idx_1 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.V_B_ISS_ISS[1];
    rtb_Diff[1] = ctl_controller0_U_ctl_input_msg_l->cmd_state_b.A_B_ISS_ISS[1];
    rtb_Switch8_idx_2 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.omega_B_ISS_B[2];
    rtb_Sum2_idx_2 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_a.alpha_B_ISS_B[2];
    rtb_Gain1_j_idx_2 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.P_B_ISS_ISS[2];
    rtb_Switch_c_idx_2 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.V_B_ISS_ISS[2];
    rtb_Diff[2] = ctl_controller0_U_ctl_input_msg_l->cmd_state_b.A_B_ISS_ISS[2];
    rtb_Merge_pa[0] = ctl_controller0_U_ctl_input_msg_l->cmd_state_b.quat_ISS2B
      [0];
    rtb_Merge_pa[1] = ctl_controller0_U_ctl_input_msg_l->cmd_state_b.quat_ISS2B
      [1];
    rtb_Merge_pa[2] = ctl_controller0_U_ctl_input_msg_l->cmd_state_b.quat_ISS2B
      [2];
    rtb_Merge_pa[3] = ctl_controller0_U_ctl_input_msg_l->cmd_state_b.quat_ISS2B
      [3];
    rtb_Switch_ctl_mode_cmd = ctl_controller0_U_ctl_input_msg_l->ctl_mode_cmd;
    rtb_Switch_current_time_sec =
      ctl_controller0_U_ctl_input_msg_l->current_time_sec;
    rtb_Switch_current_time_nsec =
      ctl_controller0_U_ctl_input_msg_l->current_time_nsec;
    rtb_Switch_speed_gain_cmd =
      ctl_controller0_U_ctl_input_msg_l->speed_gain_cmd;
    rtb_Divide_f[0] =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.omega_B_ISS_B[0];
    rtb_Switch_cmd_state_b_alpha__1 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.alpha_B_ISS_B[0];
    rtb_Switch_b[0] = ctl_controller0_U_ctl_input_msg_l->att_kp[0];
    rtb_Switch_att_ki_idx_0 = ctl_controller0_U_ctl_input_msg_l->att_ki[0];
    rtb_Switch_omega_kd_idx_0 = ctl_controller0_U_ctl_input_msg_l->omega_kd[0];
    rtb_Sum_f[0] = ctl_controller0_U_ctl_input_msg_l->pos_kp[0];
    rtb_Switch_pos_ki_idx_0 = ctl_controller0_U_ctl_input_msg_l->pos_ki[0];
    rtb_Switch_vel_kd[0] = ctl_controller0_U_ctl_input_msg_l->vel_kd[0];
    rtb_Divide_f[1] =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.omega_B_ISS_B[1];
    rtb_Switch_cmd_state_b_alpha__0 =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.alpha_B_ISS_B[1];
    rtb_Switch_b[1] = ctl_controller0_U_ctl_input_msg_l->att_kp[1];
    rtb_Switch_att_ki_idx_1 = ctl_controller0_U_ctl_input_msg_l->att_ki[1];
    rtb_Switch_omega_kd_idx_1 = ctl_controller0_U_ctl_input_msg_l->omega_kd[1];
    rtb_Sum_f[1] = ctl_controller0_U_ctl_input_msg_l->pos_kp[1];
    rtb_Switch_pos_ki_idx_1 = ctl_controller0_U_ctl_input_msg_l->pos_ki[1];
    rtb_Switch_vel_kd[1] = ctl_controller0_U_ctl_input_msg_l->vel_kd[1];
    rtb_Divide_f[2] =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.omega_B_ISS_B[2];
    rtb_Switch_cmd_state_b_alpha_B_ =
      ctl_controller0_U_ctl_input_msg_l->cmd_state_b.alpha_B_ISS_B[2];
    rtb_Switch_b[2] = ctl_controller0_U_ctl_input_msg_l->att_kp[2];
    rtb_Switch_att_ki_idx_2 = ctl_controller0_U_ctl_input_msg_l->att_ki[2];
    rtb_Switch_omega_kd_idx_2 = ctl_controller0_U_ctl_input_msg_l->omega_kd[2];
    rtb_Sum_f[2] = ctl_controller0_U_ctl_input_msg_l->pos_kp[2];
    rtb_Switch_pos_ki_idx_2 = ctl_controller0_U_ctl_input_msg_l->pos_ki[2];
    rtb_Switch_vel_kd[2] = ctl_controller0_U_ctl_input_msg_l->vel_kd[2];
    for (i = 0; i < 9; i++) {
      rtb_Switch_inertia_matrix[i] =
        ctl_controller0_U_ctl_input_msg_l->inertia_matrix[i];
    }

    rtb_Switch_mass = ctl_controller0_U_ctl_input_msg_l->mass;
  }

  // End of Switch: '<S1>/Switch'

  // DataTypeConversion: '<S20>/Conversion' incorporates:
  //   Constant: '<S19>/Constant2'

  for (i = 0; i < 9; i++) {
    rtb_Assignment_h[i] = (real32_T)ctl_controller0_P->Constant2_Value_p[i];
  }

  // End of DataTypeConversion: '<S20>/Conversion'

  // Assignment: '<S19>/Assignment' incorporates:
  //   UnitDelay: '<S2>/Unit Delay2'

  rtb_Assignment_h[0] = ctl_controller0_DW->UnitDelay2_DSTATE[3];

  // Gain: '<S16>/Gain' incorporates:
  //   UnitDelay: '<S2>/Unit Delay2'

  rtb_Sum4_o_idx_0 = ctl_controller0_P->Gain_Gain_a *
    ctl_controller0_DW->UnitDelay2_DSTATE[0];

  // Assignment: '<S19>/Assignment' incorporates:
  //   UnitDelay: '<S2>/Unit Delay2'

  rtb_Assignment_h[4] = ctl_controller0_DW->UnitDelay2_DSTATE[3];

  // Gain: '<S16>/Gain' incorporates:
  //   UnitDelay: '<S2>/Unit Delay2'

  rtb_Sum4_o_idx_1 = ctl_controller0_P->Gain_Gain_a *
    ctl_controller0_DW->UnitDelay2_DSTATE[1];

  // Assignment: '<S19>/Assignment' incorporates:
  //   UnitDelay: '<S2>/Unit Delay2'

  rtb_Assignment_h[8] = ctl_controller0_DW->UnitDelay2_DSTATE[3];

  // Gain: '<S16>/Gain' incorporates:
  //   UnitDelay: '<S2>/Unit Delay2'

  rtb_Sum4_o_idx_2 = ctl_controller0_P->Gain_Gain_a *
    ctl_controller0_DW->UnitDelay2_DSTATE[2];

  // Sum: '<S19>/Sum2' incorporates:
  //   Constant: '<S21>/Constant3'
  //   DataTypeConversion: '<S22>/Conversion'
  //   Gain: '<S21>/Gain'
  //   Gain: '<S21>/Gain1'
  //   Gain: '<S21>/Gain2'

  rtb_Assignment_l[0] = (real32_T)ctl_controller0_P->Constant3_Value_a;
  rtb_Assignment_l[1] = rtb_Sum4_o_idx_2;
  rtb_Assignment_l[2] = ctl_controller0_P->Gain_Gain_i * rtb_Sum4_o_idx_1;
  rtb_Assignment_l[3] = ctl_controller0_P->Gain1_Gain_n * rtb_Sum4_o_idx_2;
  rtb_Assignment_l[4] = (real32_T)ctl_controller0_P->Constant3_Value_a;
  rtb_Assignment_l[5] = rtb_Sum4_o_idx_0;
  rtb_Assignment_l[6] = rtb_Sum4_o_idx_1;
  rtb_Assignment_l[7] = ctl_controller0_P->Gain2_Gain_a * rtb_Sum4_o_idx_0;
  rtb_Assignment_l[8] = (real32_T)ctl_controller0_P->Constant3_Value_a;

  // Concatenate: '<S19>/Matrix Concatenate' incorporates:
  //   Gain: '<S19>/Gain1'
  //   Sum: '<S19>/Sum2'

  for (i = 0; i < 3; i++) {
    rtb_Product5[(int32_T)(i << 2)] = rtb_Assignment_h[(int32_T)(3 * i)] +
      rtb_Assignment_l[(int32_T)(3 * i)];
    rtb_Product5[(int32_T)(1 + (int32_T)(i << 2))] = rtb_Assignment_h[(int32_T)
      ((int32_T)(3 * i) + 1)] + rtb_Assignment_l[(int32_T)((int32_T)(3 * i) + 1)];
    rtb_Product5[(int32_T)(2 + (int32_T)(i << 2))] = rtb_Assignment_h[(int32_T)
      ((int32_T)(3 * i) + 2)] + rtb_Assignment_l[(int32_T)((int32_T)(3 * i) + 2)];
  }

  rtb_Product5[3] = ctl_controller0_P->Gain1_Gain_j * rtb_Sum4_o_idx_0;
  rtb_Product5[7] = ctl_controller0_P->Gain1_Gain_j * rtb_Sum4_o_idx_1;
  rtb_Product5[11] = ctl_controller0_P->Gain1_Gain_j * rtb_Sum4_o_idx_2;

  // End of Concatenate: '<S19>/Matrix Concatenate'

  // Reshape: '<S17>/Reshape1' incorporates:
  //   UnitDelay: '<S2>/Unit Delay2'

  rtb_Product5[12] = rtb_Sum4_o_idx_0;
  rtb_Product5[13] = rtb_Sum4_o_idx_1;
  rtb_Product5[14] = rtb_Sum4_o_idx_2;
  rtb_Product5[15] = ctl_controller0_DW->UnitDelay2_DSTATE[3];

  // Product: '<S17>/Product'
  for (i = 0; i < 4; i++) {
    rtb_SumofElements = rtb_Product5[(int32_T)(i + 12)] * rtb_Merge_j[3] +
      (rtb_Product5[(int32_T)(i + 8)] * rtb_Merge_j[2] + (rtb_Product5[(int32_T)
        (i + 4)] * rtb_Merge_j[1] + rtb_Product5[i] * rtb_Merge_j[0]));
    rtb_Product_b[i] = rtb_SumofElements;
  }

  // End of Product: '<S17>/Product'

  // If: '<S18>/If' incorporates:
  //   Inport: '<S23>/In1'

  if (rtb_Product_b[3] < 0.0F) {
    // Outputs for IfAction SubSystem: '<S18>/Normalize' incorporates:
    //   ActionPort: '<S24>/Action Port'

    ctl_controller0_Normalize(rtb_Product_b, rtb_Merge,
      (P_Normalize_ctl_controller0_T *)&ctl_controller0_P->Normalize);

    // End of Outputs for SubSystem: '<S18>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S18>/No-op' incorporates:
    //   ActionPort: '<S23>/Action Port'

    rtb_Merge[0] = rtb_Product_b[0];
    rtb_Merge[1] = rtb_Product_b[1];
    rtb_Merge[2] = rtb_Product_b[2];
    rtb_Merge[3] = rtb_Product_b[3];

    // End of Outputs for SubSystem: '<S18>/No-op'
  }

  // End of If: '<S18>/If'

  // Sqrt: '<S29>/Sqrt' incorporates:
  //   DotProduct: '<S29>/Dot Product'

  rtb_Sqrt = (real32_T)sqrt((real_T)(((rtb_Merge[0] * rtb_Merge[0] + rtb_Merge[1]
    * rtb_Merge[1]) + rtb_Merge[2] * rtb_Merge[2]) + rtb_Merge[3] * rtb_Merge[3]));

  // If: '<S25>/If' incorporates:
  //   DataTypeConversion: '<S25>/Data Type Conversion'
  //   Inport: '<S27>/In1'

  if ((real_T)rtb_Sqrt > 1.0E-7) {
    // Outputs for IfAction SubSystem: '<S25>/Normalize' incorporates:
    //   ActionPort: '<S28>/Action Port'

    ctl_controller0_Normalize_e(rtb_Merge, rtb_Sqrt, rtb_Product_b);

    // End of Outputs for SubSystem: '<S25>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S25>/No-op' incorporates:
    //   ActionPort: '<S27>/Action Port'

    rtb_Product_b[3] = rtb_Merge[3];

    // End of Outputs for SubSystem: '<S25>/No-op'
  }

  // End of If: '<S25>/If'

  // Sum: '<S13>/SumA21' incorporates:
  //   Delay: '<S13>/Delay11'
  //   Gain: '<S13>/a(2)(1)'
  //   Gain: '<S13>/s(1)'

  rtb_Sum4_o_idx_2 = (real32_T)(ctl_controller0_P->s1_Gain * (real_T)
    rtb_Switch_est_V_B_ISS_ISS_idx_) - (real32_T)(ctl_controller0_P->a21_Gain *
    (real_T)ctl_controller0_DW->Delay11_DSTATE[0]);

  // Sum: '<S13>/SumB21' incorporates:
  //   Delay: '<S13>/Delay11'

  rtb_Sum4_p = rtb_Sum4_o_idx_2 + ctl_controller0_DW->Delay11_DSTATE[0];

  // Math: '<S2>/Math Function'
  rtb_Sum4_p *= rtb_Sum4_p;

  // Sum: '<S13>/SumA21'
  rtb_SumA21_idx_0 = rtb_Sum4_o_idx_2;

  // Sum: '<S13>/SumB21'
  rtb_Sum4_o_idx_0 = rtb_Sum4_p;

  // Sum: '<S13>/SumA21' incorporates:
  //   Delay: '<S13>/Delay11'
  //   Gain: '<S13>/a(2)(1)'
  //   Gain: '<S13>/s(1)'

  rtb_Sum4_o_idx_2 = (real32_T)(ctl_controller0_P->s1_Gain * (real_T)
    rtb_Switch_est_V_B_ISS_ISS_id_0) - (real32_T)(ctl_controller0_P->a21_Gain *
    (real_T)ctl_controller0_DW->Delay11_DSTATE[1]);

  // Sum: '<S13>/SumB21' incorporates:
  //   Delay: '<S13>/Delay11'

  rtb_Sum4_p = rtb_Sum4_o_idx_2 + ctl_controller0_DW->Delay11_DSTATE[1];

  // Math: '<S2>/Math Function'
  rtb_Sum4_p *= rtb_Sum4_p;

  // Sum: '<S13>/SumA21'
  rtb_SumA21_idx_1 = rtb_Sum4_o_idx_2;

  // Sum: '<S13>/SumB21'
  rtb_Sum4_o_idx_1 = rtb_Sum4_p;

  // Sum: '<S13>/SumA21' incorporates:
  //   Delay: '<S13>/Delay11'
  //   Gain: '<S13>/a(2)(1)'
  //   Gain: '<S13>/s(1)'

  rtb_Sum4_o_idx_2 = (real32_T)(ctl_controller0_P->s1_Gain * (real_T)
    rtb_Switch_est_V_B_ISS_ISS_id_1) - (real32_T)(ctl_controller0_P->a21_Gain *
    (real_T)ctl_controller0_DW->Delay11_DSTATE[2]);

  // Sum: '<S13>/SumB21' incorporates:
  //   Delay: '<S13>/Delay11'

  rtb_Sum4_p = rtb_Sum4_o_idx_2 + ctl_controller0_DW->Delay11_DSTATE[2];

  // Math: '<S2>/Math Function'
  rtb_Sum4_p *= rtb_Sum4_p;

  // Sum: '<S2>/Sum of Elements'
  rtb_SumofElements = (rtb_Sum4_o_idx_0 + rtb_Sum4_o_idx_1) + rtb_Sum4_p;

  // Sum: '<S14>/SumA21' incorporates:
  //   Delay: '<S14>/Delay11'
  //   Gain: '<S14>/a(2)(1)'
  //   Gain: '<S14>/s(1)'

  rtb_SumA21_f = (real32_T)(ctl_controller0_P->s1_Gain_b * (real_T)
    rtb_Switch_est_omega_B_ISS_B_id) - (real32_T)(ctl_controller0_P->a21_Gain_l *
    (real_T)ctl_controller0_DW->Delay11_DSTATE_i[0]);

  // Sum: '<S14>/SumB21' incorporates:
  //   Delay: '<S14>/Delay11'

  rtb_SumofElements1 = rtb_SumA21_f + ctl_controller0_DW->Delay11_DSTATE_i[0];

  // Math: '<S2>/Math Function1'
  rtb_SumofElements1 *= rtb_SumofElements1;

  // Sum: '<S14>/SumA21'
  rtb_SumA21_d_idx_0 = rtb_SumA21_f;

  // Sum: '<S14>/SumB21'
  rtb_Sum3_k_idx_0 = rtb_SumofElements1;

  // Sum: '<S14>/SumA21' incorporates:
  //   Delay: '<S14>/Delay11'
  //   Gain: '<S14>/a(2)(1)'
  //   Gain: '<S14>/s(1)'

  rtb_SumA21_f = (real32_T)(ctl_controller0_P->s1_Gain_b * (real_T)
    rtb_Switch_est_omega_B_ISS_B__0) - (real32_T)(ctl_controller0_P->a21_Gain_l *
    (real_T)ctl_controller0_DW->Delay11_DSTATE_i[1]);

  // Sum: '<S14>/SumB21' incorporates:
  //   Delay: '<S14>/Delay11'

  rtb_SumofElements1 = rtb_SumA21_f + ctl_controller0_DW->Delay11_DSTATE_i[1];

  // Math: '<S2>/Math Function1'
  rtb_SumofElements1 *= rtb_SumofElements1;

  // Sum: '<S14>/SumA21'
  rtb_SumA21_d_idx_1 = rtb_SumA21_f;

  // Sum: '<S14>/SumB21'
  rtb_Sum3_k_idx_1 = rtb_SumofElements1;

  // Sum: '<S14>/SumA21' incorporates:
  //   Delay: '<S14>/Delay11'
  //   Gain: '<S14>/a(2)(1)'
  //   Gain: '<S14>/s(1)'

  rtb_SumA21_f = (real32_T)(ctl_controller0_P->s1_Gain_b * (real_T)
    rtb_Switch_est_omega_B_ISS_B__1) - (real32_T)(ctl_controller0_P->a21_Gain_l *
    (real_T)ctl_controller0_DW->Delay11_DSTATE_i[2]);

  // Sum: '<S14>/SumB21' incorporates:
  //   Delay: '<S14>/Delay11'

  rtb_SumofElements1 = rtb_SumA21_f + ctl_controller0_DW->Delay11_DSTATE_i[2];

  // Math: '<S2>/Math Function1'
  rtb_SumofElements1 *= rtb_SumofElements1;

  // Sum: '<S2>/Sum of Elements1'
  rtb_SumofElements1 += rtb_Sum3_k_idx_0 + rtb_Sum3_k_idx_1;

  // Logic: '<S78>/Logical Operator1' incorporates:
  //   Logic: '<S78>/Logical Operator'
  //   RelationalOperator: '<S78>/Relational Operator'
  //   RelationalOperator: '<S78>/Relational Operator1'
  //   RelationalOperator: '<S78>/Relational Operator2'

  rtb_LogicalOperator2_c = ((rtb_Switch_cmd_state_b_timestam >
    rtb_Switch_current_time_sec) || ((rtb_Switch_current_time_sec ==
    rtb_Switch_cmd_state_b_timestam) && (rtb_Switch_current_time_nsec <
    rtb_Switch_cmd_state_b_timest_0)));

  // Switch: '<S78>/Switch'
  if (rtb_LogicalOperator2_c) {
    rtb_Switch_cmd_state_b_timestam = rtb_Switch_cmd_state_a_timestam;
    rtb_Switch_cmd_state_b_timest_0 = rtb_Switch_cmd_state_a_timest_0;
    rtb_Gain1_j_idx_0 = normA;
    rtb_Switch_c_idx_0 = b_s;
    rtb_Diff[0] = rtb_Switch_cmd_state_a_A_B_ISS_;
    rtb_Gain1_j_idx_1 = rtb_Switch_cmd_state_a_P_B_ISS_;
    rtb_Switch_c_idx_1 = rtb_Switch_cmd_state_a_V_B_ISS_;
    rtb_Diff[1] = rtb_Switch_cmd_state_a_A_B_IS_0;
    rtb_Gain1_j_idx_2 = rtb_Switch_cmd_state_a_P_B_IS_0;
    rtb_Switch_c_idx_2 = rtb_Sum3_k_idx_2;
    rtb_Diff[2] = rtb_Switch_cmd_state_a_A_B_IS_1;
    rtb_Merge_pa[0] = rtb_Product1_m[0];
    rtb_Merge_pa[1] = rtb_Product1_m[1];
    rtb_Merge_pa[2] = rtb_Product1_m[2];
    rtb_Merge_pa[3] = rtb_Product1_m[3];
    rtb_Divide_f[0] = rtb_Switch8_idx_0;
    rtb_Switch_cmd_state_b_alpha__1 = rtb_Sum2_idx_0;
    rtb_Divide_f[1] = rtb_Switch8_idx_1;
    rtb_Switch_cmd_state_b_alpha__0 = rtb_Sum2_idx_1;
    rtb_Divide_f[2] = rtb_Switch8_idx_2;
    rtb_Switch_cmd_state_b_alpha_B_ = rtb_Sum2_idx_2;
  }

  // End of Switch: '<S78>/Switch'

  // Logic: '<S78>/Logical Operator2'
  rtb_LogicalOperator2_c = !rtb_LogicalOperator2_c;

  // Sum: '<S78>/Sum' incorporates:
  //   DataTypeConversion: '<S78>/Data Type Conversion4'
  //   Gain: '<S78>/Gain'
  //   Sum: '<S78>/Subtract3'
  //   Sum: '<S78>/Subtract4'

  rtb_Sqrt = (real32_T)(int32_T)((int32_T)rtb_Switch_current_time_nsec -
    (int32_T)rtb_Switch_cmd_state_b_timest_0) * ctl_controller0_P->Gain_Gain_h +
    (real32_T)(int32_T)((int32_T)rtb_Switch_current_time_sec - (int32_T)
                        rtb_Switch_cmd_state_b_timestam);

  // Sum: '<S80>/Sum1' incorporates:
  //   Product: '<S80>/Product1'

  rtb_Sum3_k_idx_0 = rtb_Diff[0] * rtb_Sqrt + rtb_Switch_c_idx_0;

  // Sum: '<S80>/Sum3' incorporates:
  //   Constant: '<S80>/Constant'
  //   Product: '<S80>/Product'
  //   Product: '<S80>/Product2'

  rtb_Gain1_j_idx_0 = (ctl_controller0_P->Constant_Value * rtb_Diff[0] *
                       rtb_Sqrt * rtb_Sqrt + rtb_Gain1_j_idx_0) +
    rtb_Switch_c_idx_0 * rtb_Sqrt;

  // Sum: '<S80>/Sum1' incorporates:
  //   Product: '<S80>/Product1'

  rtb_Sum3_k_idx_1 = rtb_Diff[1] * rtb_Sqrt + rtb_Switch_c_idx_1;

  // Sum: '<S80>/Sum3' incorporates:
  //   Constant: '<S80>/Constant'
  //   Product: '<S80>/Product'
  //   Product: '<S80>/Product2'

  rtb_Gain1_j_idx_1 = (ctl_controller0_P->Constant_Value * rtb_Diff[1] *
                       rtb_Sqrt * rtb_Sqrt + rtb_Gain1_j_idx_1) +
    rtb_Switch_c_idx_1 * rtb_Sqrt;
  rtb_Switch_cmd_state_a_P_B_IS_0 = (ctl_controller0_P->Constant_Value *
    rtb_Diff[2] * rtb_Sqrt * rtb_Sqrt + rtb_Gain1_j_idx_2) + rtb_Switch_c_idx_2 *
    rtb_Sqrt;

  // Sum: '<S80>/Sum1' incorporates:
  //   Product: '<S80>/Product1'

  rtb_Sum3_k_idx_2 = rtb_Diff[2] * rtb_Sqrt + rtb_Switch_c_idx_2;

  // Constant: '<S84>/Constant3'
  rtb_VectorConcatenate[0] = ctl_controller0_P->Constant3_Value_dz;

  // Gain: '<S84>/Gain'
  rtb_VectorConcatenate[1] = ctl_controller0_P->Gain_Gain_n *
    rtb_Switch_cmd_state_b_alpha_B_;

  // SignalConversion: '<S84>/ConcatBufferAtVector ConcatenateIn3'
  rtb_VectorConcatenate[2] = rtb_Switch_cmd_state_b_alpha__0;

  // Gain: '<S84>/Gain1'
  rtb_VectorConcatenate[3] = ctl_controller0_P->Gain1_Gain_b *
    rtb_Switch_cmd_state_b_alpha__1;

  // SignalConversion: '<S84>/ConcatBufferAtVector ConcatenateIn5'
  rtb_VectorConcatenate[4] = rtb_Switch_cmd_state_b_alpha_B_;

  // Constant: '<S84>/Constant2'
  rtb_VectorConcatenate[5] = ctl_controller0_P->Constant2_Value_d;

  // Gain: '<S84>/Gain2'
  rtb_VectorConcatenate[6] = ctl_controller0_P->Gain2_Gain_kx *
    rtb_Switch_cmd_state_b_alpha__1;

  // Gain: '<S84>/Gain3'
  rtb_VectorConcatenate[7] = ctl_controller0_P->Gain3_Gain *
    rtb_Switch_cmd_state_b_alpha__0;

  // Gain: '<S84>/Gain4'
  rtb_VectorConcatenate[8] = ctl_controller0_P->Gain4_Gain *
    rtb_Switch_cmd_state_b_alpha__0;

  // SignalConversion: '<S84>/ConcatBufferAtVector ConcatenateIn10'
  rtb_VectorConcatenate[9] = rtb_Switch_cmd_state_b_alpha__1;

  // Constant: '<S84>/Constant1'
  rtb_VectorConcatenate[10] = ctl_controller0_P->Constant1_Value_l;

  // Gain: '<S84>/Gain5'
  rtb_VectorConcatenate[11] = ctl_controller0_P->Gain5_Gain *
    rtb_Switch_cmd_state_b_alpha_B_;

  // SignalConversion: '<S84>/ConcatBufferAtVector ConcatenateIn13'
  rtb_VectorConcatenate[12] = rtb_Switch_cmd_state_b_alpha__1;

  // SignalConversion: '<S84>/ConcatBufferAtVector ConcatenateIn14'
  rtb_VectorConcatenate[13] = rtb_Switch_cmd_state_b_alpha__0;

  // SignalConversion: '<S84>/ConcatBufferAtVector ConcatenateIn15'
  rtb_VectorConcatenate[14] = rtb_Switch_cmd_state_b_alpha_B_;

  // Constant: '<S84>/Constant'
  rtb_VectorConcatenate[15] = ctl_controller0_P->Constant_Value_g;

  // Constant: '<S83>/Constant3'
  rtb_VectorConcatenate_m[0] = ctl_controller0_P->Constant3_Value_f;

  // Gain: '<S83>/Gain'
  rtb_VectorConcatenate_m[1] = ctl_controller0_P->Gain_Gain_g * rtb_Divide_f[2];

  // SignalConversion: '<S83>/ConcatBufferAtVector ConcatenateIn3'
  rtb_VectorConcatenate_m[2] = rtb_Divide_f[1];

  // Gain: '<S83>/Gain1'
  rtb_VectorConcatenate_m[3] = ctl_controller0_P->Gain1_Gain_ja * rtb_Divide_f[0];

  // SignalConversion: '<S83>/ConcatBufferAtVector ConcatenateIn5'
  rtb_VectorConcatenate_m[4] = rtb_Divide_f[2];

  // Constant: '<S83>/Constant2'
  rtb_VectorConcatenate_m[5] = ctl_controller0_P->Constant2_Value_gq;

  // Gain: '<S83>/Gain2'
  rtb_VectorConcatenate_m[6] = ctl_controller0_P->Gain2_Gain_c * rtb_Divide_f[0];

  // Gain: '<S83>/Gain3'
  rtb_VectorConcatenate_m[7] = ctl_controller0_P->Gain3_Gain_o * rtb_Divide_f[1];

  // Gain: '<S83>/Gain4'
  rtb_VectorConcatenate_m[8] = ctl_controller0_P->Gain4_Gain_c * rtb_Divide_f[1];

  // SignalConversion: '<S83>/ConcatBufferAtVector ConcatenateIn10'
  rtb_VectorConcatenate_m[9] = rtb_Divide_f[0];

  // Constant: '<S83>/Constant1'
  rtb_VectorConcatenate_m[10] = ctl_controller0_P->Constant1_Value_gk;

  // Gain: '<S83>/Gain5'
  rtb_VectorConcatenate_m[11] = ctl_controller0_P->Gain5_Gain_a * rtb_Divide_f[2];

  // SignalConversion: '<S83>/ConcatBufferAtVector ConcatenateIn13'
  rtb_VectorConcatenate_m[12] = rtb_Divide_f[0];

  // SignalConversion: '<S83>/ConcatBufferAtVector ConcatenateIn14'
  rtb_VectorConcatenate_m[13] = rtb_Divide_f[1];

  // SignalConversion: '<S83>/ConcatBufferAtVector ConcatenateIn15'
  rtb_VectorConcatenate_m[14] = rtb_Divide_f[2];

  // Constant: '<S83>/Constant'
  rtb_VectorConcatenate_m[15] = ctl_controller0_P->Constant_Value_m;

  // Product: '<S79>/Product3' incorporates:
  //   Constant: '<S79>/Constant1'
  //   Constant: '<S79>/Constant3'
  //   Product: '<S79>/Product'
  //   Sum: '<S79>/Add'

  for (i = 0; i < 16; i++) {
    rtb_Product5[i] = (ctl_controller0_P->Constant3_Value_d *
                       rtb_VectorConcatenate[i] * rtb_Sqrt +
                       rtb_VectorConcatenate_m[i]) *
      ctl_controller0_P->Constant1_Value_g * rtb_Sqrt;
  }

  // End of Product: '<S79>/Product3'

  // MATLAB Function: '<S79>/MATLAB Function'
  // MATLAB Function 'cmd_command_shaper/generate_cmd_attitude/MATLAB Function': '<S82>:1' 
  // '<S82>:1:4'
  normA = 0.0F;
  i = 0;
  exitg2 = false;
  while ((!exitg2) && (i < 4)) {
    b_s = (((real32_T)fabs((real_T)rtb_Product5[(int32_T)((int32_T)(i << 2) + 1)])
            + (real32_T)fabs((real_T)rtb_Product5[(int32_T)(i << 2)])) +
           (real32_T)fabs((real_T)rtb_Product5[(int32_T)((int32_T)(i << 2) + 2)]))
      + (real32_T)fabs((real_T)rtb_Product5[(int32_T)((int32_T)(i << 2) + 3)]);
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
        mglnkfkfmglfjekn_PadeApproximantOfDegree(rtb_Product5, (uint8_T)(int32_T)
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
      rtb_Product5_0[i] = rtb_Product5[i] / b_s;
    }

    mglnkfkfmglfjekn_PadeApproximantOfDegree(rtb_Product5_0, 7U, rtb_y);
    for (eint = 0; eint <= (int32_T)((int32_T)normA - 1); eint++) {
      for (i = 0; i < 4; i++) {
        for (i_0 = 0; i_0 < 4; i_0++) {
          rtb_Product5[(int32_T)(i + (int32_T)(i_0 << 2))] = 0.0F;
          rtb_Product5[(int32_T)(i + (int32_T)(i_0 << 2))] += rtb_y[(int32_T)
            (i_0 << 2)] * rtb_y[i];
          rtb_Product5[(int32_T)(i + (int32_T)(i_0 << 2))] += rtb_y[(int32_T)
            ((int32_T)(i_0 << 2) + 1)] * rtb_y[(int32_T)(i + 4)];
          rtb_Product5[(int32_T)(i + (int32_T)(i_0 << 2))] += rtb_y[(int32_T)
            ((int32_T)(i_0 << 2) + 2)] * rtb_y[(int32_T)(i + 8)];
          rtb_Product5[(int32_T)(i + (int32_T)(i_0 << 2))] += rtb_y[(int32_T)
            ((int32_T)(i_0 << 2) + 3)] * rtb_y[(int32_T)(i + 12)];
        }
      }

      for (i = 0; i < 4; i++) {
        rtb_y[(int32_T)(i << 2)] = rtb_Product5[(int32_T)(i << 2)];
        rtb_y[(int32_T)(1 + (int32_T)(i << 2))] = rtb_Product5[(int32_T)
          ((int32_T)(i << 2) + 1)];
        rtb_y[(int32_T)(2 + (int32_T)(i << 2))] = rtb_Product5[(int32_T)
          ((int32_T)(i << 2) + 2)];
        rtb_y[(int32_T)(3 + (int32_T)(i << 2))] = rtb_Product5[(int32_T)
          ((int32_T)(i << 2) + 3)];
      }
    }
  }

  // End of MATLAB Function: '<S79>/MATLAB Function'

  // Product: '<S79>/Product6'
  normA = rtb_Sqrt * rtb_Sqrt * rtb_Sqrt;
  for (i = 0; i < 4; i++) {
    for (i_0 = 0; i_0 < 4; i_0++) {
      // Product: '<S79>/Product5' incorporates:
      //   Sum: '<S79>/Add2'

      rtb_Product5_0[(int32_T)(i_0 + (int32_T)(i << 2))] = 0.0F;

      // Product: '<S79>/Product4' incorporates:
      //   Sum: '<S79>/Add2'

      rtb_VectorConcatenate_p[(int32_T)(i_0 + (int32_T)(i << 2))] = 0.0F;

      // Product: '<S79>/Product5' incorporates:
      //   Sum: '<S79>/Add2'

      rtb_Product5_0[(int32_T)(i_0 + (int32_T)(i << 2))] +=
        rtb_VectorConcatenate_m[(int32_T)(i << 2)] * rtb_VectorConcatenate[i_0];

      // Product: '<S79>/Product4' incorporates:
      //   Sum: '<S79>/Add2'

      rtb_VectorConcatenate_p[(int32_T)(i_0 + (int32_T)(i << 2))] +=
        rtb_VectorConcatenate[(int32_T)(i << 2)] * rtb_VectorConcatenate_m[i_0];

      // Product: '<S79>/Product5' incorporates:
      //   Sum: '<S79>/Add2'

      rtb_Product5_0[(int32_T)(i_0 + (int32_T)(i << 2))] +=
        rtb_VectorConcatenate_m[(int32_T)((int32_T)(i << 2) + 1)] *
        rtb_VectorConcatenate[(int32_T)(i_0 + 4)];

      // Product: '<S79>/Product4' incorporates:
      //   Sum: '<S79>/Add2'

      rtb_VectorConcatenate_p[(int32_T)(i_0 + (int32_T)(i << 2))] +=
        rtb_VectorConcatenate[(int32_T)((int32_T)(i << 2) + 1)] *
        rtb_VectorConcatenate_m[(int32_T)(i_0 + 4)];

      // Product: '<S79>/Product5' incorporates:
      //   Sum: '<S79>/Add2'

      rtb_Product5_0[(int32_T)(i_0 + (int32_T)(i << 2))] +=
        rtb_VectorConcatenate_m[(int32_T)((int32_T)(i << 2) + 2)] *
        rtb_VectorConcatenate[(int32_T)(i_0 + 8)];

      // Product: '<S79>/Product4' incorporates:
      //   Sum: '<S79>/Add2'

      rtb_VectorConcatenate_p[(int32_T)(i_0 + (int32_T)(i << 2))] +=
        rtb_VectorConcatenate[(int32_T)((int32_T)(i << 2) + 2)] *
        rtb_VectorConcatenate_m[(int32_T)(i_0 + 8)];

      // Product: '<S79>/Product5' incorporates:
      //   Sum: '<S79>/Add2'

      rtb_Product5_0[(int32_T)(i_0 + (int32_T)(i << 2))] +=
        rtb_VectorConcatenate_m[(int32_T)((int32_T)(i << 2) + 3)] *
        rtb_VectorConcatenate[(int32_T)(i_0 + 12)];

      // Product: '<S79>/Product4' incorporates:
      //   Sum: '<S79>/Add2'

      rtb_VectorConcatenate_p[(int32_T)(i_0 + (int32_T)(i << 2))] +=
        rtb_VectorConcatenate[(int32_T)((int32_T)(i << 2) + 3)] *
        rtb_VectorConcatenate_m[(int32_T)(i_0 + 12)];
    }
  }

  // Sum: '<S79>/Add1' incorporates:
  //   Constant: '<S79>/Constant2'
  //   Product: '<S79>/Product1'
  //   Product: '<S79>/Product6'
  //   Sum: '<S79>/Add2'

  for (i = 0; i < 4; i++) {
    rtb_Product5[(int32_T)(i << 2)] = (rtb_Product5_0[(int32_T)(i << 2)] -
      rtb_VectorConcatenate_p[(int32_T)(i << 2)]) * normA /
      ctl_controller0_P->Constant2_Value_it + rtb_y[(int32_T)(i << 2)];
    rtb_Product5[(int32_T)(1 + (int32_T)(i << 2))] = (rtb_Product5_0[(int32_T)
      ((int32_T)(i << 2) + 1)] - rtb_VectorConcatenate_p[(int32_T)((int32_T)(i <<
      2) + 1)]) * normA / ctl_controller0_P->Constant2_Value_it + rtb_y[(int32_T)
      ((int32_T)(i << 2) + 1)];
    rtb_Product5[(int32_T)(2 + (int32_T)(i << 2))] = (rtb_Product5_0[(int32_T)
      ((int32_T)(i << 2) + 2)] - rtb_VectorConcatenate_p[(int32_T)((int32_T)(i <<
      2) + 2)]) * normA / ctl_controller0_P->Constant2_Value_it + rtb_y[(int32_T)
      ((int32_T)(i << 2) + 2)];
    rtb_Product5[(int32_T)(3 + (int32_T)(i << 2))] = (rtb_Product5_0[(int32_T)
      ((int32_T)(i << 2) + 3)] - rtb_VectorConcatenate_p[(int32_T)((int32_T)(i <<
      2) + 3)]) * normA / ctl_controller0_P->Constant2_Value_it + rtb_y[(int32_T)
      ((int32_T)(i << 2) + 3)];
  }

  // End of Sum: '<S79>/Add1'

  // Product: '<S79>/Product1'
  for (i = 0; i < 4; i++) {
    rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Product5[(int32_T)(i + 12)] *
      rtb_Merge_pa[3] + (rtb_Product5[(int32_T)(i + 8)] * rtb_Merge_pa[2] +
                         (rtb_Product5[(int32_T)(i + 4)] * rtb_Merge_pa[1] +
                          rtb_Product5[i] * rtb_Merge_pa[0]));
    rtb_Product1_m[i] = rtb_Switch_cmd_state_a_A_B_IS_1;
  }

  // If: '<S85>/If' incorporates:
  //   Inport: '<S86>/In1'

  if (rtb_Product1_m[3] < 0.0F) {
    // Outputs for IfAction SubSystem: '<S85>/Normalize' incorporates:
    //   ActionPort: '<S87>/Action Port'

    ctl_controller0_Normalize(rtb_Product1_m, rtb_Merge,
      (P_Normalize_ctl_controller0_T *)&ctl_controller0_P->Normalize_eo);

    // End of Outputs for SubSystem: '<S85>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S85>/No-op' incorporates:
    //   ActionPort: '<S86>/Action Port'

    rtb_Merge[0] = rtb_Product1_m[0];
    rtb_Merge[1] = rtb_Product1_m[1];
    rtb_Merge[2] = rtb_Product1_m[2];
    rtb_Merge[3] = rtb_Product1_m[3];

    // End of Outputs for SubSystem: '<S85>/No-op'
  }

  // End of If: '<S85>/If'

  // Sqrt: '<S92>/Sqrt' incorporates:
  //   DotProduct: '<S92>/Dot Product'

  normA = (real32_T)sqrt((real_T)(((rtb_Merge[0] * rtb_Merge[0] + rtb_Merge[1] *
    rtb_Merge[1]) + rtb_Merge[2] * rtb_Merge[2]) + rtb_Merge[3] * rtb_Merge[3]));

  // If: '<S88>/If' incorporates:
  //   DataTypeConversion: '<S88>/Data Type Conversion'
  //   Inport: '<S90>/In1'

  if ((real_T)normA > 1.0E-7) {
    // Outputs for IfAction SubSystem: '<S88>/Normalize' incorporates:
    //   ActionPort: '<S91>/Action Port'

    ctl_controller0_Normalize_e(rtb_Merge, normA, rtb_Merge_pa);

    // End of Outputs for SubSystem: '<S88>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S88>/No-op' incorporates:
    //   ActionPort: '<S90>/Action Port'

    rtb_Merge_pa[0] = rtb_Merge[0];
    rtb_Merge_pa[1] = rtb_Merge[1];
    rtb_Merge_pa[2] = rtb_Merge[2];
    rtb_Merge_pa[3] = rtb_Merge[3];

    // End of Outputs for SubSystem: '<S88>/No-op'
  }

  // End of If: '<S88>/If'

  // Switch: '<S2>/Switch12' incorporates:
  //   Constant: '<S10>/Constant'
  //   Constant: '<S2>/Constant4'
  //   RelationalOperator: '<S10>/Compare'

  if (rtb_Switch12 != ctl_controller0_P->ase_status_converged) {
    rtb_Switch12 = ctl_controller0_P->ctl_idle_mode;
  } else {
    rtb_Switch12 = rtb_Switch_ctl_mode_cmd;
  }

  // End of Switch: '<S2>/Switch12'

  // Sum: '<S79>/Sum1' incorporates:
  //   Product: '<S79>/Product2'

  rtb_Sum4_p = rtb_Switch_cmd_state_b_alpha__1 * rtb_Sqrt + rtb_Divide_f[0];

  // BusCreator: '<S4>/Bus Creator1'
  normA = rtb_Gain1_j_idx_0;
  b_s = rtb_Sum3_k_idx_0;
  rtb_Switch_cmd_state_a_A_B_ISS_ = rtb_Sum4_p;

  // Sum: '<S2>/Sum1' incorporates:
  //   UnitDelay: '<S2>/Unit Delay1'

  rtb_Switch8_idx_0 = ctl_controller0_DW->UnitDelay1_DSTATE[0] -
    rtb_Switch_est_P_B_ISS_ISS_idx_;

  // Math: '<S2>/Math Function2'
  rtb_Switch8_idx_0 *= rtb_Switch8_idx_0;

  // Sum: '<S2>/Sum1'
  rtb_Divide_f[0] = rtb_Switch8_idx_0;

  // Sum: '<S79>/Sum1' incorporates:
  //   Product: '<S79>/Product2'

  rtb_Sum4_o_idx_0 = rtb_Sum4_p;
  rtb_Sum4_p = rtb_Switch_cmd_state_b_alpha__0 * rtb_Sqrt + rtb_Divide_f[1];

  // BusCreator: '<S4>/Bus Creator1'
  rtb_Switch_cmd_state_a_P_B_ISS_ = rtb_Gain1_j_idx_1;
  rtb_Switch_cmd_state_a_V_B_ISS_ = rtb_Sum3_k_idx_1;
  rtb_Switch_cmd_state_a_A_B_IS_0 = rtb_Sum4_p;

  // Sum: '<S2>/Sum1' incorporates:
  //   UnitDelay: '<S2>/Unit Delay1'

  rtb_Switch8_idx_0 = ctl_controller0_DW->UnitDelay1_DSTATE[1] -
    rtb_Switch_est_P_B_ISS_ISS_id_0;

  // Math: '<S2>/Math Function2'
  rtb_Switch8_idx_0 *= rtb_Switch8_idx_0;

  // Sum: '<S2>/Sum1'
  rtb_Divide_f[1] = rtb_Switch8_idx_0;

  // Sum: '<S79>/Sum1' incorporates:
  //   Product: '<S79>/Product2'

  rtb_Sum4_o_idx_1 = rtb_Sum4_p;
  rtb_Sum4_p = rtb_Switch_cmd_state_b_alpha_B_ * rtb_Sqrt + rtb_Divide_f[2];

  // Sum: '<S2>/Sum1' incorporates:
  //   UnitDelay: '<S2>/Unit Delay1'

  rtb_Switch8_idx_0 = ctl_controller0_DW->UnitDelay1_DSTATE[2] -
    rtb_Switch_est_P_B_ISS_ISS_id_1;

  // Math: '<S2>/Math Function2'
  rtb_Switch8_idx_0 *= rtb_Switch8_idx_0;

  // Logic: '<S2>/Logical Operator2' incorporates:
  //   Constant: '<S6>/Constant'
  //   Constant: '<S7>/Constant'
  //   Logic: '<S2>/Logical Operator'
  //   Logic: '<S2>/Logical Operator4'
  //   RelationalOperator: '<S6>/Compare'
  //   RelationalOperator: '<S7>/Compare'
  //   S-Function (sfix_udelay): '<S2>/Tapped Delay'

  rtb_LogicalOperator2 = (ctl_controller0_DW->TappedDelay_X[0] &&
    ctl_controller0_DW->TappedDelay_X[1] && ctl_controller0_DW->TappedDelay_X[2]
    && ctl_controller0_DW->TappedDelay_X[3] && ((rtb_SumofElements <
    ctl_controller0_P->tun_ctl_stopping_vel_thresh) && (rtb_SumofElements1 <
    ctl_controller0_P->tun_ctl_stopping_omega_thresh)));

  // Trigonometry: '<S2>/Trigonometric Function'
  if (rtb_Product_b[3] > 1.0F) {
    rtb_SumofElements = 1.0F;
  } else if (rtb_Product_b[3] < -1.0F) {
    rtb_SumofElements = -1.0F;
  } else {
    rtb_SumofElements = rtb_Product_b[3];
  }

  // Switch: '<S2>/Switch3' incorporates:
  //   Abs: '<S2>/Abs1'
  //   Constant: '<S11>/Constant'
  //   Constant: '<S12>/Constant'
  //   Constant: '<S2>/Constant6'
  //   Constant: '<S9>/Constant'
  //   Gain: '<S2>/Gain2'
  //   Logic: '<S2>/Logical Operator1'
  //   Logic: '<S2>/Logical Operator3'
  //   RelationalOperator: '<S11>/Compare'
  //   RelationalOperator: '<S12>/Compare'
  //   RelationalOperator: '<S9>/Compare'
  //   Sum: '<S2>/Sum of Elements2'
  //   Switch: '<S2>/Switch2'
  //   Trigonometry: '<S2>/Trigonometric Function'

  if ((rtb_Switch12 == ctl_controller0_P->ctl_stopped_mode) && (((rtb_Divide_f[0]
         + rtb_Divide_f[1]) + rtb_Switch8_idx_0 >
        ctl_controller0_P->tun_ctl_stopped_pos_thresh) || ((real32_T)fabs
        ((real_T)(ctl_controller0_P->Gain2_Gain_h * (real32_T)acos((real_T)
           rtb_SumofElements))) > ctl_controller0_P->tun_ctl_stopped_quat_thresh)))
  {
    rtb_Switch3 = ctl_controller0_P->ctl_stopping_mode;
  } else if (rtb_LogicalOperator2) {
    // Switch: '<S2>/Switch2' incorporates:
    //   Constant: '<S2>/Constant'

    rtb_Switch3 = ctl_controller0_P->ctl_stopped_mode;
  } else {
    rtb_Switch3 = rtb_Switch12;
  }

  // End of Switch: '<S2>/Switch3'

  // Switch: '<S2>/Switch8' incorporates:
  //   Switch: '<S2>/Switch9'
  //   UnitDelay: '<S2>/Unit Delay1'
  //   UnitDelay: '<S2>/Unit Delay2'

  if (rtb_LogicalOperator2) {
    rtb_Switch8_idx_0 = ctl_controller0_DW->UnitDelay1_DSTATE[0];
    rtb_Switch8_idx_1 = ctl_controller0_DW->UnitDelay1_DSTATE[1];
    rtb_Switch8_idx_2 = ctl_controller0_DW->UnitDelay1_DSTATE[2];
    rtb_Product_b[0] = ctl_controller0_DW->UnitDelay2_DSTATE[0];
    rtb_Product_b[1] = ctl_controller0_DW->UnitDelay2_DSTATE[1];
    rtb_Product_b[2] = ctl_controller0_DW->UnitDelay2_DSTATE[2];
    rtb_Product_b[3] = ctl_controller0_DW->UnitDelay2_DSTATE[3];
  } else {
    rtb_Switch8_idx_0 = rtb_Switch_est_P_B_ISS_ISS_idx_;
    rtb_Switch8_idx_1 = rtb_Switch_est_P_B_ISS_ISS_id_0;
    rtb_Switch8_idx_2 = rtb_Switch_est_P_B_ISS_ISS_id_1;
    rtb_Product_b[0] = rtb_Merge_j[0];
    rtb_Product_b[1] = rtb_Merge_j[1];
    rtb_Product_b[2] = rtb_Merge_j[2];
    rtb_Product_b[3] = rtb_Merge_j[3];
  }

  // End of Switch: '<S2>/Switch8'

  // Sum: '<S45>/Sum' incorporates:
  //   Constant: '<S45>/Constant1'
  //   DataTypeConversion: '<S47>/Conversion'
  //   Gain: '<S45>/Gain'
  //   Math: '<S45>/Math Function'

  rtb_SumofElements = rtb_Merge_j[3] * rtb_Merge_j[3] *
    ctl_controller0_P->Gain_Gain_l - (real32_T)
    ctl_controller0_P->Constant1_Value;
  for (i = 0; i < 9; i++) {
    // Assignment: '<S45>/Assignment' incorporates:
    //   Constant: '<S45>/Constant2'
    //   DataTypeConversion: '<S46>/Conversion'

    rtb_Assignment_h[i] = (real32_T)ctl_controller0_P->Constant2_Value_pu[i];

    // Assignment: '<S40>/Assignment' incorporates:
    //   Constant: '<S40>/Constant2'
    //   DataTypeConversion: '<S41>/Conversion'

    rtb_Assignment_l[i] = (real32_T)ctl_controller0_P->Constant2_Value_k[i];
  }

  // Assignment: '<S45>/Assignment'
  rtb_Assignment_h[0] = rtb_SumofElements;
  rtb_Assignment_h[4] = rtb_SumofElements;
  rtb_Assignment_h[8] = rtb_SumofElements;

  // Gain: '<S45>/Gain1'
  rtb_SumofElements = ctl_controller0_P->Gain1_Gain_ej * rtb_Merge_j[3];

  // Sum: '<S40>/Sum' incorporates:
  //   Constant: '<S40>/Constant1'
  //   DataTypeConversion: '<S42>/Conversion'
  //   Gain: '<S40>/Gain'
  //   Math: '<S40>/Math Function'

  rtb_SumofElements1 = rtb_Merge_j[3] * rtb_Merge_j[3] *
    ctl_controller0_P->Gain_Gain_b - (real32_T)
    ctl_controller0_P->Constant1_Value_f;

  // Assignment: '<S40>/Assignment'
  rtb_Assignment_l[0] = rtb_SumofElements1;
  rtb_Assignment_l[4] = rtb_SumofElements1;
  rtb_Assignment_l[8] = rtb_SumofElements1;

  // Gain: '<S40>/Gain1'
  rtb_SumofElements1 = ctl_controller0_P->Gain1_Gain_g * rtb_Merge_j[3];

  // Switch: '<S30>/Switch2' incorporates:
  //   BusCreator: '<S4>/Bus Creator1'
  //   Constant: '<S30>/Constant4'
  //   Constant: '<S34>/Constant'
  //   RelationalOperator: '<S34>/Compare'
  //   Switch: '<S2>/Switch10'

  if (rtb_Switch3 <= ctl_controller0_P->CompareToConstant2_const) {
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

  // End of Switch: '<S30>/Switch2'

  // Switch: '<S2>/Switch6' incorporates:
  //   BusCreator: '<S4>/Bus Creator1'

  if (rtb_LogicalOperator2) {
    rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Switch8_idx_0;
  } else {
    rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Gain1_j_idx_0;
  }

  // Sum: '<S30>/Sum2'
  rtb_Sum2_idx_0 = rtb_Switch_cmd_state_a_A_B_IS_1 -
    rtb_Switch_est_P_B_ISS_ISS_idx_;

  // Switch: '<S2>/Switch6' incorporates:
  //   BusCreator: '<S4>/Bus Creator1'

  if (rtb_LogicalOperator2) {
    rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Switch8_idx_1;
  } else {
    rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Gain1_j_idx_1;
  }

  // Sum: '<S30>/Sum2'
  rtb_Sum2_idx_1 = rtb_Switch_cmd_state_a_A_B_IS_1 -
    rtb_Switch_est_P_B_ISS_ISS_id_0;

  // Switch: '<S2>/Switch6'
  if (rtb_LogicalOperator2) {
    rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Switch8_idx_2;
  } else {
    rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Switch_cmd_state_a_P_B_IS_0;
  }

  // Sum: '<S30>/Sum2'
  rtb_Sum2_idx_2 = rtb_Switch_cmd_state_a_A_B_IS_1 -
    rtb_Switch_est_P_B_ISS_ISS_id_1;

  // RelationalOperator: '<S32>/Compare' incorporates:
  //   Constant: '<S32>/Constant'

  rtb_Compare_j = (rtb_Switch3 <= ctl_controller0_P->CompareToConstant_const);

  // DiscreteIntegrator: '<S30>/Discrete-Time Integrator1'
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

  // Switch: '<S30>/Switch1' incorporates:
  //   Constant: '<S30>/Constant1'
  //   Constant: '<S33>/Constant'
  //   DiscreteIntegrator: '<S30>/Discrete-Time Integrator1'
  //   Product: '<S30>/Product'
  //   RelationalOperator: '<S33>/Compare'
  //   Sum: '<S30>/Sum4'
  //   Switch: '<S37>/Switch'

  if (rtb_Switch3 <= ctl_controller0_P->CompareToConstant1_const) {
    rtb_Divide_f[0] = ctl_controller0_P->Constant1_Value_k[0];
    rtb_Divide_f[1] = ctl_controller0_P->Constant1_Value_k[1];
    rtb_Divide_f[2] = ctl_controller0_P->Constant1_Value_k[2];
  } else {
    if (rtb_Switch_vel_kd[0] != 0.0F) {
      // Switch: '<S37>/Switch' incorporates:
      //   Product: '<S37>/Divide1'

      rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Sum_f[0] / rtb_Switch_vel_kd[0];
    } else {
      // Switch: '<S37>/Switch'
      rtb_Switch_cmd_state_a_A_B_IS_1 = 0.0F;
    }

    rtb_Divide_f[0] = rtb_Switch_cmd_state_a_A_B_IS_1 * rtb_Sum2_idx_0 +
      ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0];

    // Switch: '<S37>/Switch' incorporates:
    //   DiscreteIntegrator: '<S30>/Discrete-Time Integrator1'
    //   Product: '<S30>/Product'
    //   Product: '<S37>/Divide1'
    //   Sum: '<S30>/Sum4'

    if (rtb_Switch_vel_kd[1] != 0.0F) {
      rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Sum_f[1] / rtb_Switch_vel_kd[1];
    } else {
      rtb_Switch_cmd_state_a_A_B_IS_1 = 0.0F;
    }

    rtb_Divide_f[1] = rtb_Switch_cmd_state_a_A_B_IS_1 * rtb_Sum2_idx_1 +
      ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1];

    // Switch: '<S37>/Switch' incorporates:
    //   DiscreteIntegrator: '<S30>/Discrete-Time Integrator1'
    //   Product: '<S30>/Product'
    //   Product: '<S37>/Divide1'
    //   Sum: '<S30>/Sum4'

    if (rtb_Switch_vel_kd[2] != 0.0F) {
      rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Sum_f[2] / rtb_Switch_vel_kd[2];
    } else {
      rtb_Switch_cmd_state_a_A_B_IS_1 = 0.0F;
    }

    rtb_Divide_f[2] = rtb_Switch_cmd_state_a_A_B_IS_1 * rtb_Sum2_idx_2 +
      ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2];
  }

  // End of Switch: '<S30>/Switch1'

  // Sum: '<S30>/Sum1'
  rtb_Switch_c_idx_0 = (rtb_Switch_c_idx_0 + rtb_Divide_f[0]) -
    rtb_Switch_est_V_B_ISS_ISS_idx_;
  rtb_Switch_c_idx_1 = (rtb_Switch_c_idx_1 + rtb_Divide_f[1]) -
    rtb_Switch_est_V_B_ISS_ISS_id_0;
  rtb_Switch_c_idx_2 = (rtb_Switch_c_idx_2 + rtb_Divide_f[2]) -
    rtb_Switch_est_V_B_ISS_ISS_id_1;

  // Product: '<S45>/Product' incorporates:
  //   Constant: '<S48>/Constant3'
  //   DataTypeConversion: '<S49>/Conversion'
  //   Gain: '<S48>/Gain'
  //   Gain: '<S48>/Gain1'
  //   Gain: '<S48>/Gain2'

  tmp_0[0] = (real32_T)ctl_controller0_P->Constant3_Value_o;
  tmp_0[1] = rtb_Merge_j[2];
  tmp_0[2] = ctl_controller0_P->Gain_Gain_ib * rtb_Merge_j[1];
  tmp_0[3] = ctl_controller0_P->Gain1_Gain_k * rtb_Merge_j[2];
  tmp_0[4] = (real32_T)ctl_controller0_P->Constant3_Value_o;
  tmp_0[5] = rtb_Merge_j[0];
  tmp_0[6] = rtb_Merge_j[1];
  tmp_0[7] = ctl_controller0_P->Gain2_Gain_b * rtb_Merge_j[0];
  tmp_0[8] = (real32_T)ctl_controller0_P->Constant3_Value_o;

  // Product: '<S45>/Product1' incorporates:
  //   Gain: '<S45>/Gain2'

  for (i = 0; i < 3; i++) {
    rtb_Merge_j_0[i] = rtb_Merge_j[i] * rtb_Merge_j[0];
    rtb_Merge_j_0[(int32_T)(i + 3)] = rtb_Merge_j[i] * rtb_Merge_j[1];
    rtb_Merge_j_0[(int32_T)(i + 6)] = rtb_Merge_j[i] * rtb_Merge_j[2];
  }

  // End of Product: '<S45>/Product1'
  for (i = 0; i < 3; i++) {
    // Sum: '<S45>/Sum1' incorporates:
    //   Gain: '<S45>/Gain2'
    //   Product: '<S36>/Product'
    //   Product: '<S45>/Product'

    rtb_Assignment_b_0[(int32_T)(3 * i)] = (rtb_Assignment_h[(int32_T)(3 * i)] -
      tmp_0[(int32_T)(3 * i)] * rtb_SumofElements) + rtb_Merge_j_0[(int32_T)(3 *
      i)] * ctl_controller0_P->Gain2_Gain_k1;
    rtb_Assignment_b_0[(int32_T)(1 + (int32_T)(3 * i))] = (rtb_Assignment_h
      [(int32_T)((int32_T)(3 * i) + 1)] - tmp_0[(int32_T)((int32_T)(3 * i) + 1)]
      * rtb_SumofElements) + rtb_Merge_j_0[(int32_T)((int32_T)(3 * i) + 1)] *
      ctl_controller0_P->Gain2_Gain_k1;
    rtb_Assignment_b_0[(int32_T)(2 + (int32_T)(3 * i))] = (rtb_Assignment_h
      [(int32_T)((int32_T)(3 * i) + 2)] - tmp_0[(int32_T)((int32_T)(3 * i) + 2)]
      * rtb_SumofElements) + rtb_Merge_j_0[(int32_T)((int32_T)(3 * i) + 2)] *
      ctl_controller0_P->Gain2_Gain_k1;

    // Gain: '<S30>/Gain' incorporates:
    //   Constant: '<S2>/Constant5'
    //   Product: '<S36>/Product'
    //   Switch: '<S2>/Switch14'

    if (rtb_LogicalOperator2) {
      rtb_Switch_cmd_state_a_A_B_IS_1 = ctl_controller0_P->Constant5_Value[i];
    } else {
      rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Diff[i];
    }

    rtb_Sum_f[i] = ctl_controller0_P->tun_accel_gain[i] *
      rtb_Switch_cmd_state_a_A_B_IS_1;

    // End of Gain: '<S30>/Gain'
  }

  // Product: '<S40>/Product' incorporates:
  //   Constant: '<S43>/Constant3'
  //   DataTypeConversion: '<S44>/Conversion'
  //   Gain: '<S43>/Gain'
  //   Gain: '<S43>/Gain1'
  //   Gain: '<S43>/Gain2'

  tmp_1[0] = (real32_T)ctl_controller0_P->Constant3_Value_b;
  tmp_1[1] = rtb_Merge_j[2];
  tmp_1[2] = ctl_controller0_P->Gain_Gain_d * rtb_Merge_j[1];
  tmp_1[3] = ctl_controller0_P->Gain1_Gain_ji * rtb_Merge_j[2];
  tmp_1[4] = (real32_T)ctl_controller0_P->Constant3_Value_b;
  tmp_1[5] = rtb_Merge_j[0];
  tmp_1[6] = rtb_Merge_j[1];
  tmp_1[7] = ctl_controller0_P->Gain2_Gain_bs * rtb_Merge_j[0];
  tmp_1[8] = (real32_T)ctl_controller0_P->Constant3_Value_b;
  for (i = 0; i < 3; i++) {
    // Product: '<S40>/Product1' incorporates:
    //   Gain: '<S40>/Gain2'

    rtb_Merge_j_0[i] = rtb_Merge_j[i] * rtb_Merge_j[0];
    rtb_Merge_j_0[(int32_T)(i + 3)] = rtb_Merge_j[i] * rtb_Merge_j[1];
    rtb_Merge_j_0[(int32_T)(i + 6)] = rtb_Merge_j[i] * rtb_Merge_j[2];

    // Product: '<S36>/Product' incorporates:
    //   Product: '<S30>/Divide3'

    rtb_Assignment_b_1[i] = rtb_Assignment_b_0[(int32_T)(i + 6)] * rtb_Sum_f[2]
      + (rtb_Assignment_b_0[(int32_T)(i + 3)] * rtb_Sum_f[1] +
         rtb_Assignment_b_0[i] * rtb_Sum_f[0]);
  }

  // Sum: '<S40>/Sum1' incorporates:
  //   Gain: '<S40>/Gain2'
  //   Product: '<S35>/Product'
  //   Product: '<S40>/Product'

  for (i = 0; i < 3; i++) {
    rtb_Assignment_h[(int32_T)(3 * i)] = (rtb_Assignment_l[(int32_T)(3 * i)] -
      tmp_1[(int32_T)(3 * i)] * rtb_SumofElements1) + rtb_Merge_j_0[(int32_T)(3 *
      i)] * ctl_controller0_P->Gain2_Gain_kh;
    rtb_Assignment_h[(int32_T)(1 + (int32_T)(3 * i))] = (rtb_Assignment_l
      [(int32_T)((int32_T)(3 * i) + 1)] - tmp_1[(int32_T)((int32_T)(3 * i) + 1)]
      * rtb_SumofElements1) + rtb_Merge_j_0[(int32_T)((int32_T)(3 * i) + 1)] *
      ctl_controller0_P->Gain2_Gain_kh;
    rtb_Assignment_h[(int32_T)(2 + (int32_T)(3 * i))] = (rtb_Assignment_l
      [(int32_T)((int32_T)(3 * i) + 2)] - tmp_1[(int32_T)((int32_T)(3 * i) + 2)]
      * rtb_SumofElements1) + rtb_Merge_j_0[(int32_T)((int32_T)(3 * i) + 2)] *
      ctl_controller0_P->Gain2_Gain_kh;
  }

  // End of Sum: '<S40>/Sum1'

  // DotProduct: '<S54>/Dot Product'
  rtb_SumofElements = 0.0F;
  for (i = 0; i < 3; i++) {
    // Sum: '<S30>/Sum' incorporates:
    //   Product: '<S30>/Divide3'
    //   Product: '<S30>/Product2'
    //   Product: '<S30>/Product3'
    //   Product: '<S35>/Product'

    rtb_SumofElements1 = rtb_Switch_vel_kd[i] * rtb_Switch_mass *
      (rtb_Assignment_h[(int32_T)(i + 6)] * rtb_Switch_c_idx_2 +
       (rtb_Assignment_h[(int32_T)(i + 3)] * rtb_Switch_c_idx_1 +
        rtb_Assignment_h[i] * rtb_Switch_c_idx_0)) + rtb_Switch_mass *
      rtb_Assignment_b_1[i];

    // DotProduct: '<S54>/Dot Product'
    rtb_SumofElements += rtb_SumofElements1 * rtb_SumofElements1;

    // Sum: '<S30>/Sum' incorporates:
    //   Product: '<S30>/Divide3'

    rtb_Sum_f[i] = rtb_SumofElements1;
  }

  // Sqrt: '<S54>/Sqrt' incorporates:
  //   DotProduct: '<S54>/Dot Product'

  rtb_SumofElements = (real32_T)sqrt((real_T)rtb_SumofElements);

  // If: '<S51>/If' incorporates:
  //   DataTypeConversion: '<S51>/Data Type Conversion'
  //   Inport: '<S52>/In1'

  if ((real_T)rtb_SumofElements > 1.0E-7) {
    // Outputs for IfAction SubSystem: '<S51>/Normalize' incorporates:
    //   ActionPort: '<S53>/Action Port'

    // Product: '<S53>/Divide'
    rtb_Switch_c_idx_0 = rtb_Sum_f[0] / rtb_SumofElements;
    rtb_Switch_c_idx_1 = rtb_Sum_f[1] / rtb_SumofElements;
    rtb_Switch_c_idx_2 = rtb_Sum_f[2] / rtb_SumofElements;

    // End of Outputs for SubSystem: '<S51>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S51>/No-op' incorporates:
    //   ActionPort: '<S52>/Action Port'

    rtb_Switch_c_idx_0 = rtb_Sum_f[0];
    rtb_Switch_c_idx_1 = rtb_Sum_f[1];
    rtb_Switch_c_idx_2 = rtb_Sum_f[2];

    // End of Outputs for SubSystem: '<S51>/No-op'
  }

  // End of If: '<S51>/If'

  // Switch: '<S30>/Switch' incorporates:
  //   Constant: '<S30>/Constant3'

  if ((int32_T)rtb_Switch3 != 0) {
    // Switch: '<S39>/Switch' incorporates:
    //   Constant: '<S30>/Constant2'
    //   DotProduct: '<S50>/Dot Product'
    //   Product: '<S39>/Product'
    //   RelationalOperator: '<S39>/Relational Operator'
    //   Sqrt: '<S50>/Sqrt'

    if (!((real32_T)sqrt((real_T)((rtb_Sum_f[0] * rtb_Sum_f[0] + rtb_Sum_f[1] *
            rtb_Sum_f[1]) + rtb_Sum_f[2] * rtb_Sum_f[2])) <
          ctl_controller0_P->tun_ctl_linear_force_limit)) {
      rtb_Sum_f[0] = ctl_controller0_P->tun_ctl_linear_force_limit *
        rtb_Switch_c_idx_0;
      rtb_Sum_f[1] = ctl_controller0_P->tun_ctl_linear_force_limit *
        rtb_Switch_c_idx_1;
      rtb_Sum_f[2] = ctl_controller0_P->tun_ctl_linear_force_limit *
        rtb_Switch_c_idx_2;
    }

    // End of Switch: '<S39>/Switch'
    rtb_Switch_c_idx_0 = rtb_Sum_f[0];
    rtb_Switch_c_idx_1 = rtb_Sum_f[1];
    rtb_Switch_c_idx_2 = rtb_Sum_f[2];
  } else {
    rtb_Switch_c_idx_0 = ctl_controller0_P->Constant3_Value_e[0];
    rtb_Switch_c_idx_1 = ctl_controller0_P->Constant3_Value_e[1];
    rtb_Switch_c_idx_2 = ctl_controller0_P->Constant3_Value_e[2];
  }

  // End of Switch: '<S30>/Switch'

  // Switch: '<S31>/Switch3' incorporates:
  //   BusCreator: '<S4>/Bus Creator1'
  //   Constant: '<S31>/Constant2'
  //   Constant: '<S57>/Constant'
  //   RelationalOperator: '<S57>/Compare'
  //   Switch: '<S2>/Switch11'

  if (rtb_Switch3 <= ctl_controller0_P->CompareToConstant2_const_o) {
    rtb_Divide_f[0] = ctl_controller0_P->Constant2_Value_o[0];
    rtb_Divide_f[1] = ctl_controller0_P->Constant2_Value_o[1];
    rtb_Divide_f[2] = ctl_controller0_P->Constant2_Value_o[2];
  } else if (rtb_LogicalOperator2) {
    // Switch: '<S2>/Switch11' incorporates:
    //   Constant: '<S2>/Constant3'

    rtb_Divide_f[0] = ctl_controller0_P->Constant3_Value_j[0];
    rtb_Divide_f[1] = ctl_controller0_P->Constant3_Value_j[1];
    rtb_Divide_f[2] = ctl_controller0_P->Constant3_Value_j[2];
  } else {
    rtb_Divide_f[0] = rtb_Sum4_o_idx_0;
    rtb_Divide_f[1] = rtb_Sum4_o_idx_1;
    rtb_Divide_f[2] = rtb_Sum4_p;
  }

  // End of Switch: '<S31>/Switch3'

  // DataTypeConversion: '<S68>/Conversion' incorporates:
  //   Constant: '<S67>/Constant2'

  for (i = 0; i < 9; i++) {
    rtb_Assignment_h[i] = (real32_T)ctl_controller0_P->Constant2_Value_g[i];
  }

  // End of DataTypeConversion: '<S68>/Conversion'

  // Assignment: '<S67>/Assignment'
  rtb_Assignment_h[0] = rtb_Merge_j[3];

  // Gain: '<S64>/Gain'
  rtb_Sum_f[0] = ctl_controller0_P->Gain_Gain_pf * rtb_Merge_j[0];

  // Assignment: '<S67>/Assignment'
  rtb_Assignment_h[4] = rtb_Merge_j[3];

  // Gain: '<S64>/Gain'
  rtb_Sum_f[1] = ctl_controller0_P->Gain_Gain_pf * rtb_Merge_j[1];

  // Assignment: '<S67>/Assignment'
  rtb_Assignment_h[8] = rtb_Merge_j[3];

  // Gain: '<S64>/Gain'
  rtb_Sum_f[2] = ctl_controller0_P->Gain_Gain_pf * rtb_Merge_j[2];

  // Sum: '<S67>/Sum2' incorporates:
  //   Constant: '<S69>/Constant3'
  //   DataTypeConversion: '<S70>/Conversion'
  //   Gain: '<S69>/Gain'
  //   Gain: '<S69>/Gain1'
  //   Gain: '<S69>/Gain2'

  tmp_2[0] = (real32_T)ctl_controller0_P->Constant3_Value_a5;
  tmp_2[1] = rtb_Sum_f[2];
  tmp_2[2] = ctl_controller0_P->Gain_Gain_k * rtb_Sum_f[1];
  tmp_2[3] = ctl_controller0_P->Gain1_Gain_jz * rtb_Sum_f[2];
  tmp_2[4] = (real32_T)ctl_controller0_P->Constant3_Value_a5;
  tmp_2[5] = rtb_Sum_f[0];
  tmp_2[6] = rtb_Sum_f[1];
  tmp_2[7] = ctl_controller0_P->Gain2_Gain_n * rtb_Sum_f[0];
  tmp_2[8] = (real32_T)ctl_controller0_P->Constant3_Value_a5;

  // Concatenate: '<S67>/Matrix Concatenate' incorporates:
  //   Gain: '<S67>/Gain1'
  //   Sum: '<S67>/Sum2'

  for (i = 0; i < 3; i++) {
    rtb_VectorConcatenate[(int32_T)(i << 2)] = rtb_Assignment_h[(int32_T)(3 * i)]
      + tmp_2[(int32_T)(3 * i)];
    rtb_VectorConcatenate[(int32_T)(1 + (int32_T)(i << 2))] = rtb_Assignment_h
      [(int32_T)((int32_T)(3 * i) + 1)] + tmp_2[(int32_T)((int32_T)(3 * i) + 1)];
    rtb_VectorConcatenate[(int32_T)(2 + (int32_T)(i << 2))] = rtb_Assignment_h
      [(int32_T)((int32_T)(3 * i) + 2)] + tmp_2[(int32_T)((int32_T)(3 * i) + 2)];
  }

  rtb_VectorConcatenate[3] = ctl_controller0_P->Gain1_Gain_od * rtb_Sum_f[0];
  rtb_VectorConcatenate[7] = ctl_controller0_P->Gain1_Gain_od * rtb_Sum_f[1];
  rtb_VectorConcatenate[11] = ctl_controller0_P->Gain1_Gain_od * rtb_Sum_f[2];

  // End of Concatenate: '<S67>/Matrix Concatenate'

  // Reshape: '<S65>/Reshape1'
  rtb_VectorConcatenate[12] = rtb_Sum_f[0];
  rtb_VectorConcatenate[13] = rtb_Sum_f[1];
  rtb_VectorConcatenate[14] = rtb_Sum_f[2];
  rtb_VectorConcatenate[15] = rtb_Merge_j[3];

  // Product: '<S65>/Product' incorporates:
  //   BusCreator: '<S4>/Bus Creator1'
  //   Switch: '<S2>/Switch7'

  if (rtb_LogicalOperator2) {
    rtb_SumofElements = rtb_Product_b[0];
    rtb_SumofElements1 = rtb_Product_b[1];
    rtb_Sqrt = rtb_Product_b[2];
    rtb_Gain1_j_idx_2 = rtb_Product_b[3];
  } else {
    rtb_SumofElements = rtb_Merge_pa[0];
    rtb_SumofElements1 = rtb_Merge_pa[1];
    rtb_Sqrt = rtb_Merge_pa[2];
    rtb_Gain1_j_idx_2 = rtb_Merge_pa[3];
  }

  for (i = 0; i < 4; i++) {
    rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_VectorConcatenate[(int32_T)(i + 12)] *
      rtb_Gain1_j_idx_2 + (rtb_VectorConcatenate[(int32_T)(i + 8)] * rtb_Sqrt +
      (rtb_VectorConcatenate[(int32_T)(i + 4)] * rtb_SumofElements1 +
       rtb_VectorConcatenate[i] * rtb_SumofElements));
    rtb_Product1_m[i] = rtb_Switch_cmd_state_a_A_B_IS_1;
  }

  // End of Product: '<S65>/Product'

  // If: '<S66>/If' incorporates:
  //   Inport: '<S71>/In1'

  if (rtb_Product1_m[3] < 0.0F) {
    // Outputs for IfAction SubSystem: '<S66>/Normalize' incorporates:
    //   ActionPort: '<S72>/Action Port'

    ctl_controller0_Normalize(rtb_Product1_m, rtb_Merge,
      (P_Normalize_ctl_controller0_T *)&ctl_controller0_P->Normalize_l);

    // End of Outputs for SubSystem: '<S66>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S66>/No-op' incorporates:
    //   ActionPort: '<S71>/Action Port'

    rtb_Merge[0] = rtb_Product1_m[0];
    rtb_Merge[1] = rtb_Product1_m[1];
    rtb_Merge[2] = rtb_Product1_m[2];
    rtb_Merge[3] = rtb_Product1_m[3];

    // End of Outputs for SubSystem: '<S66>/No-op'
  }

  // End of If: '<S66>/If'

  // Sqrt: '<S77>/Sqrt' incorporates:
  //   DotProduct: '<S77>/Dot Product'

  rtb_SumofElements = (real32_T)sqrt((real_T)(((rtb_Merge[0] * rtb_Merge[0] +
    rtb_Merge[1] * rtb_Merge[1]) + rtb_Merge[2] * rtb_Merge[2]) + rtb_Merge[3] *
    rtb_Merge[3]));

  // If: '<S73>/If' incorporates:
  //   DataTypeConversion: '<S73>/Data Type Conversion'
  //   Inport: '<S75>/In1'

  if ((real_T)rtb_SumofElements > 1.0E-7) {
    // Outputs for IfAction SubSystem: '<S73>/Normalize' incorporates:
    //   ActionPort: '<S76>/Action Port'

    ctl_controller0_Normalize_e(rtb_Merge, rtb_SumofElements, rtb_Product1_m);

    // End of Outputs for SubSystem: '<S73>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S73>/No-op' incorporates:
    //   ActionPort: '<S75>/Action Port'

    rtb_Product1_m[0] = rtb_Merge[0];
    rtb_Product1_m[1] = rtb_Merge[1];
    rtb_Product1_m[2] = rtb_Merge[2];
    rtb_Product1_m[3] = rtb_Merge[3];

    // End of Outputs for SubSystem: '<S73>/No-op'
  }

  // End of If: '<S73>/If'

  // RelationalOperator: '<S55>/Compare' incorporates:
  //   Constant: '<S55>/Constant'

  rtb_Compare_fc = (rtb_Switch3 <= ctl_controller0_P->CompareToConstant_const_f);

  // DiscreteIntegrator: '<S31>/Discrete-Time Integrator'
  if (rtb_Compare_fc || ((int32_T)
                         ctl_controller0_DW->DiscreteTimeIntegrator_PrevRese !=
                         0)) {
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

  // Switch: '<S31>/Switch2' incorporates:
  //   Constant: '<S31>/Constant1'
  //   Constant: '<S56>/Constant'
  //   DiscreteIntegrator: '<S31>/Discrete-Time Integrator'
  //   Product: '<S31>/Product'
  //   RelationalOperator: '<S56>/Compare'
  //   Sum: '<S31>/Sum3'
  //   Switch: '<S60>/Switch'

  if (rtb_Switch3 <= ctl_controller0_P->CompareToConstant1_const_m) {
    rtb_Switch_b[0] = ctl_controller0_P->Constant1_Value_o[0];
    rtb_Switch_b[1] = ctl_controller0_P->Constant1_Value_o[1];
    rtb_Switch_b[2] = ctl_controller0_P->Constant1_Value_o[2];
  } else {
    if (rtb_Switch_omega_kd_idx_0 != 0.0F) {
      // Sum: '<S31>/Sum3' incorporates:
      //   Product: '<S31>/Product'
      //   Product: '<S60>/Divide1'
      //   Switch: '<S60>/Switch'

      rtb_SumofElements = rtb_Switch_b[0] / rtb_Switch_omega_kd_idx_0;
    } else {
      // Sum: '<S31>/Sum3' incorporates:
      //   Switch: '<S60>/Switch'

      rtb_SumofElements = 0.0F;
    }

    rtb_Switch_b[0] = rtb_Product1_m[0] * rtb_SumofElements +
      ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0];

    // Sum: '<S31>/Sum3' incorporates:
    //   DiscreteIntegrator: '<S31>/Discrete-Time Integrator'
    //   Product: '<S31>/Product'
    //   Product: '<S60>/Divide1'
    //   Switch: '<S60>/Switch'

    if (rtb_Switch_omega_kd_idx_1 != 0.0F) {
      rtb_SumofElements = rtb_Switch_b[1] / rtb_Switch_omega_kd_idx_1;
    } else {
      rtb_SumofElements = 0.0F;
    }

    rtb_Switch_b[1] = rtb_Product1_m[1] * rtb_SumofElements +
      ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1];

    // Sum: '<S31>/Sum3' incorporates:
    //   DiscreteIntegrator: '<S31>/Discrete-Time Integrator'
    //   Product: '<S31>/Product'
    //   Product: '<S60>/Divide1'
    //   Switch: '<S60>/Switch'

    if (rtb_Switch_omega_kd_idx_2 != 0.0F) {
      rtb_SumofElements = rtb_Switch_b[2] / rtb_Switch_omega_kd_idx_2;
    } else {
      rtb_SumofElements = 0.0F;
    }

    rtb_Switch_b[2] = rtb_Product1_m[2] * rtb_SumofElements +
      ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[2];
  }

  // End of Switch: '<S31>/Switch2'

  // Product: '<S31>/Product3' incorporates:
  //   Product: '<S31>/Product4'
  //   Sum: '<S31>/Sum'

  rtb_Sum_f[0] = ((rtb_Divide_f[0] + rtb_Switch_b[0]) -
                  rtb_Switch_est_omega_B_ISS_B_id) * (rtb_Switch_omega_kd_idx_0 *
    rtb_Switch_inertia_matrix[0]);
  rtb_Sum_f[1] = ((rtb_Divide_f[1] + rtb_Switch_b[1]) -
                  rtb_Switch_est_omega_B_ISS_B__0) * (rtb_Switch_omega_kd_idx_1 *
    rtb_Switch_inertia_matrix[4]);
  rtb_Sum_f[2] = ((rtb_Divide_f[2] + rtb_Switch_b[2]) -
                  rtb_Switch_est_omega_B_ISS_B__1) * (rtb_Switch_omega_kd_idx_2 *
    rtb_Switch_inertia_matrix[8]);

  // Product: '<S31>/Product2'
  for (i = 0; i < 3; i++) {
    rtb_Divide_f[i] = rtb_Switch_inertia_matrix[(int32_T)(i + 6)] *
      rtb_Switch_est_omega_B_ISS_B__1 + (rtb_Switch_inertia_matrix[(int32_T)(i +
      3)] * rtb_Switch_est_omega_B_ISS_B__0 + rtb_Switch_inertia_matrix[i] *
      rtb_Switch_est_omega_B_ISS_B_id);
  }

  // End of Product: '<S31>/Product2'

  // Switch: '<S31>/Switch1' incorporates:
  //   Constant: '<S31>/Constant4'

  if ((int32_T)rtb_Switch3 != 0) {
    // Product: '<S31>/Divide3' incorporates:
    //   Constant: '<S2>/Constant1'
    //   Gain: '<S31>/Gain'
    //   Switch: '<S2>/Switch1'

    if (rtb_LogicalOperator2) {
      rtb_Switch_cmd_state_a_A_B_IS_1 = ctl_controller0_P->Constant1_Value_a[0];
    } else {
      rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Switch_cmd_state_b_alpha__1;
    }

    rtb_SumofElements = ctl_controller0_P->tun_alpha_gain[0] *
      rtb_Switch_cmd_state_a_A_B_IS_1;
    if (rtb_LogicalOperator2) {
      rtb_Switch_cmd_state_a_A_B_IS_1 = ctl_controller0_P->Constant1_Value_a[1];
    } else {
      rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Switch_cmd_state_b_alpha__0;
    }

    rtb_SumofElements1 = ctl_controller0_P->tun_alpha_gain[1] *
      rtb_Switch_cmd_state_a_A_B_IS_1;
    if (rtb_LogicalOperator2) {
      rtb_Switch_cmd_state_a_A_B_IS_1 = ctl_controller0_P->Constant1_Value_a[2];
    } else {
      rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Switch_cmd_state_b_alpha_B_;
    }

    rtb_Switch_cmd_state_a_A_B_IS_1 *= ctl_controller0_P->tun_alpha_gain[2];

    // SignalConversion: '<S58>/TmpSignal ConversionAtProductInport1' incorporates:
    //   Constant: '<S62>/Constant3'
    //   DataTypeConversion: '<S63>/Conversion'
    //   Gain: '<S62>/Gain'
    //   Gain: '<S62>/Gain1'
    //   Gain: '<S62>/Gain2'
    //   Product: '<S58>/Product'

    tmp_3[0] = (real32_T)ctl_controller0_P->Constant3_Value_c;
    tmp_3[1] = rtb_Divide_f[2];
    tmp_3[2] = ctl_controller0_P->Gain_Gain * rtb_Divide_f[1];
    tmp_3[3] = ctl_controller0_P->Gain1_Gain * rtb_Divide_f[2];
    tmp_3[4] = (real32_T)ctl_controller0_P->Constant3_Value_c;
    tmp_3[5] = rtb_Divide_f[0];
    tmp_3[6] = rtb_Divide_f[1];
    tmp_3[7] = ctl_controller0_P->Gain2_Gain * rtb_Divide_f[0];
    tmp_3[8] = (real32_T)ctl_controller0_P->Constant3_Value_c;

    // Product: '<S58>/Product' incorporates:
    //   Product: '<S31>/Divide3'
    //   Sum: '<S31>/Sum1'

    for (i = 0; i < 3; i++) {
      rtb_Switch_b[i] = (((rtb_Switch_inertia_matrix[(int32_T)(i + 3)] *
                           rtb_SumofElements1 + rtb_Switch_inertia_matrix[i] *
                           rtb_SumofElements) + rtb_Switch_inertia_matrix
                          [(int32_T)(i + 6)] * rtb_Switch_cmd_state_a_A_B_IS_1)
                         + rtb_Sum_f[i]) - (tmp_3[(int32_T)(i + 6)] *
        rtb_Switch_est_omega_B_ISS_B__1 + (tmp_3[(int32_T)(i + 3)] *
        rtb_Switch_est_omega_B_ISS_B__0 + tmp_3[i] *
        rtb_Switch_est_omega_B_ISS_B_id));
    }
  } else {
    rtb_Switch_b[0] = ctl_controller0_P->Constant4_Value_f[0];
    rtb_Switch_b[1] = ctl_controller0_P->Constant4_Value_f[1];
    rtb_Switch_b[2] = ctl_controller0_P->Constant4_Value_f[2];
  }

  // End of Switch: '<S31>/Switch1'

  // Sum: '<S81>/Sum1'
  rtb_Gain1_j_idx_0 -= rtb_Switch_est_P_B_ISS_ISS_idx_;
  rtb_Gain1_j_idx_1 -= rtb_Switch_est_P_B_ISS_ISS_id_0;
  rtb_Switch_est_P_B_ISS_ISS_idx_ = rtb_Switch_cmd_state_a_P_B_IS_0 -
    rtb_Switch_est_P_B_ISS_ISS_id_1;

  // DataTypeConversion: '<S101>/Conversion' incorporates:
  //   Constant: '<S100>/Constant2'

  for (i = 0; i < 9; i++) {
    rtb_Assignment_h[i] = (real32_T)ctl_controller0_P->Constant2_Value_i[i];
  }

  // End of DataTypeConversion: '<S101>/Conversion'

  // Assignment: '<S100>/Assignment'
  rtb_Assignment_h[0] = rtb_Merge_j[3];

  // Gain: '<S97>/Gain'
  rtb_Divide_f[0] = ctl_controller0_P->Gain_Gain_is * rtb_Merge_j[0];

  // Assignment: '<S100>/Assignment'
  rtb_Assignment_h[4] = rtb_Merge_j[3];

  // Gain: '<S97>/Gain'
  rtb_Divide_f[1] = ctl_controller0_P->Gain_Gain_is * rtb_Merge_j[1];

  // Assignment: '<S100>/Assignment'
  rtb_Assignment_h[8] = rtb_Merge_j[3];

  // Gain: '<S97>/Gain'
  rtb_Divide_f[2] = ctl_controller0_P->Gain_Gain_is * rtb_Merge_j[2];

  // Sum: '<S100>/Sum2' incorporates:
  //   Constant: '<S102>/Constant3'
  //   DataTypeConversion: '<S103>/Conversion'
  //   Gain: '<S102>/Gain'
  //   Gain: '<S102>/Gain1'
  //   Gain: '<S102>/Gain2'

  tmp_4[0] = (real32_T)ctl_controller0_P->Constant3_Value_n;
  tmp_4[1] = rtb_Divide_f[2];
  tmp_4[2] = ctl_controller0_P->Gain_Gain_m2 * rtb_Divide_f[1];
  tmp_4[3] = ctl_controller0_P->Gain1_Gain_bs * rtb_Divide_f[2];
  tmp_4[4] = (real32_T)ctl_controller0_P->Constant3_Value_n;
  tmp_4[5] = rtb_Divide_f[0];
  tmp_4[6] = rtb_Divide_f[1];
  tmp_4[7] = ctl_controller0_P->Gain2_Gain_o * rtb_Divide_f[0];
  tmp_4[8] = (real32_T)ctl_controller0_P->Constant3_Value_n;

  // Concatenate: '<S100>/Matrix Concatenate' incorporates:
  //   Gain: '<S100>/Gain1'
  //   Sum: '<S100>/Sum2'

  for (i = 0; i < 3; i++) {
    rtb_VectorConcatenate[(int32_T)(i << 2)] = rtb_Assignment_h[(int32_T)(3 * i)]
      + tmp_4[(int32_T)(3 * i)];
    rtb_VectorConcatenate[(int32_T)(1 + (int32_T)(i << 2))] = rtb_Assignment_h
      [(int32_T)((int32_T)(3 * i) + 1)] + tmp_4[(int32_T)((int32_T)(3 * i) + 1)];
    rtb_VectorConcatenate[(int32_T)(2 + (int32_T)(i << 2))] = rtb_Assignment_h
      [(int32_T)((int32_T)(3 * i) + 2)] + tmp_4[(int32_T)((int32_T)(3 * i) + 2)];
  }

  rtb_VectorConcatenate[3] = ctl_controller0_P->Gain1_Gain_ed * rtb_Divide_f[0];
  rtb_VectorConcatenate[7] = ctl_controller0_P->Gain1_Gain_ed * rtb_Divide_f[1];
  rtb_VectorConcatenate[11] = ctl_controller0_P->Gain1_Gain_ed * rtb_Divide_f[2];

  // End of Concatenate: '<S100>/Matrix Concatenate'

  // Reshape: '<S98>/Reshape1'
  rtb_VectorConcatenate[12] = rtb_Divide_f[0];
  rtb_VectorConcatenate[13] = rtb_Divide_f[1];
  rtb_VectorConcatenate[14] = rtb_Divide_f[2];
  rtb_VectorConcatenate[15] = rtb_Merge_j[3];

  // Product: '<S98>/Product'
  for (i = 0; i < 4; i++) {
    rtb_Switch_est_P_B_ISS_ISS_id_0 = rtb_VectorConcatenate[(int32_T)(i + 12)] *
      rtb_Merge_pa[3] + (rtb_VectorConcatenate[(int32_T)(i + 8)] * rtb_Merge_pa
                         [2] + (rtb_VectorConcatenate[(int32_T)(i + 4)] *
      rtb_Merge_pa[1] + rtb_VectorConcatenate[i] * rtb_Merge_pa[0]));
    rtb_Merge[i] = rtb_Switch_est_P_B_ISS_ISS_id_0;
  }

  // End of Product: '<S98>/Product'

  // If: '<S99>/If' incorporates:
  //   Inport: '<S104>/In1'

  if (rtb_Merge[3] < 0.0F) {
    // Outputs for IfAction SubSystem: '<S99>/Normalize' incorporates:
    //   ActionPort: '<S105>/Action Port'

    ctl_controller0_Normalize(rtb_Merge, rtb_Merge_j,
      (P_Normalize_ctl_controller0_T *)&ctl_controller0_P->Normalize_k);

    // End of Outputs for SubSystem: '<S99>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S99>/No-op' incorporates:
    //   ActionPort: '<S104>/Action Port'

    rtb_Merge_j[0] = rtb_Merge[0];
    rtb_Merge_j[1] = rtb_Merge[1];
    rtb_Merge_j[2] = rtb_Merge[2];
    rtb_Merge_j[3] = rtb_Merge[3];

    // End of Outputs for SubSystem: '<S99>/No-op'
  }

  // End of If: '<S99>/If'

  // Sqrt: '<S110>/Sqrt' incorporates:
  //   DotProduct: '<S110>/Dot Product'

  rtb_SumofElements = (real32_T)sqrt((real_T)(((rtb_Merge_j[0] * rtb_Merge_j[0]
    + rtb_Merge_j[1] * rtb_Merge_j[1]) + rtb_Merge_j[2] * rtb_Merge_j[2]) +
    rtb_Merge_j[3] * rtb_Merge_j[3]));

  // If: '<S106>/If' incorporates:
  //   DataTypeConversion: '<S106>/Data Type Conversion'
  //   Inport: '<S108>/In1'

  if ((real_T)rtb_SumofElements > 1.0E-7) {
    // Outputs for IfAction SubSystem: '<S106>/Normalize' incorporates:
    //   ActionPort: '<S109>/Action Port'

    ctl_controller0_Normalize_e(rtb_Merge_j, rtb_SumofElements, rtb_Merge);

    // End of Outputs for SubSystem: '<S106>/Normalize'
  } else {
    // Outputs for IfAction SubSystem: '<S106>/No-op' incorporates:
    //   ActionPort: '<S108>/Action Port'

    rtb_Merge[3] = rtb_Merge_j[3];

    // End of Outputs for SubSystem: '<S106>/No-op'
  }

  // End of If: '<S106>/If'

  // Sum: '<S81>/Sum3'
  rtb_Sum3_k_idx_0 -= rtb_Switch_est_V_B_ISS_ISS_idx_;

  // DiscreteIntegrator: '<S31>/Discrete-Time Integrator'
  rtb_Switch_est_V_B_ISS_ISS_idx_ =
    ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0];

  // Sum: '<S81>/Sum4'
  rtb_Sum4_o_idx_0 -= rtb_Switch_est_omega_B_ISS_B_id;

  // DiscreteIntegrator: '<S30>/Discrete-Time Integrator1'
  rtb_Switch_est_omega_B_ISS_B_id =
    ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0];

  // Sum: '<S81>/Sum3'
  rtb_Sum3_k_idx_1 -= rtb_Switch_est_V_B_ISS_ISS_id_0;

  // DiscreteIntegrator: '<S31>/Discrete-Time Integrator'
  rtb_Switch_est_V_B_ISS_ISS_id_0 =
    ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1];

  // Sum: '<S81>/Sum4'
  rtb_Sum4_o_idx_1 -= rtb_Switch_est_omega_B_ISS_B__0;

  // DiscreteIntegrator: '<S30>/Discrete-Time Integrator1'
  rtb_Switch_est_omega_B_ISS_B__0 =
    ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1];

  // Sum: '<S81>/Sum3'
  rtb_SumofElements1 = rtb_Sum3_k_idx_2 - rtb_Switch_est_V_B_ISS_ISS_id_1;

  // Sum: '<S81>/Sum4'
  rtb_Switch_est_omega_B_ISS_B__1 = rtb_Sum4_p - rtb_Switch_est_omega_B_ISS_B__1;

  // BusCreator: '<S3>/bus_creator' incorporates:
  //   DiscreteIntegrator: '<S30>/Discrete-Time Integrator1'
  //   DiscreteIntegrator: '<S31>/Discrete-Time Integrator'

  rtb_Switch_est_V_B_ISS_ISS_id_1 =
    ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2];
  rtb_Switch_est_P_B_ISS_ISS_id_0 =
    ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[2];

  // Update for UnitDelay: '<S115>/UD'
  ctl_controller0_DW->UD_DSTATE[0] = rtb_TSamp[0];
  ctl_controller0_DW->UD_DSTATE[1] = rtb_TSamp[1];
  ctl_controller0_DW->UD_DSTATE[2] = rtb_TSamp[2];
  ctl_controller0_DW->UD_DSTATE[3] = rtb_TSamp[3];

  // Update for UnitDelay: '<S112>/UD'
  ctl_controller0_DW->UD_DSTATE_e[0] = rtb_Product[0];
  ctl_controller0_DW->UD_DSTATE_e[1] = rtb_Product[1];
  ctl_controller0_DW->UD_DSTATE_e[2] = rtb_Product_f;

  // Update for UnitDelay: '<S2>/Unit Delay2'
  ctl_controller0_DW->UnitDelay2_DSTATE[0] = rtb_Product_b[0];
  ctl_controller0_DW->UnitDelay2_DSTATE[1] = rtb_Product_b[1];
  ctl_controller0_DW->UnitDelay2_DSTATE[2] = rtb_Product_b[2];
  ctl_controller0_DW->UnitDelay2_DSTATE[3] = rtb_Product_b[3];

  // Update for Delay: '<S13>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE[0] = rtb_SumA21_idx_0;

  // Update for Delay: '<S14>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE_i[0] = rtb_SumA21_d_idx_0;

  // Update for UnitDelay: '<S2>/Unit Delay1'
  ctl_controller0_DW->UnitDelay1_DSTATE[0] = rtb_Switch8_idx_0;

  // Update for S-Function (sfix_udelay): '<S2>/Tapped Delay'
  ctl_controller0_DW->TappedDelay_X[0] = ctl_controller0_DW->TappedDelay_X[1];

  // Update for DiscreteIntegrator: '<S30>/Discrete-Time Integrator1' incorporates:
  //   Product: '<S30>/Product1'
  //   Product: '<S38>/Divide1'
  //   Switch: '<S38>/Switch'

  if (rtb_Switch_vel_kd[0] != 0.0F) {
    rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Switch_pos_ki_idx_0 /
      rtb_Switch_vel_kd[0];
  } else {
    rtb_Switch_cmd_state_a_A_B_IS_1 = 0.0F;
  }

  ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0] += rtb_Sum2_idx_0 *
    rtb_Switch_cmd_state_a_A_B_IS_1 *
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

  // Update for DiscreteIntegrator: '<S31>/Discrete-Time Integrator' incorporates:
  //   Product: '<S31>/Product1'
  //   Product: '<S61>/Divide1'
  //   Switch: '<S61>/Switch'

  if (rtb_Switch_omega_kd_idx_0 != 0.0F) {
    rtb_SumofElements = rtb_Switch_att_ki_idx_0 / rtb_Switch_omega_kd_idx_0;
  } else {
    rtb_SumofElements = 0.0F;
  }

  ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0] += rtb_Product1_m[0] *
    rtb_SumofElements * ctl_controller0_P->DiscreteTimeIntegrator_gainval;
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
  ctl_controller0_Y_cmd_msg_f->traj_pos[0] = normA;
  ctl_controller0_Y_cmd_msg_f->traj_vel[0] = b_s;
  ctl_controller0_Y_cmd_msg_f->traj_accel[0] = rtb_Diff[0];

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  // Update for Delay: '<S13>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE[1] = rtb_SumA21_idx_1;

  // Update for Delay: '<S14>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE_i[1] = rtb_SumA21_d_idx_1;

  // Update for UnitDelay: '<S2>/Unit Delay1'
  ctl_controller0_DW->UnitDelay1_DSTATE[1] = rtb_Switch8_idx_1;

  // Update for S-Function (sfix_udelay): '<S2>/Tapped Delay'
  ctl_controller0_DW->TappedDelay_X[1] = ctl_controller0_DW->TappedDelay_X[2];

  // Update for DiscreteIntegrator: '<S30>/Discrete-Time Integrator1' incorporates:
  //   Product: '<S30>/Product1'
  //   Product: '<S38>/Divide1'
  //   Switch: '<S38>/Switch'

  if (rtb_Switch_vel_kd[1] != 0.0F) {
    rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Switch_pos_ki_idx_1 /
      rtb_Switch_vel_kd[1];
  } else {
    rtb_Switch_cmd_state_a_A_B_IS_1 = 0.0F;
  }

  ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1] += rtb_Sum2_idx_1 *
    rtb_Switch_cmd_state_a_A_B_IS_1 *
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

  // Update for DiscreteIntegrator: '<S31>/Discrete-Time Integrator' incorporates:
  //   Product: '<S31>/Product1'
  //   Product: '<S61>/Divide1'
  //   Switch: '<S61>/Switch'

  if (rtb_Switch_omega_kd_idx_1 != 0.0F) {
    rtb_SumofElements = rtb_Switch_att_ki_idx_1 / rtb_Switch_omega_kd_idx_1;
  } else {
    rtb_SumofElements = 0.0F;
  }

  ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1] += rtb_Product1_m[1] *
    rtb_SumofElements * ctl_controller0_P->DiscreteTimeIntegrator_gainval;
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
  ctl_controller0_Y_cmd_msg_f->traj_pos[1] = rtb_Switch_cmd_state_a_P_B_ISS_;
  ctl_controller0_Y_cmd_msg_f->traj_vel[1] = rtb_Switch_cmd_state_a_V_B_ISS_;
  ctl_controller0_Y_cmd_msg_f->traj_accel[1] = rtb_Diff[1];

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  // Update for Delay: '<S13>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE[2] = rtb_Sum4_o_idx_2;

  // Update for Delay: '<S14>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE_i[2] = rtb_SumA21_f;

  // Update for UnitDelay: '<S2>/Unit Delay1'
  ctl_controller0_DW->UnitDelay1_DSTATE[2] = rtb_Switch8_idx_2;

  // Update for S-Function (sfix_udelay): '<S2>/Tapped Delay'
  ctl_controller0_DW->TappedDelay_X[2] = ctl_controller0_DW->TappedDelay_X[3];

  // Update for DiscreteIntegrator: '<S30>/Discrete-Time Integrator1' incorporates:
  //   Product: '<S30>/Product1'
  //   Product: '<S38>/Divide1'
  //   Switch: '<S38>/Switch'

  if (rtb_Switch_vel_kd[2] != 0.0F) {
    rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Switch_pos_ki_idx_2 /
      rtb_Switch_vel_kd[2];
  } else {
    rtb_Switch_cmd_state_a_A_B_IS_1 = 0.0F;
  }

  ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2] += rtb_Sum2_idx_2 *
    rtb_Switch_cmd_state_a_A_B_IS_1 *
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

  // Update for DiscreteIntegrator: '<S31>/Discrete-Time Integrator' incorporates:
  //   Product: '<S31>/Product1'
  //   Product: '<S61>/Divide1'
  //   Switch: '<S61>/Switch'

  if (rtb_Switch_omega_kd_idx_2 != 0.0F) {
    rtb_SumofElements = rtb_Switch_att_ki_idx_2 / rtb_Switch_omega_kd_idx_2;
  } else {
    rtb_SumofElements = 0.0F;
  }

  ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[2] += rtb_Product1_m[2] *
    rtb_SumofElements * ctl_controller0_P->DiscreteTimeIntegrator_gainval;
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

  ctl_controller0_Y_cmd_msg_f->traj_pos[2] = rtb_Switch_cmd_state_a_P_B_IS_0;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_cmd_msg_f->traj_vel[2] = rtb_Sum3_k_idx_2;

  // End of Outputs for SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_cmd_msg_f->traj_accel[2] = rtb_Diff[2];

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  // Update for S-Function (sfix_udelay): '<S2>/Tapped Delay' incorporates:
  //   Constant: '<S8>/Constant'
  //   RelationalOperator: '<S8>/Compare'

  ctl_controller0_DW->TappedDelay_X[3] = (rtb_Switch12 ==
    ctl_controller0_P->ctl_stopping_mode);

  // Update for DiscreteIntegrator: '<S30>/Discrete-Time Integrator1'
  ctl_controller0_DW->DiscreteTimeIntegrator1_PrevRes = (int8_T)rtb_Compare_j;

  // Update for DiscreteIntegrator: '<S31>/Discrete-Time Integrator'
  ctl_controller0_DW->DiscreteTimeIntegrator_PrevRese = (int8_T)rtb_Compare_fc;

  // End of Outputs for SubSystem: '<Root>/ctl_controller'

  // Outport: '<Root>/cmd_msg' incorporates:
  //   BusCreator: '<S4>/Bus Creator1'
  //   DataTypeConversion: '<S78>/Data Type Conversion'

  ctl_controller0_Y_cmd_msg_f->cmd_timestamp_sec =
    rtb_Switch_cmd_state_b_timestam;
  ctl_controller0_Y_cmd_msg_f->cmd_timestamp_nsec =
    rtb_Switch_cmd_state_b_timest_0;
  ctl_controller0_Y_cmd_msg_f->cmd_mode = rtb_Switch_ctl_mode_cmd;
  ctl_controller0_Y_cmd_msg_f->speed_gain_cmd = rtb_Switch_speed_gain_cmd;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_cmd_msg_f->cmd_B_inuse = (uint8_T)rtb_LogicalOperator2_c;
  ctl_controller0_Y_cmd_msg_f->traj_quat[0] = rtb_Merge_pa[0];
  ctl_controller0_Y_cmd_msg_f->traj_quat[1] = rtb_Merge_pa[1];
  ctl_controller0_Y_cmd_msg_f->traj_quat[2] = rtb_Merge_pa[2];
  ctl_controller0_Y_cmd_msg_f->traj_quat[3] = rtb_Merge_pa[3];

  // Product: '<S31>/Divide' incorporates:
  //   Outport: '<Root>/ctl_msg'

  rt_mldivide_U1f3x3_U2f_XeZWzB4d(rtb_Switch_inertia_matrix, rtb_Sum_f,
    ctl_controller0_Y_ctl_msg_n->body_alpha_cmd);

  // Trigonometry: '<S31>/Trigonometric Function'
  if (rtb_Product1_m[3] > 1.0F) {
    rtb_Switch_cmd_state_a_A_B_IS_1 = 1.0F;
  } else if (rtb_Product1_m[3] < -1.0F) {
    rtb_Switch_cmd_state_a_A_B_IS_1 = -1.0F;
  } else {
    rtb_Switch_cmd_state_a_A_B_IS_1 = rtb_Product1_m[3];
  }

  // Outport: '<Root>/ctl_msg' incorporates:
  //   BusCreator: '<S3>/bus_creator'
  //   Gain: '<S31>/Gain2'
  //   Trigonometry: '<S31>/Trigonometric Function'

  ctl_controller0_Y_ctl_msg_n->att_err_mag = ctl_controller0_P->Gain2_Gain_j *
    (real32_T)acos((real_T)rtb_Switch_cmd_state_a_A_B_IS_1);
  ctl_controller0_Y_ctl_msg_n->ctl_status = rtb_Switch3;

  // End of Outputs for SubSystem: '<Root>/ctl_controller'

  // Outport: '<Root>/cmd_msg'
  ctl_controller0_Y_cmd_msg_f->traj_omega[0] = rtb_Switch_cmd_state_a_A_B_ISS_;
  ctl_controller0_Y_cmd_msg_f->traj_alpha[0] = rtb_Switch_cmd_state_b_alpha__1;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  // Outport: '<Root>/ctl_msg' incorporates:
  //   BusCreator: '<S3>/bus_creator'
  //   Product: '<S30>/Divide'

  ctl_controller0_Y_ctl_msg_n->body_force_cmd[0] = rtb_Switch_c_idx_0;
  ctl_controller0_Y_ctl_msg_n->body_accel_cmd[0] = 1.0F / rtb_Switch_mass *
    rtb_Switch_c_idx_0;
  ctl_controller0_Y_ctl_msg_n->pos_err[0] = rtb_Sum2_idx_0;

  // End of Outputs for SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_n->pos_err_int[0] = rtb_Switch_est_omega_B_ISS_B_id;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_n->body_torque_cmd[0] = rtb_Switch_b[0];
  ctl_controller0_Y_ctl_msg_n->att_err[0] = rtb_Product1_m[0];

  // End of Outputs for SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_n->att_err_int[0] = rtb_Switch_est_V_B_ISS_ISS_idx_;

  // Outport: '<Root>/cmd_msg'
  ctl_controller0_Y_cmd_msg_f->traj_omega[1] = rtb_Switch_cmd_state_a_A_B_IS_0;
  ctl_controller0_Y_cmd_msg_f->traj_alpha[1] = rtb_Switch_cmd_state_b_alpha__0;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  // Outport: '<Root>/ctl_msg' incorporates:
  //   BusCreator: '<S3>/bus_creator'
  //   Product: '<S30>/Divide'

  ctl_controller0_Y_ctl_msg_n->body_force_cmd[1] = rtb_Switch_c_idx_1;
  ctl_controller0_Y_ctl_msg_n->body_accel_cmd[1] = 1.0F / rtb_Switch_mass *
    rtb_Switch_c_idx_1;
  ctl_controller0_Y_ctl_msg_n->pos_err[1] = rtb_Sum2_idx_1;

  // End of Outputs for SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_n->pos_err_int[1] = rtb_Switch_est_omega_B_ISS_B__0;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_n->body_torque_cmd[1] = rtb_Switch_b[1];
  ctl_controller0_Y_ctl_msg_n->att_err[1] = rtb_Product1_m[1];

  // End of Outputs for SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_n->att_err_int[1] = rtb_Switch_est_V_B_ISS_ISS_id_0;

  // Outport: '<Root>/cmd_msg'
  ctl_controller0_Y_cmd_msg_f->traj_omega[2] = rtb_Sum4_p;
  ctl_controller0_Y_cmd_msg_f->traj_alpha[2] = rtb_Switch_cmd_state_b_alpha_B_;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  // Outport: '<Root>/ctl_msg' incorporates:
  //   BusCreator: '<S3>/bus_creator'
  //   DotProduct: '<S95>/Dot Product'
  //   Product: '<S30>/Divide'
  //   Sqrt: '<S95>/Sqrt'

  ctl_controller0_Y_ctl_msg_n->body_force_cmd[2] = rtb_Switch_c_idx_2;
  ctl_controller0_Y_ctl_msg_n->body_accel_cmd[2] = 1.0F / rtb_Switch_mass *
    rtb_Switch_c_idx_2;
  ctl_controller0_Y_ctl_msg_n->pos_err[2] = rtb_Sum2_idx_2;

  // End of Outputs for SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_n->pos_err_int[2] = rtb_Switch_est_V_B_ISS_ISS_id_1;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_n->body_torque_cmd[2] = rtb_Switch_b[2];
  ctl_controller0_Y_ctl_msg_n->att_err[2] = rtb_Product1_m[2];

  // End of Outputs for SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_n->att_err_int[2] = rtb_Switch_est_P_B_ISS_ISS_id_0;

  // Outputs for Atomic SubSystem: '<Root>/ctl_controller'
  ctl_controller0_Y_ctl_msg_n->traj_error_pos = (real32_T)sqrt((real_T)
    ((rtb_Gain1_j_idx_0 * rtb_Gain1_j_idx_0 + rtb_Gain1_j_idx_1 *
      rtb_Gain1_j_idx_1) + rtb_Switch_est_P_B_ISS_ISS_idx_ *
     rtb_Switch_est_P_B_ISS_ISS_idx_));

  // Trigonometry: '<S81>/Trigonometric Function'
  if (rtb_Merge[3] > 1.0F) {
    rtb_Switch_est_P_B_ISS_ISS_id_0 = 1.0F;
  } else if (rtb_Merge[3] < -1.0F) {
    rtb_Switch_est_P_B_ISS_ISS_id_0 = -1.0F;
  } else {
    rtb_Switch_est_P_B_ISS_ISS_id_0 = rtb_Merge[3];
  }

  // Outport: '<Root>/ctl_msg' incorporates:
  //   Abs: '<S81>/Abs'
  //   DotProduct: '<S94>/Dot Product'
  //   DotProduct: '<S96>/Dot Product'
  //   Gain: '<S81>/Gain'
  //   Sqrt: '<S94>/Sqrt'
  //   Sqrt: '<S96>/Sqrt'
  //   Trigonometry: '<S81>/Trigonometric Function'

  ctl_controller0_Y_ctl_msg_n->traj_error_att = ctl_controller0_P->Gain_Gain_hg *
    (real32_T)fabs((real_T)(real32_T)acos((real_T)
    rtb_Switch_est_P_B_ISS_ISS_id_0));
  ctl_controller0_Y_ctl_msg_n->traj_error_vel = (real32_T)sqrt((real_T)
    ((rtb_Sum3_k_idx_0 * rtb_Sum3_k_idx_0 + rtb_Sum3_k_idx_1 * rtb_Sum3_k_idx_1)
     + rtb_SumofElements1 * rtb_SumofElements1));
  ctl_controller0_Y_ctl_msg_n->traj_error_omega = (real32_T)sqrt((real_T)
    ((rtb_Sum4_o_idx_0 * rtb_Sum4_o_idx_0 + rtb_Sum4_o_idx_1 * rtb_Sum4_o_idx_1)
     + rtb_Switch_est_omega_B_ISS_B__1 * rtb_Switch_est_omega_B_ISS_B__1));

  // End of Outputs for SubSystem: '<Root>/ctl_controller'
}

// Model initialize function
void ctl_controller0_initialize(RT_MODEL_ctl_controller0_T *const
  ctl_controller0_M, ctl_input_msg *ctl_controller0_U_ctl_input_msg_l, cmd_msg
  *ctl_controller0_Y_cmd_msg_f, ctl_msg *ctl_controller0_Y_ctl_msg_n)
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
  // InitializeConditions for UnitDelay: '<S115>/UD'
  ctl_controller0_DW->UD_DSTATE[0] =
    ctl_controller0_P->DiscreteDerivative_ICPrevScaled;
  ctl_controller0_DW->UD_DSTATE[1] =
    ctl_controller0_P->DiscreteDerivative_ICPrevScaled;
  ctl_controller0_DW->UD_DSTATE[2] =
    ctl_controller0_P->DiscreteDerivative_ICPrevScaled;
  ctl_controller0_DW->UD_DSTATE[3] =
    ctl_controller0_P->DiscreteDerivative_ICPrevScaled;

  // InitializeConditions for UnitDelay: '<S112>/UD'
  ctl_controller0_DW->UD_DSTATE_e[0] =
    ctl_controller0_P->DiscreteDerivative_ICPrevScal_k;
  ctl_controller0_DW->UD_DSTATE_e[1] =
    ctl_controller0_P->DiscreteDerivative_ICPrevScal_k;
  ctl_controller0_DW->UD_DSTATE_e[2] =
    ctl_controller0_P->DiscreteDerivative_ICPrevScal_k;

  // InitializeConditions for UnitDelay: '<S2>/Unit Delay2'
  ctl_controller0_DW->UnitDelay2_DSTATE[0] =
    ctl_controller0_P->UnitDelay2_InitialCondition;
  ctl_controller0_DW->UnitDelay2_DSTATE[1] =
    ctl_controller0_P->UnitDelay2_InitialCondition;
  ctl_controller0_DW->UnitDelay2_DSTATE[2] =
    ctl_controller0_P->UnitDelay2_InitialCondition;
  ctl_controller0_DW->UnitDelay2_DSTATE[3] =
    ctl_controller0_P->UnitDelay2_InitialCondition;

  // InitializeConditions for Delay: '<S13>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE[0] =
    ctl_controller0_P->Delay11_InitialCondition;

  // InitializeConditions for Delay: '<S14>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE_i[0] =
    ctl_controller0_P->Delay11_InitialCondition_k;

  // InitializeConditions for UnitDelay: '<S2>/Unit Delay1'
  ctl_controller0_DW->UnitDelay1_DSTATE[0] =
    ctl_controller0_P->UnitDelay1_InitialCondition;

  // InitializeConditions for Delay: '<S13>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE[1] =
    ctl_controller0_P->Delay11_InitialCondition;

  // InitializeConditions for Delay: '<S14>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE_i[1] =
    ctl_controller0_P->Delay11_InitialCondition_k;

  // InitializeConditions for UnitDelay: '<S2>/Unit Delay1'
  ctl_controller0_DW->UnitDelay1_DSTATE[1] =
    ctl_controller0_P->UnitDelay1_InitialCondition;

  // InitializeConditions for Delay: '<S13>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE[2] =
    ctl_controller0_P->Delay11_InitialCondition;

  // InitializeConditions for Delay: '<S14>/Delay11'
  ctl_controller0_DW->Delay11_DSTATE_i[2] =
    ctl_controller0_P->Delay11_InitialCondition_k;

  // InitializeConditions for UnitDelay: '<S2>/Unit Delay1'
  ctl_controller0_DW->UnitDelay1_DSTATE[2] =
    ctl_controller0_P->UnitDelay1_InitialCondition;

  // InitializeConditions for S-Function (sfix_udelay): '<S2>/Tapped Delay'
  ctl_controller0_DW->TappedDelay_X[0] = ((int32_T)
    ctl_controller0_P->TappedDelay_vinit != 0);
  ctl_controller0_DW->TappedDelay_X[1] = ((int32_T)
    ctl_controller0_P->TappedDelay_vinit != 0);
  ctl_controller0_DW->TappedDelay_X[2] = ((int32_T)
    ctl_controller0_P->TappedDelay_vinit != 0);
  ctl_controller0_DW->TappedDelay_X[3] = ((int32_T)
    ctl_controller0_P->TappedDelay_vinit != 0);

  // InitializeConditions for DiscreteIntegrator: '<S30>/Discrete-Time Integrator1' 
  ctl_controller0_DW->DiscreteTimeIntegrator1_PrevRes = 0;
  ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[0] =
    ctl_controller0_P->DiscreteTimeIntegrator1_IC[0];

  // InitializeConditions for DiscreteIntegrator: '<S31>/Discrete-Time Integrator' 
  ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[0] =
    ctl_controller0_P->DiscreteTimeIntegrator_IC[0];

  // InitializeConditions for DiscreteIntegrator: '<S30>/Discrete-Time Integrator1' 
  ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[1] =
    ctl_controller0_P->DiscreteTimeIntegrator1_IC[1];

  // InitializeConditions for DiscreteIntegrator: '<S31>/Discrete-Time Integrator' 
  ctl_controller0_DW->DiscreteTimeIntegrator_DSTATE[1] =
    ctl_controller0_P->DiscreteTimeIntegrator_IC[1];

  // InitializeConditions for DiscreteIntegrator: '<S30>/Discrete-Time Integrator1' 
  ctl_controller0_DW->DiscreteTimeIntegrator1_DSTATE[2] =
    ctl_controller0_P->DiscreteTimeIntegrator1_IC[2];

  // InitializeConditions for DiscreteIntegrator: '<S31>/Discrete-Time Integrator' 
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
RT_MODEL_ctl_controller0_T *ctl_controller0(ctl_input_msg
  *ctl_controller0_U_ctl_input_msg_l, cmd_msg *ctl_controller0_Y_cmd_msg_f,
  ctl_msg *ctl_controller0_Y_ctl_msg_n)
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
    *ctl_controller0_U_ctl_input_msg_l = ctl_controller0_rtZctl_input_ms;

    // external outputs
    (*ctl_controller0_Y_cmd_msg_f) = ctl_controller0_rtZcmd_msg;
    (*ctl_controller0_Y_ctl_msg_n) = ctl_controller0_rtZctl_msg;
  }

  return ctl_controller0_M;
}

//
// File trailer for generated code.
//
// [EOF]
//
