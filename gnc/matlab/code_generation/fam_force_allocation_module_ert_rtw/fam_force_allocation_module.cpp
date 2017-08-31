//
// File: fam_force_allocation_module.cpp
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
#include "fam_force_allocation_module.h"
#include "fam_force_allocation_module_private.h"

const act_msg fam_force_allocation_module_rtZact_msg = {
  0U,                                  // act_timestamp_sec
  0U,                                  // act_timestamp_nsec

  {
    0U, 0U }
  ,                                    // act_impeller_speed_cmd

  {
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F }
  ,                                    // act_servo_pwm_cmd

  {
    0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F, 0.0F }
  ,                                    // act_nozzle_theta

  {
    0.0F, 0.0F, 0.0F }
  ,                                    // act_predicted_force_B

  {
    0.0F, 0.0F, 0.0F }
  // act_predicted_torque_B
} ;                                    // act_msg ground

const cmd_msg fam_force_allocation_module_r_0 = { 0U,// cmd_timestamp_sec
  0U,                                  // cmd_timestamp_nsec
  0U,                                  // cmd_mode
  0U,                                  // speed_gain_cmd
  0U,                                  // cmd_B_inuse
  { 0.0F, 0.0F, 0.0F },                // traj_pos
  { 0.0F, 0.0F, 0.0F },                // traj_vel
  { 0.0F, 0.0F, 0.0F },                // traj_accel
  { 0.0F, 0.0F, 0.0F, 0.0F },          // traj_quat
  { 0.0F, 0.0F, 0.0F },                // traj_omega
  { 0.0F, 0.0F, 0.0F }                 // traj_alpha
};

const ctl_msg fam_force_allocation_module_r_1 = { { 0.0F, 0.0F, 0.0F },// body_force_cmd 
  { 0.0F, 0.0F, 0.0F },                // body_accel_cmd
  { 0.0F, 0.0F, 0.0F },                // pos_err
  { 0.0F, 0.0F, 0.0F },                // pos_err_int
  { 0.0F, 0.0F, 0.0F },                // body_torque_cmd
  { 0.0F, 0.0F, 0.0F },                // body_alpha_cmd
  { 0.0F, 0.0F, 0.0F },                // att_err
  0.0F,                                // att_err_mag
  { 0.0F, 0.0F, 0.0F },                // att_err_int
  0U,                                  // ctl_status
  0.0F,                                // traj_error_pos
  0.0F,                                // traj_error_att
  0.0F,                                // traj_error_vel
  0.0F                                 // traj_error_omega
};

const cmc_msg fam_force_allocation_module_r_2 = { { 0U,// timestamp_sec
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

const ex_time_msg fam_force_allocation_module_rtZ = { 0U,// timestamp_sec
  0U                                   // timestamp_nsec
};

// Model step function
void fam_force_allocation_module_step(RT_MODEL_fam_force_allocation_T *const
  fam_force_allocation_module_M, ex_time_msg
  *fam_force_allocation_module_U_current_time, cmd_msg
  *fam_force_allocation_module_U_cmd_msg_f, ctl_msg
  *fam_force_allocation_module_U_ctl_msg_n, cmc_msg
  *fam_force_allocation_module_U_cmc_msg_h, act_msg
  *fam_force_allocation_module_Y_act_msg_c)
{
  P_fam_force_allocation_module_T *fam_force_allocation_module_P =
    ((P_fam_force_allocation_module_T *)
     fam_force_allocation_module_M->defaultParam);
  B_fam_force_allocation_module_T *fam_force_allocation_module_B =
    ((B_fam_force_allocation_module_T *) fam_force_allocation_module_M->blockIO);
  DW_fam_force_allocation_modul_T *fam_force_allocation_module_DW =
    ((DW_fam_force_allocation_modul_T *) fam_force_allocation_module_M->dwork);
  real32_T nozzle_moment_arm[36];
  int32_T ibmat;
  int32_T itilerow;
  real32_T c[36];
  uint32_T bpIdx;
  uint32_T bpIdx_0;
  real32_T rtb_positive_thrust_per_nozzle[12];
  real32_T rtb_thrust_norm_by_Cd[6];
  real32_T rtb_cmd_area_per_nozzle[6];
  real32_T rtb_calibrated_nozzle_theta[6];
  real32_T rtb_thrust_norm_by_Cd_h[6];
  real32_T rtb_cmd_area_per_nozzle_f[6];
  real32_T rtb_calibrated_nozzle_theta_h[6];
  real32_T rtb_Product9[12];
  real32_T rtb_VectorConcatenate3[12];
  real32_T rtb_Abs;
  real32_T rtb_thrust_per_nozzle[12];
  real32_T rtb_thrust2torque_B[36];
  int32_T i;
  real32_T rtb_cmd_area_per_nozzle_p;
  real32_T rtb_Switch8_idx_0;
  real32_T rtb_Switch8_idx_2;
  real32_T rtb_Switch8_idx_1;
  real32_T tmp;
  real32_T tmp_0;
  real32_T u0;

  // Lookup_n-D: '<S11>/impeller_speed_lookup' incorporates:
  //   DataTypeConversion: '<S1>/Data Type Conversion'
  //   Inport: '<Root>/cmd_msg'

  bpIdx = plook_u32f_binckpan((real32_T)
    fam_force_allocation_module_U_cmd_msg_f->speed_gain_cmd, *(real32_T (*)[3])&
    fam_force_allocation_module_P->impeller_speed_lookup_bp01Data[0], 2U,
    &fam_force_allocation_module_DW->m_bpIndex);

  // DataTypeConversion: '<S11>/Data Type Conversion' incorporates:
  //   Constant: '<S11>/Constant'
  //   Lookup_n-D: '<S11>/impeller_speed_lookup'
  //   Product: '<S11>/Divide2'
  //   Rounding: '<S11>/Rounding Function'

  tmp_0 = (real32_T)floor((real_T)rt_roundf_snf
    (fam_force_allocation_module_P->abp_impeller_speed2pwm *
     fam_force_allocation_module_P->fam_impeller_speeds[bpIdx]));
  if (rtIsNaNF(tmp_0) || rtIsInfF(tmp_0)) {
    tmp_0 = 0.0F;
  } else {
    tmp_0 = (real32_T)fmod((real_T)tmp_0, (real_T)256.0F);
  }

  // Lookup_n-D: '<S14>/impeller_speed_lookup' incorporates:
  //   DataTypeConversion: '<S1>/Data Type Conversion'
  //   Inport: '<Root>/cmd_msg'

  bpIdx_0 = plook_u32f_binckpan((real32_T)
    fam_force_allocation_module_U_cmd_msg_f->speed_gain_cmd, *(real32_T (*)[3])&
    fam_force_allocation_module_P->impeller_speed_lookup_bp01Dat_l[0], 2U,
    &fam_force_allocation_module_DW->m_bpIndex_e);

  // DataTypeConversion: '<S14>/Data Type Conversion' incorporates:
  //   Constant: '<S14>/Constant'
  //   Lookup_n-D: '<S14>/impeller_speed_lookup'
  //   Product: '<S14>/Divide2'
  //   Rounding: '<S14>/Rounding Function'

  tmp = (real32_T)floor((real_T)rt_roundf_snf
                        (fam_force_allocation_module_P->abp_impeller_speed2pwm *
    fam_force_allocation_module_P->fam_impeller_speeds[bpIdx_0]));
  if (rtIsNaNF(tmp) || rtIsInfF(tmp)) {
    tmp = 0.0F;
  } else {
    tmp = (real32_T)fmod((real_T)tmp, (real_T)256.0F);
  }

  // Outputs for Enabled SubSystem: '<S3>/latch_nozzle_thrust_matricies' incorporates:
  //   EnablePort: '<S9>/Enable'

  // UnitDelay: '<S3>/Unit Delay'
  if (fam_force_allocation_module_DW->UnitDelay_DSTATE > 0.0) {
    // MATLAB Function: '<S9>/MATLAB Function' incorporates:
    //   Constant: '<S9>/Constant1'
    //   Constant: '<S9>/Constant3'
    //   Inport: '<Root>/cmc_msg'
    //   SignalConversion: '<S1>/Signal Conversion'
    //   SignalConversion: '<S9>/OutportBufferForforce2thrust_B'
    //   SignalConversion: '<S9>/OutportBufferFortorque2thrust_B'

    // MATLAB Function 'fam_force_allocation_module/calc_nozzle_thrust/latch_nozzle_thrust_matricies/MATLAB Function': '<S10>:1' 
    // NOTE: if number of nozzles change, must change this hard coded '12' below 
    // '<S10>:1:5'
    for (i = 0; i < 3; i++) {
      ibmat = (int32_T)(i * 12);
      for (itilerow = 0; itilerow < 12; itilerow++) {
        nozzle_moment_arm[(int32_T)(ibmat + itilerow)] =
          fam_force_allocation_module_U_cmc_msg_h->center_of_mass[i];
      }
    }

    for (i = 0; i < 36; i++) {
      nozzle_moment_arm[i] = fam_force_allocation_module_P->fam_P_nozzle_B_B[i]
        - nozzle_moment_arm[i];
    }

    // [m] Distance from nozzles to CG
    // '<S10>:1:7'
    for (i = 0; i < 12; i++) {
      c[i] = nozzle_moment_arm[(int32_T)(i + 12)] *
        fam_force_allocation_module_P->fam_nozzle_orientations[(int32_T)(i + 24)]
        - nozzle_moment_arm[(int32_T)(i + 24)] *
        fam_force_allocation_module_P->fam_nozzle_orientations[(int32_T)(i + 12)];
      c[(int32_T)(i + 12)] = nozzle_moment_arm[(int32_T)(i + 24)] *
        fam_force_allocation_module_P->fam_nozzle_orientations[i] -
        fam_force_allocation_module_P->fam_nozzle_orientations[(int32_T)(i + 24)]
        * nozzle_moment_arm[i];
      c[(int32_T)(i + 24)] =
        fam_force_allocation_module_P->fam_nozzle_orientations[(int32_T)(i + 12)]
        * nozzle_moment_arm[i] - nozzle_moment_arm[(int32_T)(i + 12)] *
        fam_force_allocation_module_P->fam_nozzle_orientations[i];
      rtb_thrust2torque_B[(int32_T)(3 * i)] = -c[i];
      rtb_thrust2torque_B[(int32_T)(1 + (int32_T)(3 * i))] = -c[(int32_T)(i + 12)];
      rtb_thrust2torque_B[(int32_T)(2 + (int32_T)(3 * i))] = -c[(int32_T)(i + 24)];
    }

    // [-] Converts a nozzle thrust into resulting body torques
    // '<S10>:1:8'
    pphlopppohlnphdb_pinv(rtb_thrust2torque_B,
                          fam_force_allocation_module_B->OutportBufferFortorque2thrust_B);

    // [-] Converts body torques into nozzle thrust
    // '<S10>:1:9'
    for (i = 0; i < 12; i++) {
      nozzle_moment_arm[(int32_T)(3 * i)] =
        -fam_force_allocation_module_P->fam_nozzle_orientations[i];
      nozzle_moment_arm[(int32_T)(1 + (int32_T)(3 * i))] =
        -fam_force_allocation_module_P->fam_nozzle_orientations[(int32_T)(i + 12)];
      nozzle_moment_arm[(int32_T)(2 + (int32_T)(3 * i))] =
        -fam_force_allocation_module_P->fam_nozzle_orientations[(int32_T)(i + 24)];
    }

    // [-] Converts a nozzle thrust into resulting body forces
    // '<S10>:1:10'
    pphlopppohlnphdb_pinv(nozzle_moment_arm,
                          fam_force_allocation_module_B->OutportBufferForforce2thrust_B);

    // End of MATLAB Function: '<S9>/MATLAB Function'

    // SignalConversion: '<S9>/OutportBufferForthrust2force_B'
    // [-] Converts body forces into nozzle thrust
    memcpy(&fam_force_allocation_module_B->OutportBufferForthrust2force_B[0],
           &nozzle_moment_arm[0], (uint32_T)(36U * sizeof(real32_T)));

    // SignalConversion: '<S9>/OutportBufferForthrust2torque_B'
    memcpy(&fam_force_allocation_module_B->OutportBufferForthrust2torque_B[0],
           &rtb_thrust2torque_B[0], (uint32_T)(36U * sizeof(real32_T)));
  }

  // End of UnitDelay: '<S3>/Unit Delay'
  // End of Outputs for SubSystem: '<S3>/latch_nozzle_thrust_matricies'
  for (i = 0; i < 12; i++) {
    // Sum: '<S3>/Add' incorporates:
    //   Inport: '<Root>/ctl_msg'
    //   Product: '<S3>/Product'
    //   Product: '<S3>/Product1'

    rtb_thrust_per_nozzle[i] =
      (fam_force_allocation_module_B->OutportBufferForforce2thrust_B[(int32_T)(i
        + 24)] * fam_force_allocation_module_U_ctl_msg_n->body_force_cmd[2] +
       (fam_force_allocation_module_B->OutportBufferForforce2thrust_B[(int32_T)
        (i + 12)] * fam_force_allocation_module_U_ctl_msg_n->body_force_cmd[1] +
        fam_force_allocation_module_B->OutportBufferForforce2thrust_B[i] *
        fam_force_allocation_module_U_ctl_msg_n->body_force_cmd[0])) +
      (fam_force_allocation_module_B->OutportBufferFortorque2thrust_B[(int32_T)
       (i + 24)] * fam_force_allocation_module_U_ctl_msg_n->body_torque_cmd[2] +
       (fam_force_allocation_module_B->OutportBufferFortorque2thrust_B[(int32_T)
        (i + 12)] * fam_force_allocation_module_U_ctl_msg_n->body_torque_cmd[1]
        + fam_force_allocation_module_B->OutportBufferFortorque2thrust_B[i] *
        fam_force_allocation_module_U_ctl_msg_n->body_torque_cmd[0]));
  }

  // MinMax: '<S3>/MinMax6'
  if ((rtb_thrust_per_nozzle[0] <= rtb_thrust_per_nozzle[1]) || rtIsNaNF
      (rtb_thrust_per_nozzle[1])) {
    rtb_Abs = rtb_thrust_per_nozzle[0];
  } else {
    rtb_Abs = rtb_thrust_per_nozzle[1];
  }

  if (!((rtb_Abs <= rtb_thrust_per_nozzle[6]) || rtIsNaNF(rtb_thrust_per_nozzle
        [6]))) {
    rtb_Abs = rtb_thrust_per_nozzle[6];
  }

  if (!((rtb_Abs <= rtb_thrust_per_nozzle[7]) || rtIsNaNF(rtb_thrust_per_nozzle
        [7]))) {
    rtb_Abs = rtb_thrust_per_nozzle[7];
  }

  // Switch: '<S3>/Switch6' incorporates:
  //   Constant: '<S6>/Constant'
  //   MinMax: '<S3>/MinMax6'
  //   RelationalOperator: '<S6>/Compare'
  //   Sum: '<S3>/Add7'

  if (rtb_Abs < fam_force_allocation_module_P->Constant_Value_k) {
    // Abs: '<S3>/Abs'
    rtb_Abs = (real32_T)fabs((real_T)rtb_Abs);
    rtb_Switch8_idx_0 = rtb_thrust_per_nozzle[0] + rtb_Abs;
    rtb_Switch8_idx_2 = rtb_thrust_per_nozzle[6] + rtb_Abs;
    rtb_Switch8_idx_1 = rtb_thrust_per_nozzle[1] + rtb_Abs;
    rtb_Abs += rtb_thrust_per_nozzle[7];
  } else {
    rtb_Switch8_idx_0 = rtb_thrust_per_nozzle[0];
    rtb_Switch8_idx_2 = rtb_thrust_per_nozzle[6];
    rtb_Switch8_idx_1 = rtb_thrust_per_nozzle[1];
    rtb_Abs = rtb_thrust_per_nozzle[7];
  }

  // End of Switch: '<S3>/Switch6'

  // Assignment: '<S3>/assign_x'
  rtb_positive_thrust_per_nozzle[0] = rtb_Switch8_idx_0;
  rtb_positive_thrust_per_nozzle[1] = rtb_Switch8_idx_1;
  rtb_positive_thrust_per_nozzle[6] = rtb_Switch8_idx_2;
  rtb_positive_thrust_per_nozzle[7] = rtb_Abs;

  // MinMax: '<S3>/MinMax7'
  if ((rtb_thrust_per_nozzle[2] <= rtb_thrust_per_nozzle[3]) || rtIsNaNF
      (rtb_thrust_per_nozzle[3])) {
    rtb_Abs = rtb_thrust_per_nozzle[2];
  } else {
    rtb_Abs = rtb_thrust_per_nozzle[3];
  }

  if (!((rtb_Abs <= rtb_thrust_per_nozzle[8]) || rtIsNaNF(rtb_thrust_per_nozzle
        [8]))) {
    rtb_Abs = rtb_thrust_per_nozzle[8];
  }

  if (!((rtb_Abs <= rtb_thrust_per_nozzle[9]) || rtIsNaNF(rtb_thrust_per_nozzle
        [9]))) {
    rtb_Abs = rtb_thrust_per_nozzle[9];
  }

  // Switch: '<S3>/Switch7' incorporates:
  //   Constant: '<S7>/Constant'
  //   MinMax: '<S3>/MinMax7'
  //   RelationalOperator: '<S7>/Compare'
  //   Sum: '<S3>/Add8'

  if (rtb_Abs < fam_force_allocation_module_P->Constant_Value_i) {
    // Abs: '<S3>/Abs1'
    rtb_Abs = (real32_T)fabs((real_T)rtb_Abs);
    rtb_Switch8_idx_0 = rtb_thrust_per_nozzle[2] + rtb_Abs;
    rtb_Switch8_idx_2 = rtb_thrust_per_nozzle[8] + rtb_Abs;
    rtb_Switch8_idx_1 = rtb_thrust_per_nozzle[3] + rtb_Abs;
    rtb_Abs += rtb_thrust_per_nozzle[9];
  } else {
    rtb_Switch8_idx_0 = rtb_thrust_per_nozzle[2];
    rtb_Switch8_idx_2 = rtb_thrust_per_nozzle[8];
    rtb_Switch8_idx_1 = rtb_thrust_per_nozzle[3];
    rtb_Abs = rtb_thrust_per_nozzle[9];
  }

  // End of Switch: '<S3>/Switch7'

  // Assignment: '<S3>/assign_y'
  rtb_positive_thrust_per_nozzle[2] = rtb_Switch8_idx_0;
  rtb_positive_thrust_per_nozzle[3] = rtb_Switch8_idx_1;
  rtb_positive_thrust_per_nozzle[8] = rtb_Switch8_idx_2;
  rtb_positive_thrust_per_nozzle[9] = rtb_Abs;

  // MinMax: '<S3>/MinMax8'
  if ((rtb_thrust_per_nozzle[4] <= rtb_thrust_per_nozzle[5]) || rtIsNaNF
      (rtb_thrust_per_nozzle[5])) {
    rtb_Abs = rtb_thrust_per_nozzle[4];
  } else {
    rtb_Abs = rtb_thrust_per_nozzle[5];
  }

  if (!((rtb_Abs <= rtb_thrust_per_nozzle[10]) || rtIsNaNF
        (rtb_thrust_per_nozzle[10]))) {
    rtb_Abs = rtb_thrust_per_nozzle[10];
  }

  if (!((rtb_Abs <= rtb_thrust_per_nozzle[11]) || rtIsNaNF
        (rtb_thrust_per_nozzle[11]))) {
    rtb_Abs = rtb_thrust_per_nozzle[11];
  }

  // Switch: '<S3>/Switch8' incorporates:
  //   Constant: '<S8>/Constant'
  //   MinMax: '<S3>/MinMax8'
  //   RelationalOperator: '<S8>/Compare'
  //   Sum: '<S3>/Add9'

  if (rtb_Abs < fam_force_allocation_module_P->Constant_Value_ii) {
    // Abs: '<S3>/Abs2'
    rtb_Abs = (real32_T)fabs((real_T)rtb_Abs);
    rtb_Switch8_idx_0 = rtb_thrust_per_nozzle[4] + rtb_Abs;
    rtb_Switch8_idx_2 = rtb_thrust_per_nozzle[10] + rtb_Abs;
    rtb_Switch8_idx_1 = rtb_thrust_per_nozzle[5] + rtb_Abs;
    rtb_Abs += rtb_thrust_per_nozzle[11];
  } else {
    rtb_Switch8_idx_0 = rtb_thrust_per_nozzle[4];
    rtb_Switch8_idx_2 = rtb_thrust_per_nozzle[10];
    rtb_Switch8_idx_1 = rtb_thrust_per_nozzle[5];
    rtb_Abs = rtb_thrust_per_nozzle[11];
  }

  // End of Switch: '<S3>/Switch8'

  // Assignment: '<S3>/assign_z'
  rtb_positive_thrust_per_nozzle[4] = rtb_Switch8_idx_0;
  rtb_positive_thrust_per_nozzle[5] = rtb_Switch8_idx_1;
  rtb_positive_thrust_per_nozzle[10] = rtb_Switch8_idx_2;
  rtb_positive_thrust_per_nozzle[11] = rtb_Abs;

  // Product: '<S13>/Product2' incorporates:
  //   Constant: '<S13>/Constant5'

  for (i = 0; i < 6; i++) {
    rtb_thrust_norm_by_Cd[i] = rtb_positive_thrust_per_nozzle[i] /
      fam_force_allocation_module_P->abp_PM1_discharge_coeff[i];
  }

  // End of Product: '<S13>/Product2'

  // Sum: '<S13>/Add'
  rtb_Switch8_idx_2 = rtb_thrust_norm_by_Cd[0];
  for (i = 0; i < 5; i++) {
    rtb_Switch8_idx_2 += rtb_thrust_norm_by_Cd[(int32_T)(i + 1)];
  }

  // End of Sum: '<S13>/Add'

  // Product: '<S13>/Divide' incorporates:
  //   Lookup_n-D: '<S11>/impeller_speed_lookup'

  rtb_Switch8_idx_2 *= 1.0F / fam_force_allocation_module_P->
    fam_impeller_speeds[bpIdx] /
    fam_force_allocation_module_P->fam_impeller_speeds[bpIdx];

  // Lookup_n-D: '<S13>/fam_Cdp_lookup_pm1'
  rtb_Switch8_idx_2 = look1_iflf_pbinlcapw(rtb_Switch8_idx_2,
    fam_force_allocation_module_P->fam_PM1_lookup_thrust_breakpoints,
    fam_force_allocation_module_P->fam_PM1_lookup_Cdp_data,
    &fam_force_allocation_module_DW->m_bpIndex_n, 315U);

  // Product: '<S13>/Divide1' incorporates:
  //   Constant: '<S13>/Constant1'
  //   Constant: '<S13>/Constant2'
  //   Lookup_n-D: '<S11>/impeller_speed_lookup'

  rtb_Abs = fam_force_allocation_module_P->fam_impeller_speeds[bpIdx] *
    fam_force_allocation_module_P->fam_impeller_speeds[bpIdx] *
    fam_force_allocation_module_P->abp_impeller_diameter *
    fam_force_allocation_module_P->abp_impeller_diameter *
    fam_force_allocation_module_P->const_air_density * rtb_Switch8_idx_2;
  for (i = 0; i < 6; i++) {
    // Product: '<S12>/Product7' incorporates:
    //   Constant: '<S12>/Constant4'
    //   Constant: '<S12>/Constant5'
    //   Product: '<S12>/Product2'

    rtb_Switch8_idx_0 = fam_force_allocation_module_P->Constant4_Value_o *
      fam_force_allocation_module_P->abp_PM1_discharge_coeff[i] *
      fam_force_allocation_module_P->abp_PM1_discharge_coeff[i] * rtb_Abs;

    // Product: '<S12>/Product6'
    rtb_cmd_area_per_nozzle[i] = rtb_positive_thrust_per_nozzle[i] /
      rtb_Switch8_idx_0;

    // Product: '<S12>/Product7'
    rtb_thrust_norm_by_Cd[i] = rtb_Switch8_idx_0;
  }

  // Saturate: '<S12>/Saturation'
  rtb_Abs = (real32_T)cos((real_T)
    fam_force_allocation_module_P->abp_nozzle_min_open_angle);
  rtb_Switch8_idx_0 = (real32_T)cos((real_T)
    fam_force_allocation_module_P->abp_nozzle_max_open_angle);
  for (i = 0; i < 6; i++) {
    // Product: '<S12>/Product5' incorporates:
    //   Constant: '<S12>/Constant3'
    //   Constant: '<S12>/Constant6'
    //   Constant: '<S12>/Constant7'
    //   Constant: '<S12>/Constant9'
    //   Product: '<S12>/Product3'
    //   Product: '<S12>/Product4'
    //   Sum: '<S12>/Subtract2'

    u0 = (fam_force_allocation_module_P->abp_nozzle_intake_height -
          rtb_cmd_area_per_nozzle[i] /
          fam_force_allocation_module_P->abp_nozzle_flap_count /
          fam_force_allocation_module_P->abp_PM1_nozzle_widths[i]) /
      fam_force_allocation_module_P->abp_nozzle_flap_length;

    // Saturate: '<S12>/Saturation'
    if (u0 > rtb_Abs) {
      u0 = rtb_Abs;
    } else {
      if (u0 < rtb_Switch8_idx_0) {
        u0 = rtb_Switch8_idx_0;
      }
    }

    // Trigonometry: '<S12>/Trigonometric Function'
    if (u0 > 1.0F) {
      u0 = 1.0F;
    } else {
      if (u0 < -1.0F) {
        u0 = -1.0F;
      }
    }

    // Sum: '<S12>/Subtract3' incorporates:
    //   Constant: '<S12>/Constant8'
    //   Trigonometry: '<S12>/Trigonometric Function'

    rtb_Switch8_idx_2 = (real32_T)acos((real_T)u0) -
      fam_force_allocation_module_P->abp_nozzle_min_open_angle;

    // Sum: '<S12>/Subtract1' incorporates:
    //   Constant: '<S12>/Constant1'
    //   Constant: '<S12>/Constant2'
    //   Product: '<S12>/Product1'

    rtb_thrust_per_nozzle[i] = rtb_Switch8_idx_2 *
      fam_force_allocation_module_P->fam_nozzle_angle2pwm +
      fam_force_allocation_module_P->abp_servo_min_PWM;

    // Product: '<S16>/Product2' incorporates:
    //   Constant: '<S16>/Constant5'

    rtb_thrust_norm_by_Cd_h[i] = rtb_positive_thrust_per_nozzle[(int32_T)(i + 6)]
      / fam_force_allocation_module_P->abp_PM2_discharge_coeff[i];

    // Sum: '<S12>/Subtract3'
    rtb_calibrated_nozzle_theta[i] = rtb_Switch8_idx_2;
  }

  // Sum: '<S16>/Add'
  rtb_Switch8_idx_2 = rtb_thrust_norm_by_Cd_h[0];
  for (i = 0; i < 5; i++) {
    rtb_Switch8_idx_2 += rtb_thrust_norm_by_Cd_h[(int32_T)(i + 1)];
  }

  // End of Sum: '<S16>/Add'

  // Product: '<S16>/Divide' incorporates:
  //   Lookup_n-D: '<S14>/impeller_speed_lookup'

  rtb_Switch8_idx_2 *= 1.0F / fam_force_allocation_module_P->
    fam_impeller_speeds[bpIdx_0] /
    fam_force_allocation_module_P->fam_impeller_speeds[bpIdx_0];

  // Lookup_n-D: '<S16>/fam_Cdp_lookup_pm1'
  rtb_Switch8_idx_2 = look1_iflf_pbinlcapw(rtb_Switch8_idx_2,
    fam_force_allocation_module_P->fam_PM2_lookup_thrust_breakpoints,
    fam_force_allocation_module_P->fam_PM2_lookup_Cdp_data,
    &fam_force_allocation_module_DW->m_bpIndex_a, 315U);

  // Product: '<S16>/Divide1' incorporates:
  //   Constant: '<S16>/Constant1'
  //   Constant: '<S16>/Constant2'
  //   Lookup_n-D: '<S14>/impeller_speed_lookup'

  rtb_Switch8_idx_2 *= fam_force_allocation_module_P->
    fam_impeller_speeds[bpIdx_0] *
    fam_force_allocation_module_P->fam_impeller_speeds[bpIdx_0] *
    fam_force_allocation_module_P->abp_impeller_diameter *
    fam_force_allocation_module_P->abp_impeller_diameter *
    fam_force_allocation_module_P->const_air_density;

  // Saturate: '<S15>/Saturation'
  rtb_Abs = (real32_T)cos((real_T)
    fam_force_allocation_module_P->abp_nozzle_min_open_angle);
  rtb_Switch8_idx_0 = (real32_T)cos((real_T)
    fam_force_allocation_module_P->abp_nozzle_max_open_angle);
  for (i = 0; i < 6; i++) {
    // Product: '<S15>/Product7' incorporates:
    //   Constant: '<S15>/Constant4'
    //   Constant: '<S15>/Constant5'
    //   Product: '<S15>/Product2'

    rtb_Switch8_idx_1 = fam_force_allocation_module_P->Constant4_Value_m *
      fam_force_allocation_module_P->abp_PM2_discharge_coeff[i] *
      fam_force_allocation_module_P->abp_PM2_discharge_coeff[i] *
      rtb_Switch8_idx_2;

    // Product: '<S15>/Product6'
    rtb_cmd_area_per_nozzle_p = rtb_positive_thrust_per_nozzle[(int32_T)(i + 6)]
      / rtb_Switch8_idx_1;

    // Product: '<S15>/Product5' incorporates:
    //   Constant: '<S15>/Constant3'
    //   Constant: '<S15>/Constant6'
    //   Constant: '<S15>/Constant7'
    //   Constant: '<S15>/Constant9'
    //   Product: '<S15>/Product3'
    //   Product: '<S15>/Product4'
    //   Sum: '<S15>/Subtract2'

    u0 = (fam_force_allocation_module_P->abp_nozzle_intake_height -
          rtb_cmd_area_per_nozzle_p /
          fam_force_allocation_module_P->abp_nozzle_flap_count /
          fam_force_allocation_module_P->abp_PM2_nozzle_widths[i]) /
      fam_force_allocation_module_P->abp_nozzle_flap_length;

    // Saturate: '<S15>/Saturation'
    if (u0 > rtb_Abs) {
      u0 = rtb_Abs;
    } else {
      if (u0 < rtb_Switch8_idx_0) {
        u0 = rtb_Switch8_idx_0;
      }
    }

    // Trigonometry: '<S15>/Trigonometric Function'
    if (u0 > 1.0F) {
      u0 = 1.0F;
    } else {
      if (u0 < -1.0F) {
        u0 = -1.0F;
      }
    }

    // Sum: '<S15>/Subtract3' incorporates:
    //   Constant: '<S15>/Constant8'
    //   Trigonometry: '<S15>/Trigonometric Function'

    u0 = (real32_T)acos((real_T)u0) -
      fam_force_allocation_module_P->abp_nozzle_min_open_angle;

    // Sum: '<S15>/Subtract1' incorporates:
    //   Constant: '<S15>/Constant1'
    //   Constant: '<S15>/Constant2'
    //   Product: '<S15>/Product1'

    rtb_thrust_per_nozzle[(int32_T)(i + 6)] = u0 *
      fam_force_allocation_module_P->fam_nozzle_angle2pwm +
      fam_force_allocation_module_P->abp_servo_min_PWM;

    // Product: '<S15>/Product7'
    rtb_thrust_norm_by_Cd_h[i] = rtb_Switch8_idx_1;

    // Product: '<S15>/Product6'
    rtb_cmd_area_per_nozzle_f[i] = rtb_cmd_area_per_nozzle_p;

    // Sum: '<S15>/Subtract3'
    rtb_calibrated_nozzle_theta_h[i] = u0;
  }

  for (i = 0; i < 6; i++) {
    // SignalConversion: '<S1>/ConcatBufferAtVector Concatenate1In1'
    rtb_positive_thrust_per_nozzle[i] = rtb_calibrated_nozzle_theta[i];

    // SignalConversion: '<S1>/ConcatBufferAtVector Concatenate1In2'
    rtb_positive_thrust_per_nozzle[(int32_T)(i + 6)] =
      rtb_calibrated_nozzle_theta_h[i];

    // SignalConversion: '<S1>/ConcatBufferAtVector Concatenate2In1'
    rtb_Product9[i] = rtb_cmd_area_per_nozzle[i];

    // SignalConversion: '<S1>/ConcatBufferAtVector Concatenate2In2'
    rtb_Product9[(int32_T)(i + 6)] = rtb_cmd_area_per_nozzle_f[i];

    // SignalConversion: '<S1>/ConcatBufferAtVector Concatenate3In1'
    rtb_VectorConcatenate3[i] = rtb_thrust_norm_by_Cd[i];

    // SignalConversion: '<S1>/ConcatBufferAtVector Concatenate3In2'
    rtb_VectorConcatenate3[(int32_T)(i + 6)] = rtb_thrust_norm_by_Cd_h[i];
  }

  // Outport: '<Root>/act_msg' incorporates:
  //   BusCreator: '<S1>/Bus Creator2'
  //   DataTypeConversion: '<S11>/Data Type Conversion'
  //   DataTypeConversion: '<S14>/Data Type Conversion'
  //   Inport: '<Root>/current_time'

  fam_force_allocation_module_Y_act_msg_c->act_timestamp_sec =
    fam_force_allocation_module_U_current_time->timestamp_sec;
  fam_force_allocation_module_Y_act_msg_c->act_timestamp_nsec =
    fam_force_allocation_module_U_current_time->timestamp_nsec;
  fam_force_allocation_module_Y_act_msg_c->act_impeller_speed_cmd[0] = (uint8_T)
    (tmp_0 < 0.0F ? (int32_T)(uint8_T)(int32_T)-(int32_T)(int8_T)(uint8_T)-tmp_0
     : (int32_T)(uint8_T)tmp_0);
  fam_force_allocation_module_Y_act_msg_c->act_impeller_speed_cmd[1] = (uint8_T)
    (tmp < 0.0F ? (int32_T)(uint8_T)(int32_T)-(int32_T)(int8_T)(uint8_T)-tmp :
     (int32_T)(uint8_T)tmp);
  for (i = 0; i < 12; i++) {
    // Outport: '<Root>/act_msg' incorporates:
    //   BusCreator: '<S1>/Bus Creator2'

    fam_force_allocation_module_Y_act_msg_c->act_servo_pwm_cmd[i] =
      rtb_thrust_per_nozzle[i];
    fam_force_allocation_module_Y_act_msg_c->act_nozzle_theta[i] =
      rtb_positive_thrust_per_nozzle[i];

    // Product: '<S2>/Product9'
    rtb_Product9[i] *= rtb_VectorConcatenate3[i];
  }

  // Outport: '<Root>/act_msg' incorporates:
  //   Product: '<S2>/Product3'
  //   Product: '<S2>/Product4'

  for (i = 0; i < 3; i++) {
    fam_force_allocation_module_Y_act_msg_c->act_predicted_force_B[i] = 0.0F;
    fam_force_allocation_module_Y_act_msg_c->act_predicted_torque_B[i] = 0.0F;
    for (ibmat = 0; ibmat < 12; ibmat++) {
      fam_force_allocation_module_Y_act_msg_c->act_predicted_force_B[i] +=
        fam_force_allocation_module_B->OutportBufferForthrust2force_B[(int32_T)
        ((int32_T)(3 * ibmat) + i)] * rtb_Product9[ibmat];
      fam_force_allocation_module_Y_act_msg_c->act_predicted_torque_B[i] +=
        fam_force_allocation_module_B->OutportBufferForthrust2torque_B[(int32_T)
        ((int32_T)(3 * ibmat) + i)] * rtb_Product9[ibmat];
    }
  }

  // Update for UnitDelay: '<S3>/Unit Delay' incorporates:
  //   Constant: '<S3>/Constant'

  fam_force_allocation_module_DW->UnitDelay_DSTATE =
    fam_force_allocation_module_P->Constant_Value;
}

// Model initialize function
void fam_force_allocation_module_initialize(RT_MODEL_fam_force_allocation_T *
  const fam_force_allocation_module_M, ex_time_msg
  *fam_force_allocation_module_U_current_time, cmd_msg
  *fam_force_allocation_module_U_cmd_msg_f, ctl_msg
  *fam_force_allocation_module_U_ctl_msg_n, cmc_msg
  *fam_force_allocation_module_U_cmc_msg_h, act_msg
  *fam_force_allocation_module_Y_act_msg_c)
{
  P_fam_force_allocation_module_T *fam_force_allocation_module_P =
    ((P_fam_force_allocation_module_T *)
     fam_force_allocation_module_M->defaultParam);
  B_fam_force_allocation_module_T *fam_force_allocation_module_B =
    ((B_fam_force_allocation_module_T *) fam_force_allocation_module_M->blockIO);
  DW_fam_force_allocation_modul_T *fam_force_allocation_module_DW =
    ((DW_fam_force_allocation_modul_T *) fam_force_allocation_module_M->dwork);

  {
    int32_T i;

    // InitializeConditions for UnitDelay: '<S3>/Unit Delay'
    fam_force_allocation_module_DW->UnitDelay_DSTATE =
      fam_force_allocation_module_P->UnitDelay_InitialCondition;

    // SystemInitialize for Enabled SubSystem: '<S3>/latch_nozzle_thrust_matricies' 
    for (i = 0; i < 36; i++) {
      // SystemInitialize for Outport: '<S9>/force2thrust_B'
      fam_force_allocation_module_B->OutportBufferForforce2thrust_B[i] =
        fam_force_allocation_module_P->force2thrust_B_Y0;

      // SystemInitialize for Outport: '<S9>/torque2thrust_B'
      fam_force_allocation_module_B->OutportBufferFortorque2thrust_B[i] =
        fam_force_allocation_module_P->torque2thrust_B_Y0;

      // SystemInitialize for Outport: '<S9>/thrust2force_B'
      fam_force_allocation_module_B->OutportBufferForthrust2force_B[i] =
        fam_force_allocation_module_P->thrust2force_B_Y0;

      // SystemInitialize for Outport: '<S9>/thrust2torque_B'
      fam_force_allocation_module_B->OutportBufferForthrust2torque_B[i] =
        fam_force_allocation_module_P->thrust2torque_B_Y0;
    }

    // End of SystemInitialize for SubSystem: '<S3>/latch_nozzle_thrust_matricies' 
  }
}

// Model terminate function
void fam_force_allocation_module_terminate(RT_MODEL_fam_force_allocation_T
  * fam_force_allocation_module_M)
{
  // model code
  rt_FREE(fam_force_allocation_module_M->blockIO);
  if (fam_force_allocation_module_M->paramIsMalloced) {
    rt_FREE(fam_force_allocation_module_M->defaultParam);
  }

  rt_FREE(fam_force_allocation_module_M->dwork);
  rt_FREE(fam_force_allocation_module_M);
}

// Model data allocation function
RT_MODEL_fam_force_allocation_T *fam_force_allocation_module(ex_time_msg
  *fam_force_allocation_module_U_current_time, cmd_msg
  *fam_force_allocation_module_U_cmd_msg_f, ctl_msg
  *fam_force_allocation_module_U_ctl_msg_n, cmc_msg
  *fam_force_allocation_module_U_cmc_msg_h, act_msg
  *fam_force_allocation_module_Y_act_msg_c)
{
  RT_MODEL_fam_force_allocation_T *fam_force_allocation_module_M;
  fam_force_allocation_module_M = (RT_MODEL_fam_force_allocation_T *) malloc
    (sizeof(RT_MODEL_fam_force_allocation_T));
  if (fam_force_allocation_module_M == NULL) {
    return NULL;
  }

  (void) memset((char *)fam_force_allocation_module_M, 0,
                sizeof(RT_MODEL_fam_force_allocation_T));

  // block I/O
  {
    B_fam_force_allocation_module_T *b = (B_fam_force_allocation_module_T *)
      malloc(sizeof(B_fam_force_allocation_module_T));
    rt_VALIDATE_MEMORY(fam_force_allocation_module_M,b);
    fam_force_allocation_module_M->blockIO = (b);
  }

  // parameters
  {
    P_fam_force_allocation_module_T *p;
    static int_T pSeen = 0;

    // only malloc on multiple model instantiation
    if (pSeen == 1 ) {
      p = (P_fam_force_allocation_module_T *) malloc(sizeof
        (P_fam_force_allocation_module_T));
      rt_VALIDATE_MEMORY(fam_force_allocation_module_M,p);
      (void) memcpy(p, &fam_force_allocation_module_P,
                    sizeof(P_fam_force_allocation_module_T));
      fam_force_allocation_module_M->paramIsMalloced = (true);
    } else {
      p = &fam_force_allocation_module_P;
      fam_force_allocation_module_M->paramIsMalloced = (false);
      pSeen = 1;
    }

    fam_force_allocation_module_M->defaultParam = (p);
  }

  // states (dwork)
  {
    DW_fam_force_allocation_modul_T *dwork = (DW_fam_force_allocation_modul_T *)
      malloc(sizeof(DW_fam_force_allocation_modul_T));
    rt_VALIDATE_MEMORY(fam_force_allocation_module_M,dwork);
    fam_force_allocation_module_M->dwork = (dwork);
  }

  {
    P_fam_force_allocation_module_T *fam_force_allocation_module_P =
      ((P_fam_force_allocation_module_T *)
       fam_force_allocation_module_M->defaultParam);
    B_fam_force_allocation_module_T *fam_force_allocation_module_B =
      ((B_fam_force_allocation_module_T *)
       fam_force_allocation_module_M->blockIO);
    DW_fam_force_allocation_modul_T *fam_force_allocation_module_DW =
      ((DW_fam_force_allocation_modul_T *) fam_force_allocation_module_M->dwork);

    // initialize non-finites
    rt_InitInfAndNaN(sizeof(real_T));

    // block I/O
    (void) memset(((void *) fam_force_allocation_module_B), 0,
                  sizeof(B_fam_force_allocation_module_T));

    // states (dwork)
    (void) memset((void *)fam_force_allocation_module_DW, 0,
                  sizeof(DW_fam_force_allocation_modul_T));

    // external inputs
    *fam_force_allocation_module_U_current_time =
      fam_force_allocation_module_rtZ;
    *fam_force_allocation_module_U_cmd_msg_f = fam_force_allocation_module_r_0;
    *fam_force_allocation_module_U_ctl_msg_n = fam_force_allocation_module_r_1;
    *fam_force_allocation_module_U_cmc_msg_h = fam_force_allocation_module_r_2;

    // external outputs
    (*fam_force_allocation_module_Y_act_msg_c) =
      fam_force_allocation_module_rtZact_msg;
  }

  return fam_force_allocation_module_M;
}

//
// File trailer for generated code.
//
// [EOF]
//
