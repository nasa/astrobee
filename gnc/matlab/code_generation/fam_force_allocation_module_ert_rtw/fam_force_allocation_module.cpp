//
// File: fam_force_allocation_module.cpp
//
// Code generated for Simulink model 'fam_force_allocation_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Thu Dec  6 14:20:13 2018
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
  real32_T c[36];
  real32_T X[72];
  real32_T A[72];
  real32_T V[36];
  int32_T r;
  int32_T vcol;
  real32_T tol;
  real32_T U[72];
  boolean_T b_p;
  int32_T ar;
  int32_T ia;
  int32_T ic;
  int32_T b;
  int32_T ib;
  int32_T b_ic;
  uint32_T bpIdx;
  real32_T rtb_Switch;
  real32_T rtb_Switch_n;
  real32_T rtb_thrust_per_nozzle[12];
  real32_T rtb_positive_thrust_per_nozzle[12];
  real32_T rtb_thrust_norm_by_Cd[6];
  real32_T rtb_cmd_area_per_nozzle[6];
  real32_T rtb_calibrated_nozzle_theta[6];
  real32_T rtb_thrust_norm_by_Cd_h[6];
  real32_T rtb_cmd_area_per_nozzle_f[6];
  real32_T rtb_calibrated_nozzle_theta_h[6];
  real32_T rtb_Product9[12];
  real32_T rtb_VectorConcatenate3[12];
  real32_T rtb_thrust2torque_B[36];
  real32_T rtb_Switch8_idx_0;
  real32_T rtb_Switch8_idx_2;
  real32_T rtb_Switch8_idx_1;
  real32_T tmp;
  real32_T tmp_0;
  real32_T unusedExpr[36];
  real32_T unusedExpr_0[36];

  // Switch: '<S19>/Switch' incorporates:
  //   Constant: '<S19>/Constant1'
  //   Constant: '<S22>/Constant'
  //   DataTypeConversion: '<S1>/Data Type Conversion'
  //   Inport: '<Root>/cmd_msg'
  //   Lookup_n-D: '<S19>/impeller_speed_lookup'
  //   RelationalOperator: '<S22>/Compare'

  if (fam_force_allocation_module_U_cmd_msg_f->speed_gain_cmd ==
      fam_force_allocation_module_P->CompareToConstant_const) {
    rtb_Switch = fam_force_allocation_module_P->Constant1_Value;
  } else {
    // Lookup_n-D: '<S19>/impeller_speed_lookup'
    bpIdx = plook_u32f_binckpan((real32_T)
      fam_force_allocation_module_U_cmd_msg_f->speed_gain_cmd, *(real32_T (*)[3])
      &fam_force_allocation_module_P->impeller_speed_lookup_bp01Data[0], 2U,
      &fam_force_allocation_module_DW->m_bpIndex_b);
    rtb_Switch = fam_force_allocation_module_P->fam_impeller_speeds[bpIdx];
  }

  // End of Switch: '<S19>/Switch'

  // DataTypeConversion: '<S19>/Data Type Conversion' incorporates:
  //   Constant: '<S19>/Constant'
  //   Product: '<S19>/Divide2'
  //   Rounding: '<S19>/Rounding Function'

  tmp_0 = (real32_T)floor((real_T)rt_roundf_snf
    (fam_force_allocation_module_P->abp_impeller_speed2pwm * rtb_Switch));
  if (rtIsNaNF(tmp_0) || rtIsInfF(tmp_0)) {
    tmp_0 = 0.0F;
  } else {
    tmp_0 = (real32_T)fmod((real_T)tmp_0, (real_T)256.0F);
  }

  // Switch: '<S23>/Switch' incorporates:
  //   Constant: '<S23>/Constant1'
  //   Constant: '<S26>/Constant'
  //   DataTypeConversion: '<S1>/Data Type Conversion'
  //   Inport: '<Root>/cmd_msg'
  //   Lookup_n-D: '<S23>/impeller_speed_lookup'
  //   RelationalOperator: '<S26>/Compare'

  if (fam_force_allocation_module_U_cmd_msg_f->speed_gain_cmd ==
      fam_force_allocation_module_P->CompareToConstant_const_n) {
    rtb_Switch_n = fam_force_allocation_module_P->Constant1_Value_h;
  } else {
    // Lookup_n-D: '<S23>/impeller_speed_lookup'
    bpIdx = plook_u32f_binckpan((real32_T)
      fam_force_allocation_module_U_cmd_msg_f->speed_gain_cmd, *(real32_T (*)[3])
      &fam_force_allocation_module_P->impeller_speed_lookup_bp01Dat_l[0], 2U,
      &fam_force_allocation_module_DW->m_bpIndex_e);
    rtb_Switch_n = fam_force_allocation_module_P->fam_impeller_speeds[bpIdx];
  }

  // End of Switch: '<S23>/Switch'

  // DataTypeConversion: '<S23>/Data Type Conversion' incorporates:
  //   Constant: '<S23>/Constant'
  //   Product: '<S23>/Divide2'
  //   Rounding: '<S23>/Rounding Function'

  tmp = (real32_T)floor((real_T)rt_roundf_snf
                        (fam_force_allocation_module_P->abp_impeller_speed2pwm *
    rtb_Switch_n));
  if (rtIsNaNF(tmp) || rtIsInfF(tmp)) {
    tmp = 0.0F;
  } else {
    tmp = (real32_T)fmod((real_T)tmp, (real_T)256.0F);
  }

  // Outputs for Enabled SubSystem: '<S3>/latch_nozzle_thrust_matricies' incorporates:
  //   EnablePort: '<S12>/Enable'

  // Logic: '<S3>/Logical Operator' incorporates:
  //   Inport: '<Root>/cmc_msg'
  //   RelationalOperator: '<S11>/FixPt Relational Operator'
  //   SignalConversion: '<S1>/Signal Conversion'
  //   UnitDelay: '<S11>/Delay Input1'

  if ((fam_force_allocation_module_U_cmc_msg_h->center_of_mass[0] !=
       fam_force_allocation_module_DW->DelayInput1_DSTATE[0]) ||
      (fam_force_allocation_module_U_cmc_msg_h->center_of_mass[1] !=
       fam_force_allocation_module_DW->DelayInput1_DSTATE[1]) ||
      (fam_force_allocation_module_U_cmc_msg_h->center_of_mass[2] !=
       fam_force_allocation_module_DW->DelayInput1_DSTATE[2])) {
    // MATLAB Function: '<S12>/MATLAB Function' incorporates:
    //   Constant: '<S12>/Constant1'
    //   Constant: '<S12>/Constant3'

    // MATLAB Function 'fam_force_allocation_module/calc_nozzle_thrust/latch_nozzle_thrust_matricies/MATLAB Function': '<S13>:1' 
    // NOTE: if number of nozzles change, must change this hard coded '12' below 
    // '<S13>:1:5'
    for (r = 0; r < 3; r++) {
      vcol = (int32_T)(r * 12);
      for (ic = 0; ic < 12; ic++) {
        nozzle_moment_arm[(int32_T)(vcol + ic)] =
          fam_force_allocation_module_U_cmc_msg_h->center_of_mass[r];
      }
    }

    for (r = 0; r < 36; r++) {
      nozzle_moment_arm[r] = fam_force_allocation_module_P->fam_P_nozzle_B_B[r]
        - nozzle_moment_arm[r];
    }

    // [m] Distance from nozzles to CG
    // '<S13>:1:7'
    for (r = 0; r < 12; r++) {
      c[r] = nozzle_moment_arm[(int32_T)(r + 12)] *
        fam_force_allocation_module_P->fam_nozzle_orientations[(int32_T)(r + 24)]
        - nozzle_moment_arm[(int32_T)(r + 24)] *
        fam_force_allocation_module_P->fam_nozzle_orientations[(int32_T)(r + 12)];
      c[(int32_T)(r + 12)] = nozzle_moment_arm[(int32_T)(r + 24)] *
        fam_force_allocation_module_P->fam_nozzle_orientations[r] -
        fam_force_allocation_module_P->fam_nozzle_orientations[(int32_T)(r + 24)]
        * nozzle_moment_arm[r];
      c[(int32_T)(r + 24)] =
        fam_force_allocation_module_P->fam_nozzle_orientations[(int32_T)(r + 12)]
        * nozzle_moment_arm[r] - nozzle_moment_arm[(int32_T)(r + 12)] *
        fam_force_allocation_module_P->fam_nozzle_orientations[r];
      rtb_thrust2torque_B[(int32_T)(3 * r)] = -c[r];
      rtb_thrust2torque_B[(int32_T)(1 + (int32_T)(3 * r))] = -c[(int32_T)(r + 12)];
      rtb_thrust2torque_B[(int32_T)(2 + (int32_T)(3 * r))] = -c[(int32_T)(r + 24)];
    }

    // [-] Converts a nozzle thrust into resulting body torques
    // '<S13>:1:8'
    pphlopppohlnphdb_pinv(rtb_thrust2torque_B, unusedExpr);

    // [-] Converts body torques into nozzle thrust
    // '<S13>:1:9'
    for (r = 0; r < 12; r++) {
      nozzle_moment_arm[(int32_T)(3 * r)] =
        -fam_force_allocation_module_P->fam_nozzle_orientations[r];
      nozzle_moment_arm[(int32_T)(1 + (int32_T)(3 * r))] =
        -fam_force_allocation_module_P->fam_nozzle_orientations[(int32_T)(r + 12)];
      nozzle_moment_arm[(int32_T)(2 + (int32_T)(3 * r))] =
        -fam_force_allocation_module_P->fam_nozzle_orientations[(int32_T)(r + 24)];
    }

    // [-] Converts a nozzle thrust into resulting body forces
    // '<S13>:1:10'
    pphlopppohlnphdb_pinv(nozzle_moment_arm, unusedExpr_0);

    // [-] Converts body forces into nozzle thrust
    // '<S13>:1:12'
    for (r = 0; r < 3; r++) {
      for (vcol = 0; vcol < 12; vcol++) {
        A[(int32_T)(vcol + (int32_T)(12 * r))] = nozzle_moment_arm[(int32_T)
          ((int32_T)(3 * vcol) + r)];
        A[(int32_T)(vcol + (int32_T)(12 * (int32_T)(r + 3)))] =
          rtb_thrust2torque_B[(int32_T)((int32_T)(3 * vcol) + r)];
      }
    }

    b_p = true;
    for (r = 0; r < 72; r++) {
      X[r] = 0.0F;
      if (b_p && ((!rtIsInfF(A[r])) && (!rtIsNaNF(A[r])))) {
      } else {
        b_p = false;
      }
    }

    if (b_p) {
      ekfccjmgknopmoph_svd(A, U, c, V);
    } else {
      for (r = 0; r < 72; r++) {
        U[r] = (rtNaNF);
      }

      memset(&c[0], 0, (uint32_T)(36U * sizeof(real32_T)));
      for (r = 0; r < 6; r++) {
        c[(int32_T)(r + (int32_T)(6 * r))] = (rtNaNF);
      }

      for (r = 0; r < 36; r++) {
        V[r] = (rtNaNF);
      }
    }

    tol = 12.0F * c[0] * 1.1920929E-7F;
    r = 0;
    vcol = 0;
    while (((int32_T)(vcol + 1) < 7) && (c[(int32_T)((int32_T)(6 * vcol) + vcol)]
            > tol)) {
      r++;
      vcol++;
    }

    if (r > 0) {
      vcol = 0;
      for (ic = 0; (int32_T)(ic + 1) <= r; ic++) {
        tol = 1.0F / c[(int32_T)((int32_T)(6 * ic) + ic)];
        for (ar = vcol; (int32_T)(ar + 1) <= (int32_T)(vcol + 6); ar++) {
          V[ar] *= tol;
        }

        vcol += 6;
      }

      for (vcol = 0; vcol <= 67; vcol += 6) {
        for (ic = vcol; (int32_T)(ic + 1) <= (int32_T)(vcol + 6); ic++) {
          X[ic] = 0.0F;
        }
      }

      vcol = -1;
      for (ic = 0; ic <= 67; ic += 6) {
        ar = -1;
        vcol++;
        b = (int32_T)((int32_T)((int32_T)((int32_T)(r - 1) * 12) + vcol) + 1);
        for (ib = vcol; (int32_T)(ib + 1) <= b; ib += 12) {
          if (U[ib] != 0.0F) {
            ia = ar;
            for (b_ic = ic; (int32_T)(b_ic + 1) <= (int32_T)(ic + 6); b_ic++) {
              ia++;
              X[b_ic] += U[ib] * V[ia];
            }
          }

          ar += 6;
        }
      }
    }

    // SignalConversion: '<S12>/OutportBufferForforcetorque2thrust_B' incorporates:
    //   MATLAB Function: '<S12>/MATLAB Function'

    for (r = 0; r < 6; r++) {
      for (vcol = 0; vcol < 12; vcol++) {
        fam_force_allocation_module_B->OutportBufferForforcetorque2thr[(int32_T)
          (vcol + (int32_T)(12 * r))] = X[(int32_T)((int32_T)(6 * vcol) + r)];
      }
    }

    // End of SignalConversion: '<S12>/OutportBufferForforcetorque2thrust_B'

    // SignalConversion: '<S12>/OutportBufferForthrust2force_B'
    memcpy(&fam_force_allocation_module_B->OutportBufferForthrust2force_B[0],
           &nozzle_moment_arm[0], (uint32_T)(36U * sizeof(real32_T)));

    // SignalConversion: '<S12>/OutportBufferForthrust2torque_B'
    memcpy(&fam_force_allocation_module_B->OutportBufferForthrust2torque_B[0],
           &rtb_thrust2torque_B[0], (uint32_T)(36U * sizeof(real32_T)));
  }

  // End of Logic: '<S3>/Logical Operator'
  // End of Outputs for SubSystem: '<S3>/latch_nozzle_thrust_matricies'

  // SignalConversion: '<S3>/TmpSignal ConversionAtProduct2Inport2' incorporates:
  //   Inport: '<Root>/ctl_msg'

  rtb_thrust_norm_by_Cd[0] =
    fam_force_allocation_module_U_ctl_msg_n->body_force_cmd[0];
  rtb_thrust_norm_by_Cd[3] =
    fam_force_allocation_module_U_ctl_msg_n->body_torque_cmd[0];
  rtb_thrust_norm_by_Cd[1] =
    fam_force_allocation_module_U_ctl_msg_n->body_force_cmd[1];
  rtb_thrust_norm_by_Cd[4] =
    fam_force_allocation_module_U_ctl_msg_n->body_torque_cmd[1];
  rtb_thrust_norm_by_Cd[2] =
    fam_force_allocation_module_U_ctl_msg_n->body_force_cmd[2];
  rtb_thrust_norm_by_Cd[5] =
    fam_force_allocation_module_U_ctl_msg_n->body_torque_cmd[2];

  // Product: '<S3>/Product2' incorporates:
  //   SignalConversion: '<S3>/TmpSignal ConversionAtProduct2Inport2'

  for (r = 0; r < 12; r++) {
    rtb_thrust_per_nozzle[r] = 0.0F;
    for (vcol = 0; vcol < 6; vcol++) {
      rtb_thrust_per_nozzle[r] +=
        fam_force_allocation_module_B->OutportBufferForforcetorque2thr[(int32_T)
        ((int32_T)(12 * vcol) + r)] * rtb_thrust_norm_by_Cd[vcol];
    }
  }

  // End of Product: '<S3>/Product2'

  // MinMax: '<S3>/MinMax6'
  if ((rtb_thrust_per_nozzle[0] <= rtb_thrust_per_nozzle[1]) || rtIsNaNF
      (rtb_thrust_per_nozzle[1])) {
    tol = rtb_thrust_per_nozzle[0];
  } else {
    tol = rtb_thrust_per_nozzle[1];
  }

  if (!((tol <= rtb_thrust_per_nozzle[6]) || rtIsNaNF(rtb_thrust_per_nozzle[6])))
  {
    tol = rtb_thrust_per_nozzle[6];
  }

  if (!((tol <= rtb_thrust_per_nozzle[7]) || rtIsNaNF(rtb_thrust_per_nozzle[7])))
  {
    tol = rtb_thrust_per_nozzle[7];
  }

  // Switch: '<S3>/Switch6' incorporates:
  //   Constant: '<S8>/Constant'
  //   MinMax: '<S3>/MinMax6'
  //   RelationalOperator: '<S8>/Compare'
  //   Sum: '<S3>/Add7'

  if (tol < fam_force_allocation_module_P->Constant_Value) {
    // Abs: '<S3>/Abs'
    tol = (real32_T)fabs((real_T)tol);
    rtb_Switch8_idx_0 = rtb_thrust_per_nozzle[0] + tol;
    rtb_Switch8_idx_2 = rtb_thrust_per_nozzle[6] + tol;
    rtb_Switch8_idx_1 = rtb_thrust_per_nozzle[1] + tol;
    tol += rtb_thrust_per_nozzle[7];
  } else {
    rtb_Switch8_idx_0 = rtb_thrust_per_nozzle[0];
    rtb_Switch8_idx_2 = rtb_thrust_per_nozzle[6];
    rtb_Switch8_idx_1 = rtb_thrust_per_nozzle[1];
    tol = rtb_thrust_per_nozzle[7];
  }

  // End of Switch: '<S3>/Switch6'

  // Assignment: '<S3>/assign_x'
  rtb_positive_thrust_per_nozzle[0] = rtb_Switch8_idx_0;
  rtb_positive_thrust_per_nozzle[1] = rtb_Switch8_idx_1;
  rtb_positive_thrust_per_nozzle[6] = rtb_Switch8_idx_2;
  rtb_positive_thrust_per_nozzle[7] = tol;

  // MinMax: '<S3>/MinMax7'
  if ((rtb_thrust_per_nozzle[2] <= rtb_thrust_per_nozzle[3]) || rtIsNaNF
      (rtb_thrust_per_nozzle[3])) {
    tol = rtb_thrust_per_nozzle[2];
  } else {
    tol = rtb_thrust_per_nozzle[3];
  }

  if (!((tol <= rtb_thrust_per_nozzle[8]) || rtIsNaNF(rtb_thrust_per_nozzle[8])))
  {
    tol = rtb_thrust_per_nozzle[8];
  }

  if (!((tol <= rtb_thrust_per_nozzle[9]) || rtIsNaNF(rtb_thrust_per_nozzle[9])))
  {
    tol = rtb_thrust_per_nozzle[9];
  }

  // Switch: '<S3>/Switch7' incorporates:
  //   Constant: '<S9>/Constant'
  //   MinMax: '<S3>/MinMax7'
  //   RelationalOperator: '<S9>/Compare'
  //   Sum: '<S3>/Add8'

  if (tol < fam_force_allocation_module_P->Constant_Value_k) {
    // Abs: '<S3>/Abs1'
    tol = (real32_T)fabs((real_T)tol);
    rtb_Switch8_idx_0 = rtb_thrust_per_nozzle[2] + tol;
    rtb_Switch8_idx_2 = rtb_thrust_per_nozzle[8] + tol;
    rtb_Switch8_idx_1 = rtb_thrust_per_nozzle[3] + tol;
    tol += rtb_thrust_per_nozzle[9];
  } else {
    rtb_Switch8_idx_0 = rtb_thrust_per_nozzle[2];
    rtb_Switch8_idx_2 = rtb_thrust_per_nozzle[8];
    rtb_Switch8_idx_1 = rtb_thrust_per_nozzle[3];
    tol = rtb_thrust_per_nozzle[9];
  }

  // End of Switch: '<S3>/Switch7'

  // Assignment: '<S3>/assign_y'
  rtb_positive_thrust_per_nozzle[2] = rtb_Switch8_idx_0;
  rtb_positive_thrust_per_nozzle[3] = rtb_Switch8_idx_1;
  rtb_positive_thrust_per_nozzle[8] = rtb_Switch8_idx_2;
  rtb_positive_thrust_per_nozzle[9] = tol;

  // MinMax: '<S3>/MinMax8'
  if ((rtb_thrust_per_nozzle[4] <= rtb_thrust_per_nozzle[5]) || rtIsNaNF
      (rtb_thrust_per_nozzle[5])) {
    tol = rtb_thrust_per_nozzle[4];
  } else {
    tol = rtb_thrust_per_nozzle[5];
  }

  if (!((tol <= rtb_thrust_per_nozzle[10]) || rtIsNaNF(rtb_thrust_per_nozzle[10])))
  {
    tol = rtb_thrust_per_nozzle[10];
  }

  if (!((tol <= rtb_thrust_per_nozzle[11]) || rtIsNaNF(rtb_thrust_per_nozzle[11])))
  {
    tol = rtb_thrust_per_nozzle[11];
  }

  // Switch: '<S3>/Switch8' incorporates:
  //   Constant: '<S10>/Constant'
  //   MinMax: '<S3>/MinMax8'
  //   RelationalOperator: '<S10>/Compare'
  //   Sum: '<S3>/Add9'

  if (tol < fam_force_allocation_module_P->Constant_Value_b) {
    // Abs: '<S3>/Abs2'
    tol = (real32_T)fabs((real_T)tol);
    rtb_Switch8_idx_0 = rtb_thrust_per_nozzle[4] + tol;
    rtb_Switch8_idx_2 = rtb_thrust_per_nozzle[10] + tol;
    rtb_Switch8_idx_1 = rtb_thrust_per_nozzle[5] + tol;
    tol += rtb_thrust_per_nozzle[11];
  } else {
    rtb_Switch8_idx_0 = rtb_thrust_per_nozzle[4];
    rtb_Switch8_idx_2 = rtb_thrust_per_nozzle[10];
    rtb_Switch8_idx_1 = rtb_thrust_per_nozzle[5];
    tol = rtb_thrust_per_nozzle[11];
  }

  // End of Switch: '<S3>/Switch8'

  // Assignment: '<S3>/assign_z'
  rtb_positive_thrust_per_nozzle[4] = rtb_Switch8_idx_0;
  rtb_positive_thrust_per_nozzle[5] = rtb_Switch8_idx_1;
  rtb_positive_thrust_per_nozzle[10] = rtb_Switch8_idx_2;
  rtb_positive_thrust_per_nozzle[11] = tol;

  // Product: '<S21>/Product2' incorporates:
  //   Constant: '<S21>/Constant5'

  for (vcol = 0; vcol < 6; vcol++) {
    rtb_thrust_norm_by_Cd[vcol] = rtb_positive_thrust_per_nozzle[vcol] /
      fam_force_allocation_module_P->abp_PM1_discharge_coeff[vcol];
  }

  // End of Product: '<S21>/Product2'

  // Sum: '<S21>/Add'
  tol = rtb_thrust_norm_by_Cd[0];
  for (b = 0; b < 5; b++) {
    tol += rtb_thrust_norm_by_Cd[(int32_T)(b + 1)];
  }

  // End of Sum: '<S21>/Add'

  // Product: '<S21>/Divide'
  tol *= 1.0F / rtb_Switch / rtb_Switch;

  // Lookup_n-D: '<S21>/fam_Cdp_lookup_pm1'
  tol = look1_iflf_pbinlcapw(tol,
    fam_force_allocation_module_P->fam_PM1_lookup_thrust_breakpoints,
    fam_force_allocation_module_P->fam_PM1_lookup_Cdp_data,
    &fam_force_allocation_module_DW->m_bpIndex, 315U);

  // Product: '<S21>/Divide1' incorporates:
  //   Constant: '<S21>/Constant1'
  //   Constant: '<S21>/Constant2'

  rtb_Switch = rtb_Switch * rtb_Switch *
    fam_force_allocation_module_P->abp_impeller_diameter *
    fam_force_allocation_module_P->abp_impeller_diameter *
    fam_force_allocation_module_P->const_air_density * tol;
  for (vcol = 0; vcol < 6; vcol++) {
    // Product: '<S20>/Product7' incorporates:
    //   Constant: '<S20>/Constant4'
    //   Constant: '<S20>/Constant5'
    //   Product: '<S20>/Product2'

    tol = fam_force_allocation_module_P->Constant4_Value_o *
      fam_force_allocation_module_P->abp_PM1_discharge_coeff[vcol] *
      fam_force_allocation_module_P->abp_PM1_discharge_coeff[vcol] * rtb_Switch;

    // Product: '<S20>/Product6'
    rtb_cmd_area_per_nozzle[vcol] = rtb_positive_thrust_per_nozzle[vcol] / tol;

    // Product: '<S20>/Product7'
    rtb_thrust_norm_by_Cd[vcol] = tol;
  }

  // Saturate: '<S20>/Saturation'
  rtb_Switch = (real32_T)cos((real_T)
    fam_force_allocation_module_P->abp_nozzle_min_open_angle);
  tol = (real32_T)cos((real_T)
                      fam_force_allocation_module_P->abp_nozzle_max_open_angle);
  for (vcol = 0; vcol < 6; vcol++) {
    // Product: '<S20>/Product5' incorporates:
    //   Constant: '<S20>/Constant3'
    //   Constant: '<S20>/Constant6'
    //   Constant: '<S20>/Constant7'
    //   Constant: '<S20>/Constant9'
    //   Product: '<S20>/Product3'
    //   Product: '<S20>/Product4'
    //   Sum: '<S20>/Subtract2'

    rtb_Switch8_idx_1 = (fam_force_allocation_module_P->abp_nozzle_intake_height
                         - rtb_cmd_area_per_nozzle[vcol] /
                         fam_force_allocation_module_P->abp_nozzle_flap_count /
                         fam_force_allocation_module_P->
                         abp_PM1_nozzle_widths[vcol]) /
      fam_force_allocation_module_P->abp_nozzle_flap_length;

    // Saturate: '<S20>/Saturation'
    if (rtb_Switch8_idx_1 > rtb_Switch) {
      rtb_Switch8_idx_1 = rtb_Switch;
    } else {
      if (rtb_Switch8_idx_1 < tol) {
        rtb_Switch8_idx_1 = tol;
      }
    }

    // Trigonometry: '<S20>/Trigonometric Function'
    if (rtb_Switch8_idx_1 > 1.0F) {
      rtb_Switch8_idx_1 = 1.0F;
    } else {
      if (rtb_Switch8_idx_1 < -1.0F) {
        rtb_Switch8_idx_1 = -1.0F;
      }
    }

    // Sum: '<S20>/Subtract3' incorporates:
    //   Constant: '<S20>/Constant8'
    //   Trigonometry: '<S20>/Trigonometric Function'

    rtb_Switch8_idx_0 = (real32_T)acos((real_T)rtb_Switch8_idx_1) -
      fam_force_allocation_module_P->abp_nozzle_min_open_angle;

    // Sum: '<S20>/Subtract1' incorporates:
    //   Constant: '<S20>/Constant1'
    //   Constant: '<S20>/Constant2'
    //   Product: '<S20>/Product1'

    rtb_thrust_per_nozzle[vcol] = rtb_Switch8_idx_0 *
      fam_force_allocation_module_P->fam_nozzle_angle2pwm +
      fam_force_allocation_module_P->abp_servo_min_PWM;

    // Product: '<S25>/Product2' incorporates:
    //   Constant: '<S25>/Constant5'

    rtb_thrust_norm_by_Cd_h[vcol] = rtb_positive_thrust_per_nozzle[(int32_T)
      (vcol + 6)] / fam_force_allocation_module_P->abp_PM2_discharge_coeff[vcol];

    // Sum: '<S20>/Subtract3'
    rtb_calibrated_nozzle_theta[vcol] = rtb_Switch8_idx_0;
  }

  // Sum: '<S25>/Add'
  tol = rtb_thrust_norm_by_Cd_h[0];
  for (b = 0; b < 5; b++) {
    tol += rtb_thrust_norm_by_Cd_h[(int32_T)(b + 1)];
  }

  // End of Sum: '<S25>/Add'

  // Product: '<S25>/Divide'
  tol *= 1.0F / rtb_Switch_n / rtb_Switch_n;

  // Lookup_n-D: '<S25>/fam_Cdp_lookup_pm1'
  tol = look1_iflf_pbinlcapw(tol,
    fam_force_allocation_module_P->fam_PM2_lookup_thrust_breakpoints,
    fam_force_allocation_module_P->fam_PM2_lookup_Cdp_data,
    &fam_force_allocation_module_DW->m_bpIndex_a, 315U);

  // Product: '<S25>/Divide1' incorporates:
  //   Constant: '<S25>/Constant1'
  //   Constant: '<S25>/Constant2'

  rtb_Switch_n = rtb_Switch_n * rtb_Switch_n *
    fam_force_allocation_module_P->abp_impeller_diameter *
    fam_force_allocation_module_P->abp_impeller_diameter *
    fam_force_allocation_module_P->const_air_density * tol;

  // Saturate: '<S24>/Saturation'
  rtb_Switch = (real32_T)cos((real_T)
    fam_force_allocation_module_P->abp_nozzle_min_open_angle);
  tol = (real32_T)cos((real_T)
                      fam_force_allocation_module_P->abp_nozzle_max_open_angle);
  for (vcol = 0; vcol < 6; vcol++) {
    // Product: '<S24>/Product7' incorporates:
    //   Constant: '<S24>/Constant4'
    //   Constant: '<S24>/Constant5'
    //   Product: '<S24>/Product2'

    rtb_Switch8_idx_0 = fam_force_allocation_module_P->Constant4_Value_m *
      fam_force_allocation_module_P->abp_PM2_discharge_coeff[vcol] *
      fam_force_allocation_module_P->abp_PM2_discharge_coeff[vcol] *
      rtb_Switch_n;

    // Product: '<S24>/Product6'
    rtb_Switch8_idx_2 = rtb_positive_thrust_per_nozzle[(int32_T)(vcol + 6)] /
      rtb_Switch8_idx_0;

    // Product: '<S24>/Product5' incorporates:
    //   Constant: '<S24>/Constant3'
    //   Constant: '<S24>/Constant6'
    //   Constant: '<S24>/Constant7'
    //   Constant: '<S24>/Constant9'
    //   Product: '<S24>/Product3'
    //   Product: '<S24>/Product4'
    //   Sum: '<S24>/Subtract2'

    rtb_Switch8_idx_1 = (fam_force_allocation_module_P->abp_nozzle_intake_height
                         - rtb_Switch8_idx_2 /
                         fam_force_allocation_module_P->abp_nozzle_flap_count /
                         fam_force_allocation_module_P->
                         abp_PM2_nozzle_widths[vcol]) /
      fam_force_allocation_module_P->abp_nozzle_flap_length;

    // Saturate: '<S24>/Saturation'
    if (rtb_Switch8_idx_1 > rtb_Switch) {
      rtb_Switch8_idx_1 = rtb_Switch;
    } else {
      if (rtb_Switch8_idx_1 < tol) {
        rtb_Switch8_idx_1 = tol;
      }
    }

    // Trigonometry: '<S24>/Trigonometric Function'
    if (rtb_Switch8_idx_1 > 1.0F) {
      rtb_Switch8_idx_1 = 1.0F;
    } else {
      if (rtb_Switch8_idx_1 < -1.0F) {
        rtb_Switch8_idx_1 = -1.0F;
      }
    }

    // Sum: '<S24>/Subtract3' incorporates:
    //   Constant: '<S24>/Constant8'
    //   Trigonometry: '<S24>/Trigonometric Function'

    rtb_Switch8_idx_1 = (real32_T)acos((real_T)rtb_Switch8_idx_1) -
      fam_force_allocation_module_P->abp_nozzle_min_open_angle;

    // Sum: '<S24>/Subtract1' incorporates:
    //   Constant: '<S24>/Constant1'
    //   Constant: '<S24>/Constant2'
    //   Product: '<S24>/Product1'

    rtb_thrust_per_nozzle[(int32_T)(vcol + 6)] = rtb_Switch8_idx_1 *
      fam_force_allocation_module_P->fam_nozzle_angle2pwm +
      fam_force_allocation_module_P->abp_servo_min_PWM;

    // Product: '<S24>/Product7'
    rtb_thrust_norm_by_Cd_h[vcol] = rtb_Switch8_idx_0;

    // Product: '<S24>/Product6'
    rtb_cmd_area_per_nozzle_f[vcol] = rtb_Switch8_idx_2;

    // Sum: '<S24>/Subtract3'
    rtb_calibrated_nozzle_theta_h[vcol] = rtb_Switch8_idx_1;
  }

  for (vcol = 0; vcol < 6; vcol++) {
    // SignalConversion: '<S1>/ConcatBufferAtVector Concatenate1In1'
    rtb_positive_thrust_per_nozzle[vcol] = rtb_calibrated_nozzle_theta[vcol];

    // SignalConversion: '<S1>/ConcatBufferAtVector Concatenate1In2'
    rtb_positive_thrust_per_nozzle[(int32_T)(vcol + 6)] =
      rtb_calibrated_nozzle_theta_h[vcol];

    // SignalConversion: '<S1>/ConcatBufferAtVector Concatenate2In1'
    rtb_Product9[vcol] = rtb_cmd_area_per_nozzle[vcol];

    // SignalConversion: '<S1>/ConcatBufferAtVector Concatenate2In2'
    rtb_Product9[(int32_T)(vcol + 6)] = rtb_cmd_area_per_nozzle_f[vcol];

    // SignalConversion: '<S1>/ConcatBufferAtVector Concatenate3In1'
    rtb_VectorConcatenate3[vcol] = rtb_thrust_norm_by_Cd[vcol];

    // SignalConversion: '<S1>/ConcatBufferAtVector Concatenate3In2'
    rtb_VectorConcatenate3[(int32_T)(vcol + 6)] = rtb_thrust_norm_by_Cd_h[vcol];
  }

  // Outport: '<Root>/act_msg' incorporates:
  //   BusCreator: '<S1>/Bus Creator2'
  //   DataTypeConversion: '<S19>/Data Type Conversion'
  //   DataTypeConversion: '<S23>/Data Type Conversion'
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
  for (vcol = 0; vcol < 12; vcol++) {
    // Outport: '<Root>/act_msg' incorporates:
    //   BusCreator: '<S1>/Bus Creator2'

    fam_force_allocation_module_Y_act_msg_c->act_servo_pwm_cmd[vcol] =
      rtb_thrust_per_nozzle[vcol];
    fam_force_allocation_module_Y_act_msg_c->act_nozzle_theta[vcol] =
      rtb_positive_thrust_per_nozzle[vcol];

    // Product: '<S2>/Product9'
    rtb_Product9[vcol] *= rtb_VectorConcatenate3[vcol];
  }

  for (vcol = 0; vcol < 3; vcol++) {
    // Outport: '<Root>/act_msg' incorporates:
    //   Product: '<S2>/Product3'
    //   Product: '<S2>/Product4'

    fam_force_allocation_module_Y_act_msg_c->act_predicted_force_B[vcol] = 0.0F;
    fam_force_allocation_module_Y_act_msg_c->act_predicted_torque_B[vcol] = 0.0F;
    for (r = 0; r < 12; r++) {
      fam_force_allocation_module_Y_act_msg_c->act_predicted_force_B[vcol] +=
        fam_force_allocation_module_B->OutportBufferForthrust2force_B[(int32_T)
        ((int32_T)(3 * r) + vcol)] * rtb_Product9[r];
      fam_force_allocation_module_Y_act_msg_c->act_predicted_torque_B[vcol] +=
        fam_force_allocation_module_B->OutportBufferForthrust2torque_B[(int32_T)
        ((int32_T)(3 * r) + vcol)] * rtb_Product9[r];
    }

    // Update for UnitDelay: '<S11>/Delay Input1' incorporates:
    //   Inport: '<Root>/cmc_msg'
    //   SignalConversion: '<S1>/Signal Conversion'

    fam_force_allocation_module_DW->DelayInput1_DSTATE[vcol] =
      fam_force_allocation_module_U_cmc_msg_h->center_of_mass[vcol];
  }
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

    // InitializeConditions for UnitDelay: '<S11>/Delay Input1'
    fam_force_allocation_module_DW->DelayInput1_DSTATE[0] =
      fam_force_allocation_module_P->DetectChange_vinit;
    fam_force_allocation_module_DW->DelayInput1_DSTATE[1] =
      fam_force_allocation_module_P->DetectChange_vinit;
    fam_force_allocation_module_DW->DelayInput1_DSTATE[2] =
      fam_force_allocation_module_P->DetectChange_vinit;

    // SystemInitialize for Enabled SubSystem: '<S3>/latch_nozzle_thrust_matricies' 
    for (i = 0; i < 36; i++) {
      // SystemInitialize for Outport: '<S12>/thrust2force_B'
      fam_force_allocation_module_B->OutportBufferForthrust2force_B[i] =
        fam_force_allocation_module_P->thrust2force_B_Y0;

      // SystemInitialize for Outport: '<S12>/thrust2torque_B'
      fam_force_allocation_module_B->OutportBufferForthrust2torque_B[i] =
        fam_force_allocation_module_P->thrust2torque_B_Y0;
    }

    // SystemInitialize for Outport: '<S12>/forcetorque2thrust_B'
    for (i = 0; i < 72; i++) {
      fam_force_allocation_module_B->OutportBufferForforcetorque2thr[i] =
        fam_force_allocation_module_P->forcetorque2thrust_B_Y0;
    }

    // End of SystemInitialize for Outport: '<S12>/forcetorque2thrust_B'
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

    // non-finite (run-time) assignments
    fam_force_allocation_module_P->DetectChange_vinit = rtInfF;

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
