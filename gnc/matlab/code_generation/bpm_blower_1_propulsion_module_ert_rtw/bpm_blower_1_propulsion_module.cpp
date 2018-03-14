//
// File: bpm_blower_1_propulsion_module.cpp
//
// Code generated for Simulink model 'bpm_blower_1_propulsion_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed Jan 31 12:34:43 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "bpm_blower_1_propulsion_module.h"
#include "bpm_blower_1_propulsion_module_private.h"

// Model step function
void bpm_blower_1_propulsion_module_step(RT_MODEL_bpm_blower_1_propuls_T *const
  bpm_blower_1_propulsion_modu_M, real32_T
  bpm_blower_1_propulsion_modul_U_battery_voltage, real32_T
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
  *bpm_blower_1_propulsion_modul_Y_meas_motor_speed)
{
  P_bpm_blower_1_propulsion_mod_T *bpm_blower_1_propulsion_modul_P =
    ((P_bpm_blower_1_propulsion_mod_T *)
     bpm_blower_1_propulsion_modu_M->defaultParam);
  B_bpm_blower_1_propulsion_mod_T *bpm_blower_1_propulsion_modul_B =
    ((B_bpm_blower_1_propulsion_mod_T *) bpm_blower_1_propulsion_modu_M->blockIO);
  DW_bpm_blower_1_propulsion_mo_T *bpm_blower_1_propulsion_modu_DW =
    ((DW_bpm_blower_1_propulsion_mo_T *) bpm_blower_1_propulsion_modu_M->dwork);

  // local scratch DWork variables
  int32_T ForEach_itr;
  real32_T nozzle_moment_arm[18];
  real_T b[18];
  int32_T ibmat;
  int32_T itilerow;
  real32_T c[18];
  real32_T rtb_current[6];
  real32_T rtb_IntegralGain_p;
  real32_T rtb_Sum_j;
  real32_T rtb_SignPreIntegrator_j;
  real32_T rtb_FilterCoefficient_h;
  boolean_T rtb_NotEqual_m;
  real32_T rtb_Gain1_ii[6];
  real32_T rtb_IntegralGain[6];
  real32_T rtb_SignDeltaU[6];
  real32_T rtb_FilterCoefficient[6];
  boolean_T rtb_NotEqual[6];
  real32_T rtb_Add1[3];
  real32_T rtb_Assignment[9];
  real32_T rtb_Gain_n;
  real32_T rtb_ImpAsg_InsertedFor_rotated_[18];
  real_T rtb_Sum2[3];
  real32_T Backlash1[6];
  real32_T tmp;
  int32_T i;
  real32_T tmp_0[9];
  real32_T tmp_1[3];
  real32_T tmp_2[9];
  real32_T rtb_Switch_d_0[9];
  real32_T rtb_Assignment_0[9];
  real32_T rtb_SignDeltaU_b;
  real32_T rtb_IntegralGain_a;
  real32_T rtb_SignDeltaU_1;
  int8_T rtb_SignPreIntegrator_g;
  const real32_T *rtb_Switch_d_1;

  // Outputs for Atomic SubSystem: '<S5>/speed_controller'
  // Product: '<S27>/Divide' incorporates:
  //   Constant: '<S27>/Constant'
  //   Inport: '<Root>/impeller_cmd'

  rtb_IntegralGain_p = (real32_T)bpm_blower_1_propulsion_modul_U_impeller_cmd *
    bpm_blower_1_propulsion_modul_P->bpm_blower_1_propulsion_module_;

  // RateLimiter: '<S27>/Rate Limiter'
  rtb_FilterCoefficient_h = rtb_IntegralGain_p -
    bpm_blower_1_propulsion_modu_DW->PrevY_l;
  if (rtb_FilterCoefficient_h >
      bpm_blower_1_propulsion_modul_P->RateLimiter_RisingLim) {
    rtb_IntegralGain_p = bpm_blower_1_propulsion_modu_DW->PrevY_l +
      bpm_blower_1_propulsion_modul_P->RateLimiter_RisingLim;
  } else {
    if (rtb_FilterCoefficient_h <
        bpm_blower_1_propulsion_modul_P->RateLimiter_FallingLim) {
      rtb_IntegralGain_p = bpm_blower_1_propulsion_modu_DW->PrevY_l +
        bpm_blower_1_propulsion_modul_P->RateLimiter_FallingLim;
    }
  }

  bpm_blower_1_propulsion_modu_DW->PrevY_l = rtb_IntegralGain_p;

  // End of RateLimiter: '<S27>/Rate Limiter'

  // Sum: '<S27>/Sum' incorporates:
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
  //   DiscreteTransferFcn: '<S27>/Discrete Transfer Fcn1'

  rtb_IntegralGain_p -=
    bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator_DSTATE *
    bpm_blower_1_propulsion_modul_P->bpm_imp_speed_filt_num /
    bpm_blower_1_propulsion_modul_P->bpm_imp_speed_filt_den;

  // Gain: '<S28>/Filter Coefficient' incorporates:
  //   DiscreteIntegrator: '<S28>/Filter'
  //   Gain: '<S28>/Derivative Gain'
  //   Sum: '<S28>/SumD'

  rtb_FilterCoefficient_h = (bpm_blower_1_propulsion_modul_P->bpm_imp_ctl_kd *
    rtb_IntegralGain_p - bpm_blower_1_propulsion_modu_DW->Filter_DSTATE_o) *
    bpm_blower_1_propulsion_modul_P->bpm_imp_ctl_filt_n;

  // Sum: '<S28>/Sum' incorporates:
  //   DiscreteIntegrator: '<S28>/Integrator'
  //   Gain: '<S28>/Proportional Gain'

  rtb_Sum_j = (bpm_blower_1_propulsion_modul_P->bpm_imp_ctl_kp *
               rtb_IntegralGain_p +
               bpm_blower_1_propulsion_modu_DW->Integrator_DSTATE_k) +
    rtb_FilterCoefficient_h;

  // DeadZone: '<S30>/DeadZone'
  if (rtb_Sum_j > bpm_blower_1_propulsion_modul_P->bpm_imp_max_voltage) {
    rtb_SignPreIntegrator_j = rtb_Sum_j -
      bpm_blower_1_propulsion_modul_P->bpm_imp_max_voltage;
  } else if (rtb_Sum_j >=
             bpm_blower_1_propulsion_modul_P->DiscretePIDController_LowerSatu) {
    rtb_SignPreIntegrator_j = 0.0F;
  } else {
    rtb_SignPreIntegrator_j = rtb_Sum_j -
      bpm_blower_1_propulsion_modul_P->DiscretePIDController_LowerSatu;
  }

  // End of DeadZone: '<S30>/DeadZone'

  // RelationalOperator: '<S30>/NotEqual' incorporates:
  //   Gain: '<S30>/ZeroGain'

  rtb_NotEqual_m = (bpm_blower_1_propulsion_modul_P->ZeroGain_Gain * rtb_Sum_j
                    != rtb_SignPreIntegrator_j);

  // Signum: '<S30>/SignDeltaU'
  if (rtb_SignPreIntegrator_j < 0.0F) {
    rtb_SignPreIntegrator_j = -1.0F;
  } else if (rtb_SignPreIntegrator_j > 0.0F) {
    rtb_SignPreIntegrator_j = 1.0F;
  } else {
    if (rtb_SignPreIntegrator_j == 0.0F) {
      rtb_SignPreIntegrator_j = 0.0F;
    }
  }

  // End of Signum: '<S30>/SignDeltaU'

  // Gain: '<S28>/Integral Gain'
  rtb_IntegralGain_p *= bpm_blower_1_propulsion_modul_P->bpm_imp_ctl_ki;

  // Saturate: '<S28>/Saturate'
  if (rtb_Sum_j > bpm_blower_1_propulsion_modul_P->bpm_imp_max_voltage) {
    rtb_Sum_j = bpm_blower_1_propulsion_modul_P->bpm_imp_max_voltage;
  } else {
    if (rtb_Sum_j <
        bpm_blower_1_propulsion_modul_P->DiscretePIDController_LowerSatu) {
      rtb_Sum_j =
        bpm_blower_1_propulsion_modul_P->DiscretePIDController_LowerSatu;
    }
  }

  // End of Saturate: '<S28>/Saturate'

  // Switch: '<S29>/Switch2' incorporates:
  //   Inport: '<Root>/battery_voltage'
  //   RelationalOperator: '<S29>/LowerRelop1'

  if (rtb_Sum_j > bpm_blower_1_propulsion_modul_U_battery_voltage) {
    rtb_Sum_j = bpm_blower_1_propulsion_modul_U_battery_voltage;
  } else {
    // Gain: '<S27>/Gain'
    rtb_Gain_n = bpm_blower_1_propulsion_modul_P->Gain_Gain *
      bpm_blower_1_propulsion_modul_U_battery_voltage;

    // Switch: '<S29>/Switch' incorporates:
    //   RelationalOperator: '<S29>/UpperRelop'

    if (rtb_Sum_j < rtb_Gain_n) {
      rtb_Sum_j = rtb_Gain_n;
    }

    // End of Switch: '<S29>/Switch'
  }

  // End of Switch: '<S29>/Switch2'

  // Signum: '<S30>/SignPreIntegrator'
  if (rtb_IntegralGain_p < 0.0F) {
    rtb_IntegralGain_a = -1.0F;
  } else if (rtb_IntegralGain_p > 0.0F) {
    rtb_IntegralGain_a = 1.0F;
  } else if (rtb_IntegralGain_p == 0.0F) {
    rtb_IntegralGain_a = 0.0F;
  } else {
    rtb_IntegralGain_a = rtb_IntegralGain_p;
  }

  // End of Signum: '<S30>/SignPreIntegrator'

  // DataTypeConversion: '<S30>/DataTypeConv2'
  rtb_IntegralGain_a = (real32_T)floor((real_T)rtb_IntegralGain_a);
  if (rtIsNaNF(rtb_IntegralGain_a) || rtIsInfF(rtb_IntegralGain_a)) {
    rtb_IntegralGain_a = 0.0F;
  } else {
    rtb_IntegralGain_a = (real32_T)fmod((real_T)rtb_IntegralGain_a, (real_T)
      256.0F);
  }

  // DataTypeConversion: '<S30>/DataTypeConv1'
  if (rtb_SignPreIntegrator_j < 128.0F) {
    rtb_SignPreIntegrator_g = (int8_T)rtb_SignPreIntegrator_j;
  } else {
    rtb_SignPreIntegrator_g = MAX_int8_T;
  }

  // End of DataTypeConversion: '<S30>/DataTypeConv1'

  // Switch: '<S28>/Switch' incorporates:
  //   Constant: '<S28>/Constant'
  //   DataTypeConversion: '<S30>/DataTypeConv2'
  //   Logic: '<S30>/AND'
  //   RelationalOperator: '<S30>/Equal'

  if (rtb_NotEqual_m && ((rtb_IntegralGain_a < 0.0F ? (int32_T)(int8_T)(int32_T)
                          -(int32_T)(int8_T)(uint8_T)-rtb_IntegralGain_a :
                          (int32_T)(int8_T)(uint8_T)rtb_IntegralGain_a) ==
                         (int32_T)rtb_SignPreIntegrator_g)) {
    rtb_IntegralGain_p = bpm_blower_1_propulsion_modul_P->Constant_Value;
  }

  // End of Switch: '<S28>/Switch'

  // Update for DiscreteIntegrator: '<S28>/Integrator'
  bpm_blower_1_propulsion_modu_DW->Integrator_DSTATE_k +=
    bpm_blower_1_propulsion_modul_P->Integrator_gainval * rtb_IntegralGain_p;

  // Update for DiscreteIntegrator: '<S28>/Filter'
  bpm_blower_1_propulsion_modu_DW->Filter_DSTATE_o +=
    bpm_blower_1_propulsion_modul_P->Filter_gainval * rtb_FilterCoefficient_h;

  // End of Outputs for SubSystem: '<S5>/speed_controller'

  // Outputs for Atomic SubSystem: '<S5>/dc_motor_model'
  // Gain: '<S26>/Gain5' incorporates:
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
  //   Gain: '<S26>/Gain4'
  //   Sum: '<S26>/Add'

  rtb_FilterCoefficient_h = (rtb_Sum_j - 1.0F /
    bpm_blower_1_propulsion_modul_P->bpm_imp_motor_speed_k *
    bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator_DSTATE) * (1.0F /
    bpm_blower_1_propulsion_modul_P->bpm_imp_motor_r);

  // Sum: '<S26>/Add1' incorporates:
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
  //   Gain: '<S26>/Gain1'
  //   Gain: '<S26>/Gain6'

  rtb_IntegralGain_p = bpm_blower_1_propulsion_modul_P->bpm_imp_motor_torque_k *
    rtb_FilterCoefficient_h -
    bpm_blower_1_propulsion_modul_P->bpm_imp_motor_friction_coeff *
    bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator_DSTATE;

  // End of Outputs for SubSystem: '<S5>/dc_motor_model'

  // Outport: '<Root>/impeller_current'
  *bpm_blower_1_propulsion_modul_Y_impeller_current = rtb_FilterCoefficient_h;

  // Outputs for Atomic SubSystem: '<S1>/servo_model'
  // Backlash: '<S31>/Backlash1'
  rtb_FilterCoefficient_h =
    bpm_blower_1_propulsion_modul_P->bpm_servo_motor_backlash_deadband / 2.0F;

  // Gain: '<S31>/Gain5'
  rtb_SignPreIntegrator_j = 1.0F /
    bpm_blower_1_propulsion_modul_P->bpm_servo_motor_r;
  for (i = 0; i < 6; i++) {
    // Backlash: '<S31>/Backlash1' incorporates:
    //   DiscreteIntegrator: '<S31>/Discrete-Time Integrator4'

    if (bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator4_DSTATE[i] <
        bpm_blower_1_propulsion_modu_DW->PrevY[i] - rtb_FilterCoefficient_h) {
      rtb_Sum_j =
        bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator4_DSTATE[i] +
        rtb_FilterCoefficient_h;
    } else if (bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator4_DSTATE[i]
               <= bpm_blower_1_propulsion_modu_DW->PrevY[i] +
               rtb_FilterCoefficient_h) {
      rtb_Sum_j = bpm_blower_1_propulsion_modu_DW->PrevY[i];
    } else {
      rtb_Sum_j =
        bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator4_DSTATE[i] -
        rtb_FilterCoefficient_h;
    }

    // Gain: '<S31>/Gain1'
    rtb_Gain_n = bpm_blower_1_propulsion_modul_P->bpm_servo_motor_gear_ratio *
      rtb_Sum_j;

    // Sum: '<S32>/Sum1' incorporates:
    //   Constant: '<S32>/Constant'
    //   Gain: '<S32>/Gain'
    //   Inport: '<Root>/servo_cmd'
    //   Sum: '<S32>/Sum'

    rtb_IntegralGain_a = (bpm_blower_1_propulsion_modul_U_servo_cmd[i] +
                          bpm_blower_1_propulsion_modul_P->bpm_servo_pwm2angle_bias)
      * bpm_blower_1_propulsion_modul_P->bpm_servo_pwm2angle - rtb_Gain_n;

    // Gain: '<S33>/Filter Coefficient' incorporates:
    //   DiscreteIntegrator: '<S33>/Filter'
    //   Gain: '<S33>/Derivative Gain'
    //   Sum: '<S33>/SumD'

    tmp = (bpm_blower_1_propulsion_modul_P->bpm_servo_ctl_kd *
           rtb_IntegralGain_a - bpm_blower_1_propulsion_modu_DW->Filter_DSTATE[i])
      * bpm_blower_1_propulsion_modul_P->bpm_servo_ctl_filt_n;

    // Sum: '<S33>/Sum' incorporates:
    //   DiscreteIntegrator: '<S33>/Integrator'
    //   Gain: '<S33>/Proportional Gain'

    rtb_SignDeltaU_b = (bpm_blower_1_propulsion_modul_P->bpm_servo_ctl_kp *
                        rtb_IntegralGain_a +
                        bpm_blower_1_propulsion_modu_DW->Integrator_DSTATE[i]) +
      tmp;

    // Saturate: '<S33>/Saturate'
    if (rtb_SignDeltaU_b >
        bpm_blower_1_propulsion_modul_P->bpm_servo_max_voltage) {
      rtb_SignDeltaU_1 = bpm_blower_1_propulsion_modul_P->bpm_servo_max_voltage;
    } else if (rtb_SignDeltaU_b <
               bpm_blower_1_propulsion_modul_P->DiscretePIDController_LowerSa_b)
    {
      rtb_SignDeltaU_1 =
        bpm_blower_1_propulsion_modul_P->DiscretePIDController_LowerSa_b;
    } else {
      rtb_SignDeltaU_1 = rtb_SignDeltaU_b;
    }

    // End of Saturate: '<S33>/Saturate'

    // Gain: '<S31>/Gain5' incorporates:
    //   DiscreteIntegrator: '<S31>/Discrete-Time Integrator3'
    //   Gain: '<S31>/Gain7'
    //   Sum: '<S31>/Add3'

    rtb_current[i] = (rtb_SignDeltaU_1 -
                      bpm_blower_1_propulsion_modul_P->bpm_servo_motor_k *
                      bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator3_DSTATE
                      [i]) * rtb_SignPreIntegrator_j;

    // Gain: '<S34>/ZeroGain'
    rtb_SignDeltaU_1 = bpm_blower_1_propulsion_modul_P->ZeroGain_Gain_e *
      rtb_SignDeltaU_b;

    // DeadZone: '<S34>/DeadZone'
    if (rtb_SignDeltaU_b >
        bpm_blower_1_propulsion_modul_P->bpm_servo_max_voltage) {
      rtb_SignDeltaU_b -= bpm_blower_1_propulsion_modul_P->bpm_servo_max_voltage;
    } else if (rtb_SignDeltaU_b >=
               bpm_blower_1_propulsion_modul_P->DiscretePIDController_LowerSa_b)
    {
      rtb_SignDeltaU_b = 0.0F;
    } else {
      rtb_SignDeltaU_b -=
        bpm_blower_1_propulsion_modul_P->DiscretePIDController_LowerSa_b;
    }

    // End of DeadZone: '<S34>/DeadZone'

    // RelationalOperator: '<S34>/NotEqual'
    rtb_NotEqual[i] = (rtb_SignDeltaU_1 != rtb_SignDeltaU_b);

    // Signum: '<S34>/SignDeltaU'
    if (rtb_SignDeltaU_b < 0.0F) {
      rtb_SignDeltaU_b = -1.0F;
    } else if (rtb_SignDeltaU_b > 0.0F) {
      rtb_SignDeltaU_b = 1.0F;
    } else {
      if (rtb_SignDeltaU_b == 0.0F) {
        rtb_SignDeltaU_b = 0.0F;
      }
    }

    // End of Signum: '<S34>/SignDeltaU'

    // Gain: '<S33>/Integral Gain'
    rtb_IntegralGain_a *= bpm_blower_1_propulsion_modul_P->bpm_servo_ctl_ki;

    // Update for DiscreteIntegrator: '<S31>/Discrete-Time Integrator4' incorporates:
    //   DiscreteIntegrator: '<S31>/Discrete-Time Integrator3'

    bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator4_DSTATE[i] +=
      bpm_blower_1_propulsion_modul_P->DiscreteTimeIntegrator4_gainval *
      bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator3_DSTATE[i];

    // Backlash: '<S31>/Backlash1'
    Backlash1[i] = rtb_Sum_j;

    // Gain: '<S31>/Gain1'
    rtb_Gain1_ii[i] = rtb_Gain_n;

    // Sum: '<S32>/Sum1'
    rtb_IntegralGain[i] = rtb_IntegralGain_a;

    // Gain: '<S33>/Filter Coefficient'
    rtb_FilterCoefficient[i] = tmp;

    // Sum: '<S33>/Sum'
    rtb_SignDeltaU[i] = rtb_SignDeltaU_b;
  }

  // Gain: '<S31>/Gain6'
  rtb_SignPreIntegrator_j = 1.0F /
    bpm_blower_1_propulsion_modul_P->bpm_servo_motor_gear_box_inertia;

  // Update for DiscreteIntegrator: '<S31>/Discrete-Time Integrator4'
  rtb_FilterCoefficient_h = bpm_blower_1_propulsion_modul_P->bpm_servo_max_theta
    / bpm_blower_1_propulsion_modul_P->bpm_servo_motor_gear_ratio;
  rtb_Sum_j = (real32_T)bpm_blower_1_propulsion_modul_P->bpm_servo_min_theta /
    bpm_blower_1_propulsion_modul_P->bpm_servo_motor_gear_ratio;
  rtb_Gain_n = bpm_blower_1_propulsion_modul_P->bpm_servo_max_theta /
    bpm_blower_1_propulsion_modul_P->bpm_servo_motor_gear_ratio;
  tmp = (real32_T)bpm_blower_1_propulsion_modul_P->bpm_servo_min_theta /
    bpm_blower_1_propulsion_modul_P->bpm_servo_motor_gear_ratio;
  for (i = 0; i < 6; i++) {
    if (bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator4_DSTATE[i] >=
        rtb_FilterCoefficient_h) {
      bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator4_DSTATE[i] =
        rtb_Gain_n;
    } else {
      if (bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator4_DSTATE[i] <=
          rtb_Sum_j) {
        bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator4_DSTATE[i] = tmp;
      }
    }

    // Update for Backlash: '<S31>/Backlash1'
    bpm_blower_1_propulsion_modu_DW->PrevY[i] = Backlash1[i];

    // Update for DiscreteIntegrator: '<S33>/Integrator' incorporates:
    //   Constant: '<S33>/Constant'
    //   DataTypeConversion: '<S34>/DataTypeConv1'
    //   DataTypeConversion: '<S34>/DataTypeConv2'
    //   Logic: '<S34>/AND'
    //   RelationalOperator: '<S34>/Equal'
    //   Signum: '<S34>/SignPreIntegrator'
    //   Switch: '<S33>/Switch'

    if (rtb_IntegralGain[i] < 0.0F) {
      rtb_IntegralGain_a = -1.0F;
    } else if (rtb_IntegralGain[i] > 0.0F) {
      rtb_IntegralGain_a = 1.0F;
    } else if (rtb_IntegralGain[i] == 0.0F) {
      rtb_IntegralGain_a = 0.0F;
    } else {
      rtb_IntegralGain_a = rtb_IntegralGain[i];
    }

    rtb_IntegralGain_a = (real32_T)floor((real_T)rtb_IntegralGain_a);
    if (rtIsNaNF(rtb_IntegralGain_a) || rtIsInfF(rtb_IntegralGain_a)) {
      rtb_IntegralGain_a = 0.0F;
    } else {
      rtb_IntegralGain_a = (real32_T)fmod((real_T)rtb_IntegralGain_a, (real_T)
        256.0F);
    }

    if (rtb_SignDeltaU[i] < 128.0F) {
      rtb_SignPreIntegrator_g = (int8_T)rtb_SignDeltaU[i];
    } else {
      rtb_SignPreIntegrator_g = MAX_int8_T;
    }

    if (rtb_NotEqual[i] && ((rtb_IntegralGain_a < 0.0F ? (int32_T)(int8_T)
          (int32_T)-(int32_T)(int8_T)(uint8_T)-rtb_IntegralGain_a : (int32_T)
          (int8_T)(uint8_T)rtb_IntegralGain_a) == (int32_T)
                            rtb_SignPreIntegrator_g)) {
      rtb_IntegralGain_a = bpm_blower_1_propulsion_modul_P->Constant_Value_n;
    } else {
      rtb_IntegralGain_a = rtb_IntegralGain[i];
    }

    bpm_blower_1_propulsion_modu_DW->Integrator_DSTATE[i] +=
      bpm_blower_1_propulsion_modul_P->Integrator_gainval_f * rtb_IntegralGain_a;

    // End of Update for DiscreteIntegrator: '<S33>/Integrator'

    // Update for DiscreteIntegrator: '<S33>/Filter'
    bpm_blower_1_propulsion_modu_DW->Filter_DSTATE[i] +=
      bpm_blower_1_propulsion_modul_P->Filter_gainval_n *
      rtb_FilterCoefficient[i];

    // Update for DiscreteIntegrator: '<S31>/Discrete-Time Integrator3' incorporates:
    //   DiscreteIntegrator: '<S31>/Discrete-Time Integrator3'
    //   Gain: '<S31>/Gain11'
    //   Gain: '<S31>/Gain6'
    //   Gain: '<S31>/Gain9'
    //   Sum: '<S31>/Add1'

    bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator3_DSTATE[i] +=
      (bpm_blower_1_propulsion_modul_P->bpm_servo_motor_k * rtb_current[i] -
       bpm_blower_1_propulsion_modul_P->bpm_servo_motor_friction_coeff *
       bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator3_DSTATE[i]) *
      rtb_SignPreIntegrator_j *
      bpm_blower_1_propulsion_modul_P->DiscreteTimeIntegrator3_gainval;
  }

  // End of Outputs for SubSystem: '<S1>/servo_model'

  // Sum: '<S3>/Add1' incorporates:
  //   Constant: '<S3>/Constant1'
  //   Constant: '<S3>/Constant3'
  //   Gain: '<S3>/Gain'

  rtb_SignPreIntegrator_j =
    bpm_blower_1_propulsion_modul_P->tun_bpm_noise_on_flag *
    bpm_blower_1_propulsion_modul_P->bmp_PM1_impeller_orientation_error[0] +
    bpm_blower_1_propulsion_modul_P->abp_pm1_impeller_orientation[0];

  // DotProduct: '<S25>/Dot Product'
  rtb_IntegralGain_a = rtb_SignPreIntegrator_j * rtb_SignPreIntegrator_j;

  // Sum: '<S3>/Add1' incorporates:
  //   Constant: '<S3>/Constant1'
  //   Constant: '<S3>/Constant3'
  //   Gain: '<S3>/Gain'

  rtb_Add1[0] = rtb_SignPreIntegrator_j;
  rtb_SignPreIntegrator_j =
    bpm_blower_1_propulsion_modul_P->tun_bpm_noise_on_flag *
    bpm_blower_1_propulsion_modul_P->bmp_PM1_impeller_orientation_error[1] +
    bpm_blower_1_propulsion_modul_P->abp_pm1_impeller_orientation[1];

  // DotProduct: '<S25>/Dot Product'
  rtb_IntegralGain_a += rtb_SignPreIntegrator_j * rtb_SignPreIntegrator_j;

  // Sum: '<S3>/Add1' incorporates:
  //   Constant: '<S3>/Constant1'
  //   Constant: '<S3>/Constant3'
  //   Gain: '<S3>/Gain'

  rtb_Add1[1] = rtb_SignPreIntegrator_j;
  rtb_SignPreIntegrator_j =
    bpm_blower_1_propulsion_modul_P->tun_bpm_noise_on_flag *
    bpm_blower_1_propulsion_modul_P->bmp_PM1_impeller_orientation_error[2] +
    bpm_blower_1_propulsion_modul_P->abp_pm1_impeller_orientation[2];

  // DotProduct: '<S25>/Dot Product'
  rtb_IntegralGain_a += rtb_SignPreIntegrator_j * rtb_SignPreIntegrator_j;

  // Sum: '<S3>/Add1'
  rtb_Add1[2] = rtb_SignPreIntegrator_j;

  // Sqrt: '<S25>/Sqrt' incorporates:
  //   DotProduct: '<S25>/Dot Product'

  rtb_FilterCoefficient_h = (real32_T)sqrt((real_T)rtb_IntegralGain_a);

  // If: '<S12>/If' incorporates:
  //   DataTypeConversion: '<S12>/Data Type Conversion'
  //   Product: '<S24>/Divide'

  if ((real_T)rtb_FilterCoefficient_h > 1.0E-7) {
    // Outputs for IfAction SubSystem: '<S12>/Normalize' incorporates:
    //   ActionPort: '<S24>/Action Port'

    rtb_Add1[0] /= rtb_FilterCoefficient_h;
    rtb_Add1[1] /= rtb_FilterCoefficient_h;
    rtb_Add1[2] = rtb_SignPreIntegrator_j / rtb_FilterCoefficient_h;

    // End of Outputs for SubSystem: '<S12>/Normalize'
  }

  // End of If: '<S12>/If'

  // Gain: '<S6>/Gain1'
  rtb_Sum_j = bpm_blower_1_propulsion_modul_P->Gain1_Gain * rtb_IntegralGain_p;

  // Product: '<S3>/Product1' incorporates:
  //   Constant: '<S3>/Constant2'
  //   Constant: '<S3>/Constant4'
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
  //   Gain: '<S3>/Gain1'
  //   Sum: '<S3>/Add3'

  rtb_FilterCoefficient_h =
    (bpm_blower_1_propulsion_modul_P->tun_bpm_noise_on_flag *
     bpm_blower_1_propulsion_modul_P->bpm_impeller_inertia_error +
     bpm_blower_1_propulsion_modul_P->bpm_impeller_inertia) *
    bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator_DSTATE;

  // Outputs for Enabled SubSystem: '<S3>/latch_nozzle_thrust_matricies' incorporates:
  //   EnablePort: '<S11>/Enable'

  // UnitDelay: '<S3>/Unit Delay' incorporates:
  //   Constant: '<S11>/Constant2'
  //   Constant: '<S11>/Constant5'
  //   Constant: '<S11>/Constant8'
  //   Switch: '<S11>/Switch'

  if (bpm_blower_1_propulsion_modu_DW->UnitDelay_DSTATE) {
    if (bpm_blower_1_propulsion_modul_P->tun_bpm_noise_on_flag >
        bpm_blower_1_propulsion_modul_P->Switch_Threshold) {
      rtb_Switch_d_1 =
        &bpm_blower_1_propulsion_modul_P->bpm_PM1_Q_nozzle2misaligned[0];
    } else {
      rtb_Switch_d_1 = &bpm_blower_1_propulsion_modul_P->Constant2_Value[0];
    }

    // Outputs for Iterator SubSystem: '<S11>/For Each Subsystem' incorporates:
    //   ForEach: '<S15>/For Each'

    for (ForEach_itr = 0; ForEach_itr < 6; ForEach_itr++) {
      // Sum: '<S18>/Sum' incorporates:
      //   Constant: '<S18>/Constant1'
      //   DataTypeConversion: '<S20>/Conversion'
      //   ForEachSliceSelector: '<S15>/ImpSel_InsertedFor_misalignment_quats_at_outport_0'
      //   Gain: '<S18>/Gain'
      //   Math: '<S18>/Math Function'

      rtb_SignPreIntegrator_j = rtb_Switch_d_1[(int32_T)(18 + ForEach_itr)] *
        rtb_Switch_d_1[(int32_T)(18 + ForEach_itr)] *
        bpm_blower_1_propulsion_modul_P->CoreSubsys.Gain_Gain - (real32_T)
        bpm_blower_1_propulsion_modul_P->CoreSubsys.Constant1_Value;

      // Assignment: '<S18>/Assignment' incorporates:
      //   Constant: '<S18>/Constant2'
      //   DataTypeConversion: '<S19>/Conversion'

      for (i = 0; i < 9; i++) {
        rtb_Assignment[i] = (real32_T)
          bpm_blower_1_propulsion_modul_P->CoreSubsys.Constant2_Value[i];
      }

      rtb_Assignment[0] = rtb_SignPreIntegrator_j;
      rtb_Assignment[4] = rtb_SignPreIntegrator_j;
      rtb_Assignment[8] = rtb_SignPreIntegrator_j;

      // End of Assignment: '<S18>/Assignment'

      // Gain: '<S18>/Gain1' incorporates:
      //   ForEachSliceSelector: '<S15>/ImpSel_InsertedFor_misalignment_quats_at_outport_0'

      rtb_SignPreIntegrator_j = rtb_Switch_d_1[(int32_T)(18 + ForEach_itr)] *
        bpm_blower_1_propulsion_modul_P->CoreSubsys.Gain1_Gain;

      // Product: '<S18>/Product' incorporates:
      //   Constant: '<S21>/Constant3'
      //   DataTypeConversion: '<S22>/Conversion'
      //   ForEachSliceSelector: '<S15>/ImpSel_InsertedFor_misalignment_quats_at_outport_0'
      //   Gain: '<S21>/Gain'
      //   Gain: '<S21>/Gain1'
      //   Gain: '<S21>/Gain2'

      tmp_2[0] = (real32_T)
        bpm_blower_1_propulsion_modul_P->CoreSubsys.Constant3_Value;
      tmp_2[1] = rtb_Switch_d_1[(int32_T)(12 + ForEach_itr)];
      tmp_2[2] = rtb_Switch_d_1[(int32_T)(6 + ForEach_itr)] *
        bpm_blower_1_propulsion_modul_P->CoreSubsys.Gain_Gain_a;
      tmp_2[3] = rtb_Switch_d_1[(int32_T)(12 + ForEach_itr)] *
        bpm_blower_1_propulsion_modul_P->CoreSubsys.Gain1_Gain_m;
      tmp_2[4] = (real32_T)
        bpm_blower_1_propulsion_modul_P->CoreSubsys.Constant3_Value;
      tmp_2[5] = rtb_Switch_d_1[ForEach_itr];
      tmp_2[6] = rtb_Switch_d_1[(int32_T)(ForEach_itr + 6)];
      tmp_2[7] = bpm_blower_1_propulsion_modul_P->CoreSubsys.Gain2_Gain *
        rtb_Switch_d_1[ForEach_itr];
      tmp_2[8] = (real32_T)
        bpm_blower_1_propulsion_modul_P->CoreSubsys.Constant3_Value;

      // Product: '<S18>/Product1' incorporates:
      //   ForEachSliceSelector: '<S15>/ImpSel_InsertedFor_misalignment_quats_at_outport_0'
      //   Gain: '<S18>/Gain2'

      for (i = 0; i < 3; i++) {
        rtb_Switch_d_0[i] = rtb_Switch_d_1[(int32_T)((int32_T)(6 * i) +
          ForEach_itr)] * rtb_Switch_d_1[ForEach_itr];
        rtb_Switch_d_0[(int32_T)(i + 3)] = rtb_Switch_d_1[(int32_T)((int32_T)(6 *
          i) + ForEach_itr)] * rtb_Switch_d_1[(int32_T)(ForEach_itr + 6)];
        rtb_Switch_d_0[(int32_T)(i + 6)] = rtb_Switch_d_1[(int32_T)((int32_T)(6 *
          i) + ForEach_itr)] * rtb_Switch_d_1[(int32_T)(ForEach_itr + 12)];
      }

      // End of Product: '<S18>/Product1'

      // Sum: '<S18>/Sum1' incorporates:
      //   Gain: '<S18>/Gain2'
      //   Product: '<S17>/Product'
      //   Product: '<S18>/Product'

      for (i = 0; i < 3; i++) {
        rtb_Assignment_0[(int32_T)(3 * i)] = (rtb_Assignment[(int32_T)(3 * i)] -
          tmp_2[(int32_T)(3 * i)] * rtb_SignPreIntegrator_j) + rtb_Switch_d_0
          [(int32_T)(3 * i)] *
          bpm_blower_1_propulsion_modul_P->CoreSubsys.Gain2_Gain_i;
        rtb_Assignment_0[(int32_T)(1 + (int32_T)(3 * i))] = (rtb_Assignment
          [(int32_T)((int32_T)(3 * i) + 1)] - tmp_2[(int32_T)((int32_T)(3 * i) +
          1)] * rtb_SignPreIntegrator_j) + rtb_Switch_d_0[(int32_T)((int32_T)(3 *
          i) + 1)] * bpm_blower_1_propulsion_modul_P->CoreSubsys.Gain2_Gain_i;
        rtb_Assignment_0[(int32_T)(2 + (int32_T)(3 * i))] = (rtb_Assignment
          [(int32_T)((int32_T)(3 * i) + 2)] - tmp_2[(int32_T)((int32_T)(3 * i) +
          2)] * rtb_SignPreIntegrator_j) + rtb_Switch_d_0[(int32_T)((int32_T)(3 *
          i) + 2)] * bpm_blower_1_propulsion_modul_P->CoreSubsys.Gain2_Gain_i;
      }

      // End of Sum: '<S18>/Sum1'
      for (i = 0; i < 3; i++) {
        // ForEachSliceAssignment: '<S15>/ImpAsg_InsertedFor_rotated_vectors_at_inport_0' incorporates:
        //   Constant: '<S11>/Constant3'
        //   ForEachSliceSelector: '<S15>/ImpSel_InsertedFor_unit_vectors_at_outport_0'
        //   Product: '<S17>/Product'

        rtb_ImpAsg_InsertedFor_rotated_[(int32_T)(ForEach_itr + (int32_T)(6 * i))]
          = rtb_Assignment_0[(int32_T)(i + 6)] *
          bpm_blower_1_propulsion_modul_P->abp_PM1_nozzle_orientations[(int32_T)
          (ForEach_itr + 12)] + (rtb_Assignment_0[(int32_T)(i + 3)] *
          bpm_blower_1_propulsion_modul_P->abp_PM1_nozzle_orientations[(int32_T)
          (ForEach_itr + 6)] + rtb_Assignment_0[i] *
          bpm_blower_1_propulsion_modul_P->
          abp_PM1_nozzle_orientations[ForEach_itr]);
      }
    }

    // End of Outputs for SubSystem: '<S11>/For Each Subsystem'
    // MATLAB Function 'bpm_blower_1_propulsion_module/blower_body_dynamics/latch_nozzle_thrust_matricies/MATLAB Function': '<S16>:1' 
    // NOTE: if number of nozzles change, must change this hard coded '6' below
    // '<S16>:1:5'
    for (i = 0; i < 3; i++) {
      // Sum: '<S11>/Sum2' incorporates:
      //   Constant: '<S11>/Constant7'
      //   Gain: '<S11>/Gain2'
      //   Inport: '<Root>/veh_cm'

      rtb_Sum2[i] = (real_T)
        bpm_blower_1_propulsion_modul_P->tun_bpm_noise_on_flag *
        bpm_blower_1_propulsion_modul_P->abp_P_CG_B_B_error[i] + (real_T)
        bpm_blower_1_propulsion_modul_U_veh_cm[i];

      // MATLAB Function: '<S11>/MATLAB Function'
      ibmat = (int32_T)(i * 6);
      for (itilerow = 0; itilerow < 6; itilerow++) {
        b[(int32_T)(ibmat + itilerow)] = rtb_Sum2[i];
      }
    }

    // MATLAB Function: '<S11>/MATLAB Function' incorporates:
    //   Constant: '<S11>/Constant1'
    //   Constant: '<S11>/Constant2'
    //   Constant: '<S11>/Constant4'
    //   Constant: '<S11>/Constant5'
    //   Constant: '<S11>/Constant8'
    //   Gain: '<S11>/Gain'
    //   Sum: '<S11>/Sum'
    //   Switch: '<S11>/Switch'

    for (i = 0; i < 18; i++) {
      nozzle_moment_arm[i] =
        (bpm_blower_1_propulsion_modul_P->tun_bpm_noise_on_flag *
         bpm_blower_1_propulsion_modul_P->bpm_PM1_P_nozzle_B_B_error[i] +
         bpm_blower_1_propulsion_modul_P->abp_PM1_P_nozzle_B_B[i]) - (real32_T)
        b[i];
    }

    // [m] Distance from nozzles to CG
    // '<S16>:1:7'
    // [-] Converts a nozzle thrust into resulting body torques
    // '<S16>:1:8'
    // [-] Converts a nozzle thrust into resulting body forces
    for (i = 0; i < 6; i++) {
      c[i] = nozzle_moment_arm[(int32_T)(i + 6)] *
        rtb_ImpAsg_InsertedFor_rotated_[(int32_T)(i + 12)] - nozzle_moment_arm
        [(int32_T)(i + 12)] * rtb_ImpAsg_InsertedFor_rotated_[(int32_T)(i + 6)];
      c[(int32_T)(i + 6)] = nozzle_moment_arm[(int32_T)(i + 12)] *
        rtb_ImpAsg_InsertedFor_rotated_[i] - rtb_ImpAsg_InsertedFor_rotated_
        [(int32_T)(i + 12)] * nozzle_moment_arm[i];
      c[(int32_T)(i + 12)] = rtb_ImpAsg_InsertedFor_rotated_[(int32_T)(i + 6)] *
        nozzle_moment_arm[i] - nozzle_moment_arm[(int32_T)(i + 6)] *
        rtb_ImpAsg_InsertedFor_rotated_[i];

      // SignalConversion: '<S11>/OutportBufferForthrust2force_B'
      bpm_blower_1_propulsion_modul_B->OutportBufferForthrust2force_B[(int32_T)
        (3 * i)] = -rtb_ImpAsg_InsertedFor_rotated_[i];
      bpm_blower_1_propulsion_modul_B->OutportBufferForthrust2force_B[(int32_T)
        (1 + (int32_T)(3 * i))] = -rtb_ImpAsg_InsertedFor_rotated_[(int32_T)(i +
        6)];
      bpm_blower_1_propulsion_modul_B->OutportBufferForthrust2force_B[(int32_T)
        (2 + (int32_T)(3 * i))] = -rtb_ImpAsg_InsertedFor_rotated_[(int32_T)(i +
        12)];

      // SignalConversion: '<S11>/OutportBufferForthrust2torque_B'
      bpm_blower_1_propulsion_modul_B->OutportBufferForthrust2torque_B[(int32_T)
        (3 * i)] = -c[i];
      bpm_blower_1_propulsion_modul_B->OutportBufferForthrust2torque_B[(int32_T)
        (1 + (int32_T)(3 * i))] = -c[(int32_T)(i + 6)];
      bpm_blower_1_propulsion_modul_B->OutportBufferForthrust2torque_B[(int32_T)
        (2 + (int32_T)(3 * i))] = -c[(int32_T)(i + 12)];
    }
  }

  // End of UnitDelay: '<S3>/Unit Delay'
  // End of Outputs for SubSystem: '<S3>/latch_nozzle_thrust_matricies'

  // Outputs for Atomic SubSystem: '<S1>/calc_nozzle_area'
  // Gain: '<S4>/Gain12'
  rtb_SignPreIntegrator_j = 1.0F /
    bpm_blower_1_propulsion_modul_P->abp_nozzle_gear_ratio;
  for (i = 0; i < 6; i++) {
    rtb_Gain_n = rtb_SignPreIntegrator_j * rtb_Gain1_ii[i];

    // Product: '<S4>/Product2' incorporates:
    //   Constant: '<S4>/Constant1'
    //   Constant: '<S4>/Constant2'
    //   Constant: '<S4>/Constant4'
    //   Constant: '<S4>/Constant5'
    //   Constant: '<S4>/Constant6'
    //   Product: '<S4>/Product1'
    //   Sum: '<S4>/Subtract'
    //   Sum: '<S4>/Subtract1'
    //   Trigonometry: '<S4>/Trigonometric Function'

    Backlash1[i] = (bpm_blower_1_propulsion_modul_P->abp_nozzle_intake_height -
                    (real32_T)cos((real_T)(rtb_Gain_n +
      bpm_blower_1_propulsion_modul_P->abp_nozzle_min_open_angle)) *
                    bpm_blower_1_propulsion_modul_P->abp_nozzle_flap_length) *
      bpm_blower_1_propulsion_modul_P->abp_PM1_nozzle_widths[i] *
      bpm_blower_1_propulsion_modul_P->abp_nozzle_flap_count;
    rtb_Gain1_ii[i] = rtb_Gain_n;
  }

  // End of Gain: '<S4>/Gain12'
  // End of Outputs for SubSystem: '<S1>/calc_nozzle_area'

  // Outputs for Atomic SubSystem: '<S1>/blower_aerodynamics'

  // DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
  bpm_blower__blower_aerodynamics
    (bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator_DSTATE, Backlash1,
     &bpm_blower_1_propulsion_modul_B->blower_aerodynamics,
     &bpm_blower_1_propulsion_modu_DW->blower_aerodynamics,
     (P_blower_aerodynamics_bpm_blo_T *)
     &bpm_blower_1_propulsion_modul_P->blower_aerodynamics,
     bpm_blower_1_propulsion_modul_P->abp_pm1_zero_thrust_area,
     bpm_blower_1_propulsion_modul_P->bpm_PM1_zero_thrust_area_error,
     bpm_blower_1_propulsion_modul_P->tun_bpm_noise_on_flag,
     bpm_blower_1_propulsion_modul_P->abp_PM1_discharge_coeff,
     bpm_blower_1_propulsion_modul_P->bpm_PM1_nozzle_discharge_coeff_error,
     bpm_blower_1_propulsion_modul_P->bpm_lookup_Cdp_data,
     bpm_blower_1_propulsion_modul_P->bpm_lookup_totalarea_breakpoints,
     bpm_blower_1_propulsion_modul_P->abp_impeller_diameter,
     bpm_blower_1_propulsion_modul_P->const_air_density,
     bpm_blower_1_propulsion_modul_P->bpm_PM1_nozzle_noise_feedback_gain);

  // End of Outputs for SubSystem: '<S1>/blower_aerodynamics'

  // Outport: '<Root>/servo_current'
  for (i = 0; i < 6; i++) {
    bpm_blower_1_propulsion_modul_Y_servo_current[i] = rtb_current[i];
  }

  // End of Outport: '<Root>/servo_current'

  // SignalConversion: '<S10>/TmpSignal ConversionAtProductInport1' incorporates:
  //   Constant: '<S13>/Constant3'
  //   DataTypeConversion: '<S14>/Conversion'
  //   Gain: '<S13>/Gain'
  //   Gain: '<S13>/Gain1'
  //   Gain: '<S13>/Gain2'
  //   Inport: '<Root>/omega_B_ECI_B'
  //   Product: '<S10>/Product'

  tmp_0[0] = (real32_T)bpm_blower_1_propulsion_modul_P->Constant3_Value;
  tmp_0[1] = bpm_blower_1_propulsion_modul_U_omega_B_ECI_B[2];
  tmp_0[2] = bpm_blower_1_propulsion_modul_P->Gain_Gain_h *
    bpm_blower_1_propulsion_modul_U_omega_B_ECI_B[1];
  tmp_0[3] = bpm_blower_1_propulsion_modul_P->Gain1_Gain_f *
    bpm_blower_1_propulsion_modul_U_omega_B_ECI_B[2];
  tmp_0[4] = (real32_T)bpm_blower_1_propulsion_modul_P->Constant3_Value;
  tmp_0[5] = bpm_blower_1_propulsion_modul_U_omega_B_ECI_B[0];
  tmp_0[6] = bpm_blower_1_propulsion_modul_U_omega_B_ECI_B[1];
  tmp_0[7] = bpm_blower_1_propulsion_modul_P->Gain2_Gain *
    bpm_blower_1_propulsion_modul_U_omega_B_ECI_B[0];
  tmp_0[8] = (real32_T)bpm_blower_1_propulsion_modul_P->Constant3_Value;
  for (i = 0; i < 3; i++) {
    // Product: '<S3>/Product4' incorporates:
    //   Sum: '<S3>/Add2'

    tmp_1[i] = 0.0F;
    for (ibmat = 0; ibmat < 6; ibmat++) {
      tmp_1[i] +=
        bpm_blower_1_propulsion_modul_B->OutportBufferForthrust2torque_B
        [(int32_T)((int32_T)(3 * ibmat) + i)] *
        bpm_blower_1_propulsion_modul_B->blower_aerodynamics.Add1[ibmat];
    }

    // End of Product: '<S3>/Product4'

    // Outport: '<Root>/torque_B' incorporates:
    //   Product: '<S10>/Product'
    //   Product: '<S3>/Product'
    //   Product: '<S3>/Product2'
    //   Sum: '<S3>/Add2'

    bpm_blower_1_propulsion_modul_Y_torque_B[i] = (((rtb_Add1[0] *
      rtb_FilterCoefficient_h * tmp_0[i] + tmp_0[(int32_T)(i + 3)] * (rtb_Add1[1]
      * rtb_FilterCoefficient_h)) + tmp_0[(int32_T)(i + 6)] * (rtb_Add1[2] *
      rtb_FilterCoefficient_h)) + rtb_Add1[i] * rtb_Sum_j) + tmp_1[i];

    // Outport: '<Root>/force_B' incorporates:
    //   Product: '<S3>/Product3'

    bpm_blower_1_propulsion_modul_Y_force_B[i] = 0.0F;
    for (ibmat = 0; ibmat < 6; ibmat++) {
      bpm_blower_1_propulsion_modul_Y_force_B[i] +=
        bpm_blower_1_propulsion_modul_B->OutportBufferForthrust2force_B[(int32_T)
        ((int32_T)(3 * ibmat) + i)] *
        bpm_blower_1_propulsion_modul_B->blower_aerodynamics.Add1[ibmat];
    }

    // End of Outport: '<Root>/force_B'
  }

  // Outport: '<Root>/motor_speed' incorporates:
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator'

  *bpm_blower_1_propulsion_modul_Y_motor_speed =
    bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator_DSTATE;

  // Outport: '<Root>/nozzle_theta'
  for (i = 0; i < 6; i++) {
    bpm_blower_1_propulsion_modul_Y_nozzle_theta[i] = rtb_Gain1_ii[i];
  }

  // End of Outport: '<Root>/nozzle_theta'

  // DataTypeConversion: '<S8>/Data Type Conversion' incorporates:
  //   Gain: '<S8>/Gain1'
  //   RandomNumber: '<S8>/random_noise1'

  rtb_SignPreIntegrator_j = (real32_T)(1.0 / sqrt
    (bpm_blower_1_propulsion_modul_P->astrobee_time_step_size) *
    bpm_blower_1_propulsion_modu_DW->NextOutput);

  // Quantizer: '<S8>/Quantizer' incorporates:
  //   DataTypeConversion: '<S8>/Data Type Conversion1'
  //   DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
  //   DiscreteIntegrator: '<S8>/Discrete-Time Integrator1'
  //   Gain: '<S8>/Gain3'
  //   Gain: '<S8>/Gain4'
  //   RandomNumber: '<S8>/random_noise'
  //   Sum: '<S8>/Sum1'

  rtb_FilterCoefficient_h = rt_roundf_snf((((real32_T)(1.0 / sqrt
    (bpm_blower_1_propulsion_modul_P->astrobee_time_step_size) *
    bpm_blower_1_propulsion_modu_DW->NextOutput_e) +
    bpm_blower_1_propulsion_modul_P->bpm_sensor_sf *
    bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator_DSTATE) +
    bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator1_DSTATE) /
    bpm_blower_1_propulsion_modul_P->bpm_sensor_resolution) *
    bpm_blower_1_propulsion_modul_P->bpm_sensor_resolution;

  // Outport: '<Root>/meas_motor_speed' incorporates:
  //   Delay: '<S8>/Delay2'

  *bpm_blower_1_propulsion_modul_Y_meas_motor_speed =
    bpm_blower_1_propulsion_modu_DW->Delay2_DSTATE;

  // Sum: '<S6>/Add'
  rtb_IntegralGain_p -=
    bpm_blower_1_propulsion_modul_B->blower_aerodynamics.Constant7;

  // Switch: '<S6>/Switch' incorporates:
  //   Constant: '<S6>/Constant'
  //   Gain: '<S6>/Gain'
  //   Gain: '<S6>/Gain2'

  if (bpm_blower_1_propulsion_modul_P->tun_bpm_noise_on_flag >
      bpm_blower_1_propulsion_modul_P->Switch_Threshold_j) {
    rtb_IntegralGain_p *= 1.0F /
      bpm_blower_1_propulsion_modul_P->bpm_impeller_inertia;
  } else {
    rtb_IntegralGain_p *= 1.0F /
      (bpm_blower_1_propulsion_modul_P->bpm_impeller_inertia +
       bpm_blower_1_propulsion_modul_P->bpm_impeller_inertia_error);
  }

  // End of Switch: '<S6>/Switch'

  // Update for DiscreteIntegrator: '<S6>/Discrete-Time Integrator'
  bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator_DSTATE +=
    bpm_blower_1_propulsion_modul_P->DiscreteTimeIntegrator_gainval *
    rtb_IntegralGain_p;

  // Update for UnitDelay: '<S3>/Unit Delay' incorporates:
  //   Inport: '<Root>/veh_cm'
  //   Logic: '<S3>/Logical Operator'
  //   RelationalOperator: '<S9>/FixPt Relational Operator'
  //   UnitDelay: '<S9>/Delay Input1'

  bpm_blower_1_propulsion_modu_DW->UnitDelay_DSTATE =
    ((bpm_blower_1_propulsion_modul_U_veh_cm[0] !=
      bpm_blower_1_propulsion_modu_DW->DelayInput1_DSTATE[0]) ||
     (bpm_blower_1_propulsion_modul_U_veh_cm[1] !=
      bpm_blower_1_propulsion_modu_DW->DelayInput1_DSTATE[1]) ||
     (bpm_blower_1_propulsion_modul_U_veh_cm[2] !=
      bpm_blower_1_propulsion_modu_DW->DelayInput1_DSTATE[2]));

  // Update for RandomNumber: '<S8>/random_noise1'
  bpm_blower_1_propulsion_modu_DW->NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
    (&bpm_blower_1_propulsion_modu_DW->RandSeed) *
    bpm_blower_1_propulsion_modul_P->random_noise1_StdDev +
    bpm_blower_1_propulsion_modul_P->random_noise1_Mean;

  // Update for RandomNumber: '<S8>/random_noise'
  bpm_blower_1_propulsion_modu_DW->NextOutput_e = rt_nrand_Upu32_Yd_f_pw_snf
    (&bpm_blower_1_propulsion_modu_DW->RandSeed_j) *
    bpm_blower_1_propulsion_modul_P->random_noise_StdDev +
    bpm_blower_1_propulsion_modul_P->random_noise_Mean;

  // Saturate: '<S8>/Saturation'
  if (rtb_FilterCoefficient_h > bpm_blower_1_propulsion_modul_P->bpm_sensor_max)
  {
    // Update for Delay: '<S8>/Delay2'
    bpm_blower_1_propulsion_modu_DW->Delay2_DSTATE =
      bpm_blower_1_propulsion_modul_P->bpm_sensor_max;
  } else if (rtb_FilterCoefficient_h <
             bpm_blower_1_propulsion_modul_P->bpm_sensor_min) {
    // Update for Delay: '<S8>/Delay2'
    bpm_blower_1_propulsion_modu_DW->Delay2_DSTATE =
      bpm_blower_1_propulsion_modul_P->bpm_sensor_min;
  } else {
    // Update for Delay: '<S8>/Delay2'
    bpm_blower_1_propulsion_modu_DW->Delay2_DSTATE = rtb_FilterCoefficient_h;
  }

  // End of Saturate: '<S8>/Saturation'

  // Update for DiscreteIntegrator: '<S8>/Discrete-Time Integrator1'
  bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator1_DSTATE +=
    bpm_blower_1_propulsion_modul_P->DiscreteTimeIntegrator1_gainval *
    rtb_SignPreIntegrator_j;

  // Update for UnitDelay: '<S9>/Delay Input1' incorporates:
  //   Update for Inport: '<Root>/veh_cm'

  bpm_blower_1_propulsion_modu_DW->DelayInput1_DSTATE[0] =
    bpm_blower_1_propulsion_modul_U_veh_cm[0];
  bpm_blower_1_propulsion_modu_DW->DelayInput1_DSTATE[1] =
    bpm_blower_1_propulsion_modul_U_veh_cm[1];
  bpm_blower_1_propulsion_modu_DW->DelayInput1_DSTATE[2] =
    bpm_blower_1_propulsion_modul_U_veh_cm[2];
}

// Model initialize function
void bpm_blower_1_propulsion_module_initialize(RT_MODEL_bpm_blower_1_propuls_T *
  const bpm_blower_1_propulsion_modu_M, real32_T
  *bpm_blower_1_propulsion_modul_U_battery_voltage, real32_T
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
  *bpm_blower_1_propulsion_modul_Y_meas_motor_speed)
{
  P_bpm_blower_1_propulsion_mod_T *bpm_blower_1_propulsion_modul_P =
    ((P_bpm_blower_1_propulsion_mod_T *)
     bpm_blower_1_propulsion_modu_M->defaultParam);
  B_bpm_blower_1_propulsion_mod_T *bpm_blower_1_propulsion_modul_B =
    ((B_bpm_blower_1_propulsion_mod_T *) bpm_blower_1_propulsion_modu_M->blockIO);
  DW_bpm_blower_1_propulsion_mo_T *bpm_blower_1_propulsion_modu_DW =
    ((DW_bpm_blower_1_propulsion_mo_T *) bpm_blower_1_propulsion_modu_M->dwork);

  {
    uint32_T tseed;
    int32_T t;
    real32_T y;
    int32_T i;
    real_T tmp;

    // InitializeConditions for DiscreteIntegrator: '<S6>/Discrete-Time Integrator' 
    bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator_DSTATE =
      bpm_blower_1_propulsion_modul_P->bpm_impeller_init_speed;

    // InitializeConditions for UnitDelay: '<S3>/Unit Delay'
    bpm_blower_1_propulsion_modu_DW->UnitDelay_DSTATE =
      bpm_blower_1_propulsion_modul_P->UnitDelay_InitialCondition;

    // InitializeConditions for RandomNumber: '<S8>/random_noise1'
    tmp = floor(bpm_blower_1_propulsion_modul_P->bpm_sensor_rand_seed + 1.0);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-tmp : (uint32_T)
      tmp;
    i = (int32_T)(uint32_T)(tseed >> 16U);
    t = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)((uint32_T)i << 16U)) + (uint32_T)t) << 16U) + (uint32_T)t) +
                       (uint32_T)i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    bpm_blower_1_propulsion_modu_DW->RandSeed = tseed;
    bpm_blower_1_propulsion_modu_DW->NextOutput = rt_nrand_Upu32_Yd_f_pw_snf
      (&bpm_blower_1_propulsion_modu_DW->RandSeed) *
      bpm_blower_1_propulsion_modul_P->random_noise1_StdDev +
      bpm_blower_1_propulsion_modul_P->random_noise1_Mean;

    // End of InitializeConditions for RandomNumber: '<S8>/random_noise1'

    // InitializeConditions for RandomNumber: '<S8>/random_noise'
    tmp = floor(bpm_blower_1_propulsion_modul_P->bpm_sensor_rand_seed);
    if (rtIsNaN(tmp) || rtIsInf(tmp)) {
      tmp = 0.0;
    } else {
      tmp = fmod(tmp, 4.294967296E+9);
    }

    tseed = tmp < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-tmp : (uint32_T)
      tmp;
    i = (int32_T)(uint32_T)(tseed >> 16U);
    t = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)((uint32_T)i << 16U)) + (uint32_T)t) << 16U) + (uint32_T)t) +
                       (uint32_T)i);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    bpm_blower_1_propulsion_modu_DW->RandSeed_j = tseed;
    bpm_blower_1_propulsion_modu_DW->NextOutput_e = rt_nrand_Upu32_Yd_f_pw_snf
      (&bpm_blower_1_propulsion_modu_DW->RandSeed_j) *
      bpm_blower_1_propulsion_modul_P->random_noise_StdDev +
      bpm_blower_1_propulsion_modul_P->random_noise_Mean;

    // End of InitializeConditions for RandomNumber: '<S8>/random_noise'

    // InitializeConditions for Delay: '<S8>/Delay2'
    bpm_blower_1_propulsion_modu_DW->Delay2_DSTATE =
      bpm_blower_1_propulsion_modul_P->Delay2_InitialCondition;

    // InitializeConditions for DiscreteIntegrator: '<S8>/Discrete-Time Integrator1' 
    bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator1_DSTATE =
      bpm_blower_1_propulsion_modul_P->DiscreteTimeIntegrator1_IC;

    // InitializeConditions for UnitDelay: '<S9>/Delay Input1'
    bpm_blower_1_propulsion_modu_DW->DelayInput1_DSTATE[0] =
      bpm_blower_1_propulsion_modul_P->DetectChange_vinit;
    bpm_blower_1_propulsion_modu_DW->DelayInput1_DSTATE[1] =
      bpm_blower_1_propulsion_modul_P->DetectChange_vinit;
    bpm_blower_1_propulsion_modu_DW->DelayInput1_DSTATE[2] =
      bpm_blower_1_propulsion_modul_P->DetectChange_vinit;

    // SystemInitialize for Atomic SubSystem: '<S5>/speed_controller'
    // InitializeConditions for RateLimiter: '<S27>/Rate Limiter'
    bpm_blower_1_propulsion_modu_DW->PrevY_l =
      bpm_blower_1_propulsion_modul_P->RateLimiter_IC;

    // InitializeConditions for DiscreteIntegrator: '<S28>/Integrator'
    bpm_blower_1_propulsion_modu_DW->Integrator_DSTATE_k =
      bpm_blower_1_propulsion_modul_P->Integrator_IC;

    // InitializeConditions for DiscreteIntegrator: '<S28>/Filter'
    bpm_blower_1_propulsion_modu_DW->Filter_DSTATE_o =
      bpm_blower_1_propulsion_modul_P->Filter_IC;

    // End of SystemInitialize for SubSystem: '<S5>/speed_controller'

    // SystemInitialize for Atomic SubSystem: '<S1>/servo_model'
    // InitializeConditions for DiscreteIntegrator: '<S31>/Discrete-Time Integrator4' 
    y = (real32_T)bpm_blower_1_propulsion_modul_P->bpm_servo_min_theta /
      bpm_blower_1_propulsion_modul_P->bpm_servo_motor_gear_ratio;
    for (i = 0; i < 6; i++) {
      bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator4_DSTATE[i] = y;

      // InitializeConditions for Backlash: '<S31>/Backlash1'
      bpm_blower_1_propulsion_modu_DW->PrevY[i] =
        bpm_blower_1_propulsion_modul_P->Backlash1_InitialOutput;

      // InitializeConditions for DiscreteIntegrator: '<S33>/Integrator'
      bpm_blower_1_propulsion_modu_DW->Integrator_DSTATE[i] =
        bpm_blower_1_propulsion_modul_P->Integrator_IC_n;

      // InitializeConditions for DiscreteIntegrator: '<S33>/Filter'
      bpm_blower_1_propulsion_modu_DW->Filter_DSTATE[i] =
        bpm_blower_1_propulsion_modul_P->Filter_IC_g;

      // InitializeConditions for DiscreteIntegrator: '<S31>/Discrete-Time Integrator3' 
      bpm_blower_1_propulsion_modu_DW->DiscreteTimeIntegrator3_DSTATE[i] =
        bpm_blower_1_propulsion_modul_P->DiscreteTimeIntegrator3_IC;
    }

    // End of InitializeConditions for DiscreteIntegrator: '<S31>/Discrete-Time Integrator4' 
    // End of SystemInitialize for SubSystem: '<S1>/servo_model'

    // SystemInitialize for Enabled SubSystem: '<S3>/latch_nozzle_thrust_matricies' 
    for (i = 0; i < 18; i++) {
      // SystemInitialize for Outport: '<S11>/thrust2torque_B'
      bpm_blower_1_propulsion_modul_B->OutportBufferForthrust2torque_B[i] =
        bpm_blower_1_propulsion_modul_P->thrust2torque_B_Y0;

      // SystemInitialize for Outport: '<S11>/thrust2force_B'
      bpm_blower_1_propulsion_modul_B->OutportBufferForthrust2force_B[i] =
        bpm_blower_1_propulsion_modul_P->thrust2force_B_Y0;
    }

    // End of SystemInitialize for SubSystem: '<S3>/latch_nozzle_thrust_matricies' 

    // SystemInitialize for Atomic SubSystem: '<S1>/blower_aerodynamics'
    bpm_bl_blower_aerodynamics_Init
      (&bpm_blower_1_propulsion_modu_DW->blower_aerodynamics,
       (P_blower_aerodynamics_bpm_blo_T *)
       &bpm_blower_1_propulsion_modul_P->blower_aerodynamics,
       bpm_blower_1_propulsion_modul_P->bpm_PM1_randn_noise_seed);

    // End of SystemInitialize for SubSystem: '<S1>/blower_aerodynamics'
  }
}

// Model terminate function
void bpm_blower_1_propulsion_module_terminate(RT_MODEL_bpm_blower_1_propuls_T
  * bpm_blower_1_propulsion_modu_M)
{
  // model code
  rt_FREE(bpm_blower_1_propulsion_modu_M->blockIO);
  if (bpm_blower_1_propulsion_modu_M->paramIsMalloced) {
    rt_FREE(bpm_blower_1_propulsion_modu_M->defaultParam);
  }

  rt_FREE(bpm_blower_1_propulsion_modu_M->dwork);
  rt_FREE(bpm_blower_1_propulsion_modu_M);
}

// Model data allocation function
RT_MODEL_bpm_blower_1_propuls_T *bpm_blower_1_propulsion_module(real32_T
  *bpm_blower_1_propulsion_modul_U_battery_voltage, real32_T
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
  *bpm_blower_1_propulsion_modul_Y_meas_motor_speed)
{
  RT_MODEL_bpm_blower_1_propuls_T *bpm_blower_1_propulsion_modu_M;
  bpm_blower_1_propulsion_modu_M = (RT_MODEL_bpm_blower_1_propuls_T *) malloc
    (sizeof(RT_MODEL_bpm_blower_1_propuls_T));
  if (bpm_blower_1_propulsion_modu_M == NULL) {
    return NULL;
  }

  (void) memset((char *)bpm_blower_1_propulsion_modu_M, 0,
                sizeof(RT_MODEL_bpm_blower_1_propuls_T));

  // block I/O
  {
    B_bpm_blower_1_propulsion_mod_T *b = (B_bpm_blower_1_propulsion_mod_T *)
      malloc(sizeof(B_bpm_blower_1_propulsion_mod_T));
    rt_VALIDATE_MEMORY(bpm_blower_1_propulsion_modu_M,b);
    bpm_blower_1_propulsion_modu_M->blockIO = (b);
  }

  // parameters
  {
    P_bpm_blower_1_propulsion_mod_T *p;
    static int_T pSeen = 0;

    // only malloc on multiple model instantiation
    if (pSeen == 1 ) {
      p = (P_bpm_blower_1_propulsion_mod_T *) malloc(sizeof
        (P_bpm_blower_1_propulsion_mod_T));
      rt_VALIDATE_MEMORY(bpm_blower_1_propulsion_modu_M,p);
      (void) memcpy(p, &bpm_blower_1_propulsion_modul_P,
                    sizeof(P_bpm_blower_1_propulsion_mod_T));
      bpm_blower_1_propulsion_modu_M->paramIsMalloced = (true);
    } else {
      p = &bpm_blower_1_propulsion_modul_P;
      bpm_blower_1_propulsion_modu_M->paramIsMalloced = (false);
      pSeen = 1;
    }

    bpm_blower_1_propulsion_modu_M->defaultParam = (p);
  }

  // states (dwork)
  {
    DW_bpm_blower_1_propulsion_mo_T *dwork = (DW_bpm_blower_1_propulsion_mo_T *)
      malloc(sizeof(DW_bpm_blower_1_propulsion_mo_T));
    rt_VALIDATE_MEMORY(bpm_blower_1_propulsion_modu_M,dwork);
    bpm_blower_1_propulsion_modu_M->dwork = (dwork);
  }

  {
    P_bpm_blower_1_propulsion_mod_T *bpm_blower_1_propulsion_modul_P =
      ((P_bpm_blower_1_propulsion_mod_T *)
       bpm_blower_1_propulsion_modu_M->defaultParam);
    B_bpm_blower_1_propulsion_mod_T *bpm_blower_1_propulsion_modul_B =
      ((B_bpm_blower_1_propulsion_mod_T *)
       bpm_blower_1_propulsion_modu_M->blockIO);
    DW_bpm_blower_1_propulsion_mo_T *bpm_blower_1_propulsion_modu_DW =
      ((DW_bpm_blower_1_propulsion_mo_T *) bpm_blower_1_propulsion_modu_M->dwork);

    // initialize non-finites
    rt_InitInfAndNaN(sizeof(real_T));

    // block I/O
    (void) memset(((void *) bpm_blower_1_propulsion_modul_B), 0,
                  sizeof(B_bpm_blower_1_propulsion_mod_T));

    // states (dwork)
    (void) memset((void *)bpm_blower_1_propulsion_modu_DW, 0,
                  sizeof(DW_bpm_blower_1_propulsion_mo_T));

    // external inputs
    (void)memset(&bpm_blower_1_propulsion_modul_U_omega_B_ECI_B[0], 0, 3U *
                 sizeof(real32_T));
    (void)memset(&bpm_blower_1_propulsion_modul_U_veh_cm[0], 0, 3U * sizeof
                 (real32_T));
    (void)memset(&bpm_blower_1_propulsion_modul_U_servo_cmd[0], 0, 6U * sizeof
                 (real32_T));
    *bpm_blower_1_propulsion_modul_U_battery_voltage = 0.0F;
    *bpm_blower_1_propulsion_modul_U_impeller_cmd = 0U;

    // external outputs
    (*bpm_blower_1_propulsion_modul_Y_impeller_current) = 0.0F;
    (void) memset(&bpm_blower_1_propulsion_modul_Y_servo_current[0], 0,
                  6U*sizeof(real32_T));
    (void) memset(&bpm_blower_1_propulsion_modul_Y_torque_B[0], 0,
                  3U*sizeof(real32_T));
    (void) memset(&bpm_blower_1_propulsion_modul_Y_force_B[0], 0,
                  3U*sizeof(real32_T));
    (*bpm_blower_1_propulsion_modul_Y_motor_speed) = 0.0F;
    (void) memset(&bpm_blower_1_propulsion_modul_Y_nozzle_theta[0], 0,
                  6U*sizeof(real32_T));
    (*bpm_blower_1_propulsion_modul_Y_meas_motor_speed) = 0.0F;
  }

  return bpm_blower_1_propulsion_modu_M;
}

//
// File trailer for generated code.
//
// [EOF]
//
