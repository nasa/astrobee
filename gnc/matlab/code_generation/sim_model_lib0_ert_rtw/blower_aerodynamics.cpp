//
// File: blower_aerodynamics.cpp
//
// Code generated for Simulink model 'sim_model_lib0'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Thu Aug 31 10:22:10 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "blower_aerodynamics.h"

// Include model header file for global data
#include "sim_model_lib0.h"
#include "sim_model_lib0_private.h"

// System initialize for atomic system: '<S184>/blower_aerodynamics'
void si_blower_aerodynamics_Init(DW_blower_aerodynamics_sim_mo_T *localDW,
  P_blower_aerodynamics_sim_mod_T *localP, real_T rtp_noz_randn_seed)
{
  uint32_T tseed;
  int32_T r;
  int32_T t;
  real_T y1;
  int32_T i;
  for (i = 0; i < 6; i++) {
    // InitializeConditions for DiscreteIntegrator: '<S187>/Discrete-Time Integrator' 
    localDW->DiscreteTimeIntegrator_DSTATE[i] =
      localP->DiscreteTimeIntegrator_IC;

    // InitializeConditions for RandomNumber: '<S187>/random_noise'
    y1 = floor(rtp_noz_randn_seed + rtCP_pooled1[i]);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (int32_T)(uint32_T)(tseed >> 16U);
    t = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)((uint32_T)r << 16U)) + (uint32_T)t) << 16U) + (uint32_T)t) +
                       (uint32_T)r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) * localP->random_noise_StdDev[i] +
      localP->random_noise_Mean;
    localDW->NextOutput[i] = y1;
    localDW->RandSeed[i] = tseed;

    // End of InitializeConditions for RandomNumber: '<S187>/random_noise'
  }
}

// Output and update for atomic system: '<S184>/blower_aerodynamics'
void sim_mod_blower_aerodynamics(real32_T rtu_rotor_speed, const real32_T
  rtu_nozzle_areas[6], B_blower_aerodynamics_sim_mod_T *localB,
  DW_blower_aerodynamics_sim_mo_T *localDW, P_blower_aerodynamics_sim_mod_T
  *localP, real32_T rtp_imp_zero_thrust_area, real32_T
  rtp_imp_zero_thrust_area_error, real32_T rtp_noise_on_flag, const real32_T
  rtp_noz_cd[6], const real32_T rtp_noz_cd_error[6], const real32_T
  rtp_imp_cdp_lookup[334], const real32_T rtp_imp_area_lookup[334], real32_T
  rtp_imp_diameter, real32_T rtp_const_air_den, const real_T
  rtp_noz_thrust_noise_feedback[6])
{
  real32_T rtb_walking_bias[6];
  real32_T rtb_Product2_g[6];
  real32_T rtb_Cdp;
  int32_T i;
  for (i = 0; i < 6; i++) {
    // Sum: '<S187>/Add3' incorporates:
    //   Constant: '<S187>/Constant3'
    //   Constant: '<S187>/Constant8'
    //   Gain: '<S187>/Gain1'

    rtb_Cdp = rtp_noise_on_flag * rtp_noz_cd_error[i] + rtp_noz_cd[i];

    // Product: '<S187>/Product2'
    rtb_Product2_g[i] = rtb_Cdp * rtu_nozzle_areas[i];

    // Sum: '<S187>/Add3'
    rtb_walking_bias[i] = rtb_Cdp;
  }

  // Sum: '<S187>/Sum of Elements1'
  rtb_Cdp = rtb_Product2_g[0];
  for (i = 0; i < 5; i++) {
    rtb_Cdp += rtb_Product2_g[(int32_T)(i + 1)];
  }

  // End of Sum: '<S187>/Sum of Elements1'

  // Sum: '<S187>/Add' incorporates:
  //   Constant: '<S187>/Constant5'
  //   Constant: '<S187>/Constant6'
  //   Gain: '<S187>/Gain2'

  rtb_Cdp += rtp_noise_on_flag * rtp_imp_zero_thrust_area_error +
    rtp_imp_zero_thrust_area;

  // Lookup_n-D: '<S187>/bpm_Cdp_lookup'
  rtb_Cdp = look1_iflf_binlxpw(rtb_Cdp, rtp_imp_area_lookup, rtp_imp_cdp_lookup,
    333U);

  // Product: '<S187>/Product3' incorporates:
  //   Constant: '<S187>/Constant1'
  //   Constant: '<S187>/Constant2'

  rtb_Cdp = rtu_rotor_speed * rtu_rotor_speed * rtb_Cdp * rtp_imp_diameter *
    rtp_imp_diameter * rtp_const_air_den;

  // Constant: '<S187>/Constant7'
  localB->Constant7 = localP->Constant7_Value;
  for (i = 0; i < 6; i++) {
    // Sum: '<S187>/Add1' incorporates:
    //   Constant: '<S187>/Constant4'
    //   DataTypeConversion: '<S187>/Data Type Conversion'
    //   DiscreteIntegrator: '<S187>/Discrete-Time Integrator'
    //   Product: '<S187>/Product1'
    //   Product: '<S187>/Product6'
    //   Product: '<S187>/Product7'

    localB->Add1[i] = localP->Constant4_Value * rtb_walking_bias[i] *
      rtb_walking_bias[i] * rtb_Cdp * rtu_nozzle_areas[i] + (real32_T)
      localDW->DiscreteTimeIntegrator_DSTATE[i];

    // Update for DiscreteIntegrator: '<S187>/Discrete-Time Integrator' incorporates:
    //   DiscreteIntegrator: '<S187>/Discrete-Time Integrator'
    //   Gain: '<S187>/Gain3'
    //   RandomNumber: '<S187>/random_noise'
    //   Sum: '<S187>/Sum'

    localDW->DiscreteTimeIntegrator_DSTATE[i] += (localDW->NextOutput[i] -
      rtp_noz_thrust_noise_feedback[i] * localDW->
      DiscreteTimeIntegrator_DSTATE[i]) * localP->DiscreteTimeIntegrator_gainval;

    // Update for RandomNumber: '<S187>/random_noise'
    localDW->NextOutput[i] = rt_nrand_Upu32_Yd_f_pw_snf(&localDW->RandSeed[i]) *
      localP->random_noise_StdDev[i] + localP->random_noise_Mean;
  }
}

// Termination for atomic system: '<S184>/blower_aerodynamics'
void si_blower_aerodynamics_Term(void)
{
}

// System initialize for atomic system: '<S185>/blower_aerodynamics'
void blower_aerodynamics_d_Init(DW_blower_aerodynamics_sim__c_T *localDW,
  P_blower_aerodynamics_sim_m_m_T *localP, real_T rtp_noz_randn_seed)
{
  uint32_T tseed;
  int32_T r;
  int32_T t;
  real_T y1;
  int32_T i;
  for (i = 0; i < 6; i++) {
    // InitializeConditions for DiscreteIntegrator: '<S220>/Discrete-Time Integrator' 
    localDW->DiscreteTimeIntegrator_DSTATE[i] =
      localP->DiscreteTimeIntegrator_IC;

    // InitializeConditions for RandomNumber: '<S220>/random_noise'
    y1 = floor(rtp_noz_randn_seed + rtCP_pooled1[i]);
    if (rtIsNaN(y1) || rtIsInf(y1)) {
      y1 = 0.0;
    } else {
      y1 = fmod(y1, 4.294967296E+9);
    }

    tseed = y1 < 0.0 ? (uint32_T)(int32_T)-(int32_T)(uint32_T)-y1 : (uint32_T)y1;
    r = (int32_T)(uint32_T)(tseed >> 16U);
    t = (int32_T)(uint32_T)(tseed & 32768U);
    tseed = (uint32_T)((uint32_T)((uint32_T)((uint32_T)((uint32_T)(tseed -
      (uint32_T)((uint32_T)r << 16U)) + (uint32_T)t) << 16U) + (uint32_T)t) +
                       (uint32_T)r);
    if (tseed < 1U) {
      tseed = 1144108930U;
    } else {
      if (tseed > 2147483646U) {
        tseed = 2147483646U;
      }
    }

    y1 = rt_nrand_Upu32_Yd_f_pw_snf(&tseed) * localP->random_noise_StdDev[i] +
      localP->random_noise_Mean;
    localDW->NextOutput[i] = y1;
    localDW->RandSeed[i] = tseed;

    // End of InitializeConditions for RandomNumber: '<S220>/random_noise'
  }
}

// Output and update for atomic system: '<S185>/blower_aerodynamics'
void sim_m_blower_aerodynamics_j(real32_T rtu_rotor_speed, const real32_T
  rtu_nozzle_areas[6], B_blower_aerodynamics_sim_m_f_T *localB,
  DW_blower_aerodynamics_sim__c_T *localDW, P_blower_aerodynamics_sim_m_m_T
  *localP, real32_T rtp_imp_zero_thrust_area, real32_T
  rtp_imp_zero_thrust_area_error, real32_T rtp_noise_on_flag, const real32_T
  rtp_noz_cd[6], const real32_T rtp_noz_cd_error[6], const real32_T
  rtp_imp_cdp_lookup[334], const real32_T rtp_imp_area_lookup[334], real32_T
  rtp_imp_diameter, real32_T rtp_const_air_den, const real_T
  rtp_noz_thrust_noise_feedback[6])
{
  real32_T rtb_walking_bias[6];
  real32_T rtb_Product2_h[6];
  real32_T rtb_Cdp;
  int32_T i;
  for (i = 0; i < 6; i++) {
    // Sum: '<S220>/Add3' incorporates:
    //   Constant: '<S220>/Constant3'
    //   Constant: '<S220>/Constant8'
    //   Gain: '<S220>/Gain1'

    rtb_Cdp = rtp_noise_on_flag * rtp_noz_cd_error[i] + rtp_noz_cd[i];

    // Product: '<S220>/Product2'
    rtb_Product2_h[i] = rtb_Cdp * rtu_nozzle_areas[i];

    // Sum: '<S220>/Add3'
    rtb_walking_bias[i] = rtb_Cdp;
  }

  // Sum: '<S220>/Sum of Elements1'
  rtb_Cdp = rtb_Product2_h[0];
  for (i = 0; i < 5; i++) {
    rtb_Cdp += rtb_Product2_h[(int32_T)(i + 1)];
  }

  // End of Sum: '<S220>/Sum of Elements1'

  // Sum: '<S220>/Add' incorporates:
  //   Constant: '<S220>/Constant5'
  //   Constant: '<S220>/Constant6'
  //   Gain: '<S220>/Gain2'

  rtb_Cdp += rtp_noise_on_flag * rtp_imp_zero_thrust_area_error +
    rtp_imp_zero_thrust_area;

  // Lookup_n-D: '<S220>/bpm_Cdp_lookup'
  rtb_Cdp = look1_iflf_binlxpw(rtb_Cdp, rtp_imp_area_lookup, rtp_imp_cdp_lookup,
    333U);

  // Product: '<S220>/Product3' incorporates:
  //   Constant: '<S220>/Constant1'
  //   Constant: '<S220>/Constant2'

  rtb_Cdp = rtu_rotor_speed * rtu_rotor_speed * rtb_Cdp * rtp_imp_diameter *
    rtp_imp_diameter * rtp_const_air_den;

  // Constant: '<S220>/Constant7'
  localB->Constant7 = localP->Constant7_Value;
  for (i = 0; i < 6; i++) {
    // Sum: '<S220>/Add1' incorporates:
    //   Constant: '<S220>/Constant4'
    //   DataTypeConversion: '<S220>/Data Type Conversion'
    //   DiscreteIntegrator: '<S220>/Discrete-Time Integrator'
    //   Product: '<S220>/Product1'
    //   Product: '<S220>/Product6'
    //   Product: '<S220>/Product7'

    localB->Add1[i] = localP->Constant4_Value * rtb_walking_bias[i] *
      rtb_walking_bias[i] * rtb_Cdp * rtu_nozzle_areas[i] + (real32_T)
      localDW->DiscreteTimeIntegrator_DSTATE[i];

    // Update for DiscreteIntegrator: '<S220>/Discrete-Time Integrator' incorporates:
    //   DiscreteIntegrator: '<S220>/Discrete-Time Integrator'
    //   Gain: '<S220>/Gain3'
    //   RandomNumber: '<S220>/random_noise'
    //   Sum: '<S220>/Sum'

    localDW->DiscreteTimeIntegrator_DSTATE[i] += (localDW->NextOutput[i] -
      rtp_noz_thrust_noise_feedback[i] * localDW->
      DiscreteTimeIntegrator_DSTATE[i]) * localP->DiscreteTimeIntegrator_gainval;

    // Update for RandomNumber: '<S220>/random_noise'
    localDW->NextOutput[i] = rt_nrand_Upu32_Yd_f_pw_snf(&localDW->RandSeed[i]) *
      localP->random_noise_StdDev[i] + localP->random_noise_Mean;
  }
}

// Termination for atomic system: '<S185>/blower_aerodynamics'
void blower_aerodynamics_f_Term(void)
{
}

//
// File trailer for generated code.
//
// [EOF]
//
