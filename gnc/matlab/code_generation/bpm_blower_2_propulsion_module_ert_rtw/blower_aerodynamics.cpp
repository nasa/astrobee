//
// File: blower_aerodynamics.cpp
//
// Code generated for Simulink model 'bpm_blower_2_propulsion_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Mon Sep 23 17:47:03 2019
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#include "blower_aerodynamics.h"

// Include model header file for global data
#include "bpm_blower_2_propulsion_module.h"
#include "bpm_blower_2_propulsion_module_private.h"

// System initialize for atomic system: '<S1>/blower_aerodynamics'
void bpm_bl_blower_aerodynamics_Init(DW_blower_aerodynamics_bpm_bl_T *localDW,
  P_blower_aerodynamics_bpm_blo_T *localP, real_T rtp_noz_randn_seed)
{
  uint32_T tseed;
  int32_T r;
  int32_T t;
  real_T y1;
  int32_T i;
  for (i = 0; i < 6; i++) {
    // InitializeConditions for DiscreteIntegrator: '<S2>/Discrete-Time Integrator' 
    localDW->DiscreteTimeIntegrator_DSTATE[i] =
      localP->DiscreteTimeIntegrator_IC;

    // InitializeConditions for RandomNumber: '<S2>/random_noise'
    y1 = floor(rtp_noz_randn_seed + rtCP_random_noise_rtw_collapsed[i]);
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

    // End of InitializeConditions for RandomNumber: '<S2>/random_noise'
  }
}

// Output and update for atomic system: '<S1>/blower_aerodynamics'
void bpm_blower__blower_aerodynamics(real32_T rtu_rotor_speed, const real32_T
  rtu_nozzle_areas[6], B_blower_aerodynamics_bpm_blo_T *localB,
  DW_blower_aerodynamics_bpm_bl_T *localDW, P_blower_aerodynamics_bpm_blo_T
  *localP, real32_T rtp_imp_zero_thrust_area, real32_T
  rtp_imp_zero_thrust_area_error, real32_T rtp_noise_on_flag, const real32_T
  rtp_noz_cd[6], const real32_T rtp_noz_cd_error[6], const real32_T
  rtp_imp_cdp_lookup[334], const real32_T rtp_imp_area_lookup[334], real32_T
  rtp_imp_diameter, real32_T rtp_const_air_den, real32_T rtp_noz_thrust_sf,
  const real_T rtp_noz_thrust_noise_feedback[6])
{
  real32_T rtb_walking_bias[6];
  real32_T rtb_Product2_k[6];
  real32_T rtb_Cdp;
  int32_T i;
  for (i = 0; i < 6; i++) {
    // Sum: '<S2>/Add3' incorporates:
    //   Constant: '<S2>/Constant3'
    //   Constant: '<S2>/Constant8'
    //   Gain: '<S2>/Gain1'

    rtb_Cdp = rtp_noise_on_flag * rtp_noz_cd_error[i] + rtp_noz_cd[i];

    // Product: '<S2>/Product2'
    rtb_Product2_k[i] = rtb_Cdp * rtu_nozzle_areas[i];

    // Sum: '<S2>/Add3'
    rtb_walking_bias[i] = rtb_Cdp;
  }

  // Sum: '<S2>/Sum of Elements1'
  rtb_Cdp = rtb_Product2_k[0];
  for (i = 0; i < 5; i++) {
    rtb_Cdp += rtb_Product2_k[(int32_T)(i + 1)];
  }

  // End of Sum: '<S2>/Sum of Elements1'

  // Sum: '<S2>/Add' incorporates:
  //   Constant: '<S2>/Constant5'
  //   Constant: '<S2>/Constant6'
  //   Gain: '<S2>/Gain2'

  rtb_Cdp += rtp_noise_on_flag * rtp_imp_zero_thrust_area_error +
    rtp_imp_zero_thrust_area;

  // Lookup_n-D: '<S2>/bpm_Cdp_lookup'
  rtb_Cdp = look1_iflf_binlxpw(rtb_Cdp, rtp_imp_area_lookup, rtp_imp_cdp_lookup,
    333U);

  // Product: '<S2>/Product3' incorporates:
  //   Constant: '<S2>/Constant1'
  //   Constant: '<S2>/Constant2'

  rtb_Cdp = rtu_rotor_speed * rtu_rotor_speed * rtb_Cdp * rtp_imp_diameter *
    rtp_imp_diameter * rtp_const_air_den;

  // Constant: '<S2>/Constant7'
  localB->Constant7 = localP->Constant7_Value;
  for (i = 0; i < 6; i++) {
    // Sum: '<S2>/Add1' incorporates:
    //   Constant: '<S2>/Constant4'
    //   Constant: '<S2>/Constant9'
    //   DataTypeConversion: '<S2>/Data Type Conversion'
    //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
    //   Product: '<S2>/Product1'
    //   Product: '<S2>/Product6'
    //   Product: '<S2>/Product7'

    localB->Add1[i] = localP->Constant4_Value * rtb_walking_bias[i] *
      rtb_walking_bias[i] * rtb_Cdp * rtu_nozzle_areas[i] * rtp_noz_thrust_sf +
      (real32_T)localDW->DiscreteTimeIntegrator_DSTATE[i];

    // Update for DiscreteIntegrator: '<S2>/Discrete-Time Integrator' incorporates:
    //   DiscreteIntegrator: '<S2>/Discrete-Time Integrator'
    //   Gain: '<S2>/Gain3'
    //   RandomNumber: '<S2>/random_noise'
    //   Sum: '<S2>/Sum'

    localDW->DiscreteTimeIntegrator_DSTATE[i] += (localDW->NextOutput[i] -
      rtp_noz_thrust_noise_feedback[i] * localDW->
      DiscreteTimeIntegrator_DSTATE[i]) * localP->DiscreteTimeIntegrator_gainval;

    // Update for RandomNumber: '<S2>/random_noise'
    localDW->NextOutput[i] = rt_nrand_Upu32_Yd_f_pw_snf(&localDW->RandSeed[i]) *
      localP->random_noise_StdDev[i] + localP->random_noise_Mean;
  }
}

// Termination for atomic system: '<S1>/blower_aerodynamics'
void bpm_bl_blower_aerodynamics_Term(void)
{
}

//
// File trailer for generated code.
//
// [EOF]
//
