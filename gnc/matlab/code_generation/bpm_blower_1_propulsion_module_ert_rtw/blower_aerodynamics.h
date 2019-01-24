//
// File: blower_aerodynamics.h
//
// Code generated for Simulink model 'bpm_blower_1_propulsion_module'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Thu Dec  6 14:21:20 2018
//
// Target selection: ert.tlc
// Embedded hardware selection: 32-bit Generic
// Code generation objectives: Unspecified
// Validation result: Not run
//
#ifndef RTW_HEADER_blower_aerodynamics_h_
#define RTW_HEADER_blower_aerodynamics_h_
#include <math.h>
#ifndef bpm_blower_1_propulsion_module_COMMON_INCLUDES_
# define bpm_blower_1_propulsion_module_COMMON_INCLUDES_
#include <stdlib.h>
#include "rtwtypes.h"
#endif                                 // bpm_blower_1_propulsion_module_COMMON_INCLUDES_ 

#include "bpm_blower_1_propulsion_module_types.h"
#include "look1_iflf_binlxpw.h"
#include "rt_nonfinite.h"
#include "rt_nrand_Upu32_Yd_f_pw_snf.h"

// Block signals for system '<S1>/blower_aerodynamics'
typedef struct {
  real32_T Add1[6];                    // '<S2>/Add1'
  real32_T Constant7;                  // '<S2>/Constant7'
} B_blower_aerodynamics_bpm_blo_T;

// Block states (auto storage) for system '<S1>/blower_aerodynamics'
typedef struct {
  real_T DiscreteTimeIntegrator_DSTATE[6];// '<S2>/Discrete-Time Integrator'
  real_T NextOutput[6];                // '<S2>/random_noise'
  uint32_T RandSeed[6];                // '<S2>/random_noise'
} DW_blower_aerodynamics_bpm_bl_T;

// Parameters for system: '<S1>/blower_aerodynamics'
struct P_blower_aerodynamics_bpm_blo_T_ {
  real_T DiscreteTimeIntegrator_gainval;// Computed Parameter: DiscreteTimeIntegrator_gainval
                                        //  Referenced by: '<S2>/Discrete-Time Integrator'

  real_T DiscreteTimeIntegrator_IC;    // Expression: 0
                                       //  Referenced by: '<S2>/Discrete-Time Integrator'

  real_T random_noise_Mean;            // Expression: 0
                                       //  Referenced by: '<S2>/random_noise'

  real_T random_noise_StdDev[6];       // Computed Parameter: random_noise_StdDev
                                       //  Referenced by: '<S2>/random_noise'

  real32_T Constant4_Value;            // Expression: single(2)
                                       //  Referenced by: '<S2>/Constant4'

  real32_T Constant7_Value;            // Expression: single(0)
                                       //  Referenced by: '<S2>/Constant7'

};

void bpm_bl_blower_aerodynamics_Init(DW_blower_aerodynamics_bpm_bl_T *localDW,
  P_blower_aerodynamics_bpm_blo_T *localP, real_T rtp_noz_randn_seed);
void bpm_blower__blower_aerodynamics(real32_T rtu_rotor_speed, const real32_T
  rtu_nozzle_areas[6], B_blower_aerodynamics_bpm_blo_T *localB,
  DW_blower_aerodynamics_bpm_bl_T *localDW, P_blower_aerodynamics_bpm_blo_T
  *localP, real32_T rtp_imp_zero_thrust_area, real32_T
  rtp_imp_zero_thrust_area_error, real32_T rtp_noise_on_flag, const real32_T
  rtp_noz_cd[6], const real32_T rtp_noz_cd_error[6], const real32_T
  rtp_imp_cdp_lookup[334], const real32_T rtp_imp_area_lookup[334], real32_T
  rtp_imp_diameter, real32_T rtp_const_air_den, real32_T rtp_noz_thrust_sf,
  const real_T rtp_noz_thrust_noise_feedback[6]);

#endif                                 // RTW_HEADER_blower_aerodynamics_h_

//
// File trailer for generated code.
//
// [EOF]
//
