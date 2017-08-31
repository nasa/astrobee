//
// File: aieciecjdjmgglfc_eulers_to_quat.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Aug 22 16:52:08 2017
//
#include "rtwtypes.h"
#include <math.h>
#include "aieciecjdjmgglfc_eulers_to_quat.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void aieciecjdjmgglfc_eulers_to_quat(real32_T phi, real32_T theta, real32_T psi,
  real32_T quat[4])
{
  real32_T c_psi;
  real32_T s_psi;
  real32_T c_theta;
  real32_T s_theta;
  real32_T c_phi;
  real32_T s_phi;

  //  Convert Euler angles to quaternions.  Assumes 3-2-1 rotation sequence for
  //  euler angles.  Corrected typo where first and 3rd rows were flipped.
  //  From: Fundamentals of Spacecraft Attitude Determination and Control, F.
  //  Landis Markley and John L. Crassisdis. Table: B.5
  //
  //  Jesse. C. Fusco           jesse.c.fusco@nasa.gov
  //
  c_psi = (real32_T)cos((real_T)(psi / 2.0F));
  s_psi = (real32_T)sin((real_T)(psi / 2.0F));
  c_theta = (real32_T)cos((real_T)(theta / 2.0F));
  s_theta = (real32_T)sin((real_T)(theta / 2.0F));
  c_phi = (real32_T)cos((real_T)(phi / 2.0F));
  s_phi = (real32_T)sin((real_T)(phi / 2.0F));
  quat[0] = s_phi * c_theta * c_psi - c_phi * s_theta * s_psi;
  quat[2] = c_phi * c_theta * s_psi - s_phi * s_theta * c_psi;
  quat[1] = c_phi * s_theta * c_psi + s_phi * c_theta * s_psi;
  quat[3] = c_phi * c_theta * c_psi + s_phi * s_theta * s_psi;
}

//
// File trailer for generated code.
//
// [EOF]
//
