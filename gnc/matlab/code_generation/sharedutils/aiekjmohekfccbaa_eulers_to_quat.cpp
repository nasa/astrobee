//
// File: aiekjmohekfccbaa_eulers_to_quat.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1137
// Simulink Coder version         : 8.9 (R2015b) 13-Aug-2015
// C/C++ source code generated on : Wed May  3 15:13:05 2017
//
#include "rtwtypes.h"
#include <math.h>
#include "aiekjmohekfccbaa_eulers_to_quat.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void aiekjmohekfccbaa_eulers_to_quat(real32_T rz, real32_T ry, real32_T rx,
  real32_T quat[4])
{
  real32_T result;
  int32_T i;
  int32_T loop_ub;
  real32_T quat_data[4];
  int32_T ii_sizes_idx_0;
  int32_T ii_sizes_idx_1;
  int32_T quat_sizes_idx_0;
  quat[0] = (real32_T)cos((real_T)(rz / 2.0F)) * (real32_T)cos((real_T)(ry /
    2.0F)) * (real32_T)sin((real_T)(rx / 2.0F)) - (real32_T)sin((real_T)(rz /
    2.0F)) * (real32_T)sin((real_T)(ry / 2.0F)) * (real32_T)cos((real_T)(rx /
    2.0F));
  quat[1] = (real32_T)cos((real_T)(rz / 2.0F)) * (real32_T)sin((real_T)(ry /
    2.0F)) * (real32_T)cos((real_T)(rx / 2.0F)) + (real32_T)sin((real_T)(rz /
    2.0F)) * (real32_T)cos((real_T)(ry / 2.0F)) * (real32_T)sin((real_T)(rx /
    2.0F));
  quat[2] = (real32_T)sin((real_T)(rz / 2.0F)) * (real32_T)cos((real_T)(ry /
    2.0F)) * (real32_T)cos((real_T)(rx / 2.0F)) - (real32_T)cos((real_T)(rz /
    2.0F)) * (real32_T)sin((real_T)(ry / 2.0F)) * (real32_T)sin((real_T)(rx /
    2.0F));
  quat[3] = (real32_T)cos((real_T)(rz / 2.0F)) * (real32_T)cos((real_T)(ry /
    2.0F)) * (real32_T)cos((real_T)(rx / 2.0F)) + (real32_T)sin((real_T)(rz /
    2.0F)) * (real32_T)sin((real_T)(ry / 2.0F)) * (real32_T)sin((real_T)(rx /
    2.0F));

  // Normalize the quaternion and make the scalar term positive
  if (quat[3] < 0.0F) {
    ii_sizes_idx_0 = 1;
    ii_sizes_idx_1 = 1;
  } else {
    ii_sizes_idx_0 = 0;
    ii_sizes_idx_1 = 0;
  }

  quat_sizes_idx_0 = ii_sizes_idx_0 * ii_sizes_idx_1;
  loop_ub = ii_sizes_idx_0 * ii_sizes_idx_1;
  for (i = 0; i < loop_ub; i++) {
    quat_data[i] = -quat[0];
  }

  loop_ub = ii_sizes_idx_0 * ii_sizes_idx_1;
  for (i = 0; i < loop_ub; i++) {
    quat_data[i + quat_sizes_idx_0] = -quat[1];
  }

  loop_ub = ii_sizes_idx_0 * ii_sizes_idx_1;
  for (i = 0; i < loop_ub; i++) {
    quat_data[i + (quat_sizes_idx_0 << 1)] = -quat[2];
  }

  loop_ub = ii_sizes_idx_0 * ii_sizes_idx_1;
  for (i = 0; i < loop_ub; i++) {
    quat_data[i + quat_sizes_idx_0 * 3] = -quat[3];
  }

  for (i = 0; i < quat_sizes_idx_0; i++) {
    quat[0] = quat_data[i];
  }

  for (i = 0; i < quat_sizes_idx_0; i++) {
    quat[1] = quat_data[i + quat_sizes_idx_0];
  }

  for (i = 0; i < quat_sizes_idx_0; i++) {
    quat[2] = quat_data[(quat_sizes_idx_0 << 1) + i];
  }

  for (i = 0; i < quat_sizes_idx_0; i++) {
    quat[3] = quat_data[quat_sizes_idx_0 * 3 + i];
  }

  //
  //  Column_Vector = RSSROW(Matrix)
  //
  //  where Matrix is an n-by-m matrix.  This function will return the RSS of
  //  each row into an n-by-1 column vector for any number of m columns.
  //
  //  Douglas Adams, Feb, 2005
  //
  result = 1.0F / (real32_T)sqrt((real_T)(((quat[0] * quat[0] + quat[1] * quat[1])
    + quat[2] * quat[2]) + quat[3] * quat[3]));
  quat[0] *= result;
  quat[1] *= result;
  quat[2] *= result;
  quat[3] *= result;
}

//
// File trailer for generated code.
//
// [EOF]
//
