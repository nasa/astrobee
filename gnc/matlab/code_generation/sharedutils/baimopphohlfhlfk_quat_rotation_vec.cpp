//
// File: baimopphohlfhlfk_quat_rotation_vec.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Wed May 17 14:42:31 2017
//
#include "rtwtypes.h"
#include "baimopphohlfhlfk_quat_rotation_vec.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void baimopphohlfhlfk_quat_rotation_vec(const real_T vector[3], const real32_T
  Q2[4], real32_T p_new[3])
{
  real32_T qOut_idx_0;
  real32_T qOut_idx_1;
  real32_T qOut_idx_2;
  real32_T qOut_idx_3;

  //
  //  Q2 = q_a2b, rotation represents a vector rotating from frame b2a
  //  p_new = quat_rotation(vector, Q2)
  //
  //  rotates a vector of 3-vectors by a vector of quaternion Q2
  //
  //  if Q2 represents the rotation from frame 1 to frame 2 OR
  //  equivalently, the rotation of frame 2 with respect to
  //  frame 1, then rotating a vector p_expr_in_2 by quaternion
  //  q changes the vector to p_expr_in_1
  //
  //  p_new = vector part of Q2 x Q1 x Q2*
  //  where Q2* = [-1 -1 -1 1]' .* Q2
  qOut_idx_0 = ((0.0F * Q2[0] + (real32_T)vector[2] * Q2[1]) - (real32_T)vector
                [1] * Q2[2]) + (real32_T)vector[0] * Q2[3];
  qOut_idx_1 = (((real32_T)-vector[2] * Q2[0] + 0.0F * Q2[1]) + (real32_T)
                vector[0] * Q2[2]) + (real32_T)vector[1] * Q2[3];
  qOut_idx_2 = (((real32_T)vector[1] * Q2[0] - (real32_T)vector[0] * Q2[1]) +
                0.0F * Q2[2]) + (real32_T)vector[2] * Q2[3];
  qOut_idx_3 = (((real32_T)-vector[0] * Q2[0] - (real32_T)vector[1] * Q2[1]) -
                (real32_T)vector[2] * Q2[2]) + 0.0F * Q2[3];

  //            q_mid = [    q2(4)  q2(3) -q2(2)  q2(1);
  //             -q2(3)  q2(4)  q2(1)  q2(2);
  //              q2(2) -q2(1)  q2(4)  q2(3);
  //             -q2(1) -q2(2) -q2(3)  q2(4)];
  //            q_mid = [    q2(4)  q2(3) -q2(2)  q2(1);
  //             -q2(3)  q2(4)  q2(1)  q2(2);
  //              q2(2) -q2(1)  q2(4)  q2(3);
  //             -q2(1) -q2(2) -q2(3)  q2(4)];
  p_new[0] = ((Q2[3] * qOut_idx_0 + -Q2[2] * qOut_idx_1) - -Q2[1] * qOut_idx_2)
    + -Q2[0] * qOut_idx_3;
  p_new[1] = ((-(-Q2[2]) * qOut_idx_0 + Q2[3] * qOut_idx_1) + -Q2[0] *
              qOut_idx_2) + -Q2[1] * qOut_idx_3;
  p_new[2] = ((-Q2[1] * qOut_idx_0 - -Q2[0] * qOut_idx_1) + Q2[3] * qOut_idx_2)
    + -Q2[2] * qOut_idx_3;
}

//
// File trailer for generated code.
//
// [EOF]
//
