//
// File: pphlpphdeknodjek_quat_rotation_vec.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Aug 22 16:52:08 2017
//
#include "rtwtypes.h"
#include "aaaahdbaecbaiecj_quatmult.h"
#include "pphlpphdeknodjek_quat_rotation_vec.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void pphlpphdeknodjek_quat_rotation_vec(real_T vector[3], const real32_T Q[4],
  real32_T vec_out[3])
{
  real32_T V[4];
  real32_T Q_0[4];
  real32_T tmp[4];

  //
  //  Q = q_a2b, rotation represents a vector rotating from frame b2a
  //  q_out = quat_rotation(vector, Q2)
  //
  //  rotates a row-vector of 3-vectors by a row-vector of quaternion Q2
  //
  //  From: Indirect Kalman Filter for 3D Attitude Estimation: A tutorial for Quaternion Algebra 
  //  Equation below is from Eq. 77, with the the quaternions inverted because
  //  our quat multiplication convention has the reverse order from the
  //  mulitplication used in the paper
  //
  //  Check to see if the vector is a single row
  //  Vec_out = Q * [V 0]' * Q^-1
  //  Quaternion Multiplication:
  //  Uses Hamilton's convention where the rotation order is left to right,
  //  q1*q2 corresponds to the first rotation q1, followed by the second
  //  rotation q2.
  //
  //  Fundamentals of Spacecraft Attitude Determination and Control,
  //  F. Landis Markley and John L. Crassidis
  //  Equation: 2.82b
  //
  //  Jesse C. Fusco            jesse.c.fusco@nasa.gov
  V[0] = (0.0F * Q[0] + Q[3] * (real32_T)vector[0]) + (Q[1] * (real32_T)vector[2]
    - Q[2] * (real32_T)vector[1]);
  V[1] = (0.0F * Q[1] + Q[3] * (real32_T)vector[1]) + (Q[2] * (real32_T)vector[0]
    - Q[0] * (real32_T)vector[2]);
  V[2] = (0.0F * Q[2] + Q[3] * (real32_T)vector[2]) + (Q[0] * (real32_T)vector[1]
    - Q[1] * (real32_T)vector[0]);
  V[3] = Q[3] * 0.0F - ((Q[0] * (real32_T)vector[0] + Q[1] * (real32_T)vector[1])
                        + Q[2] * (real32_T)vector[2]);
  Q_0[0] = -Q[0];
  Q_0[1] = -Q[1];
  Q_0[2] = -Q[2];
  Q_0[3] = Q[3];
  aaaahdbaecbaiecj_quatmult(V, Q_0, tmp);
  vec_out[0] = tmp[0];
  vec_out[1] = tmp[1];
  vec_out[2] = tmp[2];
}

//
// File trailer for generated code.
//
// [EOF]
//
