//
// File: aaaahdbaecbaiecj_quatmult.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue Aug 22 16:52:08 2017
//
#include "rtwtypes.h"
#include "aaaahdbaecbaiecj_quatmult.h"

// Function for MATLAB Function: '<S16>/Compute Global positions of Handrail Features'
void aaaahdbaecbaiecj_quatmult(const real32_T p[4], const real32_T q[4],
  real32_T qOut[4])
{
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
  qOut[0] = (q[3] * p[0] + p[3] * q[0]) + (p[1] * q[2] - p[2] * q[1]);
  qOut[1] = (q[3] * p[1] + p[3] * q[1]) + (p[2] * q[0] - p[0] * q[2]);
  qOut[2] = (q[3] * p[2] + p[3] * q[2]) + (p[0] * q[1] - p[1] * q[0]);
  qOut[3] = p[3] * q[3] - ((p[0] * q[0] + p[1] * q[1]) + p[2] * q[2]);
}

//
// File trailer for generated code.
//
// [EOF]
//
