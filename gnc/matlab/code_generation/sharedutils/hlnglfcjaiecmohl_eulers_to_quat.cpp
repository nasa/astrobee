//
// File: hlnglfcjaiecmohl_eulers_to_quat.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Thu Aug 17 13:31:41 2017
//
#include "rtwtypes.h"
#include <math.h>
#include "hlnglfcjaiecmohl_eulers_to_quat.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void hlnglfcjaiecmohl_eulers_to_quat(real32_T r1, real32_T r2, real32_T r3,
  real32_T quat[4])
{
  real32_T cx;
  real32_T sx;
  real32_T cy;
  real32_T sy;
  real32_T cz;
  real32_T sz;

  //  Convert Euler angles to quaternions.  Assumes 3-2-1 rotation sequence for
  //  euler angles.
  //  From: Fundamentals of Spacecraft Attitude Determination and Control, F.
  //  Landix Markley and John L. Crassisdis. Table: B.5
  //
  //  Jesse. C. Fusco           jesse.c.fusco@nasa.gov
  cx = (real32_T)cos((real_T)(r3 / 2.0F));
  sx = (real32_T)sin((real_T)(r3 / 2.0F));
  cy = (real32_T)cos((real_T)(r2 / 2.0F));
  sy = (real32_T)sin((real_T)(r2 / 2.0F));
  cz = (real32_T)cos((real_T)(r1 / 2.0F));
  sz = (real32_T)sin((real_T)(r1 / 2.0F));
  quat[0] = cx * cy * sz - sx * sy * cz;
  quat[1] = cx * sy * cz + sx * cy * sz;
  quat[2] = sx * cy * cz - cx * sy * sz;
  quat[3] = cx * cy * cz + sx * sy * sz;

  //  function quat = eulers_to_quat(rz, ry, rx)
  //
  //  quat = zeros(size(rz, 1), 4, 'like', rx);
  //
  //  quat(:,1) = cos(rz/2).*cos(ry/2).*sin(rx/2) - sin(rz/2).*sin(ry/2).*cos(rx/2); 
  //
  //  quat(:,2) = cos(rz/2).*sin(ry/2).*cos(rx/2) + sin(rz/2).*cos(ry/2).*sin(rx/2); 
  //
  //  quat(:,3) = sin(rz/2).*cos(ry/2).*cos(rx/2) - cos(rz/2).*sin(ry/2).*sin(rx/2); 
  //
  //  quat(:,4) = cos(rz/2).*cos(ry/2).*cos(rx/2) + sin(rz/2).*sin(ry/2).*sin(rx/2); 
  //
  //  %Normalize the quaternion and make the scalar term positive
  //  idx = find(quat(:,4) < 0);
  //  quat(idx,:) = quat(idx,:).*-1;
  //  quat = diag(1./rssrow(quat))*quat;
}

//
// File trailer for generated code.
//
// [EOF]
//
