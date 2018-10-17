//
// File: gdjmmglnmgdjlfkf_quat_rotation_vec.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 11:58:17 2018
//
#include "rtwtypes.h"
#include "gdjmmglnmgdjlfkf_quat_rotation_vec.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void gdjmmglnmgdjlfkf_quat_rotation_vec(real_T vector[3], const real32_T Q[4],
  real_T vec_out[3])
{
  int32_T jcol;
  int8_T I[9];
  real32_T b_a;
  real32_T y;
  real32_T tmp[9];
  real32_T Q_0[9];
  real32_T b_a_0[9];

  //  Copyright (c) 2017, United States Government, as represented by the
  //  Administrator of the National Aeronautics and Space Administration.
  //
  //  All rights reserved.
  //
  //  The Astrobee platform is licensed under the Apache License, Version 2.0
  //  (the "License"); you may not use this file except in compliance with the
  //  License. You may obtain a copy of the License at
  //
  //      http://www.apache.org/licenses/LICENSE-2.0
  //
  //  Unless required by applicable law or agreed to in writing, software
  //  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
  //  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
  //  License for the specific language governing permissions and limitations
  //  under the License.
  //
  //  Q = q_a2b, rotation represents a vector rotating from frame b2a
  //  q_out = quat_rotation(vector, Q2)
  //
  //  rotates a row-vector of 3-vectors by a row-vector of quaternion Q2
  //
  //  We accomplish this rotation by using DCMs as an intermediate step
  //
  //  Check to see if the vector is a single row
  //  Copyright (c) 2017, United States Government, as represented by the
  //  Administrator of the National Aeronautics and Space Administration.
  //
  //  All rights reserved.
  //
  //  The Astrobee platform is licensed under the Apache License, Version 2.0
  //  (the "License"); you may not use this file except in compliance with the
  //  License. You may obtain a copy of the License at
  //
  //      http://www.apache.org/licenses/LICENSE-2.0
  //
  //  Unless required by applicable law or agreed to in writing, software
  //  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
  //  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
  //  License for the specific language governing permissions and limitations
  //  under the License.
  //  Convert quaterion to a DCM.  DCM will rotate a vector V by Q_A2B from
  //  reference frame A to reference frame B.
  //
  //  Indirect Kalman Filter for 3D Attitude Estimation: A Tutorial for Aquaternion Algebra. 
  //  Nikolas Trawny and Stergios I. Roumeliotis
  //  Equation 78
  for (jcol = 0; jcol < 9; jcol++) {
    I[jcol] = 0;
  }

  b_a = Q[3] * Q[3] * 2.0F - 1.0F;
  y = 2.0F * Q[3];

  //  Copyright (c) 2017, United States Government, as represented by the
  //  Administrator of the National Aeronautics and Space Administration.
  //
  //  All rights reserved.
  //
  //  The Astrobee platform is licensed under the Apache License, Version 2.0
  //  (the "License"); you may not use this file except in compliance with the
  //  License. You may obtain a copy of the License at
  //
  //      http://www.apache.org/licenses/LICENSE-2.0
  //
  //  Unless required by applicable law or agreed to in writing, software
  //  distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
  //  WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
  //  License for the specific language governing permissions and limitations
  //  under the License.
  //  construct swew matrix from a vector
  //  From Zack and Brian's ekf.m  Used as a nested function in the optical
  //  flow update
  tmp[0] = 0.0F;
  tmp[3] = -Q[2];
  tmp[6] = Q[1];
  tmp[1] = Q[2];
  tmp[4] = 0.0F;
  tmp[7] = -Q[0];
  tmp[2] = -Q[1];
  tmp[5] = Q[0];
  tmp[8] = 0.0F;
  for (jcol = 0; jcol < 3; jcol++) {
    I[(int32_T)(jcol + (int32_T)(3 * jcol))] = 1;
    Q_0[jcol] = Q[jcol] * Q[0];
    Q_0[(int32_T)(jcol + 3)] = Q[jcol] * Q[1];
    Q_0[(int32_T)(jcol + 6)] = Q[jcol] * Q[2];
  }

  for (jcol = 0; jcol < 3; jcol++) {
    b_a_0[(int32_T)(3 * jcol)] = ((real32_T)I[(int32_T)(3 * jcol)] * b_a - tmp
      [(int32_T)(3 * jcol)] * y) + Q_0[(int32_T)(3 * jcol)] * 2.0F;
    b_a_0[(int32_T)(1 + (int32_T)(3 * jcol))] = ((real32_T)I[(int32_T)((int32_T)
      (3 * jcol) + 1)] * b_a - tmp[(int32_T)((int32_T)(3 * jcol) + 1)] * y) +
      Q_0[(int32_T)((int32_T)(3 * jcol) + 1)] * 2.0F;
    b_a_0[(int32_T)(2 + (int32_T)(3 * jcol))] = ((real32_T)I[(int32_T)((int32_T)
      (3 * jcol) + 2)] * b_a - tmp[(int32_T)((int32_T)(3 * jcol) + 2)] * y) +
      Q_0[(int32_T)((int32_T)(3 * jcol) + 2)] * 2.0F;
  }

  for (jcol = 0; jcol < 3; jcol++) {
    vec_out[jcol] = (real_T)((b_a_0[(int32_T)(jcol + 3)] * (real32_T)vector[1] +
      b_a_0[jcol] * (real32_T)vector[0]) + b_a_0[(int32_T)(jcol + 6)] *
      (real32_T)vector[2]);
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
