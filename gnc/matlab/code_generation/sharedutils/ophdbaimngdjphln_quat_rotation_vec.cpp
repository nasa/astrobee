//
// File: ophdbaimngdjphln_quat_rotation_vec.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Thu Aug 31 10:20:56 2017
//
#include "rtwtypes.h"
#include "ophdbaimngdjphln_quat_rotation_vec.h"

// Function for MATLAB Function: '<S16>/Compute Residual and H'
void ophdbaimngdjphln_quat_rotation_vec(real_T vector[3], const real32_T Q[4],
  real32_T vec_out[3])
{
  real32_T qOut_idx_3;
  real32_T qOut_idx_1;
  real32_T qOut_idx_2;
  real32_T qOut_idx_0;

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
  //  From: Indirect Kalman Filter for 3D Attitude Estimation: A tutorial for Quaternion Algebra 
  //  Equation below is from Eq. 77, with the the quaternions inverted because
  //  our quat multiplication convention has the reverse order from the
  //  mulitplication used in the paper
  //
  //  Check to see if the vector is a single row
  //  Vec_out = Q * [V 0]' * Q^-1
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
  //  Quaternion Multiplication:
  //  Uses Hamilton's convention where the rotation order is left to right,
  //  q1*q2 corresponds to the first rotation q1, followed by the second
  //  rotation q2.
  //
  //  Fundamentals of Spacecraft Attitude Determination and Control,
  //  F. Landis Markley and John L. Crassidis
  //  Equation: 2.82b
  qOut_idx_0 = (0.0F * Q[0] + Q[3] * (real32_T)vector[0]) + (Q[1] * (real32_T)
    vector[2] - Q[2] * (real32_T)vector[1]);
  qOut_idx_1 = (0.0F * Q[1] + Q[3] * (real32_T)vector[1]) + (Q[2] * (real32_T)
    vector[0] - Q[0] * (real32_T)vector[2]);
  qOut_idx_2 = (0.0F * Q[2] + Q[3] * (real32_T)vector[2]) + (Q[0] * (real32_T)
    vector[1] - Q[1] * (real32_T)vector[0]);
  qOut_idx_3 = Q[3] * 0.0F - ((Q[0] * (real32_T)vector[0] + Q[1] * (real32_T)
    vector[1]) + Q[2] * (real32_T)vector[2]);

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
  //  Quaternion Multiplication:
  //  Uses Hamilton's convention where the rotation order is left to right,
  //  q1*q2 corresponds to the first rotation q1, followed by the second
  //  rotation q2.
  //
  //  Fundamentals of Spacecraft Attitude Determination and Control,
  //  F. Landis Markley and John L. Crassidis
  //  Equation: 2.82b
  vec_out[0] = (Q[3] * qOut_idx_0 + qOut_idx_3 * -Q[0]) + (qOut_idx_1 * -Q[2] -
    qOut_idx_2 * -Q[1]);
  vec_out[1] = (Q[3] * qOut_idx_1 + qOut_idx_3 * -Q[1]) + (qOut_idx_2 * -Q[0] -
    qOut_idx_0 * -Q[2]);
  vec_out[2] = (Q[3] * qOut_idx_2 + qOut_idx_3 * -Q[2]) + (qOut_idx_0 * -Q[1] -
    qOut_idx_1 * -Q[0]);
}

//
// File trailer for generated code.
//
// [EOF]
//
