//
// File: fkfcbaiengdjgdje_quaternion_to_rotation.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1139
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Thu Aug 31 10:20:56 2017
//
#include "rtwtypes.h"
#include "fkfcbaiengdjgdje_quaternion_to_rotation.h"

// Function for MATLAB Function: '<S17>/Compute Residual and H'
void fkfcbaiengdjgdje_quaternion_to_rotation(const real32_T q[4], real32_T R[9])
{
  real32_T S[9];
  static const int8_T b[9] = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };

  real32_T q_0;
  real32_T q_1;
  real32_T q_2[12];
  real32_T q_3[12];
  int32_T i;
  int32_T i_0;

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
  //  construct rotation matrix from quaternion
  //  From Zack and Brian's ekf.m  Used as a nested function in the optical
  //  flow update
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
  S[0] = 0.0F;
  S[3] = -q[2];
  S[6] = q[1];
  S[1] = q[2];
  S[4] = 0.0F;
  S[7] = -q[0];
  S[2] = -q[1];
  S[5] = q[0];
  S[8] = 0.0F;
  q_0 = q[3];
  q_1 = q[3];
  for (i = 0; i < 3; i++) {
    q_2[(int32_T)(3 * i)] = q_0 * (real32_T)b[i] + S[i];
    q_3[(int32_T)(i << 2)] = (real32_T)b[(int32_T)(3 * i)] * q_1 - S[(int32_T)(3
      * i)];
    q_2[(int32_T)(1 + (int32_T)(3 * i))] = (real32_T)b[(int32_T)(i + 3)] * q_0 +
      S[(int32_T)(i + 3)];
    q_3[(int32_T)(1 + (int32_T)(i << 2))] = (real32_T)b[(int32_T)((int32_T)(3 *
      i) + 1)] * q_1 - S[(int32_T)((int32_T)(3 * i) + 1)];
    q_2[(int32_T)(2 + (int32_T)(3 * i))] = (real32_T)b[(int32_T)(i + 6)] * q_0 +
      S[(int32_T)(i + 6)];
    q_3[(int32_T)(2 + (int32_T)(i << 2))] = (real32_T)b[(int32_T)((int32_T)(3 *
      i) + 2)] * q_1 - S[(int32_T)((int32_T)(3 * i) + 2)];
    q_2[(int32_T)(9 + i)] = -q[i];
    q_3[(int32_T)(3 + (int32_T)(i << 2))] = -q[i];
  }

  for (i = 0; i < 3; i++) {
    for (i_0 = 0; i_0 < 3; i_0++) {
      R[(int32_T)(i_0 + (int32_T)(3 * i))] = 0.0F;
      R[(int32_T)(i_0 + (int32_T)(3 * i))] += q_3[(int32_T)(i << 2)] * q_2[i_0];
      R[(int32_T)(i_0 + (int32_T)(3 * i))] += q_3[(int32_T)((int32_T)(i << 2) +
        1)] * q_2[(int32_T)(i_0 + 3)];
      R[(int32_T)(i_0 + (int32_T)(3 * i))] += q_3[(int32_T)((int32_T)(i << 2) +
        2)] * q_2[(int32_T)(i_0 + 6)];
      R[(int32_T)(i_0 + (int32_T)(3 * i))] += q_3[(int32_T)((int32_T)(i << 2) +
        3)] * q_2[(int32_T)(i_0 + 9)];
    }
  }
}

//
// File trailer for generated code.
//
// [EOF]
//
