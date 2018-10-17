//
// File: iecjopppiecjmgln_quatmult.cpp
//
// Code generated for Simulink model 'est_estimator'.
//
// Model version                  : 1.1142
// Simulink Coder version         : 8.11 (R2016b) 25-Aug-2016
// C/C++ source code generated on : Tue May  1 11:58:17 2018
//
#include "rtwtypes.h"
#include <math.h>
#include "iecjopppiecjmgln_quatmult.h"

// Function for MATLAB Function: '<S16>/Compute Global positions of Handrail Features'
void iecjopppiecjmgln_quatmult(const real32_T p[4], const real32_T q[4],
  real32_T qOut[4])
{
  real32_T c;

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
  qOut[0] = (q[3] * p[0] + p[3] * q[0]) + (p[1] * q[2] - p[2] * q[1]);
  qOut[1] = (q[3] * p[1] + p[3] * q[1]) + (p[2] * q[0] - p[0] * q[2]);
  qOut[2] = (q[3] * p[2] + p[3] * q[2]) + (p[0] * q[1] - p[1] * q[0]);
  qOut[3] = p[3] * q[3] - ((p[0] * q[0] + p[1] * q[1]) + p[2] * q[2]);

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
  //  Takes the root-sum square of each row of the matrix a
  c = (real32_T)sqrt((real_T)(((qOut[0] * qOut[0] + qOut[1] * qOut[1]) + qOut[2]
    * qOut[2]) + qOut[3] * qOut[3]));
  qOut[0] /= c;
  qOut[1] /= c;
  qOut[2] /= c;
  qOut[3] /= c;

  //  Normalize
}

//
// File trailer for generated code.
//
// [EOF]
//
