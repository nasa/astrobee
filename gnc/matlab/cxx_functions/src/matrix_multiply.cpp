/* Copyright (c) 2017, United States Government, as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * 
 * All rights reserved.
 * 
 * The Astrobee platform is licensed under the Apache License, Version 2.0
 * (the "License"); you may not use this file except in compliance with the
 * License. You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations
 * under the License.
 */

#include <Eigen/Dense>

#include <iostream>

using namespace Eigen;

void matrix_multiply(float* m1, int m1_rows, int m1_cols, float* m2, int m2_rows, int m2_cols, float* m_out) {
  Map<MatrixXf> A(m1, m1_rows, m1_cols);
  Map<MatrixXf> B(m2, m2_rows, m2_cols);
  Map<MatrixXf> out(m_out, m1_rows, m2_cols);

  out.noalias() = A * B;
}

