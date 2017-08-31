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
#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues> 

#include <vector>
#include <iostream>
//#include <ros/ros.h>

using namespace Eigen;

int compute_delta_state_and_cov(float* residual_in, int error_in, float* H_in,
        int H_rows, int H_cols, unsigned int augs_bitmask, float* R_mat_in, float* P_in,
        float* delta_state_out_out, float* P_out_out) {
  //struct timeval secs1, secs2;
  //gettimeofday(&secs1, 0);
  Map<MatrixXf> H_full(H_in, H_rows, H_cols);
  Map<VectorXf> residual_full(residual_in, H_rows);
  Map<MatrixXf> R_mat_full(R_mat_in, H_rows, H_rows);
  Map<MatrixXf> P(P_in, H_cols, H_cols);
  Map<VectorXf> delta_state_out(delta_state_out_out, H_cols);
  Map<MatrixXf> P_out(P_out_out, H_cols, H_cols);

  if (error_in) {
    delta_state_out.setZero();
    P_out = P;
    //ROS_INFO("compute_delta_state stop 1");
    return 1;
  }

  std::vector<int> used_augs;
  for (int i = 0; i < (H_cols - 15) / 6; i++)
    if (augs_bitmask & (1 << i))
      used_augs.push_back(i);
  VectorBlock<Map<VectorXf> > residual = residual_full.segment(0, used_augs.size() * 6);
  Block<Map<MatrixXf> > R_mat = R_mat_full.block(0, 0, used_augs.size() * 6, used_augs.size() * 6);
  Block<Map<MatrixXf> > reduced_H = H_full.block(0, 0, used_augs.size() * 6, used_augs.size() * 6);
  MatrixXf reduced_P(used_augs.size() * 6, used_augs.size() * 6);
  for (int i = 0; i < used_augs.size(); i++)
    for (int j = 0; j < used_augs.size(); j++)
      reduced_P.block<6, 6>(6 * i, 6 * j) = P.block<6, 6>(15 + 6 * used_augs[i], 15 + 6 * used_augs[j]);
  MatrixXf reduced_cols_P(H_cols, used_augs.size() * 6);
  for (int i = 0; i < used_augs.size(); i++)
    reduced_cols_P.block(0, 6 * i, H_cols, 6) = P.block(0, 15 + 6 * used_augs[i], H_cols, 6);
  //gettimeofday(&secs2, 0);
  //ROS_INFO("compute_delta_state %d create Ps", (((secs2.tv_sec - secs1.tv_sec) * 1000) + (secs2.tv_usec - secs1.tv_usec)));
  //secs1 = secs2;

  MatrixXf T(reduced_cols_P.rows(), reduced_H.rows()), S(reduced_H.rows(), reduced_H.rows());
  T.noalias() = reduced_cols_P * reduced_H.transpose();
  //gettimeofday(&secs2, 0);
  //ROS_INFO("compute_delta_state %d multiplies0", (((secs2.tv_sec - secs1.tv_sec) * 1000) + (secs2.tv_usec - secs1.tv_usec)));
  //secs1 = secs2;
  S.triangularView<Lower>() = reduced_H * reduced_P * reduced_H.transpose();
  S.triangularView<Lower>() += R_mat;
  //gettimeofday(&secs2, 0);
  //ROS_INFO("compute_delta_state %d multiplies1", (((secs2.tv_sec - secs1.tv_sec) * 1000) + (secs2.tv_usec - secs1.tv_usec)));
  //secs1 = secs2;

  LDLT<MatrixXf,Lower> chol(S);
  if (chol.info() != Success) {
    fprintf(stderr, "S not positive definite.\n");
    delta_state_out.setZero();
    P_out = P;
    //ROS_INFO("compute_delta_state stop 2");
    return 1;
  }
  //gettimeofday(&secs2, 0);
  //ROS_INFO("compute_delta_state %d cholesky", (((secs2.tv_sec - secs1.tv_sec) * 1000) + (secs2.tv_usec - secs1.tv_usec)));
  //secs1 = secs2;
  MatrixXf sinv = MatrixXf::Identity(S.cols(), S.cols());
  chol.solveInPlace(sinv);
  //gettimeofday(&secs2, 0);
  //ROS_INFO("compute_delta_state %d inverse", (((secs2.tv_sec - secs1.tv_sec) * 1000) + (secs2.tv_usec - secs1.tv_usec)));
  //secs1 = secs2;
  
  MatrixXf K;
  K.noalias() = T * sinv.selfadjointView<Lower>();
  delta_state_out.noalias() = K * residual;
  //gettimeofday(&secs2, 0);
  //ROS_INFO("compute_delta_state %d multiplies2", (((secs2.tv_sec - secs1.tv_sec) * 1000) + (secs2.tv_usec - secs1.tv_usec)));
  //secs1 = secs2;

  // F = I - KH; P = FPF' + KRK';
  // P = P - (KHP)' - KHP + K(HPH' + R)K'
  // P = P - KHP
  P_out.triangularView<Lower>() = P - K * T.transpose(); // P - KHP
  P_out.triangularView<StrictlyUpper>() = P_out.transpose();
  //P_out.noalias() = 0.5 * (P_out + P_out.transpose());

  //gettimeofday(&secs2, 0);
  //ROS_INFO("compute_delta_state %d %d stop 0", (((secs2.tv_sec - secs1.tv_sec) * 1000) + (secs2.tv_usec - secs1.tv_usec)), H_rows);
  return 0;
}
