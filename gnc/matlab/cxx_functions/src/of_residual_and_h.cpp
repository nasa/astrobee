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
#include <Eigen/Jacobi>
#include <Eigen/SVD>

#include <vector>
#include <iostream>
#include <limits>
//#include <ros/ros.h>

using Eigen::MatrixXd;
using namespace Eigen;
//using namespace std;

void apply_givens(MatrixXf & H_f_j, MatrixXf & H_x_j, VectorXf & r_j, int row1, int row2, int col,
    int H_x_j_col_start, int H_x_j_num_cols) {
  JacobiRotation<float> g;
  g.makeGivens(H_f_j(row1, col), H_f_j(row2, col));
  H_f_j.applyOnTheLeft(row1, row2, g.adjoint());
  H_x_j.block(0, H_x_j_col_start, H_x_j.rows(), H_x_j_num_cols).applyOnTheLeft(row1, row2, g.adjoint());
  r_j.applyOnTheLeft(row1, row2, g.adjoint());
}

void apply_givens(MatrixXf & H, VectorXf & r, int row1, int row2, int col) {
  JacobiRotation<float> g;
  g.makeGivens(H(row1, col), H(row2, col));
  H.block(0, col, H.rows(), H.cols() - col).applyOnTheLeft(row1, row2, g.adjoint());
  r.applyOnTheLeft(row1, row2, g.adjoint());
}

Matrix3f skew(const Vector3f & v) {
  Matrix3f ret;
  ret << 0, -v(2), v(1),
         v(2), 0, -v(0),
         -v(1), v(0), 0;
  return ret;
}

// S is lower triangular
int pinv(const MatrixXf & S, MatrixXf & out) {
  LDLT<MatrixXf,Lower> chol(S);
  if (chol.info() != Success) {
    return 1;
  }
  out = MatrixXf::Identity(out.rows(), out.cols());
  chol.solveInPlace(out);
  return 0;
}

int of_residual_and_h(float* of_measured_p, float* global_points_p, float* camera_tf_global_p,
          int* valid_p, int num_points, int ase_of_num_aug, int ase_of_num_features,
          float tun_ase_mahal_distance_max, float ase_of_r_mag, float ase_inv_focal_length, float ase_distortion, float* P_p,
          float* r_out_p, float* H_out_p, unsigned int* augs_bitmask, unsigned char* num_of_tracks_out, float* mahal_dists_out_p, float* R_out_p)
{
  //struct timeval secs1, secs2;
  //gettimeofday(&secs1, 0);
  int covariance_size = 21 + 6 * ase_of_num_aug;
  Map<VectorXf> of_measured(of_measured_p, ase_of_num_features * 2 * ase_of_num_aug);
  Map<MatrixXf> global_points(global_points_p, 4, ase_of_num_features);
  Map<MatrixXf> camera_tf_global(camera_tf_global_p, 4 * ase_of_num_aug, 4);
  Map<MatrixXi> valid(valid_p, ase_of_num_features, ase_of_num_aug);
  Map<MatrixXf> P(P_p, covariance_size, covariance_size);
  Map<VectorXf> r_out(r_out_p, 6 * ase_of_num_aug);
  Map<MatrixXf> H_out(H_out_p, 6 * ase_of_num_aug, covariance_size);
  Map<MatrixXf> R_out(R_out_p, 6 * ase_of_num_aug, 6 * ase_of_num_aug);
  Map<VectorXf> mahal_dists_out(mahal_dists_out_p, ase_of_num_features);
  
  r_out.setZero();
  H_out.setZero();
  R_out.setZero();
  *num_of_tracks_out = 0;
  *augs_bitmask = 0;
  
  // figure out which augmentations were used (this helps with speed)
  std::vector<int> used_augs;
  for (int i = 0; i < ase_of_num_aug; i++) {
    if ((valid.col(i).array() > 0).any()) {
      used_augs.push_back(i);
      *augs_bitmask |= 2 << i;
    }
  }
  if (used_augs.size() < 3) {
    r_out(1) = 1;
    return 1;
  }

  MatrixXf reduced_P(used_augs.size() * 6, used_augs.size() * 6);
  for (int i = 0; i < used_augs.size(); i++)
    for (int j = 0; j < used_augs.size(); j++)
      reduced_P.block<6, 6>(6 * i, 6 * j) = P.block<6, 6>(21 + 6 * i, 21 + 6 * j);

  VectorXf r((2 * used_augs.size() - 3) * ase_of_num_features);
  MatrixXf H((2 * used_augs.size() - 3) * ase_of_num_features, 6 * used_augs.size());
  H.setZero();
  //gettimeofday(&secs2, 0);
  //ROS_INFO("of_residual_and_h %d cast", (((secs2.tv_sec - secs1.tv_sec) * 1000) + (secs2.tv_usec - secs1.tv_usec)));
  //secs1 = secs2;

  int cur_row = 0;
  int num_tracks = 0;
  for (int j = 0; j < num_points; j++) {
    int valid_augs = valid.row(j).count();
    if (valid_augs < 2)
      continue;
    int rows = valid_augs * 2;
    VectorXf r_j(rows);
    MatrixXf H_f_j(rows, 3);
    MatrixXf H_x_j(rows, 6 * used_augs.size());
    H_x_j.setZero();
    int aug_ind = 0;
    // construct r_j, H_x_j, and H_f_j
    for (int i = 0; i < valid_augs; i++) {
      while (!valid(j, used_augs[aug_ind]))
        aug_ind++;
      assert(aug_ind < used_augs.size());
      
      Matrix<float, 3, 4> c_tf_g = camera_tf_global.block<3, 4>(4 * used_augs[aug_ind], 0);
      Vector3f camera_landmark = c_tf_g * global_points.col(j);
      Vector2f z_est(camera_landmark(0), camera_landmark(1));
      z_est *= 1.0 / camera_landmark(2);
 
      Matrix<float, 2, 3> prefix;
      prefix << 1, 0, -z_est(0), 0, 1, -z_est(1);
      prefix *= 1.0 / camera_landmark(2);
      Matrix<float, 2, 3> H_theta_ji = prefix * skew(camera_landmark);
      Matrix<float, 2, 3> H_p_ji = -prefix * c_tf_g.block<3, 3>(0, 0);

      r_j.segment<2>(2 * i) = of_measured.segment<2>(2 * ase_of_num_aug * j + 2 * used_augs[aug_ind]) - z_est;
      H_x_j.block<2, 3>(2 * i, aug_ind * 6)     = H_theta_ji;
      H_x_j.block<2, 3>(2 * i, aug_ind * 6 + 3) = H_p_ji;
      H_f_j.block<2, 3>(2 * i, 0) = -H_p_ji;
      aug_ind++;
    }

    // do givens rotations to make H_f_j upper triangular
    // optimized version only if we observed all 4 augmentations
    if (used_augs.size() == valid_augs && valid_augs == 4) {
      // first make each block of 2 rows x x x; 0 x x
      for (int i = 0; i < rows; i+=2)
        apply_givens(H_f_j, H_x_j, r_j, i, i + 1, 0, 6 * (i/2), 6);
      // now make each block of 4 rows x x x; 0 x x; 0 0 x; 0 0 0
      for (int i = 0; i < rows; i+=4) {
        apply_givens(H_f_j, H_x_j, r_j, i    , i + 2, 0, 6 * (i/4), 12);
        apply_givens(H_f_j, H_x_j, r_j, i + 1, i + 2, 1, 6 * (i/4), 12);
        apply_givens(H_f_j, H_x_j, r_j, i + 1, i + 3, 1, 6 * (i/4), 12);
        apply_givens(H_f_j, H_x_j, r_j, i + 2, i + 3, 2, 6 * (i/4), 12);
      }
      // finally eliminate the second block of four
      apply_givens(H_f_j, H_x_j, r_j, 0, 4, 0, 0, 24);
      apply_givens(H_f_j, H_x_j, r_j, 1, 4, 1, 0, 24);
      apply_givens(H_f_j, H_x_j, r_j, 1, 5, 1, 0, 24);
      apply_givens(H_f_j, H_x_j, r_j, 2, 4, 2, 0, 24);
      apply_givens(H_f_j, H_x_j, r_j, 2, 5, 2, 0, 24);
      apply_givens(H_f_j, H_x_j, r_j, 2, 6, 2, 0, 24);
    } else {
      // normal version
      for (int col = 0; col < 3; col++)
       for (int row = rows - 1; row > col; row--)
         apply_givens(H_f_j, H_x_j, r_j, row - 1, row, col, 0, H_x_j.cols());
    }
      
    // then delete the top three rows which are nonzero in H_f_j
    Block<MatrixXf> H_x_j_reduced = H_x_j.block(3, 0, rows - 3, H_x_j.cols());
    VectorBlock<VectorXf> r_j_reduced = r_j.segment(3, rows - 3);

    // now check the mahalanobis distance
    MatrixXf S(H_x_j_reduced.rows(), H_x_j_reduced.rows());
    S.triangularView<Lower>() = H_x_j_reduced * reduced_P * H_x_j_reduced.transpose();
    for (int i = 0; i < S.rows(); i++)
      S(i, i) += (ase_inv_focal_length * ase_of_r_mag) * (ase_inv_focal_length * ase_of_r_mag);
    MatrixXf S_inv(S.rows(), S.cols());
    if (pinv(S, S_inv)) {
      fprintf(stderr, "Failed to take inverse for Mahalanobis distance.\n");
      continue;
    }
    float mahal_dist = sqrt(r_j_reduced.transpose() * S_inv * r_j_reduced);
    mahal_dists_out(j) = mahal_dist;
    if (mahal_dist > tun_ase_mahal_distance_max)
      continue;

    // add to full r and H
    r.segment(cur_row, r_j_reduced.rows()) = r_j_reduced;
    H.block(cur_row, 0, r_j_reduced.rows(), H_x_j_reduced.cols()) = H_x_j_reduced;
    cur_row += r_j_reduced.rows();
    num_tracks++;
  }
  //gettimeofday(&secs2, 0);
  //ROS_INFO("of_residual_and_h %d r and h", (((secs2.tv_sec - secs1.tv_sec) * 1000) + (secs2.tv_usec - secs1.tv_usec)));
  //secs1 = secs2;

  // set the rest to uninitialized
  for (int j = num_points; j < ase_of_num_features; j++)
    mahal_dists_out(j) = std::numeric_limits<double>::quiet_NaN();

  r.conservativeResize(cur_row);
  H.conservativeResize(cur_row, NoChange);
  if (r.rows() < 6 * used_augs.size()) {
    r_out(1) = 1;
    //ROS_INFO("of_residual_and_h end 1");
    return 1;
  }

  *num_of_tracks_out = static_cast<unsigned char>(num_tracks);
  //gettimeofday(&secs2, 0);
  //ROS_INFO("of_residual_and_h %d output", (((secs2.tv_sec - secs1.tv_sec) * 1000) + (secs2.tv_usec - secs1.tv_usec)));
  //secs1 = secs2;

  // now we do the compression
  // do givens rotations to make H upper triangular and remove as many rows as possible
  for (int col = 0; col < H.cols(); col++)
    for (int row = H.rows() - 1; row > col; row--)
      apply_givens(H, r, col, row, col);
  //gettimeofday(&secs2, 0);
  //ROS_INFO("of_residual_and_h %d compress", (((secs2.tv_sec - secs1.tv_sec) * 1000) + (secs2.tv_usec - secs1.tv_usec)));
  //secs1 = secs2;

  r_out.segment(0, 6 * used_augs.size()) = r.segment(0, 6 * used_augs.size());
  H_out.block(0, 0, 6 * used_augs.size(), 6 * used_augs.size()) = H.block(0, 0, 6 * used_augs.size(), 6 * used_augs.size());
  for (int i = 0; i < 6 * ase_of_num_aug; i++)
    R_out(i, i) = (ase_of_r_mag * ase_inv_focal_length) * (ase_of_r_mag * ase_inv_focal_length);
  //gettimeofday(&secs2, 0);
  //ROS_INFO("of_residual_and_h %d end 0", (((secs2.tv_sec - secs1.tv_sec) * 1000) + (secs2.tv_usec - secs1.tv_usec)));

  return 0;
}

