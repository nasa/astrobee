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
#include <optimization_common/utilities.h>
#include <localization_common/logger.h>

namespace optimization_common {
Eigen::Matrix<double, 6, 1> VectorFromIsometry3d(const Eigen::Isometry3d& isometry_3d) {
  // Isometry3d linear().data() returns the data pointer to the full Isometry3d matrix rather than just the rotation
  const Eigen::Matrix3d rotation = isometry_3d.linear();
  Eigen::Matrix<double, 6, 1> isometry_3d_vector;
  ceres::RotationMatrixToAngleAxis(rotation.data(), &(isometry_3d_vector.data()[0]));
  isometry_3d_vector.block<3, 1>(3, 0) = isometry_3d.translation();
  return isometry_3d_vector;
}

Eigen::Matrix<double, 7, 1> VectorFromAffine3d(const Eigen::Affine3d& affine_3d) {
  Eigen::Matrix3d rotation;
  Eigen::Matrix3d scale_matrix;
  affine_3d.computeRotationScaling(&rotation, &scale_matrix);
  // Assumes uniform scaling, which is the case for Affine3d
  const double scale = scale_matrix(0, 0);
  Eigen::Matrix<double, 7, 1> affine_3d_vector;
  ceres::RotationMatrixToAngleAxis(rotation.data(), &(affine_3d_vector.data()[0]));
  affine_3d_vector.block<3, 1>(3, 0) = affine_3d.translation();
  affine_3d_vector(6, 0) = scale;
  return affine_3d_vector;
}

Eigen::Isometry3d Isometry3d(const Eigen::Matrix<double, 6, 1>& isometry_vector) {
  return Isometry3(isometry_vector.data());
}

Eigen::Affine3d Affine3d(const Eigen::Matrix<double, 7, 1>& affine_vector) { return Affine3(affine_vector.data()); }

Eigen::Matrix3d Intrinsics(const Eigen::Vector2d& focal_lengths, const Eigen::Vector2d& principal_points) {
  Eigen::Matrix3d intrinsics(Eigen::Matrix3d::Identity());
  intrinsics(0, 0) = focal_lengths(0);
  intrinsics(1, 1) = focal_lengths(1);
  intrinsics(0, 2) = principal_points(0);
  intrinsics(1, 2) = principal_points(1);
  return intrinsics;
}

void AddParameterBlock(const int num_parameters, double* const parameters, ceres::Problem& problem,
                       const bool set_constant) {
  problem.AddParameterBlock(parameters, num_parameters);
  if (set_constant) problem.SetParameterBlockConstant(parameters);
}

double ResidualNorm(const std::vector<double>& residual, const int index, const int residual_size) {
  double norm = 0;
  for (int i = 0; i < residual_size; ++i) {
    norm += residual[index * residual_size + i];
  }
  return norm;
}

// Assumes each residual is the same size
void CheckResiduals(const int residual_size, ceres::Problem& problem, const double outlier_threshold) {
  double total_cost = 0.0;
  std::vector<double> residual_norms;
  std::vector<double> residuals;
  ceres::Problem::EvaluateOptions eval_options;
  eval_options.num_threads = 1;
  eval_options.apply_loss_function = false;
  problem.Evaluate(eval_options, &total_cost, &residuals, NULL, NULL);
  double total_cost_huber = 0.0;

  std::vector<double> residual_norms_huber;
  std::vector<double> residuals_huber;
  ceres::Problem::EvaluateOptions eval_options_huber;
  eval_options_huber.num_threads = 1;
  eval_options_huber.apply_loss_function = true;
  problem.Evaluate(eval_options_huber, &total_cost_huber, &residuals_huber, NULL, NULL);
  const int num_residuals = residuals.size() / residual_size;
  double mean_residual = 0.0;
  int huber_outliers = 0;
  for (int index = 0; index < num_residuals; ++index) {
    double norm = ResidualNorm(residuals, index, residual_size);
    mean_residual += norm;
    residual_norms.push_back(norm);

    double norm_huber = ResidualNorm(residuals_huber, index, residual_size);
    residual_norms_huber.push_back(norm_huber);
    if (norm_huber / norm < outlier_threshold) {
      ++huber_outliers;
    }
  }
  mean_residual /= num_residuals;
  std::sort(residual_norms.begin(), residual_norms.end());
  LogInfo("Residual min, mean, median and max error: " << residual_norms[0] << ' ' << mean_residual << ' '
                                                       << residual_norms[residual_norms.size() / 2] << ' '
                                                       << residual_norms.back());
  LogInfo("Huber outlier percentage: " << huber_outliers / static_cast<double>(residuals.size()));
}
}  // namespace optimization_common
