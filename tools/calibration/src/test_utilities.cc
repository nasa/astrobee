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

#include <calibration/camera_utilities.h>
#include <calibration/test_utilities.h>
#include <localization_common/test_utilities.h>

namespace calibration {
namespace lc = localization_common;

OptimizationParams DefaultOptimizationParams() {
  OptimizationParams params;
  params.solver_options.linear_solver_type = ceres::ITERATIVE_SCHUR;
  params.solver_options.use_explicit_schur_complement = false;
  params.solver_options.max_num_iterations = 100;
  params.solver_options.function_tolerance = 1e-8;
  params.solver_options.parameter_tolerance = 1e-8;
  params.verbose = false;
  params.huber_loss = 1.345;
  return params;
}

RansacPnPParams DefaultRansacPnPParams() {
  RansacPnPParams params;
  params.max_inlier_threshold = 3;
  params.num_iterations = 100;
  params.min_num_inliers = 4;
  // TODO(rsoussan): Change this to p3p when p3p bug or opencv version fixed
  // Currenty p3p leads to significant errors even with perfect data
  params.pnp_method = cv::SOLVEPNP_EPNP;
  return params;
}

ReprojectionPoseEstimateParams DefaultReprojectionPoseEstimateParams() {
  ReprojectionPoseEstimateParams params;
  params.optimization = DefaultOptimizationParams();
  params.ransac_pnp = DefaultRansacPnPParams();
  params.optimize_estimate = true;
  return params;
}

RandomRegistrationCorrespondences::RandomRegistrationCorrespondences() {
  static constexpr double x_min = -10.0;
  static constexpr double x_max = 10.0;
  static constexpr double y_min = -10.0;
  static constexpr double y_max = 10.0;
  static constexpr double z_min = 0.1;
  static constexpr double z_max = 30.0;

  // Sample points in front of camera
  const double x = lc::RandomDouble(x_min, x_max);
  const double y = lc::RandomDouble(y_min, y_max);
  const double z = lc::RandomDouble(z_min, z_max);
  // TODO(rsoussan): Sample orientation such that it is facing camera to some extent
  camera_T_target_ = Eigen::Isometry3d::Identity();           // lc::RandomIsometry3d();
  camera_T_target_.translation() = Eigen::Vector3d(0, 0, 3);  // Eigen::Vector3d(x, y, z);
  intrinsics_ = Eigen::Matrix3d::Identity();                  // lc::RandomIntrinsics();
  intrinsics_(0, 0) = 500;
  intrinsics_(1, 1) = 500;
  intrinsics_(0, 2) = 500;
  intrinsics_(1, 2) = 500;
  const std::vector<Eigen::Vector3d> target_t_target_points = TargetPoints();
  for (const auto& target_t_target_point : target_t_target_points) {
    const Eigen::Vector3d camera_t_target_point = camera_T_target_ * target_t_target_point;
    if (camera_t_target_point.z() <= 0) continue;
    const Eigen::Vector2d image_point = Project3dPointToImageSpace(camera_t_target_point, intrinsics_);
    std::cout << "image point: " << image_point.x() << ", " << image_point.y() << std::endl;
    correspondences_.AddCorrespondence(image_point, target_t_target_point);
  }
}

std::vector<Eigen::Vector3d> RandomRegistrationCorrespondences::TargetPoints() {
  static constexpr double kRowSpacing = 0.1;
  static constexpr double kColSpacing = 0.1;
  static constexpr int kNumPointsPerRow = 3;  // 10;
  static constexpr int kNumPointsPerCol = 3;  // 10;

  std::vector<Eigen::Vector3d> target_points;
  for (int i = 0; i < kNumPointsPerCol; ++i) {
    for (int j = 0; j < kNumPointsPerRow; ++j) {
      target_points.emplace_back(Eigen::Vector3d(i * kColSpacing, j * kRowSpacing, 0));
    }
  }
  return target_points;
}
}  // namespace calibration
