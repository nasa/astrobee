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
  params.pnp_method = cv::SOLVEPNP_ITERATIVE;
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

  static constexpr double yaw_min = -45.0;
  static constexpr double yaw_max = 45.0;
  static constexpr double pitch_min = -45.0;
  static constexpr double pitch_max = 45.0;
  static constexpr double roll_min = -15.0;
  static constexpr double roll_max = 15.0;

  camera_T_target_ = RandomFrontFacingPose(x_min, x_max, y_min, y_max, z_min, z_max, yaw_min, yaw_max, pitch_min,
                                           pitch_max, roll_min, roll_max);
  intrinsics_ = lc::RandomIntrinsics();
  /*intrinsics_ = Eigen::Matrix3d::Identity();
  intrinsics_(0, 0) = 500;
  intrinsics_(1, 1) = 500;
  intrinsics_(0, 2) = 500;
  intrinsics_(1, 2) = 500;*/

  const std::vector<Eigen::Vector3d> target_t_target_points = TargetPoints();
  for (const auto& target_t_target_point : target_t_target_points) {
    const Eigen::Vector3d camera_t_target_point = camera_T_target_ * target_t_target_point;
    if (camera_t_target_point.z() <= 0) continue;
    const Eigen::Vector2d image_point = Project3dPointToImageSpace(camera_t_target_point, intrinsics_);
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

Eigen::Isometry3d RandomFrontFacingPose(const double x_min, const double x_max, const double y_min, const double y_max,
                                        const double z_min, const double z_max, const double yaw_min,
                                        const double yaw_max, const double pitch_min, const double pitch_max,
                                        const double roll_min, const double roll_max) {
  // Translation
  const double x = lc::RandomDouble(x_min, x_max);
  const double y = lc::RandomDouble(y_min, y_max);
  const double z = lc::RandomDouble(z_min, z_max);

  // Rotation using intrinsic Euler Angles, ypr convention
  const Eigen::AngleAxisd yaw =
    Eigen::AngleAxisd(M_PI / 180.0 * lc::RandomDouble(yaw_min, yaw_max), Eigen::Vector3d::UnitZ());
  const Eigen::AngleAxisd pitch =
    Eigen::AngleAxisd(M_PI / 180.0 * lc::RandomDouble(pitch_min, pitch_max), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd roll =
    Eigen::AngleAxisd(M_PI / 180.0 * lc::RandomDouble(roll_min, roll_max), Eigen::Vector3d::UnitX());
  const Eigen::Matrix3d rotation(yaw * yaw * pitch * yaw * pitch * roll);

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = Eigen::Vector3d(x, y, z);
  pose.linear() = rotation;
  return pose;
}
}  // namespace calibration
