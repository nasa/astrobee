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
  params.pnp_method = cv::SOLVEPNP_EPNP;  // cv::SOLVEPNP_ITERATIVE;
  return params;
}

ReprojectionPoseEstimateParams DefaultReprojectionPoseEstimateParams() {
  ReprojectionPoseEstimateParams params;
  params.optimization = DefaultOptimizationParams();
  params.ransac_pnp = DefaultRansacPnPParams();
  params.optimize_estimate = true;
  params.max_inlier_threshold = 3.0;
  return params;
}

CameraTargetBasedIntrinsicsCalibratorParams DefaultCameraTargetBasedIntrinsicsCalibratorParams() {
  CameraTargetBasedIntrinsicsCalibratorParams params;
  params.optimization = DefaultOptimizationParams();
  params.reprojection_pose_estimate = DefaultReprojectionPoseEstimateParams();
  params.calibrate_focal_lengths = true;
  params.calibrate_principal_points = true;
  params.calibrate_distortion = true;
  params.calibrate_target_poses = true;
  params.scale_loss_radially = false;
  params.radial_scale_power = 1.0;
  params.only_use_inliers = true;
  params.max_num_match_sets = 10000000;
  params.min_num_target_inliers = 4;
  params.save_individual_initial_reprojection_images = false;
  params.save_final_reprojection_image = false;
  params.max_visualization_error_norm = 50;
  params.individual_max_visualization_error_norm = 50;
  params.image_size = Eigen::Vector2i(1280, 960);
  return params;
}

Eigen::VectorXd RandomFovDistortion() {
  Eigen::VectorXd distortion(1);
  distortion[0] = lc::RandomDouble(0.1, 3.0);
  return distortion;
}

Eigen::VectorXd RandomRadDistortion() {
  Eigen::VectorXd distortion(2);
  distortion[0] = lc::RandomDouble(-1.0, 1.0);
  distortion[1] = lc::RandomDouble(-1.0, 1.0);
  return distortion;
}

Eigen::VectorXd RandomRadTanDistortion() {
  Eigen::VectorXd distortion(4);
  distortion[0] = lc::RandomDouble(-1.0, 1.0);
  distortion[1] = lc::RandomDouble(-1.0, 1.0);
  distortion[2] = lc::RandomDouble(-1.0, 1.0);
  distortion[3] = lc::RandomDouble(-1.0, 1.0);
  return distortion;
}

std::vector<Eigen::Vector3d> TargetPoints(const int points_per_row, const int points_per_col, const double row_spacing,
                                          const double col_spacing) {
  std::vector<Eigen::Vector3d> target_points;
  for (int i = 0; i < points_per_col; ++i) {
    for (int j = 0; j < points_per_row; ++j) {
      target_points.emplace_back(Eigen::Vector3d(i * col_spacing, j * row_spacing, 0));
    }
  }
  return target_points;
}

std::vector<Eigen::Vector3d> RandomFrontFacingPoints(const int num_points) {
  std::vector<Eigen::Vector3d> points;
  for (int i = 0; i < num_points; ++i) {
    points.emplace_back(RandomFrontFacingPoint());
  }
  return points;
}

Eigen::Vector3d RandomFrontFacingPoint() {
  static constexpr double x_min = -10.0;
  static constexpr double x_max = 10.0;
  static constexpr double y_min = -10.0;
  static constexpr double y_max = 10.0;
  static constexpr double z_min = 0.1;
  static constexpr double z_max = 30.0;

  const double x = lc::RandomDouble(x_min, x_max);
  const double y = lc::RandomDouble(y_min, y_max);
  const double z = lc::RandomDouble(z_min, z_max);
  return Eigen::Vector3d(x, y, z);
}

Eigen::Isometry3d RandomFrontFacingPose() {
  static constexpr double x_min = -10.0;
  static constexpr double x_max = 10.0;
  static constexpr double y_min = -10.0;
  static constexpr double y_max = 10.0;
  static constexpr double z_min = 0.1;
  static constexpr double z_max = 30.0;

  static constexpr double yaw_min = -45.0;
  static constexpr double yaw_max = 45.0;
  // Target points can handle larger orientations than random points
  static constexpr double pitch_min = 0;
  static constexpr double pitch_max = 0;
  static constexpr double roll_min = 0;
  static constexpr double roll_max = 0;

  return RandomFrontFacingPose(x_min, x_max, y_min, y_max, z_min, z_max, yaw_min, yaw_max, pitch_min, pitch_max,
                               roll_min, roll_max);
}

Eigen::Isometry3d RandomFrontFacingPose(const double x_min, const double x_max, const double y_min, const double y_max,
                                        const double z_min, const double z_max, const double yaw_min,
                                        const double yaw_max, const double pitch_min, const double pitch_max,
                                        const double roll_min, const double roll_max) {
  // Translation
  const double x = lc::RandomDouble(x_min, x_max);
  const double y = lc::RandomDouble(y_min, y_max);
  const double z = lc::RandomDouble(z_min, z_max);

  const double yaw = lc::RandomDouble(yaw_min, yaw_max);
  const double pitch = lc::RandomDouble(pitch_min, pitch_max);
  const double roll = lc::RandomDouble(roll_min, roll_max);
  const Eigen::Matrix3d rotation = lc::RotationFromEulerAngles(yaw, pitch, roll);
  return lc::Isometry3d(Eigen::Vector3d(x, y, z), rotation);
}

StateParameters AddNoiseToStateParameters(const StateParameters& state_parameters, const double focal_lengths_stddev,
                                          const double principal_points_stddev, const double distortion_stddev,
                                          const bool ensure_distortion_positive) {
  StateParameters noisy_state_parameters;
  noisy_state_parameters.focal_lengths = lc::AddNoiseToVector(state_parameters.focal_lengths, focal_lengths_stddev);
  noisy_state_parameters.principal_points =
    lc::AddNoiseToVector(state_parameters.principal_points, principal_points_stddev);
  noisy_state_parameters.distortion = lc::AddNoiseToVector(state_parameters.distortion, distortion_stddev);
  if (ensure_distortion_positive) {
    for (int i = 0; i < static_cast<int>(noisy_state_parameters.distortion.size()); ++i) {
      if (noisy_state_parameters.distortion[i] < 0) noisy_state_parameters.distortion[i] = 0.0001;
    }
  }
  return noisy_state_parameters;
}
}  // namespace calibration
