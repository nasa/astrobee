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

#include "test_utilities.h"  // NOLINT

#include <localization_common/test_utilities.h>
#include <vision_common/camera_utilities.h>

#include <algorithm>

namespace vision_common {
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
  // TODO(rsoussan): Change this to cv::SOLVEPNP_IPPE when available.
  // Currenty cv::SOLVEPNP_P3P and cv::SOLVEPNP_ITERATIVE lead to significant errors even with perfect data
  params.pnp_method = cv::SOLVEPNP_EPNP;
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
  distortion[0] = lc::RandomDouble(0.1, 0.3);
  return distortion;
}

Eigen::VectorXd RandomRadDistortion() {
  Eigen::VectorXd distortion(2);
  distortion[0] = lc::RandomDouble(-0.1, 0.1);
  distortion[1] = lc::RandomDouble(-0.1, 0.1);
  return distortion;
}

Eigen::VectorXd RandomRadTanDistortion() {
  Eigen::VectorXd distortion(4);
  distortion[0] = lc::RandomDouble(-0.1, 0.1);
  distortion[1] = lc::RandomDouble(-0.1, 0.1);
  distortion[2] = lc::RandomDouble(0, 0.1);
  distortion[3] = lc::RandomDouble(0, 0.1);
  return distortion;
}

std::vector<Eigen::Vector3d> TargetPoints(const int points_per_row, const int points_per_col, const double row_spacing,
                                          const double col_spacing) {
  Eigen::Vector3d target_center(points_per_col * col_spacing / 2.0, points_per_row * row_spacing / 2.0, 0.0);
  std::vector<Eigen::Vector3d> target_points;
  for (int i = 0; i < points_per_col; ++i) {
    for (int j = 0; j < points_per_row; ++j) {
      // Center target points about (0,0)
      target_points.emplace_back(Eigen::Vector3d(i * col_spacing, j * row_spacing, 0) - target_center);
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
  static constexpr double rho_min = 1.0;
  static constexpr double rho_max = 3.0;
  static constexpr double phi_min = -25.0;
  static constexpr double phi_max = 25.0;
  static constexpr double z_rho_scale = 0.5;
  return RandomFrontFacingPoint(rho_min, rho_max, phi_min, phi_max, z_rho_scale);
}

Eigen::Vector3d RandomFrontFacingPoint(const double rho_min, const double rho_max, const double phi_min,
                                       const double phi_max, const double z_rho_scale) {
  const double rho = lc::RandomDouble(rho_min, rho_max);
  const double phi = lc::RandomDouble(phi_min, phi_max);
  const double z = lc::RandomDouble(-1.0 * z_rho_scale * rho, z_rho_scale * rho);
  // Z and x are swapped so z defines distance from camera rather than height
  const Eigen::Vector3d tmp = lc::CylindricalToCartesian(Eigen::Vector3d(rho, phi, z));
  const Eigen::Vector3d random_point(tmp.z(), tmp.y(), tmp.x());
  return random_point;
}

Eigen::Isometry3d RandomFrontFacingPose() {
  static constexpr double rho_min = 1.0;
  static constexpr double rho_max = 3.0;
  static constexpr double phi_min = -25.0;
  static constexpr double phi_max = 25.0;
  static constexpr double z_rho_scale = 0.5;

  // Pitch acts like yaw since z axis points outwards in camera frame
  static constexpr double yaw_min = -10.0;
  static constexpr double yaw_max = 10.0;
  static constexpr double pitch_min = -45;
  static constexpr double pitch_max = 45;
  static constexpr double roll_min = -10;
  static constexpr double roll_max = 10;

  return RandomFrontFacingPose(rho_min, rho_max, phi_min, phi_max, z_rho_scale, yaw_min, yaw_max, pitch_min, pitch_max,
                               roll_min, roll_max);
}

Eigen::Isometry3d RandomFrontFacingPose(const double rho_min, const double rho_max, const double phi_min,
                                        const double phi_max, const double z_rho_scale, const double yaw_min,
                                        const double yaw_max, const double pitch_min, const double pitch_max,
                                        const double roll_min, const double roll_max) {
  const Eigen::Vector3d translation = RandomFrontFacingPoint(rho_min, rho_max, phi_min, phi_max, z_rho_scale);
  const double yaw = lc::RandomDouble(yaw_min, yaw_max);
  const double pitch = lc::RandomDouble(pitch_min, pitch_max);
  const double roll = lc::RandomDouble(roll_min, roll_max);
  const Eigen::Matrix3d rotation = lc::RotationFromEulerAngles(yaw, pitch, roll);
  return lc::Isometry3d(translation, rotation);
}

std::vector<Eigen::Isometry3d> EvenlySpacedTargetPoses(const int num_rows, const int num_cols, const int num_y_levels) {
  // Pitch acts like yaw since z axis points out of camera frame
  const lc::Sampler pitch_sampler(-15.0, 15.0, num_cols);
  const lc::Sampler roll_sampler(-15.0, 15.0, num_cols);
  const lc::Sampler yaw_sampler(-15.0, 15.0, num_cols);

  // Cylindrical coordinates for translation
  const lc::Sampler rho_sampler(1.0, 3.0, num_rows);
  constexpr double phi = 0.0;

  // Use smaller y_rho scale factor since image is shorter than it is wide
  constexpr double y_rho_scale = 0.235;
  constexpr double z_rho_scale = 0.5;

  std::vector<Eigen::Isometry3d> poses;
  for (int i = 0; i < num_rows; ++i) {
    const double rho = rho_sampler.Sample(i);
    const lc::Sampler z_sampler(-1.0 * rho * z_rho_scale, rho * z_rho_scale, num_cols);
    for (int j = 0; j < num_cols; ++j) {
      const double pitch = pitch_sampler.Sample(j);
      const double roll = roll_sampler.Sample(j);
      const double yaw = yaw_sampler.Sample(j);
      const double z = z_sampler.Sample(j);
      // Z and x are swapped so z defines distance from camera rather than height
      const Eigen::Vector3d tmp = lc::CylindricalToCartesian(Eigen::Vector3d(rho, phi, z));
      const Eigen::Matrix3d rotation = lc::RotationFromEulerAngles(yaw, pitch, roll);
      const lc::Sampler y_sampler(-1.0 * rho * y_rho_scale, rho * y_rho_scale, num_y_levels);
      for (int k = 0; k < num_y_levels; ++k) {
        const double y = y_sampler.Sample(k);
        const Eigen::Vector3d translation(tmp.z(), y, tmp.x());
        poses.emplace_back(lc::Isometry3d(translation, rotation));
      }
    }
  }

  return poses;
}

StateParameters AddNoiseToStateParameters(const StateParameters& state_parameters, const double focal_lengths_stddev,
                                          const double principal_points_stddev, const double distortion_stddev,
                                          const bool fov) {
  StateParameters noisy_state_parameters;
  noisy_state_parameters.focal_lengths = lc::AddNoiseToVector(state_parameters.focal_lengths, focal_lengths_stddev);
  noisy_state_parameters.principal_points =
    lc::AddNoiseToVector(state_parameters.principal_points, principal_points_stddev);
  noisy_state_parameters.distortion = lc::AddNoiseToVector(state_parameters.distortion, distortion_stddev);
  if (fov) {
    // Close to zero fov values cause issues for solver, neg values are same as positive values
    noisy_state_parameters.distortion[0] = std::max(0.1, noisy_state_parameters.distortion[0]);
  }
  return noisy_state_parameters;
}
}  // namespace vision_common
