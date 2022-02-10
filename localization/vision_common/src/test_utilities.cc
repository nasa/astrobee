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

#include <localization_common/test_utilities.h>
#include <vision_common/pose_estimation.h>
#include <vision_common/test_utilities.h>

#include <opencv2/imgproc.hpp>

#include <algorithm>

namespace vision_common {
namespace lc = localization_common;

void SetFocalLengths(const Eigen::Vector2d& focal_lengths, Eigen::Matrix3d& intrinsics) {
  intrinsics(0, 0) = focal_lengths[0];
  intrinsics(1, 1) = focal_lengths[1];
}

void SetPrincipalPoints(const Eigen::Vector2d& principal_points, Eigen::Matrix3d& intrinsics) {
  intrinsics(0, 2) = principal_points[0];
  intrinsics(1, 2) = principal_points[1];
}

LKOpticalFlowFeatureDetectorAndMatcherParams DefaultLKOpticalFlowFeatureDetectorAndMatcherParams() {
  LKOpticalFlowFeatureDetectorAndMatcherParams params;
  params.max_iterations = 10;
  params.termination_epsilon = 0.03;
  params.window_length = 10;
  params.max_level = 3;
  params.min_eigen_threshold = 0.2;
  params.max_flow_distance = 50;
  params.max_backward_match_distance = 0.1;
  params.good_features_to_track.max_corners = 100;
  params.good_features_to_track.quality_level = 0.01;
  params.good_features_to_track.min_distance = 20;
  params.good_features_to_track.block_size = 3;
  params.good_features_to_track.use_harris_detector = false;
  params.good_features_to_track.k = 0.04;
  return params;
}

cv::Mat MarkerImage(const int row_spacing, const int col_spacing, int& num_markers_added, const cv::Point2i& offset) {
  cv::Mat image(cv::Mat(cv::Size(640, 480), CV_8UC1, cv::Scalar(255)));
  num_markers_added = AddMarkers(row_spacing, col_spacing, image, offset);
  return image;
}

int AddMarkers(const int row_spacing, const int col_spacing, cv::Mat& image, const cv::Point2i& offset) {
  int num_markers = 0;
  // Don't start at zero so all markers are candidates for matches
  // End before edge to add some buffer to markers
  for (int row = row_spacing; row < image.rows - row_spacing; row += row_spacing) {
    for (int col = col_spacing; col < image.cols - col_spacing; col += col_spacing) {
      cv::drawMarker(image, offset + cv::Point2i(col, row), cv::Scalar(0), cv::MARKER_CROSS);
      ++num_markers;
    }
  }
  return num_markers;
}

optimization_common::OptimizationParams DefaultOptimizationParams() {
  optimization_common::OptimizationParams params;
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
}  // namespace vision_common
