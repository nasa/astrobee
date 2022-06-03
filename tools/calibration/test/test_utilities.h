/* Copyright (c) 2017, United S/ates Government, as represented by the
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
#ifndef CALIBRATION_TEST_UTILITIES_H_  // NOLINT
#define CALIBRATION_TEST_UTILITIES_H_  // NOLINT

#include <calibration/camera_target_based_intrinsics_calibrator_params.h>
#include <calibration/match_set.h>
#include <calibration/state_parameters.h>
#include <calibration/utilities.h>
#include <ff_common/eigen_vectors.h>
#include <localization_common/image_correspondences.h>
#include <localization_common/test_utilities.h>
#include <vision_common/test_utilities.h>

#include <vector>

namespace calibration {
CameraTargetBasedIntrinsicsCalibratorParams DefaultCameraTargetBasedIntrinsicsCalibratorParams();

template <typename DISTORTER>
std::vector<MatchSet> RandomTargetMatchSets(const int num_match_sets, const int num_target_points_per_row_and_col,
                                            const Eigen::Matrix3d& intrinsics,
                                            const Eigen::VectorXd& distortion = Eigen::VectorXd());

template <typename DISTORTER>
std::vector<MatchSet> EvenlySpacedTargetMatchSets(const int num_pose_rows, const int num_pose_cols,
                                                  const int num_pose_y_levels,
                                                  const int num_target_points_per_row_and_col,
                                                  const Eigen::Matrix3d& intrinsics,
                                                  const Eigen::VectorXd& distortion = Eigen::VectorXd());

template <typename DISTORTER>
std::vector<MatchSet> RandomTargetMatchSets(const int num_match_sets, const int num_target_points_per_row_and_col,
                                            const Eigen::Matrix3d& intrinsics, const Eigen::VectorXd& distortion) {
  std::vector<MatchSet> match_sets;
  match_sets.reserve(num_match_sets);
  for (int i = 0; i < num_match_sets; ++i) {
    const auto correspondences = vision_common::RegistrationCorrespondences<DISTORTER>(
      localization_common::RandomFrontFacingPose(), intrinsics,
      vision_common::TargetPoints(num_target_points_per_row_and_col, num_target_points_per_row_and_col), distortion);
    // Set inliers using correspondence point size since correspondence points with negative z are not
    // included in RegistrationCorrespondences
    std::vector<int> inliers(correspondences.correspondences().size());
    std::iota(inliers.begin(), inliers.end(), 0);
    match_sets.emplace_back(correspondences.correspondences(), correspondences.camera_T_target(), inliers);
  }
  return match_sets;
}

template <typename DISTORTER>
std::vector<MatchSet> EvenlySpacedTargetMatchSets(const int num_pose_rows, const int num_pose_cols,
                                                  const int num_pose_y_levels,
                                                  const int num_target_points_per_row_and_col,
                                                  const Eigen::Matrix3d& intrinsics,
                                                  const Eigen::VectorXd& distortion) {
  const std::vector<Eigen::Isometry3d> target_poses =
    vision_common::EvenlySpacedTargetPoses(num_pose_rows, num_pose_cols, num_pose_y_levels);
  std::vector<MatchSet> match_sets;
  const int num_match_sets = target_poses.size();
  match_sets.reserve(num_match_sets);
  const std::vector<Eigen::Vector3d> target_points =
    vision_common::TargetPoints(num_target_points_per_row_and_col, num_target_points_per_row_and_col);
  for (int i = 0; i < num_match_sets; ++i) {
    const auto correspondences =
      vision_common::RegistrationCorrespondences<DISTORTER>(target_poses[i], intrinsics, target_points, distortion);
    // Set inliers using correspondence point size since correspondence points with negative z are not
    // included in RegistrationCorrespondences
    std::vector<int> inliers(correspondences.correspondences().size());
    std::iota(inliers.begin(), inliers.end(), 0);
    match_sets.emplace_back(correspondences.correspondences(), correspondences.camera_T_target(), inliers);
  }
  return match_sets;
}

StateParameters AddNoiseToStateParameters(const StateParameters& state_parameters, const double focal_lengths_stddev,
                                          const double principal_points_stddev, const double distortion_stddev,
                                          const bool fov = false);
}  // namespace calibration

#endif  // CALIBRATION_TEST_UTILITIES_H_  // NOLINT
