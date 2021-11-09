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
#ifndef CALIBRATION_TEST_UTILITIES_H_
#define CALIBRATION_TEST_UTILITIES_H_

#include <calibration/camera_target_based_intrinsics_calibrator_params.h>
#include <calibration/match_set.h>
#include <calibration/state_parameters.h>
#include <calibration/optimization_params.h>
#include <calibration/ransac_pnp_params.h>
#include <calibration/reprojection_pose_estimate_params.h>
#include <ff_common/eigen_vectors.h>
#include <localization_common/image_correspondences.h>

#include <vector>

namespace calibration {
OptimizationParams DefaultOptimizationParams();

RansacPnPParams DefaultRansacPnPParams();

ReprojectionPoseEstimateParams DefaultReprojectionPoseEstimateParams();

CameraTargetBasedIntrinsicsCalibratorParams DefaultCameraTargetBasedIntrinsicsCalibratorParams();

Eigen::VectorXd RandomFovDistortion();

Eigen::VectorXd RandomRadDistortion();

Eigen::VectorXd RandomRadTanDistortion();

Eigen::Isometry3d RandomFrontFacingPose(const double x_min, const double x_max, const double y_min, const double y_max,
                                        const double z_min, const double z_max, const double yaw_min,
                                        const double yaw_max, const double pitch_min, const double pitch_max,
                                        const double roll_min, const double roll_max);
Eigen::Isometry3d RandomFrontFacingPose();

std::vector<Eigen::Vector3d> TargetPoints(const int points_per_row, const int points_per_col,
                                          const double row_spacing = 0.1, const double col_spacing = 0.1);

std::vector<Eigen::Vector3d> RandomFrontFacingPoints(const int num_points);

Eigen::Vector3d RandomFrontFacingPoint();

template <typename DISTORTER>
std::vector<MatchSet> RandomMatchSets(const int num_match_sets, const int num_points_per_set,
                                      const Eigen::Matrix3d& intrinsics,
                                      const Eigen::VectorXd& distortion = Eigen::VectorXd());
template <typename DISTORTER>
class RegistrationCorrespondences {
 public:
  RegistrationCorrespondences(const Eigen::Isometry3d& camera_T_target, const Eigen::Matrix3d& intrinsics,
                              const std::vector<Eigen::Vector3d>& target_t_target_point,
                              const Eigen::VectorXd& distortion = Eigen::VectorXd());

  const localization_common::ImageCorrespondences& correspondences() const { return correspondences_; }

  const Eigen::Isometry3d& camera_T_target() const { return camera_T_target_; }

  const Eigen::Matrix3d& intrinsics() const { return intrinsics_; }

 private:
  localization_common::ImageCorrespondences correspondences_;
  Eigen::Isometry3d camera_T_target_;
  Eigen::Matrix3d intrinsics_;
};

template <typename DISTORTER>
RegistrationCorrespondences<DISTORTER>::RegistrationCorrespondences(
  const Eigen::Isometry3d& camera_T_target, const Eigen::Matrix3d& intrinsics,
  const std::vector<Eigen::Vector3d>& target_t_target_points, const Eigen::VectorXd& distortion)
    : camera_T_target_(camera_T_target), intrinsics_(intrinsics) {
  for (const auto& target_t_target_point : target_t_target_points) {
    const Eigen::Vector3d camera_t_target_point = camera_T_target_ * target_t_target_point;
    if (camera_t_target_point.z() <= 0) continue;
    const Eigen::Vector2d image_point =
      Project3dPointToImageSpaceWithDistortion<DISTORTER>(camera_t_target_point, intrinsics_, distortion);
    correspondences_.AddCorrespondence(image_point, target_t_target_point);
  }
}

template <typename DISTORTER>
std::vector<MatchSet> RandomMatchSets(const int num_match_sets, const int num_points_per_set,
                                      const Eigen::Matrix3d& intrinsics, const Eigen::VectorXd& distortion) {
  std::vector<int> inliers(num_points_per_set);
  std::iota(inliers.begin(), inliers.end(), 0);
  std::vector<MatchSet> match_sets;
  match_sets.reserve(num_match_sets);
  for (int i = 0; i < num_match_sets; ++i) {
    const auto correspondences = RegistrationCorrespondences<DISTORTER>(
      RandomFrontFacingPose(), intrinsics, RandomFrontFacingPoints(num_points_per_set), distortion);
    match_sets.emplace_back(correspondences.correspondences(), correspondences.camera_T_target(), inliers);
  }
  return match_sets;
}

StateParameters AddNoiseToStateParameters(const StateParameters& state_parameters, const double focal_lengths_stddev,
                                          const double principal_points_stddev, const double distortion_stddev);
}  // namespace calibration

#endif  // CALIBRATION_TEST_UTILITIES_H_
