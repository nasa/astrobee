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
Eigen::Isometry3d RandomFrontFacingPose(const double x_min, const double x_max, const double y_min, const double y_max,
                                        const double z_min, const double z_max, const double yaw_min,
                                        const double yaw_max, const double pitch_min, const double pitch_max,
                                        const double roll_min, const double roll_max);
Eigen::Isometry3d RandomFrontFacingPose();

std::vector<Eigen::Vector3d> TargetPoints(const int points_per_row, const int points_per_col,
                                          const double row_spacing = 0.1, const double col_spacing = 0.1);

std::vector<Eigen::Vector3d> RandomFrontFacingPoints(const int num_points);

Eigen::Vector3d RandomFrontFacingPoint();

class RegistrationCorrespondences {
 public:
  RegistrationCorrespondences(const Eigen::Isometry3d& camera_T_target, const Eigen::Matrix3d& intrinsics,
                              const std::vector<Eigen::Vector3d>& target_t_target_point);

  const localization_common::ImageCorrespondences& correspondences() const { return correspondences_; }

  const Eigen::Isometry3d& camera_T_target() const { return camera_T_target_; }

  const Eigen::Matrix3d& intrinsics() const { return intrinsics_; }

 private:
  localization_common::ImageCorrespondences correspondences_;
  Eigen::Isometry3d camera_T_target_;
  Eigen::Matrix3d intrinsics_;
};
}  // namespace calibration

#endif  // CALIBRATION_TEST_UTILITIES_H_
