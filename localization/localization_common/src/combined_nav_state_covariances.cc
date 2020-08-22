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

#include <localization_common/combined_nav_state_covariances.h>

namespace localization_common {
CombinedNavStateCovariances::CombinedNavStateCovariances(const Eigen::Matrix<double, 6, 6>& pose_covariance,
                                                         const Eigen::Matrix3d& velocity_covariance,
                                                         const Eigen::Matrix<double, 6, 6>& bias_covariance)
    : pose_covariance_(pose_covariance), velocity_covariance_(velocity_covariance), bias_covariance_(bias_covariance) {}

CombinedNavStateCovariances::CombinedNavStateCovariances(const Eigen::Vector3d& position_variances,
                                                         const Eigen::Vector3d& orientation_variances,
                                                         const Eigen::Vector3d& velocity_variances,
                                                         const Eigen::Vector3d& accelerometer_bias_variances,
                                                         const Eigen::Vector3d& gyro_bias_variances) {
  pose_covariance_.block<3, 3>(0, 0) = position_variances.asDiagonal();
  pose_covariance_.block<3, 3>(3, 3) = orientation_variances.asDiagonal();
  velocity_covariance_ = velocity_variances.asDiagonal();
  bias_covariance_.block<3, 3>(0, 0) = accelerometer_bias_variances.asDiagonal();
  bias_covariance_.block<3, 3>(3, 3) = gyro_bias_variances.asDiagonal();
}

Confidence CombinedNavStateCovariances::PoseConfidence() const {
  const double position_log_det = std::log10(pose_covariance().block<3, 3>(0, 0).determinant());
  const double orientation_log_det = std::log10(pose_covariance().block<3, 3>(3, 3).determinant());

  const Confidence position_confidence =
      position_log_det < kPositionLogDetThreshold ? Confidence::kGood : Confidence::kPoor;
  const Confidence orientation_confidence =
      orientation_log_det < kOrientationLogDetThreshold ? Confidence::kGood : Confidence::kPoor;

  if (position_confidence == Confidence::kGood && orientation_confidence == Confidence::kGood) return Confidence::kGood;
  // Lost if both confidences are bad, poor if only 1 is bad
  if (position_confidence == Confidence::kPoor && orientation_confidence == Confidence::kPoor) return Confidence::kLost;
  return Confidence::kPoor;
}
}  // namespace localization_common
