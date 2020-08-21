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

#ifndef LOCALIZATION_COMMON_COMBINED_NAV_STATE_COVARIANCES_H_
#define LOCALIZATION_COMMON_COMBINED_NAV_STATE_COVARIANCES_H_

#include <Eigen/Core>

namespace localization_common {
enum Confidence { kGood, kPoor, kLost };
class CombinedNavStateCovariances {
 public:
  CombinedNavStateCovariances(const Eigen::Matrix<double, 6, 6>& pose_covariance,
                              const Eigen::Matrix3d& velocity_covariance,
                              const Eigen::Matrix<double, 6, 6>& bias_covariance);
  CombinedNavStateCovariances(const Eigen::Vector3d& position_variances, const Eigen::Vector3d& orientation_variances,
                              const Eigen::Vector3d& velocity_variances,
                              const Eigen::Vector3d& accelerometer_bias_variances,
                              const Eigen::Vector3d& gyro_bias_variances);
  CombinedNavStateCovariances() = default;
  // create constructor from marginals! -> put this in
  // localization_common!
  Confidence PoseConfidence() const;

  const Eigen::Matrix<double, 6, 6>& pose_covariance() const { return pose_covariance_; }
  const Eigen::Matrix3d& velocity_covariance() const { return velocity_covariance_; }
  const Eigen::Matrix<double, 6, 6>& bias_covariance() const { return bias_covariance_; }

  // Diagonal Accessors for Loc msg
  Eigen::Vector3d position_variances() const { return pose_covariance_.block<3, 3>(0, 0).diagonal(); }
  Eigen::Vector3d orientation_variances() const { return pose_covariance_.block<3, 3>(3, 3).diagonal(); }
  Eigen::Vector3d velocity_variances() const { return velocity_covariance_.diagonal(); }
  Eigen::Vector3d gyro_bias_variances() const { return bias_covariance_.block<3, 3>(3, 3).diagonal(); }
  Eigen::Vector3d accel_bias_variances() const { return bias_covariance_.block<3, 3>(0, 0).diagonal(); }

 private:
  Eigen::Matrix<double, 6, 6> pose_covariance_;
  Eigen::Matrix3d velocity_covariance_;
  Eigen::Matrix<double, 6, 6> bias_covariance_;
  // TODO(rsoussan): tune these!
  static constexpr double kPositionLogDetThreshold = 1;
  static constexpr double kOrientationLogDetThreshold = 1;
};
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_COMBINED_NAV_STATE_COVARIANCES_H_
