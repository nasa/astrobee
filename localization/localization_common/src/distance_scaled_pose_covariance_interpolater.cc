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

#include <localization_common/distance_scaled_pose_covariance_interpolater.h>

namespace localization_common {
DistanceScaledPoseCovarianceInterpolater::DistanceScaledPoseCovarianceInterpolater(
  const DistanceScaledPoseCovarianceInterpolaterParams& params)
    : params_(params) {}

PoseCovariance DistanceScaledPoseCovarianceInterpolater::Interpolate(const PoseWithCovariance& a,
                                                                     const PoseWithCovariance& b,
                                                                     const Time& timestamp_a, const Time& timestamp_b) {
  const Eigen::Isometry3d relative_pose = a.pose.inverse() * b.pose;
  const double translation_norm = relative_pose.translation().norm();
  const double orientation_angle = Eigen::AngleAxisd(relative_pose.linear()).angle();
  Eigen::Matrix<double, 6, 6> relative_covariance = Eigen::Matrix<double, 6, 6>::Zero();
  // Set translation part of covariance using translation norm
  relative_covariance.block<3, 3>(3, 3) =
    (relative_pose.translation().cwiseAbs() * params_.translation_scale).asDiagonal();
  // Set orientation part of covariance using orientation angle
  relative_covariance.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * orientation_angle * params_.orientation_scale;
  // Make sure no component of the covariance matrix is too small
  for (int i = 0; i < 6; ++i) {
    if (relative_covariance(i, i) < params_.min_covariance) {
      relative_covariance(i, i) = params_.min_covariance;
    }
  }
  return relative_covariance;
}
}  // namespace localization_common
