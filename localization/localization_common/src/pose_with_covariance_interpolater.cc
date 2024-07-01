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

#include <localization_common/pose_with_covariance_interpolater.h>
#include <localization_common/utilities.h>

namespace localization_common {
template <>
PoseWithCovariance PoseWithCovarianceInterpolater::Interpolate(const PoseWithCovariance& a, const PoseWithCovariance& b,
                                                               const double alpha) const {
  return localization_common::Interpolate(a, b, alpha);
}

template <>
boost::optional<PoseWithCovariance> PoseWithCovarianceInterpolater::Relative(const Time timestamp_a,
                                                                             const Time timestamp_b) const {
  const auto a = Interpolate(timestamp_a);
  const auto b = Interpolate(timestamp_b);
  if (!a || !b) {
    LogDebug("Relative: Failed to get interpolated objects for query timestamps " << timestamp_a << " and "
                                                                                  << timestamp_b << ".");
    return boost::none;
  }
  const Eigen::Isometry3d relative_pose = a->pose.inverse() * b->pose;
  const auto relative_covariance = params().pose_covariance_interpolater->Interpolate(*a, *b, timestamp_a, timestamp_b);
  return PoseWithCovariance(relative_pose, relative_covariance);
}
}  // namespace localization_common
