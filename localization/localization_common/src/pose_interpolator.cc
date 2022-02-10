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

#include <localization_common/pose_interpolator.h>
#include <localization_common/utilities.h>

namespace localization_common {
PoseInterpolator::PoseInterpolator(const std::vector<Eigen::Isometry3d>& poses, const std::vector<Time>& timestamps)
    : TimestampedSet<Eigen::Isometry3d>(timestamps, poses) {}

boost::optional<Eigen::Isometry3d> PoseInterpolator::Interpolate(const Time timestamp) const {
  const auto lower_and_upper_bound = LowerAndUpperBound(timestamp);
  // Check if equal timestamp exists
  if (lower_and_upper_bound.second && lower_and_upper_bound.second->timestamp == timestamp)
    return lower_and_upper_bound.second->value;
  if (!lower_and_upper_bound.first && !lower_and_upper_bound.second) {
    LogDebug("Interpolate: Failed to get lower and upper bound timestamps for query timestamp " << timestamp << ".");
    return boost::none;
  }

  const Time lower_bound_time = lower_and_upper_bound.first->timestamp;
  const Time upper_bound_time = lower_and_upper_bound.second->timestamp;
  const double alpha = (timestamp - lower_bound_time) / (upper_bound_time - lower_bound_time);
  return localization_common::Interpolate(lower_and_upper_bound.first->value, lower_and_upper_bound.second->value,
                                          alpha);
}
}  // namespace localization_common
