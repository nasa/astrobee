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

#include <localization_common/marginals_pose_covariance_interpolater.h>

namespace localization_common {

PoseCovariance MarginalsPoseCovarianceInterpolater::Interpolate(const PoseWithCovariance& a,
                                                                const PoseWithCovariance& b, const Time& timestamp_a,
                                                                const Time& timestamp_b) {
  // Get relative covariance_a_b
  const auto closest_t_a = nodes_->ClosestTimestamp(timestamp_a);
  const auto closest_t_b = nodes_->ClosestTimestamp(timestamp_b);
  if (!closest_t_a || !closest_t_b) {
    LogError("RelativeNodeAndNoise: Failed to get closest timestamps.");
    return boost::none;
  }

  // TODO(rsoussan): Add fallback covariance/relative covariance if gap too large
  if (std::abs(*closest_t_a - timestamp_a) > 3.0 || std::abs(*closest_t_b - timestamp_b) > 3.0) {
    LogWarning("RelativeNodeAndNoise: Timestamps far from closest timestamps available.");
  }
  const auto keys_a = nodes_->Keys(*closest_t_a);
  const auto keys_b = nodes_->Keys(*closest_t_b);
  const auto covariance_a_b = marginals_.jointMarginalCovariance({keys_a[0], keys_b[0]}).fullMatrix();
  return RelativeCovariance(a, b, covariance_a_b);
}
};  // namespace localization_common
}  // namespace localization_common
