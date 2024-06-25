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

#include <localization_common/pose_covariance_interpolater.h>
#include <localization_common/utilities.h>

namespace localization_common {
PoseCovariance PoseCovarianceInterpolater::Interpolate(const PoseWithCovariance& a, const PoseWithCovariance& b,
                                                       const Time& timestamp_a, const Time& timestamp_b) {
  // Doesn't use correlation covariance, so not the best estimation.
  // If for example a and b are odometry poses, the larger they are, the larger the relative covariance estimation is
  // without using the correlation covariance. See MarginalsPoseCovarianceInterpolater for a better estimater.
  return RelativePoseCovariance(a, b);
}
}  // namespace localization_common
