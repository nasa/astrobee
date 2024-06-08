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

#include <localization_common/pose_interpolater.h>
#include <localization_common/utilities.h>

namespace localization_common {
template <>
Eigen::Isometry3d PoseInterpolater::Interpolate(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b,
                                                const double alpha) const {
  return localization_common::Interpolate(a, b, alpha);
}

template <>
Eigen::Isometry3d PoseInterpolater::Relative(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) const {
  return a.inverse() * b;
}
}  // namespace localization_common
