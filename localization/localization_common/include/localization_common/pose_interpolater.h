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

#ifndef LOCALIZATION_COMMON_POSE_INTERPOLATER_H_
#define LOCALIZATION_COMMON_POSE_INTERPOLATER_H_

#include <localization_common/timestamped_interpolater.h>

#include <Eigen/Geometry>

namespace localization_common {
using PoseInterpolater = TimestampedInterpolater<Eigen::Isometry3d>;

template <>
Eigen::Isometry3d PoseInterpolater::Interpolate(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b,
                                                const double alpha) const;

template <>
Eigen::Isometry3d PoseInterpolater::Relative(const Eigen::Isometry3d& a, const Eigen::Isometry3d& b) const;
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_POSE_INTERPOLATER_H_
