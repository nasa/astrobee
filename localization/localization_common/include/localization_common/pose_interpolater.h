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

#include <ff_common/eigen_vectors.h>
#include <localization_common/time.h>
#include <localization_common/timestamped_set.h>

#include <Eigen/Geometry>

#include <boost/optional.hpp>

#include <vector>

namespace localization_common {
class PoseInterpolater : public TimestampedSet<Eigen::Isometry3d> {
 public:
  PoseInterpolater(const std::vector<Time>& timestamps, const std::vector<Eigen::Isometry3d>& poses);

  boost::optional<Eigen::Isometry3d> Interpolate(const Time timestamp) const;

 private:
  TimestampedSet timestamped_poses_;
};
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_POSE_INTERPOLATER_H_
