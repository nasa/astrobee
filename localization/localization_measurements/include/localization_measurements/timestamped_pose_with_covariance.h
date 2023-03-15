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

#ifndef LOCALIZATION_MEASUREMENTS_TIMESTAMPED_POSE_WITH_COVARIANCE_H_
#define LOCALIZATION_MEASUREMENTS_TIMESTAMPED_POSE_WITH_COVARIANCE_H_

#include <localization_common/pose_with_covariance.h>
#include <localization_common/time.h>

namespace localization_measurements {
struct TimestampedPoseWithCovariance {
  TimestampedPoseWithCovariance(const localization_common::PoseWithCovariance& pose_with_covariance,
                                const localization_common::Time time)
      : pose_with_covariance(pose_with_covariance), time(time) {}
  TimestampedPoseWithCovariance() = default;
  localization_common::PoseWithCovariance pose_with_covariance;
  localization_common::Time time;
};
}  // namespace localization_measurements

#endif  // LOCALIZATION_MEASUREMENTS_TIMESTAMPED_POSE_WITH_COVARIANCE_H_