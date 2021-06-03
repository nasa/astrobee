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

#ifndef LOCALIZATION_MEASUREMENTS_TIMESTAMPED_HANDRAIL_POSE_H_
#define LOCALIZATION_MEASUREMENTS_TIMESTAMPED_HANDRAIL_POSE_H_

#include <localization_measurements/timestamped_pose.h>
#include <localization_common/time.h>

namespace localization_measurements {
struct TimestampedHandrailPose : public TimestampedPose {
  TimestampedHandrailPose(const gtsam::Pose3& pose, const localization_common::Time time,
                          const bool accurate_z_position, const double length, const double distance_to_wall)
      : TimestampedPose(pose, time),
        accurate_z_position(accurate_z_position),
        length(length),
        distance_to_wall(distance_to_wall) {}
  // Sometimes the handrail endpoints aren't seen and the z translation component doesn't reflect the center
  // of the handrail
  bool accurate_z_position;
  double length;
  double distance_to_wall;
};
}  // namespace localization_measurements

#endif  // LOCALIZATION_MEASUREMENTS_TIMESTAMPED_HANDRAIL_POSE_H_
