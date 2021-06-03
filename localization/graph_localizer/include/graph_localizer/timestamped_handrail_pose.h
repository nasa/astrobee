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

#ifndef GRAPH_LOCALIZER_TIMESTAMPED_HANDRAIL_POSE_H_
#define GRAPH_LOCALIZER_TIMESTAMPED_HANDRAIL_POSE_H_

#include <graph_localizer/timestamped_pose.h>

namespace graph_localizer {
struct TimestampedHandrailPose : public TimestampedPose {
  // Sometimes the handrail endpoints aren't seen and the z translation component doesn't reflect the center
  // of the handrail
  bool accurate_z_position;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_TIMESTAMPED_HANDRAIL_POSE_H_
