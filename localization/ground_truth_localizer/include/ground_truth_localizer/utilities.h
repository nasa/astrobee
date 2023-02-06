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
#ifndef GROUND_TRUTH_LOCALIZER_UTILITIES_H_
#define GROUND_TRUTH_LOCALIZER_UTILITIES_H_

#include <ground_truth_localizer/twist.h>
#include <localization_common/time.h>

#include <ff_msgs/msg/ekf_state.hpp>
namespace ff_msgs {
typedef msg::EkfState EkfState;
}  // namespace ff_msgs

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
namespace geometry_msgs {
typedef msg::PoseStamped PoseStamped;
typedef msg::TwistStamped TwistStamped;
}  // namespace geometry_msgs

#include <Eigen/Geometry>

namespace ground_truth_localizer {
Eigen::Isometry3d PoseFromMsg(const geometry_msgs::PoseStamped& pose_msg);
Twist TwistFromMsg(const geometry_msgs::TwistStamped& twist_msg);
ff_msgs::EkfState LocStateMsg(const Eigen::Isometry3d& pose, const Twist& twist, const localization_common::Time& time);
}  // namespace ground_truth_localizer

#endif  // GROUND_TRUTH_LOCALIZER_UTILITIES_H_
