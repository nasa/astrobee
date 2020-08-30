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
#ifndef SIMULATION_LOCALIZER_UTILITIES_H_
#define SIMULATION_LOCALIZER_UTILITIES_H_

#include <ff_msgs/EkfState.h>
#include <localization_common/time.h>
#include <simulation_localizer/twist.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_msgs/Header.h>

#include <Eigen/Core>

namespace simulation_localizer {
Eigen::Isometry3d PoseFromMsg(const geometry_msgs::PoseStamped& pose_msg);
Twist TwistFromMsg(const geometry_msgs::TwistStamped& twist_msg);
ff_msgs::EkfState LocStateMsg(const Eigen::Isometry3d& pose, const Twist& twist, const localization_common::Time& time);
}  // namespace simulation_localizer

#endif  // SIMULATION_LOCALIZER_UTILITIES_H_
