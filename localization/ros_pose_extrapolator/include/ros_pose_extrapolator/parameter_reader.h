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
#ifndef ROS_POSE_EXTRAPOLATOR_PARAMETER_READER_H_
#define ROS_POSE_EXTRAPOLATOR_PARAMETER_READER_H_

#include <config_reader/config_reader.h>
#include <ros_pose_extrapolator/ros_pose_extrapolator_params.h>

#include <string>

namespace ros_pose_extrapolator {
void LoadRosPoseExtrapolatorParams(config_reader::ConfigReader& config, RosPoseExtrapolatorParams& params,
                                   const std::string& prefix = "");
}  // namespace ros_pose_extrapolator

#endif  // ROS_POSE_EXTRAPOLATOR_PARAMETER_READER_H_
