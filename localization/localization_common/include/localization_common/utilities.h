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

#ifndef LOCALIZATION_COMMON_UTILITIES_H_
#define LOCALIZATION_COMMON_UTILITIES_H_

#include <localization_common/time.h>

#include <gtsam/geometry/Pose3.h>

#include <Eigen/Core>

#include <std_msgs/Header.h>

#include <string>

namespace localization_common {
gtsam::Pose3 GtPose(const Eigen::Isometry3d& eigen_pose);

void SetEnvironmentConfigs(const std::string& astrobee_configs_path, const std::string& world);

ros::Time RosTimeFromHeader(const std_msgs::Header& header);

Time TimeFromHeader(const std_msgs::Header& header);

}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_UTILITIES_H_
