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

#include <config_reader/config_reader.h>
#include <imu_augmentor/imu_augmentor_wrapper.h>

namespace imu_augmentor {
ImuAugmentorWrapper::ImuAugmentorWrapper() {
  // Needed for ConfigReader construction
  // TODO(rsoussan): load this somewhere else/ how do other nodelets do this?
  const std::string astrobee_configs_path = "/home/rsoussan/astrobee/astrobee";
  const std::string world = "iss";
  SetEnvironmentConfigs(astrobee_configs_path, world);
  config_reader::ConfigReader config;

  config.AddFile("transforms.config");
  config.AddFile("geometry.config");

  if (!config.ReadFiles()) {
    ROS_FATAL("Failed to read config files.");
  }

  // TODO(rsoussan): load body_T_imu and gravity!!!!
}

bool LatestPoseMsg(geometry_msgs::PoseWithCovarianceStamped& latest_pose_msg) const;

void LocalizationPoseCallback(const geometry_msgs::PoseWithCovarianceStamped& localization_pose_msg);

void ImuCallback(const sensor_msgs::Imu& imu_msg);

std::unique_ptr<ImuAugmentor> imu_augmentor_;
}  // namespace imu_augmentor
