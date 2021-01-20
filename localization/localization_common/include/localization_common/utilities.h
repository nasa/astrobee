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

#include <config_reader/config_reader.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/VisualLandmarks.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/combined_nav_state_covariances.h>
#include <localization_common/time.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>

#include <Eigen/Core>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Header.h>

#include <string>

namespace localization_common {
gtsam::Pose3 LoadTransform(config_reader::ConfigReader& config, const std::string& transform_config_name);

gtsam::Vector3 LoadVector3(config_reader::ConfigReader& config, const std::string& config_name);

gtsam::Cal3_S2 LoadCameraIntrinsics(config_reader::ConfigReader& config, const std::string& intrinsics_config_name);

gtsam::Pose3 GtPose(const Eigen::Isometry3d& eigen_pose);

Eigen::Isometry3d EigenPose(const CombinedNavState& combined_nav_state);

Eigen::Isometry3d EigenPose(const gtsam::Pose3& pose);

// Returns pose in body frame
gtsam::Pose3 GtPose(const ff_msgs::VisualLandmarks& vl_features, const gtsam::Pose3& sensor_T_body);

// Returns pose in sensor frame
gtsam::Pose3 GtPose(const ff_msgs::VisualLandmarks& vl_features);

// Load either iss or granite graph localizer config depending on environment variable for ASTROBEE_WORLD
void LoadGraphLocalizerConfig(config_reader::ConfigReader& config, const std::string& path_prefix = "");

void SetEnvironmentConfigs(const std::string& astrobee_configs_path, const std::string& world,
                           const std::string& robot_config_file);

ros::Time RosTimeFromHeader(const std_msgs::Header& header);

Time TimeFromHeader(const std_msgs::Header& header);

Time TimeFromRosTime(const ros::Time& time);

void TimeToHeader(const Time timestamp, std_msgs::Header& header);

gtsam::Pose3 PoseFromMsg(const geometry_msgs::PoseStamped& msg);

gtsam::Pose3 PoseFromMsg(const geometry_msgs::Pose& msg_pose);

void PoseToMsg(const gtsam::Pose3& pose, geometry_msgs::Pose& msg_pose);

geometry_msgs::TransformStamped PoseToTF(const Eigen::Isometry3d& pose, const std::string& parent_frame,
                                         const std::string& child_frame, const Time timestamp,
                                         const std::string& platform_name = "");

geometry_msgs::TransformStamped PoseToTF(const gtsam::Pose3& pose, const std::string& parent_frame,
                                         const std::string& child_frame, const Time timestamp,
                                         const std::string& platform_name = "");

CombinedNavState CombinedNavStateFromMsg(const ff_msgs::EkfState& loc_msg);

void CombinedNavStateToMsg(const CombinedNavState& combined_nav_state, ff_msgs::EkfState& loc_msg);

CombinedNavStateCovariances CombinedNavStateCovariancesFromMsg(const ff_msgs::EkfState& loc_msg);

void CombinedNavStateCovariancesToMsg(const CombinedNavStateCovariances& covariances, ff_msgs::EkfState& loc_msg);

// Returns gravity corrected accelerometer measurement
gtsam::Vector3 RemoveGravityFromAccelerometerMeasurement(const gtsam::Vector3& global_F_gravity,
                                                         const gtsam::Pose3& body_T_imu,
                                                         const gtsam::Pose3& global_T_body,
                                                         const gtsam::Vector3& uncorrected_accelerometer_measurement);
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_UTILITIES_H_
