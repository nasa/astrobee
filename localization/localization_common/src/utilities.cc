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

#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

#include <glog/logging.h>

#include <cstdlib>
#include <string>

namespace localization_common {
Eigen::Isometry3d LoadTransform(config_reader::ConfigReader& config, const std::string& transform_config_name) {
  Eigen::Vector3d body_t_sensor;
  Eigen::Quaterniond body_Q_sensor;
  if (!msg_conversions::config_read_transform(&config, transform_config_name.c_str(), &body_t_sensor, &body_Q_sensor))
    LOG(FATAL) << "Unspecified transform config: " << transform_config_name;
  Eigen::Isometry3d body_T_sensor = Eigen::Isometry3d::Identity();
  body_T_sensor.translation() = body_t_sensor;
  body_T_sensor.linear() = body_Q_sensor.toRotationMatrix();
  return body_T_sensor;
}

gtsam::Pose3 GtPose(const Eigen::Isometry3d& eigen_pose) {
  return gtsam::Pose3(Eigen::Ref<const Eigen::MatrixXd>(eigen_pose.matrix()));
}

void SetEnvironmentConfigs(const std::string& astrobee_configs_path, const std::string& world) {
  setenv("ASTROBEE_RESOURCE_DIR", (astrobee_configs_path + "/resources").c_str(), true);
  setenv("ASTROBEE_CONFIG_DIR", (astrobee_configs_path + "/config").c_str(), true);
  // TODO(rsoussan): pass this as an argument
  setenv("ASTROBEE_ROBOT", (astrobee_configs_path + "/config/robots/bumble.config").c_str(), true);
  setenv("ASTROBEE_WORLD", world.c_str(), true);
}

ros::Time RosTimeFromHeader(const std_msgs::Header& header) { return ros::Time(header.stamp.sec, header.stamp.nsec); }

Time TimeFromHeader(const std_msgs::Header& header) { return GetTime(header.stamp.sec, header.stamp.nsec); }

gtsam::Pose3 PoseFromMsg(const geometry_msgs::Pose& msg_pose) {
  return gtsam::Pose3(RotationFromMsg<geometry_msgs::Orientation, gtsam::Rot3>(msg_pose.orientation),
                      VectorFromMsg<geometry_msgs::Point, gtsam::Point3>(msg_pose.position));
}

void PoseToMsg(const gtsam::Pose3& pose, geometry_msgs::Pose& msg_pose) {
  const gtsam::Quaternion quaternion = pose.rotation().toQuaternion();
  RotationToMsg(quaternion, msg_pose.orientation);
  VectorToMsg(pose.translation(), msg_pose.position);
}
}  // namespace localization_common
