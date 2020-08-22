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

#include <ff_msgs/EkfState.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

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

void TimeToHeader(const Time timestamp, std_msgs::Header& header) {
  ros::Time ros_time(timestamp);
  header.stamp.sec = ros_time.sec;
  header.stamp.nsec = ros_time.nsec;
}

gtsam::Pose3 PoseFromMsg(const geometry_msgs::Pose& msg_pose) {
  return gtsam::Pose3(RotationFromMsg<gtsam::Rot3, geometry_msgs::Quaternion>(msg_pose.orientation),
                      VectorFromMsg<gtsam::Point3, geometry_msgs::Point>(msg_pose.position));
}

void PoseToMsg(const gtsam::Pose3& pose, geometry_msgs::Pose& msg_pose) {
  const gtsam::Quaternion quaternion = pose.rotation().toQuaternion();
  RotationToMsg(quaternion, msg_pose.orientation);
  VectorToMsg(pose.translation(), msg_pose.position);
}

void VariancesToCovDiag(const Eigen::Vector3d& variances, float* const cov_diag) {
  cov_diag[0] = variances.x();
  cov_diag[1] = variances.y();
  cov_diag[2] = variances.z();
}

Eigen::Vector3d CovDiagToVariances(const float* const cov_diag) {
  return Eigen::Vector3d(cov_diag[0], cov_diag[1], cov_diag[2]);
}

CombinedNavState CreateCombinedNavState(const ff_msgs::EkfState& loc_msg) {
  const auto pose = PoseFromMsg(loc_msg.pose);
  const auto velocity = VectorFromMsg<gtsam::Velocity3, geometry_msgs::Vector3>(loc_msg.velocity);
  const auto accel_bias = VectorFromMsg<gtsam::Vector3, geometry_msgs::Vector3>(loc_msg.accel_bias);
  const auto gyro_bias = VectorFromMsg<gtsam::Vector3, geometry_msgs::Vector3>(loc_msg.gyro_bias);
  const auto timestamp = TimeFromHeader(loc_msg.header);
  return CombinedNavState(pose, velocity, gtsam::imuBias::ConstantBias(accel_bias, gyro_bias), timestamp);
}

void CombinedNavStateToMsg(const CombinedNavState& combined_nav_state, ff_msgs::EkfState& loc_msg) {
  PoseToMsg(combined_nav_state.pose(), loc_msg.pose);
  VectorToMsg(combined_nav_state.velocity(), loc_msg.velocity);
  VectorToMsg(combined_nav_state.bias().accelerometer(), loc_msg.accel_bias);
  VectorToMsg(combined_nav_state.bias().gyroscope(), loc_msg.gyro_bias);
  TimeToHeader(combined_nav_state.timestamp(), loc_msg.header);
}

CombinedNavStateCovariances CreateCombinedNavStateCovariances(const ff_msgs::EkfState& loc_msg) {
  const Eigen::Vector3d orientation_variances = CovDiagToVariances(&loc_msg.cov_diag[0]);
  const Eigen::Vector3d gyro_bias_variances = CovDiagToVariances(&loc_msg.cov_diag[3]);
  const Eigen::Vector3d velocity_variances = CovDiagToVariances(&loc_msg.cov_diag[6]);
  const Eigen::Vector3d accelerometer_bias_variances = CovDiagToVariances(&loc_msg.cov_diag[9]);
  const Eigen::Vector3d position_variances = CovDiagToVariances(&loc_msg.cov_diag[12]);
  return CombinedNavStateCovariances(position_variances, orientation_variances, velocity_variances,
                                     accelerometer_bias_variances, gyro_bias_variances);
}

void CombinedNavStateCovariancesToMsg(const CombinedNavStateCovariances& covariances, ff_msgs::EkfState& loc_msg) {
  // Orientation (0-2)
  VariancesToCovDiag(covariances.orientation_variances(), &loc_msg.cov_diag[0]);

  // Gyro Bias (3-5)
  VariancesToCovDiag(covariances.gyro_bias_variances(), &loc_msg.cov_diag[3]);

  // Velocity (6-8)
  VariancesToCovDiag(covariances.velocity_variances(), &loc_msg.cov_diag[6]);

  // Accel Bias (9-11)
  VariancesToCovDiag(covariances.accel_bias_variances(), &loc_msg.cov_diag[9]);

  // Position (12-14)
  VariancesToCovDiag(covariances.position_variances(), &loc_msg.cov_diag[12]);
}
}  // namespace localization_common
