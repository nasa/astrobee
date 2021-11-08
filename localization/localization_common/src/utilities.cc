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

#include <camera/camera_params.h>
#include <ff_msgs/GraphState.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

#include <cstdlib>
#include <string>

namespace localization_common {
namespace mc = msg_conversions;

gtsam::Pose3 LoadTransform(config_reader::ConfigReader& config, const std::string& transform_config_name) {
  const auto body_T_sensor = mc::LoadEigenTransform(config, transform_config_name);
  return GtPose(body_T_sensor);
}

gtsam::Vector3 LoadVector3(config_reader::ConfigReader& config, const std::string& config_name) {
  Eigen::Vector3d vec;
  mc::config_read_vector(&config, config_name.c_str(), &vec);
  return vec;
}

gtsam::Cal3_S2 LoadCameraIntrinsics(config_reader::ConfigReader& config, const std::string& intrinsics_config_name) {
  const camera::CameraParameters cam_params(&config, intrinsics_config_name.c_str());
  const auto intrinsics = cam_params.GetIntrinsicMatrix<camera::UNDISTORTED_C>();
  // Assumes zero skew
  return gtsam::Cal3_S2(intrinsics(0, 0), intrinsics(1, 1), 0, intrinsics(0, 2), intrinsics(1, 2));
}

gtsam::Pose3 GtPose(const Eigen::Isometry3d& eigen_pose) {
  return gtsam::Pose3(Eigen::Ref<const Eigen::MatrixXd>(eigen_pose.matrix()));
}

Eigen::Isometry3d EigenPose(const CombinedNavState& combined_nav_state) { return EigenPose(combined_nav_state.pose()); }

Eigen::Isometry3d EigenPose(const gtsam::Pose3& pose) { return Eigen::Isometry3d(pose.matrix()); }

gtsam::Pose3 PoseFromMsgWithExtrinsics(const geometry_msgs::Pose& pose, const gtsam::Pose3& sensor_T_body) {
  const gtsam::Pose3 global_T_sensor = PoseFromMsg(pose);
  const gtsam::Pose3 global_T_body = global_T_sensor * sensor_T_body;
  return global_T_body;
}

void LoadGraphLocalizerConfig(config_reader::ConfigReader& config, const std::string& path_prefix) {
  config.AddFile((path_prefix + "graph_localizer.config").c_str());
  LogDebug("LoadGraphLocalizerconfig: Loaded graph localizer config.");
}

void SetEnvironmentConfigs(const std::string& astrobee_configs_path, const std::string& world,
                           const std::string& robot_config_file) {
  setenv("ASTROBEE_RESOURCE_DIR", (astrobee_configs_path + "/resources").c_str(), true);
  setenv("ASTROBEE_CONFIG_DIR", (astrobee_configs_path + "/config").c_str(), true);
  setenv("ASTROBEE_ROBOT", (astrobee_configs_path + "/" + robot_config_file).c_str(), true);
  setenv("ASTROBEE_WORLD", world.c_str(), true);
}

ros::Time RosTimeFromHeader(const std_msgs::Header& header) { return ros::Time(header.stamp.sec, header.stamp.nsec); }

Time TimeFromHeader(const std_msgs::Header& header) { return GetTime(header.stamp.sec, header.stamp.nsec); }

Time TimeFromRosTime(const ros::Time& time) { return GetTime(time.sec, time.nsec); }

void TimeToHeader(const Time timestamp, std_msgs::Header& header) {
  ros::Time ros_time(timestamp);
  header.stamp.sec = ros_time.sec;
  header.stamp.nsec = ros_time.nsec;
}

gtsam::Pose3 PoseFromMsg(const geometry_msgs::PoseStamped& msg) { return PoseFromMsg(msg.pose); }

gtsam::Pose3 PoseFromMsg(const geometry_msgs::Pose& msg_pose) {
  return gtsam::Pose3(mc::RotationFromMsg<gtsam::Rot3, geometry_msgs::Quaternion>(msg_pose.orientation),
                      mc::VectorFromMsg<gtsam::Point3, geometry_msgs::Point>(msg_pose.position));
}

void PoseToMsg(const gtsam::Pose3& pose, geometry_msgs::Pose& msg_pose) {
  const gtsam::Quaternion quaternion = pose.rotation().toQuaternion();
  mc::RotationToMsg(quaternion, msg_pose.orientation);
  mc::VectorToMsg(pose.translation(), msg_pose.position);
}

geometry_msgs::TransformStamped PoseToTF(const gtsam::Pose3& pose, const std::string& parent_frame,
                                         const std::string& child_frame, const Time timestamp,
                                         const std::string& platform_name) {
  return PoseToTF(EigenPose(pose), parent_frame, child_frame, timestamp, platform_name);
}

geometry_msgs::TransformStamped PoseToTF(const Eigen::Isometry3d& pose, const std::string& parent_frame,
                                         const std::string& child_frame, const Time timestamp,
                                         const std::string& platform_name) {
  geometry_msgs::TransformStamped transform;
  TimeToHeader(timestamp, transform.header);
  transform.header.frame_id = parent_frame;
  transform.child_frame_id = platform_name + child_frame;
  mc::EigenPoseToMsg(pose, transform.transform);
  return transform;
}

CombinedNavState CombinedNavStateFromMsg(const ff_msgs::GraphState& loc_msg) {
  const auto pose = PoseFromMsg(loc_msg.pose);
  const auto velocity = mc::VectorFromMsg<gtsam::Velocity3, geometry_msgs::Vector3>(loc_msg.velocity);
  const auto accel_bias = mc::VectorFromMsg<gtsam::Vector3, geometry_msgs::Vector3>(loc_msg.accel_bias);
  const auto gyro_bias = mc::VectorFromMsg<gtsam::Vector3, geometry_msgs::Vector3>(loc_msg.gyro_bias);
  const auto timestamp = TimeFromHeader(loc_msg.header);
  return CombinedNavState(pose, velocity, gtsam::imuBias::ConstantBias(accel_bias, gyro_bias), timestamp);
}

CombinedNavStateCovariances CombinedNavStateCovariancesFromMsg(const ff_msgs::GraphState& loc_msg) {
  const Eigen::Vector3d orientation_variances = mc::CovDiagToVariances(&loc_msg.cov_diag[0]);
  const Eigen::Vector3d gyro_bias_variances = mc::CovDiagToVariances(&loc_msg.cov_diag[3]);
  const Eigen::Vector3d velocity_variances = mc::CovDiagToVariances(&loc_msg.cov_diag[6]);
  const Eigen::Vector3d accelerometer_bias_variances = mc::CovDiagToVariances(&loc_msg.cov_diag[9]);
  const Eigen::Vector3d position_variances = mc::CovDiagToVariances(&loc_msg.cov_diag[12]);
  return CombinedNavStateCovariances(position_variances, orientation_variances, velocity_variances,
                                     accelerometer_bias_variances, gyro_bias_variances);
}

gtsam::Vector3 RemoveGravityFromAccelerometerMeasurement(const gtsam::Vector3& global_F_gravity,
                                                         const gtsam::Pose3& body_T_imu,
                                                         const gtsam::Pose3& global_T_body,
                                                         const gtsam::Vector3& uncorrected_accelerometer_measurement) {
  const gtsam::Rot3 global_R_imu = global_T_body.rotation() * body_T_imu.rotation();
  const gtsam::Vector3 imu_F_gravity = global_R_imu.inverse() * global_F_gravity;
  // Add gravity correction to measurement to offset negatively measured gravity
  return (uncorrected_accelerometer_measurement + imu_F_gravity);
}

Eigen::Isometry3d Isometry3d(const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation) {
  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.translation() = translation;
  pose.linear() = rotation;
  return pose;
}

double Deg2Rad(const double degrees) { return M_PI / 180.0 * degrees; }

double Rad2Deg(const double radians) { return 180.0 / M_PI * radians; }

Eigen::Matrix3d RotationFromEulerAngles(const double yaw, const double pitch, const double roll) {
  const Eigen::AngleAxisd yaw_aa = Eigen::AngleAxisd(Deg2Rad(yaw), Eigen::Vector3d::UnitZ());
  const Eigen::AngleAxisd pitch_aa = Eigen::AngleAxisd(Deg2Rad(pitch), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd roll_aa = Eigen::AngleAxisd(Deg2Rad(roll), Eigen::Vector3d::UnitX());
  const Eigen::Matrix3d rotation(yaw_aa * yaw_aa * pitch_aa * yaw_aa * pitch_aa * roll_aa);
  return rotation;
}

std::pair<Eigen::Vector2d, Eigen::Vector2d> FocalLengthsAndPrincipalPoints(const Eigen::Matrix3d& intrinsics) {
  const Eigen::Vector2d focal_lengths(intrinsics(0, 0), intrinsics(1, 1));
  const Eigen::Vector2d principal_points(intrinsics(0, 2), intrinsics(1, 2));
  return std::make_pair(focal_lengths, principal_points);
}
}  // namespace localization_common
