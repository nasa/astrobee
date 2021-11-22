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
#include <ff_msgs/GraphState.h>
#include <ff_msgs/VisualLandmarks.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/combined_nav_state_covariances.h>
#include <localization_common/time.h>
#include <msg_conversions/msg_conversions.h>

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>

#include <Eigen/Core>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <std_msgs/Header.h>

#include <string>
#include <utility>

namespace localization_common {
gtsam::Pose3 LoadTransform(config_reader::ConfigReader& config, const std::string& transform_config_name);

gtsam::Vector3 LoadVector3(config_reader::ConfigReader& config, const std::string& config_name);

gtsam::Cal3_S2 LoadCameraIntrinsics(config_reader::ConfigReader& config, const std::string& intrinsics_config_name);

gtsam::Pose3 GtPose(const Eigen::Isometry3d& eigen_pose);

Eigen::Isometry3d EigenPose(const CombinedNavState& combined_nav_state);

Eigen::Isometry3d EigenPose(const gtsam::Pose3& pose);

// Returns pose in body frame
gtsam::Pose3 PoseFromMsgWithExtrinsics(const geometry_msgs::Pose& pose, const gtsam::Pose3& sensor_T_body);

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

CombinedNavState CombinedNavStateFromMsg(const ff_msgs::GraphState& loc_msg);

CombinedNavStateCovariances CombinedNavStateCovariancesFromMsg(const ff_msgs::GraphState& loc_msg);

// Returns gravity corrected accelerometer measurement
gtsam::Vector3 RemoveGravityFromAccelerometerMeasurement(const gtsam::Vector3& global_F_gravity,
                                                         const gtsam::Pose3& body_T_imu,
                                                         const gtsam::Pose3& global_T_body,
                                                         const gtsam::Vector3& uncorrected_accelerometer_measurement);

template <class LocMsgType>
void CombinedNavStateToMsg(const CombinedNavState& combined_nav_state, LocMsgType& loc_msg);

template <class LocMsgType>
void CombinedNavStateCovariancesToMsg(const CombinedNavStateCovariances& covariances, LocMsgType& loc_msg);

template <typename MatrixType>
double LogDeterminant(const MatrixType& matrix);

double PoseCovarianceSane(const Eigen::Matrix<double, 6, 6>& pose_covariance,
                          const double position_covariance_threshold = 0,
                          const double orientation_covariance_threshold = 0,
                          const bool check_position_covariance = true, const bool check_orientation_covariance = true);

Eigen::Isometry3d Isometry3d(const Eigen::Vector3d& translation, const Eigen::Matrix3d& rotation);

double Deg2Rad(const double degrees);

double Rad2Deg(const double radians);

// Assumes angles in degrees, ordered as rho, phi, z
Eigen::Vector3d CylindricalToCartesian(const Eigen::Vector3d& cylindrical_coordinates);

// Uses Euler Angles in intrinsic ypr representation in degrees
Eigen::Matrix3d RotationFromEulerAngles(const double yaw, const double pitch, const double roll);

Eigen::Vector2d FocalLengths(const Eigen::Matrix3d& intrinsics);

Eigen::Vector2d PrincipalPoints(const Eigen::Matrix3d& intrinsics);

Eigen::Isometry3d FrameChangeRelativeTransform(const Eigen::Isometry3d& a_F_relative_transform,
                                               const Eigen::Isometry3d& b_T_a);

// Implementations
template <class LocMsgType>
void CombinedNavStateToMsg(const CombinedNavState& combined_nav_state, LocMsgType& loc_msg) {
  PoseToMsg(combined_nav_state.pose(), loc_msg.pose);
  msg_conversions::VectorToMsg(combined_nav_state.velocity(), loc_msg.velocity);
  msg_conversions::VectorToMsg(combined_nav_state.bias().accelerometer(), loc_msg.accel_bias);
  msg_conversions::VectorToMsg(combined_nav_state.bias().gyroscope(), loc_msg.gyro_bias);
  TimeToHeader(combined_nav_state.timestamp(), loc_msg.header);
}

template <class LocMsgType>
void CombinedNavStateCovariancesToMsg(const CombinedNavStateCovariances& covariances, LocMsgType& loc_msg) {
  // Orientation (0-2)
  msg_conversions::VariancesToCovDiag(covariances.orientation_variances(), &loc_msg.cov_diag[0]);

  // Gyro Bias (3-5)
  msg_conversions::VariancesToCovDiag(covariances.gyro_bias_variances(), &loc_msg.cov_diag[3]);

  // Velocity (6-8)
  msg_conversions::VariancesToCovDiag(covariances.velocity_variances(), &loc_msg.cov_diag[6]);

  // Accel Bias (9-11)
  msg_conversions::VariancesToCovDiag(covariances.accel_bias_variances(), &loc_msg.cov_diag[9]);

  // Position (12-14)
  msg_conversions::VariancesToCovDiag(covariances.position_variances(), &loc_msg.cov_diag[12]);
}

template <typename MatrixType>
double LogDeterminant(const MatrixType& matrix) {
  return std::log10(matrix.determinant());
}
}  // namespace localization_common

#endif  // LOCALIZATION_COMMON_UTILITIES_H_
