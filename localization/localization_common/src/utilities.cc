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
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <ros/package.h>

#include <cstdlib>
#include <string>

namespace localization_common {
namespace mc = msg_conversions;

gtsam::Pose3 LoadTransform(config_reader::ConfigReader& config, const std::string& transform_config_name,
                           const std::string& prefix) {
  const auto body_T_sensor = mc::LoadEigenTransform(config, transform_config_name, prefix);
  return GtPose(body_T_sensor);
}

gtsam::Vector3 LoadVector3(config_reader::ConfigReader& config, const std::string& config_name,
                           const std::string& prefix) {
  Eigen::Vector3d vec;
  mc::config_read_vector(&config, (prefix + config_name).c_str(), &vec);
  return vec;
}

Eigen::Matrix3d LoadCameraIntrinsicsMatrix(config_reader::ConfigReader& config,
                                           const std::string& intrinsics_config_name, const std::string& prefix) {
  const camera::CameraParameters cam_params(&config, (prefix + intrinsics_config_name).c_str());
  return cam_params.GetIntrinsicMatrix<camera::UNDISTORTED_C>();
}

gtsam::Cal3_S2 LoadCameraIntrinsics(config_reader::ConfigReader& config, const std::string& intrinsics_config_name,
                                    const std::string& prefix) {
  const auto intrinsics = LoadCameraIntrinsicsMatrix(config, intrinsics_config_name, prefix);
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
  config.AddFile((path_prefix + "localization/graph_localizer.config").c_str());
  config.AddFile("transforms.config");
  config.AddFile("cameras.config");
  config.AddFile("geometry.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  LogDebug("LoadGraphLocalizerConfig: Loaded graph localizer config.");
}

void LoadGraphVIOConfig(config_reader::ConfigReader& config, const std::string& path_prefix) {
  config.AddFile((path_prefix + "localization/graph_vio.config").c_str());
  config.AddFile((path_prefix + "localization/imu_integrator.config").c_str());
  config.AddFile((path_prefix + "localization/imu_filter.config").c_str());
  config.AddFile("transforms.config");
  config.AddFile("cameras.config");
  config.AddFile("geometry.config");
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  LogDebug("LoadGraphVIOConfig: Loaded graph VIO config.");
}

void SetEnvironmentConfigs(const std::string& world, const std::string& robot_config_file) {
  const std::string astrobee_configs_path = ros::package::getPath("astrobee");
  const std::string full_robot_config_file = "config/robots/" + robot_config_file;
  setenv("ASTROBEE_RESOURCE_DIR", (astrobee_configs_path + "/resources").c_str(), true);
  setenv("ASTROBEE_CONFIG_DIR", (astrobee_configs_path + "/config").c_str(), true);
  setenv("ASTROBEE_ROBOT", (astrobee_configs_path + "/" + full_robot_config_file).c_str(), true);
  setenv("ASTROBEE_WORLD", world.c_str(), true);
}

ros::Time RosTimeFromHeader(const std_msgs::Header& header) { return ros::Time(header.stamp.sec, header.stamp.nsec); }

Time TimeFromHeader(const std_msgs::Header& header) { return GetTime(header.stamp.sec, header.stamp.nsec); }

Time TimeFromRosTime(const ros::Time& time) { return GetTime(time.sec, time.nsec); }

void TimeToHeader(const Time timestamp, std_msgs::Header& header) { TimeToMsg(timestamp, header.stamp); }

void TimeToMsg(const Time timestamp, ros::Time& time_msg) { time_msg = ros::Time(timestamp); }

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

CombinedNavState CombinedNavStateFromMsg(const ff_msgs::CombinedNavState& msg) {
  const auto pose = PoseFromMsg(msg.pose.pose.pose);
  const auto velocity = mc::VectorFromMsg<gtsam::Velocity3, geometry_msgs::Vector3>(msg.velocity.velocity);
  const auto accel_bias = mc::VectorFromMsg<gtsam::Vector3, geometry_msgs::Vector3>(msg.imu_bias.accelerometer_bias);
  const auto gyro_bias = mc::VectorFromMsg<gtsam::Vector3, geometry_msgs::Vector3>(msg.imu_bias.gyroscope_bias);
  const auto timestamp = TimeFromHeader(msg.header);
  return CombinedNavState(pose, velocity, gtsam::imuBias::ConstantBias(accel_bias, gyro_bias), timestamp);
}

CombinedNavStateCovariances CombinedNavStateCovariancesFromMsg(const ff_msgs::CombinedNavState& msg) {
  const auto pose_covariance = mc::EigenCovarianceFromMsg<6>(msg.pose.pose.covariance);
  const auto velocity_covariance = mc::EigenCovarianceFromMsg<3>(msg.velocity.covariance);
  const auto imu_bias_covariance = mc::EigenCovarianceFromMsg<6>(msg.imu_bias.covariance);
  return CombinedNavStateCovariances(pose_covariance, velocity_covariance, imu_bias_covariance);
}

TimestampedSet<PoseCovariance> CorrelationCovariancesFromMsg(const ff_msgs::CombinedNavState& msg) {
  TimestampedSet<PoseCovariance> correlation_covariances;
  for (const auto& correlation_covariance : msg.correlation_covariances) {
    correlation_covariances.Add(TimeFromRosTime(correlation_covariance.time),
                                mc::EigenCovarianceFromMsg<6>(correlation_covariance.covariance));
  }
  return correlation_covariances;
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

ff_msgs::CombinedNavState CombinedNavStateToMsg(const CombinedNavState& combined_nav_state,
                                                const PoseCovariance& pose_covariance,
                                                const Eigen::Matrix3d& velocity_covariance,
                                                const Eigen::Matrix<double, 6, 6>& imu_bias_covariance,
                                                const TimestampedSet<PoseCovariance>& correlation_covariances) {
  ff_msgs::CombinedNavState msg;
  TimeToHeader(combined_nav_state.timestamp(), msg.header);
  // Write CombinedNavState
  PoseToMsg(combined_nav_state.pose(), msg.pose.pose.pose);
  mc::VectorToMsg(combined_nav_state.velocity(), msg.velocity.velocity);
  mc::VectorToMsg(combined_nav_state.bias().accelerometer(), msg.imu_bias.accelerometer_bias);
  mc::VectorToMsg(combined_nav_state.bias().gyroscope(), msg.imu_bias.gyroscope_bias);

  // Write covariances
  mc::EigenCovarianceToMsg(pose_covariance, msg.pose.pose.covariance);
  mc::EigenCovarianceToMsg(velocity_covariance, msg.velocity.covariance);
  mc::EigenCovarianceToMsg(imu_bias_covariance, msg.imu_bias.covariance);

  // Write correlation pose covariances if available
  for (const auto& correlation_covariance : correlation_covariances.set()) {
    ff_msgs::PoseCovarianceStamped covariance_msg;
    mc::EigenCovarianceToMsg(correlation_covariance.second, covariance_msg.covariance);
    TimeToMsg(correlation_covariance.first, covariance_msg.time);
    msg.correlation_covariances.push_back(covariance_msg);
  }
  return msg;
}

double Deg2Rad(const double degrees) { return M_PI / 180.0 * degrees; }

double Rad2Deg(const double radians) { return 180.0 / M_PI * radians; }

Eigen::Vector3d CylindricalToCartesian(const Eigen::Vector3d& cylindrical_coordinates) {
  Eigen::Vector3d cartesian_coordinates;
  cartesian_coordinates.x() = cylindrical_coordinates[0] * std::cos(Deg2Rad(cylindrical_coordinates[1]));
  cartesian_coordinates.y() = cylindrical_coordinates[0] * std::sin(Deg2Rad(cylindrical_coordinates[1]));
  cartesian_coordinates.z() = cylindrical_coordinates[2];
  return cartesian_coordinates;
}

Eigen::Matrix3d RotationFromEulerAngles(const double yaw_degrees, const double pitch_degrees,
                                        const double roll_degrees) {
  const Eigen::AngleAxisd yaw_aa = Eigen::AngleAxisd(Deg2Rad(yaw_degrees), Eigen::Vector3d::UnitZ());
  const Eigen::AngleAxisd pitch_aa = Eigen::AngleAxisd(Deg2Rad(pitch_degrees), Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd roll_aa = Eigen::AngleAxisd(Deg2Rad(roll_degrees), Eigen::Vector3d::UnitX());
  // For intrinsics euler angle convention, yaw, pitch, then roll in intrinsic body frame is equivalent to
  // roll, pitch, then yaw in extrinsic global frame
  const Eigen::Matrix3d rotation(roll_aa * pitch_aa * yaw_aa);
  return rotation;
}

PoseCovariance RelativePoseCovariance(const PoseWithCovariance& a, const PoseWithCovariance& b,
                                      const boost::optional<PoseCovariance> covariance_a_b) {
  // Calculate covariance
  // See https://gtsam.org/2021/02/23/uncertainties-part3.html
  // Adjoints
  // Uses convention w_T_a and w_T_b for poses, s.t. adj_w_a is the adjoint for the
  // w_T_a pose.
  const gtsam::Pose3 w_T_a = GtPose(a.pose);
  const gtsam::Pose3 w_T_b = GtPose(b.pose);
  const gtsam::Matrix6 adj_w_a = w_T_a.AdjointMap();
  const gtsam::Matrix6 adj_b_w = w_T_b.inverse().AdjointMap();
  // Helper terms
  const gtsam::Matrix6 h = adj_b_w * adj_w_a;
  const gtsam::Matrix6 h_t = h.transpose();
  // Compute covariance without correlation terms
  gtsam::Matrix6 relative_covariance = h * a.covariance * h_t + b.covariance;
  // Add correlation terms if they exist
  // Assumes correlation_covariances stores covariances a_b rather than b_a
  // const auto covariance_a_b = b.correlation_covariances.Get(*(a.correlation_covariances.LatestTimestamp()));
  if (covariance_a_b) {
    relative_covariance = relative_covariance - h * (*covariance_a_b) - (*covariance_a_b).transpose() * h_t;
  }
  return relative_covariance;
}

Eigen::Isometry3d FrameChangeRelativePose(const Eigen::Isometry3d& a_F_relative_pose, const Eigen::Isometry3d& b_T_a) {
  // Consider for example a sensor odometry measurement, sensor_time_0_T_sensor_time_1.
  // To find the movement of the robot body given static body_T_sensor extrinsics,
  // body_time_0_T_body_time_1 = body_T_sensor * sensor_time_0_T_sensor_time_1 * sensor_T_body
  return b_T_a * a_F_relative_pose * b_T_a.inverse();
}

Eigen::Matrix<double, 6, 6> FrameChangeRelativeCovariance(
  const Eigen::Matrix<double, 6, 6>& a_F_relative_pose_covariance, const Eigen::Isometry3d& b_T_a) {
  const gtsam::Pose3 b_R_a(gtsam::Rot3(b_T_a.linear()), Eigen::Vector3d::Zero());
  return gtsam::TransformCovariance<gtsam::Pose3>(b_R_a)(a_F_relative_pose_covariance);
}

PoseWithCovariance FrameChangeRelativePoseWithCovariance(const PoseWithCovariance& a_F_relative_pose_with_covariance,
                                                         const Eigen::Isometry3d& b_T_a) {
  PoseWithCovariance b_F_relative_pose_with_covariance;
  b_F_relative_pose_with_covariance.pose = FrameChangeRelativePose(a_F_relative_pose_with_covariance.pose, b_T_a);
  b_F_relative_pose_with_covariance.covariance =
    FrameChangeRelativeCovariance(a_F_relative_pose_with_covariance.covariance, b_T_a);
  return b_F_relative_pose_with_covariance;
}

PoseWithCovariance InvertPoseWithCovariance(const PoseWithCovariance& pose_with_covariance) {
  PoseWithCovariance inverted_pose_with_covariance;
  inverted_pose_with_covariance.pose = pose_with_covariance.pose.inverse();
  // TODO(rsoussan): Do this properly!!
  inverted_pose_with_covariance.covariance = pose_with_covariance.covariance;
  return inverted_pose_with_covariance;
}

double PoseCovarianceSane(const Eigen::Matrix<double, 6, 6>& pose_covariance,
                          const double position_covariance_threshold, const double orientation_covariance_threshold,
                          const bool check_position_covariance, const bool check_orientation_covariance) {
  bool sane = true;
  if (check_position_covariance) {
    const double log_det_position_cov = LogDeterminant(pose_covariance.block<3, 3>(0, 0));
    sane &= (log_det_position_cov <= position_covariance_threshold);
  }

  if (check_orientation_covariance) {
    const double log_det_orientation_cov = LogDeterminant(pose_covariance.block<3, 3>(3, 3));
    sane &= (log_det_orientation_cov <= orientation_covariance_threshold);
  }

  return sane;
}

std::pair<std::vector<Eigen::Vector3d>, std::vector<Eigen::Vector3d>> TransformPointsWithNormals(
  const std::vector<Eigen::Vector3d>& points, const std::vector<Eigen::Vector3d>& normals,
  const Eigen::Isometry3d& b_T_a) {
  std::vector<Eigen::Vector3d> transformed_points = Transform(points, b_T_a);
  std::vector<Eigen::Vector3d> rotated_normals = Rotate(normals, b_T_a.linear());
  return std::make_pair(transformed_points, rotated_normals);
}

Eigen::Isometry3d Interpolate(const Eigen::Isometry3d& lower_bound_pose, const Eigen::Isometry3d& upper_bound_pose,
                              const double alpha) {
  const Eigen::Vector3d translation =
    (1.0 - alpha) * lower_bound_pose.translation() + alpha * upper_bound_pose.translation();
  const auto rotation =
    Eigen::Quaterniond(lower_bound_pose.linear()).slerp(alpha, Eigen::Quaterniond(upper_bound_pose.linear()));
  return Isometry3d(translation, rotation);
}

PoseWithCovariance Interpolate(const PoseWithCovariance& lower_bound_pose, const PoseWithCovariance& upper_bound_pose,
                               const double alpha) {
  const Eigen::Isometry3d interpolated_pose = Interpolate(lower_bound_pose.pose, upper_bound_pose.pose, alpha);
  // TODO(rsoussan): Implement this properly
  const PoseCovariance& interpolated_covariance =
    alpha > 0.5 ? upper_bound_pose.covariance : lower_bound_pose.covariance;
  const auto& correlation_covariances =
    alpha > 0.5 ? upper_bound_pose.correlation_covariances : lower_bound_pose.correlation_covariances;

  return PoseWithCovariance(interpolated_pose, interpolated_covariance, correlation_covariances);
}

gtsam::noiseModel::Robust::shared_ptr Robust(const gtsam::SharedNoiseModel& noise, const double huber_k) {
  return gtsam::noiseModel::Robust::Create(gtsam::noiseModel::mEstimator::Huber::Create(huber_k), noise);
}
}  // namespace localization_common
