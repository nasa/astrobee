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

#include <graph_localizer/utilities.h>
#include <imu_integration/utilities.h>
#include <localization_common/utilities.h>

#include <glog/logging.h>

#include <cstdlib>
#include <string>

namespace graph_localizer {
namespace lc = localization_common;
namespace lm = localization_measurements;
bool ValidPointSet(const std::deque<lm::FeaturePoint>& points, const double min_avg_distance_from_mean) {
  if (points.size() < 2) return false;

  // Calculate mean point and avg distance from mean
  Eigen::Vector2d sum_of_points = Eigen::Vector2d::Zero();
  for (const auto& point : points) {
    sum_of_points += point.image_point;
  }
  const Eigen::Vector2d mean_point = sum_of_points / points.size();

  double sum_of_distances_from_mean = 0;
  for (const auto& point : points) {
    const Eigen::Vector2d mean_centered_point = point.image_point - mean_point;
    sum_of_distances_from_mean += mean_centered_point.norm();
  }
  const double average_distance_from_mean = sum_of_distances_from_mean / points.size();
  return (average_distance_from_mean >= min_avg_distance_from_mean);
}

geometry_msgs::PoseWithCovarianceStamped LatestPoseMsg(const GraphLocalizer& graph_localizer) {
  Eigen::Isometry3d global_T_body_graph_latest;
  double latest_graph_timestamp;
  graph_localizer.LatestPose(global_T_body_graph_latest, latest_graph_timestamp);
  const ros::Time latest_graph_time(latest_graph_timestamp);
  std_msgs::Header header;
  header.stamp.sec = latest_graph_time.sec;
  header.stamp.nsec = latest_graph_time.nsec;
  const auto latest_graph_localization_pose_msg = PoseMsg(global_T_body_graph_latest, header);
  return latest_graph_localization_pose_msg;
}

Eigen::Isometry3d EigenPose(const ff_msgs::VisualLandmarks& vl_features, const Eigen::Isometry3d& cam_T_body) {
  Eigen::Isometry3d global_T_cam;
  global_T_cam.translation() << vl_features.pose.position.x, vl_features.pose.position.y, vl_features.pose.position.z;
  const Eigen::Quaterniond global_Q_cam(vl_features.pose.orientation.w, vl_features.pose.orientation.x,
                                        vl_features.pose.orientation.y, vl_features.pose.orientation.z);
  global_T_cam.linear() = global_Q_cam.toRotationMatrix();
  const Eigen::Isometry3d global_T_body = global_T_cam * cam_T_body;
  return global_T_body;
}

ff_msgs::EkfState EkfStateMsg(const lm::CombinedNavState& combined_nav_state, const Eigen::Vector3d& acceleration,
                              const Eigen::Vector3d& angular_velocity,
                              const lm::CombinedNavStateCovariances& covariances,
                              const int num_optical_flow_features_in_last_measurement,
                              const int num_sparse_mapping_features_in_last_measurement, const bool estimating_bias) {
  ff_msgs::EkfState loc_msg;

  // Set Header
  const ros::Time timestamp(combined_nav_state.timestamp());
  loc_msg.header.stamp.sec = timestamp.sec;
  loc_msg.header.stamp.nsec = timestamp.nsec;
  loc_msg.header.frame_id = "world";
  loc_msg.child_frame_id = "body";

  // Set Pose
  lc::PoseToMsg(combined_nav_state.pose(), loc_msg.pose);

  // Set Velocity
  lc::VectorToMsg(combined_nav_state.velocity(), loc_msg.velocity);

  // Set Gyro Bias
  lc::VectorToMsg(combined_nav_state.bias().gyroscope(), loc_msg.gyro_bias);

  // Set Acceleration Bias
  lc::VectorToMsg(combined_nav_state.bias().accelerometer(), loc_msg.accel_bias);

  // Set Acceleration
  lc::VectorToMsg(acceleration, loc_msg.accel);

  // Set Angular Velocity
  lc::VectorToMsg(angular_velocity, loc_msg.omega);

  // Set Variances
  // Orientation (0-2)
  lc::VariancesToCovDiag(covariances.orientation_variances(), &loc_msg.cov_diag[0]);

  // Gyro Bias (3-5)
  lc::VariancesToCovDiag(covariances.gyro_bias_variances(), &loc_msg.cov_diag[3]);

  // Velocity (6-8)
  lc::VariancesToCovDiag(covariances.velocity_variances(), &loc_msg.cov_diag[6]);

  // Accel Bias (9-11)
  lc::VariancesToCovDiag(covariances.accel_bias_variances(), &loc_msg.cov_diag[9]);

  // Position (12-14)
  lc::VariancesToCovDiag(covariances.position_variances(), &loc_msg.cov_diag[12]);

  // Set Confidence
  loc_msg.confidence = covariances.PoseConfidence();

  // Set Graph Feature Counts/Information
  loc_msg.of_count = num_optical_flow_features_in_last_measurement;
  loc_msg.ml_count = num_sparse_mapping_features_in_last_measurement;
  loc_msg.estimating_bias = estimating_bias;

  return loc_msg;
}

geometry_msgs::PoseWithCovarianceStamped PoseMsg(const Eigen::Isometry3d& global_T_body,
                                                 const std_msgs::Header& header) {
  geometry_msgs::PoseWithCovarianceStamped pose_msg;
  pose_msg.header = header;

  pose_msg.pose.pose.position.x = global_T_body.translation().x();
  pose_msg.pose.pose.position.y = global_T_body.translation().y();
  pose_msg.pose.pose.position.z = global_T_body.translation().z();

  const Eigen::Quaterniond global_Q_body(global_T_body.linear());
  pose_msg.pose.pose.orientation.x = global_Q_body.x();
  pose_msg.pose.pose.orientation.y = global_Q_body.y();
  pose_msg.pose.pose.orientation.z = global_Q_body.z();
  pose_msg.pose.pose.orientation.w = global_Q_body.w();

  return pose_msg;
}

void EstimateAndSetImuBiases(const lm::ImuMeasurement& imu_measurement,
                             const int num_imu_measurements_per_bias_estimate,
                             std::vector<lm::ImuMeasurement>& imu_bias_measurements,
                             GraphLocInitialization& graph_loc_initialization) {
  Eigen::Vector3d accelerometer_bias;
  Eigen::Vector3d gyro_bias;
  if (!imu_integration::EstimateAndSetImuBiases(imu_measurement, num_imu_measurements_per_bias_estimate,
                                                imu_bias_measurements, accelerometer_bias, gyro_bias))
    return;

  graph_loc_initialization.SetBiases(accelerometer_bias, gyro_bias);
}
}  // namespace graph_localizer
