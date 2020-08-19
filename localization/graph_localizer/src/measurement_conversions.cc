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

#include <graph_localizer/measurement_conversions.h>

namespace graph_localizer {
Eigen::Isometry3d EigenPose(const ff_msgs::VisualLandmarks& vl_features, const Eigen::Isometry3d& cam_T_body) {
  Eigen::Isometry3d global_T_cam;
  global_T_cam.translation() << vl_features.pose.position.x, vl_features.pose.position.y, vl_features.pose.position.z;
  const Eigen::Quaterniond global_Q_cam(vl_features.pose.orientation.w, vl_features.pose.orientation.x,
                                        vl_features.pose.orientation.y, vl_features.pose.orientation.z);
  global_T_cam.linear() = global_Q_cam.toRotationMatrix();
  const Eigen::Isometry3d global_T_body = global_T_cam * cam_T_body;
  return global_T_body;
}

MatchedProjectionsMeasurement MakeMatchedProjectionsMeasurement(const ff_msgs::VisualLandmarks& visual_landmarks) {
  MatchedProjectionsMeasurement matched_projections_measurement;
  matched_projections_measurement.matched_projections.reserve(visual_landmarks.landmarks.size());
  const Time timestamp = GetTime(visual_landmarks.header.stamp.sec, visual_landmarks.header.stamp.nsec);
  matched_projections_measurement.timestamp = timestamp;

  for (const auto& landmark : visual_landmarks.landmarks) {
    const ImagePoint image_point(landmark.u, landmark.v);
    const MapPoint map_point(landmark.x, landmark.y, landmark.z);
    matched_projections_measurement.matched_projections.emplace_back(image_point, map_point, timestamp);
  }

  return matched_projections_measurement;
}

void FrameChangeMatchedProjectionsMeasurement(MatchedProjectionsMeasurement& matched_projections_measurement,
                                              const gtsam::Pose3& b_T_a) {
  for (auto& matched_projection : matched_projections_measurement.matched_projections) {
    matched_projection.map_point = b_T_a * matched_projection.map_point;
  }
}

FeaturePointsMeasurement MakeFeaturePointsMeasurement(const ff_msgs::Feature2dArray& optical_flow_feature_points) {
  FeaturePointsMeasurement feature_points_measurement;
  feature_points_measurement.feature_points.reserve(optical_flow_feature_points.feature_array.size());
  Time timestamp = GetTime(optical_flow_feature_points.header.stamp.sec, optical_flow_feature_points.header.stamp.nsec);
  feature_points_measurement.timestamp = timestamp;
  // TODO(rsoussan): put this somewhere else?
  static int image_id = 0;
  ++image_id;

  for (const auto& feature : optical_flow_feature_points.feature_array) {
    feature_points_measurement.feature_points.emplace_back(
        FeaturePoint(feature.x, feature.y, image_id, feature.id, timestamp));
  }

  return feature_points_measurement;
}

ros::Time RosTimeFromHeader(const std_msgs::Header& header) { return ros::Time(header.stamp.sec, header.stamp.nsec); }

Time TimeFromHeader(const std_msgs::Header& header) { return GetTime(header.stamp.sec, header.stamp.nsec); }

ff_msgs::EkfState EkfStateMsg(const CombinedNavState& combined_nav_state, const Eigen::Vector3d& acceleration,
                              const Eigen::Vector3d& angular_velocity, const CombinedNavStateCovariances& covariances) {
  ff_msgs::EkfState loc_msg;

  // Set Header
  const ros::Time timestamp(combined_nav_state.timestamp());
  loc_msg.header.stamp.sec = timestamp.sec;
  loc_msg.header.stamp.nsec = timestamp.nsec;
  loc_msg.header.frame_id = "world";
  loc_msg.child_frame_id = "body";

  // Set Pose
  loc_msg.pose.position.x = combined_nav_state.pose().x();
  loc_msg.pose.position.y = combined_nav_state.pose().y();
  loc_msg.pose.position.z = combined_nav_state.pose().z();

  const auto nav_state_quaternion = combined_nav_state.pose().rotation().toQuaternion();
  loc_msg.pose.orientation.x = nav_state_quaternion.x();
  loc_msg.pose.orientation.y = nav_state_quaternion.y();
  loc_msg.pose.orientation.z = nav_state_quaternion.z();
  loc_msg.pose.orientation.w = nav_state_quaternion.w();

  // Set Velocity
  loc_msg.velocity.x = combined_nav_state.velocity().x();
  loc_msg.velocity.y = combined_nav_state.velocity().y();
  loc_msg.velocity.z = combined_nav_state.velocity().z();

  // Set Gyro Bias
  loc_msg.gyro_bias.x = combined_nav_state.bias().gyroscope().x();
  loc_msg.gyro_bias.y = combined_nav_state.bias().gyroscope().y();
  loc_msg.gyro_bias.z = combined_nav_state.bias().gyroscope().z();

  // Set Acceleration Bias
  loc_msg.accel_bias.x = combined_nav_state.bias().accelerometer().x();
  loc_msg.accel_bias.y = combined_nav_state.bias().accelerometer().y();
  loc_msg.accel_bias.z = combined_nav_state.bias().accelerometer().z();

  loc_msg.estimating_bias = true;

  // Set Acceleration
  loc_msg.accel.x = acceleration.x();
  loc_msg.accel.y = acceleration.y();
  loc_msg.accel.z = acceleration.z();

  // Set Angular Velocity
  loc_msg.omega.x = angular_velocity.x();
  loc_msg.omega.y = angular_velocity.y();
  loc_msg.omega.z = angular_velocity.z();

  // Set Variances
  // Orientation (0-2)
  loc_msg.cov_diag[0] = covariances.orientation_variances().x();
  loc_msg.cov_diag[1] = covariances.orientation_variances().y();
  loc_msg.cov_diag[2] = covariances.orientation_variances().z();
  // Gyro Bias (3-5)
  loc_msg.cov_diag[3] = covariances.gyro_bias_variances().x();
  loc_msg.cov_diag[4] = covariances.gyro_bias_variances().y();
  loc_msg.cov_diag[5] = covariances.gyro_bias_variances().z();
  // Velocity (6-8)
  loc_msg.cov_diag[6] = covariances.velocity_variances().x();
  loc_msg.cov_diag[7] = covariances.velocity_variances().y();
  loc_msg.cov_diag[8] = covariances.velocity_variances().z();
  // Accel Bias (9-11)
  loc_msg.cov_diag[9] = covariances.accel_bias_variances().x();
  loc_msg.cov_diag[10] = covariances.accel_bias_variances().y();
  loc_msg.cov_diag[11] = covariances.accel_bias_variances().z();
  // Position (12-14)
  loc_msg.cov_diag[12] = covariances.position_variances().x();
  loc_msg.cov_diag[13] = covariances.position_variances().y();
  loc_msg.cov_diag[14] = covariances.position_variances().z();

  // Set Confidence
  loc_msg.confidence = covariances.PoseConfidence();

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
}  // namespace graph_localizer
