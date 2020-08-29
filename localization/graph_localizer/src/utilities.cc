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

#include <gtsam/base/serialization.h>

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

boost::optional<geometry_msgs::PoseStamped> LatestPoseMsg(const GraphLocalizer& graph_localizer) {
  const auto latest_combined_nav_state = graph_localizer.LatestCombinedNavState();
  if (!latest_combined_nav_state) {
    LOG(ERROR) << "LatestPoseMsg: Failed to get latest combined nav state.";
    return boost::none;
  }

  const ros::Time latest_graph_time(latest_combined_nav_state->timestamp());
  std_msgs::Header header;
  header.stamp.sec = latest_graph_time.sec;
  header.stamp.nsec = latest_graph_time.nsec;
  const auto latest_graph_localization_pose_msg = PoseMsg(lc::EigenPose(*latest_combined_nav_state), header);
  return latest_graph_localization_pose_msg;
}

ff_msgs::EkfState EkfStateMsg(const lc::CombinedNavState& combined_nav_state, const Eigen::Vector3d& acceleration,
                              const Eigen::Vector3d& angular_velocity,
                              const lc::CombinedNavStateCovariances& covariances,
                              const int num_optical_flow_features_in_last_measurement,
                              const int num_sparse_mapping_features_in_last_measurement, const bool estimating_bias) {
  ff_msgs::EkfState loc_msg;

  // Set Header Frames
  loc_msg.header.frame_id = "world";
  loc_msg.child_frame_id = "body";

  // Set CombinedNavState
  lc::CombinedNavStateToMsg(combined_nav_state, loc_msg);

  // Set Acceleration
  lc::VectorToMsg(acceleration, loc_msg.accel);

  // Set Angular Velocity
  lc::VectorToMsg(angular_velocity, loc_msg.omega);

  // Set Variances
  lc::CombinedNavStateCovariancesToMsg(covariances, loc_msg);

  // Set Confidence
  loc_msg.confidence = covariances.PoseConfidence();

  // Set Graph Feature Counts/Information
  loc_msg.of_count = num_optical_flow_features_in_last_measurement;
  loc_msg.ml_count = num_sparse_mapping_features_in_last_measurement;
  loc_msg.estimating_bias = estimating_bias;

  return loc_msg;
}

ff_msgs::LocalizationGraph GraphMsg(const GraphLocalizer& graph_localizer) {
  ff_msgs::LocalizationGraph graph_msg;

  // Set Header Frames
  graph_msg.header.frame_id = "world";
  graph_msg.child_frame_id = "body";

  // TODO(rsoussan): set correct time
  lc::TimeToHeader(5, graph_msg.header);

  graph_msg.serialized_graph = gtsam::serializeBinary(graph_localizer);
  return graph_msg;
}

geometry_msgs::PoseStamped PoseMsg(const Eigen::Isometry3d& global_T_body, const std_msgs::Header& header) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header = header;
  lc::EigenPoseToMsg(global_T_body, pose_msg.pose);
  return pose_msg;
}

geometry_msgs::PoseStamped PoseMsg(const Eigen::Isometry3d& global_T_body, const lc::Time time) {
  std_msgs::Header header;
  lc::TimeToHeader(time, header);
  return PoseMsg(global_T_body, header);
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
