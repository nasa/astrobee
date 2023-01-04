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

#include <graph_localizer/serialization.h>
#include <graph_localizer/utilities.h>
#include <imu_integration/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

#include <cstdlib>
#include <string>

namespace graph_localizer {
namespace go = graph_optimizer;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;

bool ValidVLMsg(const ff_msgs::VisualLandmarks& visual_landmarks_msg, const int min_num_landmarks) {
  return (static_cast<int>(visual_landmarks_msg.landmarks.size()) >= min_num_landmarks);
}

bool ValidDepthMsg(const ff_msgs::DepthLandmarks& depth_landmarks_msg) {
  return (static_cast<int>(depth_landmarks_msg.landmarks.size()) >= 0);
}

ff_msgs::GraphState GraphStateMsg(const lc::CombinedNavState& combined_nav_state,
                                  const lc::CombinedNavStateCovariances& covariances,
                                  const FeatureCounts& detected_feature_counts, const bool estimating_bias,
                                  const double position_log_det_threshold, const double orientation_log_det_threshold,
                                  const bool standstill, const GraphLocalizerStats& graph_stats,
                                  const lm::FanSpeedMode fan_speed_mode) {
  ff_msgs::GraphState loc_msg;

  // Set Header Frames
  loc_msg.header.frame_id = "world";
  loc_msg.child_frame_id = "body";

  // Set CombinedNavState
  lc::CombinedNavStateToMsg(combined_nav_state, loc_msg);

  // Set Variances
  lc::CombinedNavStateCovariancesToMsg(covariances, loc_msg);

  // Set Confidence
  // Controller can't handle a confidence other than 0.  TODO(rsoussan): switch back when this is fixed.
  loc_msg.confidence = 0;  // covariances.PoseConfidence(position_log_det_threshold, orientation_log_det_threshold);

  // Set Graph Feature Counts/Information
  loc_msg.num_detected_of_features = detected_feature_counts.of;
  loc_msg.num_detected_ml_features = detected_feature_counts.vl;
  loc_msg.num_detected_ar_features = detected_feature_counts.ar;
  loc_msg.estimating_bias = estimating_bias;

  // Set Graph Stats
  loc_msg.iterations = graph_stats.iterations_averager_.last_value();
  loc_msg.optimization_time = graph_stats.optimization_timer_.last_value();
  loc_msg.update_time = graph_stats.update_timer_.last_value();
  loc_msg.num_factors = graph_stats.num_factors_averager_.last_value();
  loc_msg.num_of_factors = graph_stats.num_optical_flow_factors_averager_.last_value();
  loc_msg.num_ml_projection_factors = graph_stats.num_loc_proj_factors_averager_.last_value();
  loc_msg.num_ml_pose_factors = graph_stats.num_loc_pose_factors_averager_.last_value();
  loc_msg.num_states = graph_stats.num_states_averager_.last_value();
  // Other
  loc_msg.standstill = standstill;
  loc_msg.fan_speed_mode = static_cast<uint8_t>(fan_speed_mode);
  return loc_msg;
}

ff_msgs::LocalizationGraph GraphMsg(const GraphLocalizer& graph_localizer) {
  ff_msgs::LocalizationGraph graph_msg;

  // Set Header Frames
  graph_msg.header.frame_id = "world";
  graph_msg.child_frame_id = "body";

  // TODO(rsoussan): set correct time
  lc::TimeToHeader(5, graph_msg.header);

  graph_msg.serialized_graph = SerializeBinary(graph_localizer);
  return graph_msg;
}

// TODO(rsoussan): Move these
geometry_msgs::PoseStamped PoseMsg(const Eigen::Isometry3d& global_T_body, const std_msgs::Header& header) {
  geometry_msgs::PoseStamped pose_msg;
  pose_msg.header = header;
  mc::EigenPoseToMsg(global_T_body, pose_msg.pose);
  return pose_msg;
}

geometry_msgs::PoseStamped PoseMsg(const Eigen::Isometry3d& global_T_body, const lc::Time time) {
  std_msgs::Header header;
  lc::TimeToHeader(time, header);
  header.frame_id = "world";
  return PoseMsg(global_T_body, header);
}

geometry_msgs::PoseStamped PoseMsg(const gtsam::Pose3& global_T_body, const lc::Time time) {
  return PoseMsg(lc::EigenPose(global_T_body), time);
}

geometry_msgs::PoseStamped PoseMsg(const lm::TimestampedPose& timestamped_pose) {
  return PoseMsg(timestamped_pose.pose, timestamped_pose.time);
}
}  // namespace graph_localizer
