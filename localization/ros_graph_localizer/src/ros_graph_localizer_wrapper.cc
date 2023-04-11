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

#include <config_reader/config_reader.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <localization_measurements/measurement_conversions.h>
#include <msg_conversions/msg_conversions.h>
#include <ros_graph_localizer/ros_graph_localizer_wrapper.h>

#include <Eigen/Core>

namespace ros_graph_localizer {
namespace gl = graph_localizer;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;

RosGraphLocalizerWrapper::RosGraphLocalizerWrapper(const std::string& graph_config_path_prefix) {
  LoadConfigs(graph_config_path_prefix);
}

void RosGraphLocalizerWrapper::LoadConfigs(const std::string& graph_config_path_prefix) {
  config_reader::ConfigReader config;
  lc::LoadGraphLocalizerConfig(config, graph_config_path_prefix);
  config.AddFile("transforms.config");
  config.AddFile("cameras.config");
  config.AddFile("geometry.config");

  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  // TODO(rsoussan): put this back!!
  // LoadGraphLocalizerParams(config, params_);
}

void RosGraphLocalizerWrapper::SparseMapVisualLandmarksCallback(const ff_msgs::VisualLandmarks& visual_landmarks_msg) {
  const auto msg_time = lc::TimeFromHeader(visual_landmarks_msg.header);
  // Initialize with pose estimate if not initialized yet.
  // Ensure vio data exists before msg time so no gaps occur between first
  // sparse map measurement and future interpolated vio measurements.
  if (!Initialized() && !vio_measurement_buffer_.empty()) {
    const auto latest_vio_measurement_time = vio_measurement_buffer_.Latest()->timestamp;
    if (msg_time > latest_vio_measurement_time) {
      LogError(
        "SparseMapVisualLandmarksCallback: Initial vl msg time more recent than latest buffered vio time, failed to "
        "initialize graph localizer.");
      return;
    }
    const auto world_T_body = lc::PoseFromMsgWithExtrinsics(visual_landmarks_msg.pose,
                                                            params_.sparse_map_loc_factor_adder.body_T_cam.inverse());
    params_.pose_node_adder.start_node = world_T_body;
    params_.pose_node_adder.starting_time = msg_time;
    params_.pose_node_adder.Initialize();
    graph_localizer_.reset(new gl::GraphLocalizer(params_));
    // Only need the first vio measurement before the initial vl msg time
    // to ensure valid pose interpolation.
    vio_measurement_buffer_.RemoveBelowLowerBoundValues(msg_time);
    for (auto it = vio_measurement_buffer_.set().cbegin(); it != vio_measurement_buffer_.set().cend(); ++it) {
      GraphVIOStateCallback(it->second);
    }
    vio_measurement_buffer_.Clear();
  } else {  // Otherwise add measurement to graph
    graph_localizer_->AddSparseMapMatchedProjectionsMeasurement(
      lm::MakeMatchedProjectionsMeasurement(visual_landmarks_msg));
  }
}

void RosGraphLocalizerWrapper::GraphVIOStateCallback(const ff_msgs::GraphVIOState& graph_vio_state_msg) {
  const auto timestamp = lc::TimeFromHeader(graph_vio_state_msg.header);
  // Buffer measurements before initialization so they can be added once initialized.
  if (!Initialized()) {
    vio_measurement_buffer_.Add(timestamp, graph_vio_state_msg);
    return;
  }

  // Check if gap in vio msgs is too large, reset localizer if so.
  if (last_vio_msg_time_ && (timestamp - *last_vio_msg_time_) > params_.max_vio_measurement_gap) {
    LogError("GraphVIOStateCallback: VIO msg gap exceeded, resetting localizer. Msg time: "
             << timestamp << ", last msg time: " << *last_vio_msg_time_
             << ", max gap: " << params_.max_vio_measurement_gap);
    ResetLocalizer();
    return;
  }

  // Otherwise add directly to graph localizer.
  const auto& latest_combined_nav_state_msg = graph_vio_state_msg.combined_nav_states.combined_nav_states.back();
  const auto latest_combined_nav_state = lc::CombinedNavStateFromMsg(latest_combined_nav_state_msg);
  const auto latest_covariances = lc::CombinedNavStateCovariancesFromMsg(latest_combined_nav_state_msg);
  const lm::TimestampedPoseWithCovariance pose_measurement(
    lc::PoseWithCovariance(lc::EigenPose(latest_combined_nav_state.pose()), latest_covariances.pose_covariance()),
    latest_combined_nav_state.timestamp());
  graph_localizer_->AddPoseMeasurement(pose_measurement);
  last_vio_msg_time_ = timestamp;
}

void RosGraphLocalizerWrapper::Update() {
  if (Initialized()) graph_localizer_->Update();
}

bool RosGraphLocalizerWrapper::Initialized() const { return graph_localizer_ != nullptr; }

void RosGraphLocalizerWrapper::ResetLocalizer() {
  LogInfo("ResetLocalizer: Resetting vio.");
  if (!Initialized()) {
    LogError("ResetLocalizer: Localizer not initialized, nothing to do.");
    return;
  }
  graph_localizer_.reset();
  vio_measurement_buffer_.Clear();
  last_vio_msg_time_ = boost::none;
}

boost::optional<ff_msgs::GraphLocState> RosGraphLocalizerWrapper::GraphLocStateMsg() {
  const auto latest_timestamp = *(graph_localizer_->pose_nodes().LatestTimestamp());
  // Avoid sending repeat msgs.
  if (latest_msg_time_ && *latest_msg_time_ == latest_timestamp) return boost::none;
  latest_msg_time_ = latest_timestamp;
  ff_msgs::GraphLocState msg;
  const auto latest_pose = *(graph_localizer_->pose_nodes().LatestNode());
  const auto latest_keys = graph_localizer_->pose_nodes().Keys(latest_timestamp);
  const auto latest_pose_covariance = *(graph_localizer_->Covariance(latest_keys[0]));
  lc::PoseToMsg(latest_pose, msg.pose.pose);
  mc::EigenCovarianceToMsg(latest_pose_covariance, msg.pose.covariance);
  lc::TimeToHeader(latest_timestamp, msg.header);
  msg.child_frame_id = "world";
  // TODO(rsoussan): set other graph info!
  return msg;
}
}  // namespace ros_graph_localizer
