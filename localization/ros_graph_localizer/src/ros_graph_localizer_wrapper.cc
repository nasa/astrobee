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
#include <graph_factors/loc_pose_factor.h>
#include <graph_factors/loc_projection_factor.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <localization_measurements/measurement_conversions.h>
#include <msg_conversions/msg_conversions.h>
#include <parameter_reader/graph_localizer.h>
#include <ros_graph_localizer/ros_graph_localizer_wrapper.h>

#include <Eigen/Core>

namespace ros_graph_localizer {
namespace gl = graph_localizer;
namespace lc = localization_common;
namespace lm = localization_measurements;
namespace mc = msg_conversions;
namespace pr = parameter_reader;

RosGraphLocalizerWrapper::RosGraphLocalizerWrapper(const std::string& graph_config_path_prefix) {
  LoadConfigs(graph_config_path_prefix);
}

void RosGraphLocalizerWrapper::LoadConfigs(const std::string& graph_config_path_prefix) {
  config_reader::ConfigReader config;
  lc::LoadGraphLocalizerConfig(config, graph_config_path_prefix);
  pr::LoadGraphLocalizerParams(config, params_);
}

void RosGraphLocalizerWrapper::SparseMapVisualLandmarksCallback(const ff_msgs::VisualLandmarks& visual_landmarks_msg) {
  const auto msg_time = lc::TimeFromHeader(visual_landmarks_msg.header);

  // Initialize with pose estimate if not initialized yet.
  // Ensure vio data exists before msg time so no gaps occur between first
  // sparse map measurement and future interpolated vio measurements.
  if (!Initialized() && !vio_measurement_buffer_.empty()) {
    const auto oldest_vio_measurement_time = vio_measurement_buffer_.Oldest()->timestamp;
    if (msg_time < oldest_vio_measurement_time) {
      LogError(
        "SparseMapVisualLandmarksCallback: Initial vl msg time older than oldest buffered vio time, failed to "
        "initialize graph localizer.");
      return;
    }
    const auto world_T_body = lc::PoseFromMsgWithExtrinsics(visual_landmarks_msg.pose,
                                                            params_.sparse_map_loc_factor_adder.body_T_cam.inverse());
    params_.pose_node_adder.start_node = world_T_body;
    params_.pose_node_adder.starting_time = msg_time;
    LogInfo("SparseMapVisualLandmarksCallback: Initializing localizer with vl msg.");
    graph_localizer_.reset(new gl::GraphLocalizer(params_));
    // Only need the first vio measurement before the initial vl msg time
    // to ensure valid pose interpolation.
    vio_measurement_buffer_.RemoveBelowLowerBoundValues(msg_time);
    // Add all subsequent measurements, remove from buffer once added.
    while (!vio_measurement_buffer_.empty()) {
      const auto graph_vio_msg = vio_measurement_buffer_.RemoveOldest();
      if (!GraphVIOStateCallback(graph_vio_msg->value)) return;
    }
  } else if (Initialized()) {  // Otherwise add measurement to graph
    graph_localizer_->AddSparseMapMatchedProjectionsMeasurement(
      lm::MakeMatchedProjectionsMeasurement(visual_landmarks_msg));
  }
}

void RosGraphLocalizerWrapper::ARVisualLandmarksCallback(const ff_msgs::VisualLandmarks& visual_landmarks_msg) {
  // Set world_T_dock using the pose estimate from provided msg and the latest VIO extrapolated pose estimate
  // since the dock pose in the message is relative to the dock frame
  // and not the global frame
  if (!world_T_dock_) {
    const auto world_T_latest_graph_body = LatestPose();
    const auto latest_graph_timestamp = LatestTimestamp();
    if (!world_T_latest_graph_body || !latest_graph_timestamp) {
      LogError("ARVisualLandmarksCallback: Failed to get latest pose and timestamp.");
      return;
    }

    const auto dock_time = lc::TimeFromHeader(visual_landmarks_msg.header);
    const auto latest_graph_body_T_body = odom_interpolator_.Relative(*latest_graph_timestamp, dock_time);
    if (!latest_graph_body_T_body) {
      LogError("ARVisualLandmarksCallback: Failed to get latest_graph_body_T_body for provided times.");
      return;
    }
    const auto dock_T_body = lc::PoseFromMsgWithExtrinsics(
      visual_landmarks_msg.pose, params_.ar_tag_loc_factor_adder.body_T_cam.inverse());
    world_T_dock_ = *world_T_latest_graph_body * lc::GtPose(*latest_graph_body_T_body) * dock_T_body.inverse();
  }
  if (Initialized()) {
    // Frame change the ar tag measurement from the dock to world frame before
    // passing to the localizer.
    const auto frame_changed_ar_measurements = lm::FrameChangeMatchedProjectionsMeasurement(
      lm::MakeMatchedProjectionsMeasurement(visual_landmarks_msg), *world_T_dock_);
    graph_localizer_->AddArTagMatchedProjectionsMeasurement(frame_changed_ar_measurements);
  }
}

bool RosGraphLocalizerWrapper::GraphVIOStateCallback(const ff_msgs::GraphVIOState& graph_vio_state_msg) {
  const auto timestamp = lc::TimeFromHeader(graph_vio_state_msg.header);
  // Buffer measurements before initialization so they can be added once initialized.
  if (!Initialized()) {
    vio_measurement_buffer_.Add(timestamp, graph_vio_state_msg);
    return true;
  }

  // Check if gap in vio msgs is too large, reset localizer if so.
  if (last_vio_msg_time_ && (timestamp - *last_vio_msg_time_) > params_.max_vio_measurement_gap) {
    LogError("GraphVIOStateCallback: VIO msg gap exceeded, resetting localizer. Msg time: "
             << std::setprecision(15) << timestamp << ", last msg time: " << *last_vio_msg_time_
             << ", max gap: " << params_.max_vio_measurement_gap);
    ResetLocalizer();
    return false;
  }

  // Otherwise add directly to graph localizer.
  const auto& latest_combined_nav_state_msg = graph_vio_state_msg.combined_nav_states.combined_nav_states.back();
  const auto latest_combined_nav_state = lc::CombinedNavStateFromMsg(latest_combined_nav_state_msg);
  const auto latest_covariances = lc::CombinedNavStateCovariancesFromMsg(latest_combined_nav_state_msg);
  const lm::PoseWithCovarianceMeasurement pose_measurement(
    latest_combined_nav_state.pose(), latest_covariances.pose_covariance(), latest_combined_nav_state.timestamp());
  graph_localizer_->AddPoseMeasurement(pose_measurement);
  last_vio_msg_time_ = timestamp;
  odom_interpolator_.Add(latest_combined_nav_state.timestamp(), lc::EigenPose(latest_combined_nav_state.pose()));
  return true;
}

void RosGraphLocalizerWrapper::Update() {
  if (Initialized()) graph_localizer_->Update();
}

bool RosGraphLocalizerWrapper::Initialized() const { return graph_localizer_ != nullptr; }

void RosGraphLocalizerWrapper::ResetLocalizer() {
  LogInfo("ResetLocalizer: Resetting localizer.");
  if (!Initialized()) {
    LogError("ResetLocalizer: Localizer not initialized, nothing to do.");
    return;
  }
  graph_localizer_.reset();
  vio_measurement_buffer_.Clear();
  last_vio_msg_time_ = boost::none;
}

void RosGraphLocalizerWrapper::ResetWorldTDock() { world_T_dock_ = boost::none; }

boost::optional<gtsam::Pose3> RosGraphLocalizerWrapper::WorldTDock() const { return world_T_dock_; }

boost::optional<lc::Time> RosGraphLocalizerWrapper::LatestTimestamp() const {
  if (!Initialized()) {
    LogWarningEveryN(200, "LatestTimestamp: Localizer not yet initialized");
    return boost::none;
  }
  return graph_localizer_->pose_nodes().LatestTimestamp();
}

boost::optional<gtsam::Pose3> RosGraphLocalizerWrapper::LatestPose() const {
  if (!Initialized()) {
    LogWarningEveryN(200, "LatestPose: Localizer not yet initialized");
    return boost::none;
  }
  return graph_localizer_->pose_nodes().LatestNode();
}

boost::optional<ff_msgs::GraphLocState> RosGraphLocalizerWrapper::GraphLocStateMsg() {
  if (!Initialized()) {
    LogWarningEveryN(200, "GraphLocStateMsg: Localizer not yet initialized");
    return boost::none;
  }
  const auto latest_timestamp = *LatestTimestamp();
  // Avoid sending repeat msgs.
  if (latest_msg_time_ && *latest_msg_time_ == latest_timestamp) {
    LogWarningEveryN(200, "GraphLocStateMsg: No new states added.");
    return boost::none;
  }
  latest_msg_time_ = latest_timestamp;
  odom_interpolator_.RemoveBelowLowerBoundValues(latest_timestamp);
  ff_msgs::GraphLocState msg;
  const auto latest_pose = *LatestPose();
  const auto latest_keys = graph_localizer_->pose_nodes().Keys(latest_timestamp);
  const auto latest_pose_covariance = *(graph_localizer_->Covariance(latest_keys[0]));
  lc::PoseToMsg(latest_pose, msg.pose.pose);
  mc::EigenCovarianceToMsg(latest_pose_covariance, msg.pose.covariance);
  lc::TimeToHeader(latest_timestamp, msg.header);
  msg.child_frame_id = "world";
  msg.num_ml_projection_factors = graph_localizer_->NumFactors<gtsam::LocProjectionFactor<>>();
  msg.num_ml_pose_factors = graph_localizer_->NumFactors<gtsam::LocPoseFactor>();
  msg.optimization_time = graph_localizer_->optimization_timer().last_value();
  msg.update_time = graph_localizer_->update_timer().last_value();
  // TODO(rsoussan): set other graph info!
  return msg;
}
}  // namespace ros_graph_localizer
