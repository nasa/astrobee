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

#include <ff_util/ff_names.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>
#include <ros_graph_localizer/parameter_reader.h>
#include <ros_graph_localizer/ros_graph_localizer_nodelet.h>

#include <std_msgs/Empty.h>

namespace ros_graph_localizer {
namespace lc = localization_common;
namespace mc = msg_conversions;

RosGraphLocalizerNodelet::RosGraphLocalizerNodelet() : ff_util::FreeFlyerNodelet(NODE_GRAPH_LOC, true) {
  private_nh_.setCallbackQueue(&private_queue_);
  heartbeat_.node = GetName();
  heartbeat_.nodelet_manager = ros::this_node::getName();

  config_reader::ConfigReader config;
  lc::LoadGraphLocalizerConfig(config);
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  LoadRosGraphLocalizerNodeletParams(config, params_);
  last_heartbeat_time_ = ros::Time::now();
}

void RosGraphLocalizerNodelet::Initialize(ros::NodeHandle* nh) {
  // Setup the platform name
  platform_name_ = GetPlatform();
  platform_name_ = (platform_name_.empty() ? "" : platform_name_ + "/");

  ff_common::InitFreeFlyerApplication(getMyArgv());
  SubscribeAndAdvertise(nh);
  Run();
}

void RosGraphLocalizerNodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  graph_loc_pub_ = nh->advertise<std_msgs::Empty>(TOPIC_GRAPH_LOC_STATE, 10);
  reset_pub_ = nh->advertise<std_msgs::Empty>(TOPIC_GNC_EKF_RESET, 10);
  heartbeat_pub_ = nh->advertise<ff_msgs::Heartbeat>(TOPIC_HEARTBEAT, 5, true);
  graph_vio_sub_ =
    private_nh_.subscribe(TOPIC_GRAPH_VIO_STATE, params_.max_graph_vio_state_buffer_size,
                          &RosGraphLocalizerNodelet::GraphVIOStateCallback, this, ros::TransportHints().tcpNoDelay());
  sparse_map_vl_sub_ = private_nh_.subscribe(
    TOPIC_LOCALIZATION_ML_FEATURES, params_.max_vl_matched_projections_buffer_size,
    &RosGraphLocalizerNodelet::SparseMapVisualLandmarksCallback, this, ros::TransportHints().tcpNoDelay());
  // TODO(rsoussan): Should localizer reset on bias reset calls?
  // TODO(rsoussan): Add separate localizer reset calls?
  bias_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_INIT_BIAS, &RosGraphLocalizerNodelet::ResetLocalizer, this);
  bias_from_file_srv_ =
    private_nh_.advertiseService(SERVICE_GNC_EKF_INIT_BIAS_FROM_FILE, &RosGraphLocalizerNodelet::ResetLocalizer, this);
  reset_map_srv_ =
    private_nh_.advertiseService(SERVICE_LOCALIZATION_RESET_MAP, &RosGraphLocalizerNodelet::ResetMap, this);
  reset_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_RESET, &RosGraphLocalizerNodelet::ResetLocalizer, this);
  input_mode_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_SET_INPUT, &RosGraphLocalizerNodelet::SetMode, this);
}

bool RosGraphLocalizerNodelet::SetMode(ff_msgs::SetEkfInput::Request& req, ff_msgs::SetEkfInput::Response& res) {
  const auto input_mode = req.mode;
  if (input_mode == ff_msgs::SetEkfInputRequest::MODE_NONE) {
    LogInfo("Received Mode None request, turning off Localizer.");
    DisableLocalizer();
  } else if (last_mode_ == ff_msgs::SetEkfInputRequest::MODE_NONE) {
    LogInfo(
      "Received Mode request that is not None and current mode is "
      "None, resetting Localizer.");
    ResetAndEnableLocalizer();
  }

  last_mode_ = input_mode;
  return true;
}

void RosGraphLocalizerNodelet::DisableLocalizer() { localizer_enabled_ = false; }

void RosGraphLocalizerNodelet::EnableLocalizer() { localizer_enabled_ = true; }

bool RosGraphLocalizerNodelet::localizer_enabled() const { return localizer_enabled_; }

bool RosGraphLocalizerNodelet::ResetLocalizer(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  ResetAndEnableLocalizer();
  return true;
}

void RosGraphLocalizerNodelet::ResetAndEnableLocalizer() {
  DisableLocalizer();
  ros_graph_localizer_wrapper_.ResetLocalizer();
  PublishReset();
  EnableLocalizer();
}

bool RosGraphLocalizerNodelet::ResetMap(ff_msgs::ResetMap::Request& req, ff_msgs::ResetMap::Response& res) {
  // TODO(rsoussan): Better way to clear buffer?
  sparse_map_vl_sub_ = private_nh_.subscribe(
    TOPIC_LOCALIZATION_ML_FEATURES, params_.max_vl_matched_projections_buffer_size,
    &RosGraphLocalizerNodelet::SparseMapVisualLandmarksCallback, this, ros::TransportHints().tcpNoDelay());
  ResetAndEnableLocalizer();
  return true;
}

void RosGraphLocalizerNodelet::PublishGraphLocalizerState() {
  const auto msg = ros_graph_localizer_wrapper_.GraphLocStateMsg();
  if (msg)  graph_loc_pub_.publish(*msg);
}

/*void RosGraphLocalizerNodelet::PublishLocalizerGraph() {
  const auto latest_localizer_graph_msg = ros_graph_localizer_wrapper_.LatestGraphMsg();
  if (!latest_localizer_graph_msg) {
    LogDebugEveryN(100, "PublishLocalizerGraph: Failed to get latest localizer graph msg.");
    return;
  }
  graph_pub_.publish(*latest_localizer_graph_msg);
}*/

void RosGraphLocalizerNodelet::PublishReset() const {
  std_msgs::Empty msg;
  reset_pub_.publish(msg);
}

void RosGraphLocalizerNodelet::PublishHeartbeat() {
  heartbeat_.header.stamp = ros::Time::now();
  if ((heartbeat_.header.stamp - last_heartbeat_time_).toSec() < 1.0) return;
  heartbeat_pub_.publish(heartbeat_);
  last_heartbeat_time_ = heartbeat_.header.stamp;
}

void RosGraphLocalizerNodelet::PublishGraphMessages() {
  if (!localizer_enabled()) return;

  // TODO(rsoussan): Only publish if things have changed?
  PublishGraphLocalizerState();
  // if (ros_graph_localizer_wrapper_.publish_graph()) PublishLocalizerGraph();
  // if (ros_graph_localizer_wrapper_.save_graph_dot_file()) ros_graph_localizer_wrapper_.SaveGraphDotFile();
}

void RosGraphLocalizerNodelet::SparseMapVisualLandmarksCallback(
  const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg) {
  if (!localizer_enabled()) return;
  ros_graph_localizer_wrapper_.SparseMapVisualLandmarksCallback(*visual_landmarks_msg);
}

void RosGraphLocalizerNodelet::GraphVIOStateCallback(const ff_msgs::GraphVIOState::ConstPtr& graph_vio_state_msg) {
  if (!localizer_enabled()) return;
  ros_graph_localizer_wrapper_.GraphVIOStateCallback(*graph_vio_state_msg);
}

void RosGraphLocalizerNodelet::Run() {
  ros::Rate rate(100);
  ResetAndEnableLocalizer();
  while (ros::ok()) {
    private_queue_.callAvailable();
    if (localizer_enabled()) ros_graph_localizer_wrapper_.Update();
    PublishGraphMessages();
    PublishHeartbeat();
    rate.sleep();
  }
}
}  // namespace ros_graph_localizer

PLUGINLIB_EXPORT_CLASS(ros_graph_localizer::RosGraphLocalizerNodelet, nodelet::Nodelet);
