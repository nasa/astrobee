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

#include <ff_msgs/GraphState.h>
#include <ff_msgs/VIOGraph.h>
#include <ff_util/ff_names.h>
#include <graph_vio/graph_vio_nodelet.h>
#include <graph_vio/parameter_reader.h>
#include <graph_vio/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>

#include <std_msgs/Empty.h>

namespace graph_vio {
namespace lc = localization_common;
namespace mc = msg_conversions;

GraphVIONodelet::GraphVIONodelet() : ff_util::FreeFlyerNodelet(NODE_GRAPH_VIO, true) {
  private_nh_.setCallbackQueue(&private_queue_);
  heartbeat_.node = GetName();
  heartbeat_.nodelet_manager = ros::this_node::getName();

  config_reader::ConfigReader config;
  lc::LoadGraphVIOConfig(config);
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  LoadGraphVIONodeletParams(config, params_);
  last_heartbeat_time_ = ros::Time::now();
}

void GraphVIONodelet::Initialize(ros::NodeHandle* nh) {
  // Setup the platform name
  platform_name_ = GetPlatform();
  platform_name_ = (platform_name_.empty() ? "" : platform_name_ + "/");

  ff_common::InitFreeFlyerApplication(getMyArgv());
  SubscribeAndAdvertise(nh);
  Run();
}

void GraphVIONodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  state_pub_ = nh->advertise<ff_msgs::VIOGraphState>(TOPIC_VIO_GRAPH_STATE, 10);
  vio_pose_pub_ = nh->advertise<geometry_msgs::PoseStamped>(TOPIC_VIO_POSE, 10);
  graph_pub_ = nh->advertise<ff_msgs::VIOGraph>(TOPIC_VIO_GRAPH, 10);
  reset_pub_ = nh->advertise<std_msgs::Empty>(TOPIC_GNC_EKF_RESET, 10);
  heartbeat_pub_ = nh->advertise<ff_msgs::Heartbeat>(TOPIC_HEARTBEAT, 5, true);

  imu_sub_ = private_nh_.subscribe(TOPIC_HARDWARE_IMU, params_.max_imu_buffer_size, &GraphVIONodelet::ImuCallback,
                                   this, ros::TransportHints().tcpNoDelay());
  of_sub_ =
    private_nh_.subscribe(TOPIC_LOCALIZATION_OF_FEATURES, params_.max_optical_flow_buffer_size,
                          &GraphVIONodelet::OpticalFlowCallback, this, ros::TransportHints().tcpNoDelay());
  flight_mode_sub_ =
    private_nh_.subscribe(TOPIC_MOBILITY_FLIGHT_MODE, 10, &GraphVIONodelet::FlightModeCallback, this);
  bias_srv_ =
    private_nh_.advertiseService(SERVICE_GNC_EKF_INIT_BIAS, &GraphVIONodelet::ResetBiasesAndVIO, this);
  bias_from_file_srv_ = private_nh_.advertiseService(
    SERVICE_GNC_EKF_INIT_BIAS_FROM_FILE, &GraphVIONodelet::ResetBiasesFromFileAndResetVIO, this);
  reset_map_srv_ = private_nh_.advertiseService(SERVICE_LOCALIZATION_RESET_MAP, &GraphVIONodelet::ResetMap, this);
  reset_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_RESET, &GraphVIONodelet::ResetVIO, this);
  input_mode_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_SET_INPUT, &GraphVIONodelet::SetMode, this);
}

bool GraphVIONodelet::SetMode(ff_msgs::SetEkfInput::Request& req, ff_msgs::SetEkfInput::Response& res) {
  const auto input_mode = req.mode;
  if (input_mode == ff_msgs::SetEkfInputRequest::MODE_NONE) {
    LogInfo("Received Mode None request, turning off VIO.");
    DisableVIO();
  } else if (last_mode_ == ff_msgs::SetEkfInputRequest::MODE_NONE) {
    LogInfo(
      "Received Mode request that is not None and current mode is "
      "None, resetting VIO.");
    ResetAndEnableVIO();
  }

  last_mode_ = input_mode;
  return true;
}

void GraphVIONodelet::DisableVIO() { vio_enabled_ = false; }

void GraphVIONodelet::EnableVIO() { vio_enabled_ = true; }

bool GraphVIONodelet::vio_enabled() const { return vio_enabled_; }

bool GraphVIONodelet::ResetBiasesAndVIO(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  DisableVIO();
  graph_vio_wrapper_.ResetBiasesAndVIO();
  PublishReset();
  EnableVIO();
  return true;
}

bool GraphVIONodelet::ResetBiasesFromFileAndResetVIO(std_srvs::Empty::Request& req,
                                                                 std_srvs::Empty::Response& res) {
  return ResetBiasesFromFileAndResetVIO();
}

bool GraphVIONodelet::ResetBiasesFromFileAndResetVIO() {
  DisableVIO();
  graph_vio_wrapper_.ResetBiasesFromFileAndResetVIO();
  PublishReset();
  EnableVIO();
  return true;
}

bool GraphVIONodelet::ResetVIO(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  ResetAndEnableVIO();
  return true;
}

void GraphVIONodelet::ResetAndEnableVIO() {
  DisableVIO();
  graph_vio_wrapper_.ResetVIO();
  PublishReset();
  EnableVIO();
}

void GraphVIONodelet::OpticalFlowCallback(const ff_msgs::Feature2dArray::ConstPtr& feature_array_msg) {
  of_timer_.HeaderDiff(feature_array_msg->header);
  of_timer_.VlogEveryN(100, 2);

  if (!vio_enabled()) return;
  graph_vio_wrapper_.OpticalFlowCallback(*feature_array_msg);
}

void GraphVIONodelet::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  imu_timer_.HeaderDiff(imu_msg->header);
  imu_timer_.VlogEveryN(100, 2);

  if (!vio_enabled()) return;
  graph_vio_wrapper_.ImuCallback(*imu_msg);
}

void GraphVIONodelet::FlightModeCallback(ff_msgs::FlightMode::ConstPtr const& mode) {
  graph_vio_wrapper_.FlightModeCallback(*mode);
}

void GraphVIONodelet::PublishVIOState() {
  auto latest_vio_state_msg = graph_vio_wrapper_.LatestVIOStateMsg();
  if (!latest_vio_state_msg) {
    LogDebugEveryN(100, "PublishVIOState: Failed to get latest vio state msg.");
    return;
  }
  latest_vio_state_msg->callbacks_time = callbacks_timer_.last_value();
  latest_vio_state_msg->nodelet_runtime = nodelet_runtime_timer_.last_value();
  state_pub_.publish(*latest_vio_state_msg);
}

void GraphVIONodelet::PublishVIOGraph() {
  const auto latest_vio_graph_msg = graph_vio_wrapper_.LatestVIOGraphMsg();
  if (!latest_vio_graph_msg) {
    LogDebugEveryN(100, "PublishVIOGraph: Failed to get latest vio graph msg.");
    return;
  }
  graph_pub_.publish(*latest_vio_graph_msg);
}

void GraphVIONodelet::PublishReset() const {
  std_msgs::Empty msg;
  reset_pub_.publish(msg);
}

void GraphVIONodelet::PublishHeartbeat() {
  heartbeat_.header.stamp = ros::Time::now();
  if ((heartbeat_.header.stamp - last_heartbeat_time_).toSec() < 1.0) return;
  heartbeat_pub_.publish(heartbeat_);
  last_heartbeat_time_ = heartbeat_.header.stamp;
}

void GraphVIONodelet::PublishGraphMessages() {
  if (!vio_enabled()) return;

  // Publish loc information here since graph updates occur on optical flow updates
  PublishVIOState();
  if (graph_vio_wrapper_.publish_graph()) PublishGraph();
  if (graph_vio_wrapper_.save_graph_dot_file())
    graph_vio_wrapper_.SaveGraphDotFile();
}

void GraphVIONodelet::Run() {
  ros::Rate rate(100);
  // Load Biases from file by default
  // Biases reestimated if a intialize bias service call is received
  ResetBiasesFromFileAndResetVIO();
  while (ros::ok()) {
    nodelet_runtime_timer_.Start();
    callbacks_timer_.Start();
    private_queue_.callAvailable();
    callbacks_timer_.Stop();
    graph_vio_wrapper_.Update();
    nodelet_runtime_timer_.Stop();
    PublishGraphMessages();
    PublishHeartbeat();
    rate.sleep();
  }
}
}  // namespace graph_vio

PLUGINLIB_EXPORT_CLASS(graph_vio::GraphVIONodelet, nodelet::Nodelet);
