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
// #include <graph_vio/parameter_reader.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>
#include <ros_graph_vio/ros_graph_vio_nodelet.h>

#include <std_msgs/Empty.h>

namespace ros_graph_vio {
namespace lc = localization_common;
namespace mc = msg_conversions;

RosGraphVIONodelet::RosGraphVIONodelet() : ff_util::FreeFlyerNodelet(NODE_GRAPH_VIO, true) {
  private_nh_.setCallbackQueue(&private_queue_);
  heartbeat_.node = GetName();
  heartbeat_.nodelet_manager = ros::this_node::getName();

  config_reader::ConfigReader config;
  lc::LoadGraphVIOConfig(config);
  if (!config.ReadFiles()) {
    LogFatal("Failed to read config files.");
  }
  // TODO(rsoussan): put this back!!
  // LoadGraphVIONodeletParams(config, params_);
  last_heartbeat_time_ = ros::Time::now();
}

void RosGraphVIONodelet::Initialize(ros::NodeHandle* nh) {
  // Setup the platform name
  platform_name_ = GetPlatform();
  platform_name_ = (platform_name_.empty() ? "" : platform_name_ + "/");

  ff_common::InitFreeFlyerApplication(getMyArgv());
  SubscribeAndAdvertise(nh);
  Run();
}

void RosGraphVIONodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  graph_vio_state_pub_ = nh->advertise<ff_msgs::GraphVIOState>(TOPIC_GRAPH_VIO_STATE, 10);
  // graph_pub_ = nh->advertise<ff_msgs::SerializedGraph>(TOPIC_GRAPH_VIO, 10);
  reset_pub_ = nh->advertise<std_msgs::Empty>(TOPIC_GNC_EKF_RESET, 10);
  heartbeat_pub_ = nh->advertise<ff_msgs::Heartbeat>(TOPIC_HEARTBEAT, 5, true);

  imu_sub_ = private_nh_.subscribe(TOPIC_HARDWARE_IMU, params_.max_imu_buffer_size, &RosGraphVIONodelet::ImuCallback,
                                   this, ros::TransportHints().tcpNoDelay());
  fp_sub_ = private_nh_.subscribe(TOPIC_LOCALIZATION_OF_FEATURES, params_.max_feature_point_buffer_size,
                                  &RosGraphVIONodelet::FeaturePointsCallback, this, ros::TransportHints().tcpNoDelay());
  flight_mode_sub_ =
    private_nh_.subscribe(TOPIC_MOBILITY_FLIGHT_MODE, 10, &RosGraphVIONodelet::FlightModeCallback, this);
  bias_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_INIT_BIAS, &RosGraphVIONodelet::ResetBiasesAndVIO, this);
  bias_from_file_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_INIT_BIAS_FROM_FILE,
                                                     &RosGraphVIONodelet::ResetBiasesFromFileAndResetVIO, this);
  reset_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_RESET, &RosGraphVIONodelet::ResetVIO, this);
  input_mode_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_SET_INPUT, &RosGraphVIONodelet::SetMode, this);
}

bool RosGraphVIONodelet::SetMode(ff_msgs::SetEkfInput::Request& req, ff_msgs::SetEkfInput::Response& res) {
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

void RosGraphVIONodelet::DisableVIO() { vio_enabled_ = false; }

void RosGraphVIONodelet::EnableVIO() { vio_enabled_ = true; }

bool RosGraphVIONodelet::vio_enabled() const { return vio_enabled_; }

bool RosGraphVIONodelet::ResetBiasesAndVIO(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  DisableVIO();
  ros_graph_vio_wrapper_.ResetBiasesAndVIO();
  PublishReset();
  EnableVIO();
  return true;
}

bool RosGraphVIONodelet::ResetBiasesFromFileAndResetVIO(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  return ResetBiasesFromFileAndResetVIO();
}

bool RosGraphVIONodelet::ResetBiasesFromFileAndResetVIO() {
  DisableVIO();
  ros_graph_vio_wrapper_.ResetBiasesFromFileAndResetVIO();
  PublishReset();
  EnableVIO();
  return true;
}

bool RosGraphVIONodelet::ResetVIO(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  ResetAndEnableVIO();
  return true;
}

void RosGraphVIONodelet::ResetAndEnableVIO() {
  DisableVIO();
  ros_graph_vio_wrapper_.ResetVIO();
  PublishReset();
  EnableVIO();
}

void RosGraphVIONodelet::PublishGraphVIOState() {
  auto msg = ros_graph_vio_wrapper_.GraphVIOStateMsg();
  if (msg.combined_nav_states.combined_nav_states.empty()) {
    LogDebugEveryN(100, "PublishVIOState: Failed to get vio states msg.");
    return;
  }
  graph_vio_state_pub_.publish(msg);
}

void RosGraphVIONodelet::PublishReset() const {
  std_msgs::Empty msg;
  reset_pub_.publish(msg);
}

void RosGraphVIONodelet::PublishHeartbeat() {
  heartbeat_.header.stamp = ros::Time::now();
  if ((heartbeat_.header.stamp - last_heartbeat_time_).toSec() < 1.0) return;
  heartbeat_pub_.publish(heartbeat_);
  last_heartbeat_time_ = heartbeat_.header.stamp;
}

void RosGraphVIONodelet::PublishGraphMessages() {
  if (!vio_enabled()) return;

  // TODO(rsoussan): Only publish if things have changed?
  PublishGraphVIOState();
  // if (ros_graph_vio_wrapper_.publish_graph()) PublishVIOGraph();
  // if (ros_graph_vio_wrapper_.save_graph_dot_file()) ros_graph_vio_wrapper_.SaveGraphDotFile();
}

void RosGraphVIONodelet::FeaturePointsCallback(const ff_msgs::Feature2dArray::ConstPtr& feature_array_msg) {
  if (!vio_enabled()) return;
  ros_graph_vio_wrapper_.FeaturePointsCallback(*feature_array_msg);
}

void RosGraphVIONodelet::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  if (!vio_enabled()) return;
  ros_graph_vio_wrapper_.ImuCallback(*imu_msg);
}

void RosGraphVIONodelet::FlightModeCallback(ff_msgs::FlightMode::ConstPtr const& mode) {
  ros_graph_vio_wrapper_.FlightModeCallback(*mode);
}

void RosGraphVIONodelet::Run() {
  ros::Rate rate(100);
  // Load Biases from file by default
  // Biases reestimated if a intialize bias service call is received
  ResetBiasesFromFileAndResetVIO();
  while (ros::ok()) {
    private_queue_.callAvailable();
    if (vio_enabled()) ros_graph_vio_wrapper_.Update();
    PublishGraphMessages();
    PublishHeartbeat();
    rate.sleep();
  }
}
}  // namespace ros_graph_vio

PLUGINLIB_EXPORT_CLASS(ros_graph_vio::RosGraphVIONodelet, nodelet::Nodelet);
