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

#include <ff_common/ff_names.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>
#include <msg_conversions/msg_conversions.h>
#include <ros_graph_localizer/parameter_reader.h>
#include <ros_graph_localizer/ros_graph_localizer_nodelet.h>

#include <std_msgs/Empty.h>

namespace ros_graph_localizer {
namespace lc = localization_common;
namespace mc = msg_conversions;
namespace rv = ros_graph_vio;

RosGraphLocalizerNodelet::RosGraphLocalizerNodelet() : ff_util::FreeFlyerNodelet(NODE_GRAPH_LOC, true) {
  private_nh_.setCallbackQueue(&private_queue_);
  heartbeat_.node = GetName();
  heartbeat_.nodelet_manager = ros::this_node::getName();

  config_reader::ConfigReader config;
  lc::LoadGraphLocalizerConfig(config);
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
  graph_vio_state_pub_ = nh->advertise<ff_msgs::GraphVIOState>(TOPIC_GRAPH_VIO_STATE, 10);
  graph_loc_pub_ = nh->advertise<ff_msgs::GraphLocState>(TOPIC_GRAPH_LOC_STATE, 10);
  reset_pub_ = nh->advertise<std_msgs::Empty>(TOPIC_GNC_EKF_RESET, 10);
  heartbeat_pub_ = nh->advertise<ff_msgs::Heartbeat>(TOPIC_HEARTBEAT, 5, true);
  imu_sub_ = private_nh_.subscribe(TOPIC_HARDWARE_IMU, params_.max_imu_buffer_size,
                                   &RosGraphLocalizerNodelet::ImuCallback, this, ros::TransportHints().tcpNoDelay());
  fp_sub_ =
    private_nh_.subscribe(TOPIC_LOCALIZATION_OF_FEATURES, params_.max_feature_point_buffer_size,
                          &RosGraphLocalizerNodelet::FeaturePointsCallback, this, ros::TransportHints().tcpNoDelay());
  flight_mode_sub_ =
    private_nh_.subscribe(TOPIC_MOBILITY_FLIGHT_MODE, 10, &RosGraphLocalizerNodelet::FlightModeCallback, this);

  sparse_map_vl_sub_ = private_nh_.subscribe(
    TOPIC_LOCALIZATION_ML_FEATURES, params_.max_vl_matched_projections_buffer_size,
    &RosGraphLocalizerNodelet::SparseMapVisualLandmarksCallback, this, ros::TransportHints().tcpNoDelay());
  bias_srv_ =
    private_nh_.advertiseService(SERVICE_GNC_EKF_INIT_BIAS, &RosGraphLocalizerNodelet::ResetBiasesAndLocalizer, this);
  bias_from_file_srv_ = private_nh_.advertiseService(
    SERVICE_GNC_EKF_INIT_BIAS_FROM_FILE, &RosGraphLocalizerNodelet::ResetBiasesFromFileAndResetLocalizer, this);
  reset_map_srv_ =
    private_nh_.advertiseService(SERVICE_LOCALIZATION_RESET_MAP_LOC, &RosGraphLocalizerNodelet::ResetMap, this);
  // TODO(rsoussan): Reset biases from file here?
  reset_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_RESET,
                                            &RosGraphLocalizerNodelet::ResetBiasesFromFileAndResetLocalizer, this);
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

  // Reset world_T_dock when switch back to ar mode
  if (input_mode == ff_msgs::SetEkfInputRequest::MODE_AR_TAGS &&
      last_mode_ != ff_msgs::SetEkfInputRequest::MODE_AR_TAGS) {
    LogInfo("SetMode: Switching to AR_TAG mode.");
    ros_graph_localizer_wrapper_.ResetWorldTDock();
  }
  last_mode_ = input_mode;
  return true;
}

void RosGraphLocalizerNodelet::DisableLocalizer() { localizer_enabled_ = false; }

void RosGraphLocalizerNodelet::EnableLocalizer() { localizer_enabled_ = true; }

bool RosGraphLocalizerNodelet::localizer_enabled() const { return localizer_enabled_; }

bool RosGraphLocalizerNodelet::ResetBiasesAndLocalizer(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  DisableLocalizer();
  ros_graph_vio_wrapper_.ResetBiasesAndVIO();
  ros_graph_localizer_wrapper_.ResetLocalizer();
  PublishReset();
  EnableLocalizer();
  return true;
}

bool RosGraphLocalizerNodelet::ResetBiasesFromFileAndResetLocalizer(std_srvs::Empty::Request& req,
                                                                    std_srvs::Empty::Response& res) {
  return ResetBiasesFromFileAndResetLocalizer();
}

bool RosGraphLocalizerNodelet::ResetBiasesFromFileAndResetLocalizer() {
  DisableLocalizer();
  ros_graph_vio_wrapper_.ResetBiasesFromFileAndResetVIO();
  ros_graph_localizer_wrapper_.ResetLocalizer();
  PublishReset();
  EnableLocalizer();
  return true;
}

bool RosGraphLocalizerNodelet::ResetLocalizer(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  ResetAndEnableLocalizer();
  return true;
}

void RosGraphLocalizerNodelet::ResetAndEnableLocalizer() {
  // TODO(rsoussan): avoid resetting vio?
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
  if (msg) graph_loc_pub_.publish(*msg);
}

void RosGraphLocalizerNodelet::FeaturePointsCallback(const ff_msgs::Feature2dArray::ConstPtr& feature_array_msg) {
  if (!localizer_enabled()) return;
  ros_graph_vio_wrapper_.FeaturePointsCallback(*feature_array_msg);
}

void RosGraphLocalizerNodelet::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  if (!localizer_enabled()) return;
  ros_graph_vio_wrapper_.ImuCallback(*imu_msg);
  ros_graph_localizer_wrapper_.ImuCallback(*imu_msg);
}

void RosGraphLocalizerNodelet::FlightModeCallback(ff_msgs::FlightMode::ConstPtr const& mode) {
  ros_graph_vio_wrapper_.FlightModeCallback(*mode);
  ros_graph_localizer_wrapper_.FlightModeCallback(*mode);
}

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

void RosGraphLocalizerNodelet::PublishGraphLocalizerMessages() {
  if (!localizer_enabled()) return;

  // TODO(rsoussan): Only publish if things have changed?
  PublishGraphLocalizerState();
  // PublishWorldTBodyTF();
}

void RosGraphLocalizerNodelet::PublishWorldTBodyTF() {
  const auto latest_pose = ros_graph_localizer_wrapper_.LatestPose();
  const auto latest_timestamp = ros_graph_localizer_wrapper_.LatestTimestamp();
  if (!latest_pose || !latest_timestamp) {
    LogErrorEveryN(100, "PublishWorldTBodyTF: Failed to get latest pose and timestamp.");
    return;
  }

  const auto world_T_body_tf = lc::PoseToTF(*latest_pose, "world", "body", *latest_timestamp, platform_name_);
  if (world_T_body_tf.header.stamp == last_tf_body_time_) return;

  last_tf_body_time_ = world_T_body_tf.header.stamp;
  transform_pub_.sendTransform(world_T_body_tf);
}

void RosGraphLocalizerNodelet::PublishWorldTDockTF() {
  const auto world_T_dock = ros_graph_localizer_wrapper_.WorldTDock();
  if (!world_T_dock) return;
  const auto world_T_dock_tf =
    lc::PoseToTF(*world_T_dock, "world", "dock/body", lc::TimeFromRosTime(ros::Time::now()), platform_name_);
  // If the rate is higher than the sim time, prevent sending repeat tfs
  if (world_T_dock_tf.header.stamp == last_tf_dock_time_) return;
  last_tf_dock_time_ = world_T_dock_tf.header.stamp;
  transform_pub_.sendTransform(world_T_dock_tf);
}

void RosGraphLocalizerNodelet::ARVisualLandmarksCallback(
  const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg) {
  if (!localizer_enabled()) return;
  ros_graph_localizer_wrapper_.ARVisualLandmarksCallback(*visual_landmarks_msg);
  PublishWorldTDockTF();
}

void RosGraphLocalizerNodelet::SparseMapVisualLandmarksCallback(
  const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg) {
  if (!localizer_enabled()) return;
  ros_graph_vio_wrapper_.SparseMapVisualLandmarksCallback(*visual_landmarks_msg);
  ros_graph_localizer_wrapper_.SparseMapVisualLandmarksCallback(*visual_landmarks_msg);
}

void RosGraphLocalizerNodelet::Run() {
  ros::Rate rate(100);
  // ResetAndEnableLocalizer();
  // Load Biases from file by default
  // Biases reestimated if a intialize bias service call is received
  ResetBiasesFromFileAndResetLocalizer();
  while (ros::ok()) {
    private_queue_.callAvailable();
    if (localizer_enabled()) {
      ros_graph_vio_wrapper_.Update();
      // Pass data and msgs from graph vio to graph localizer
      // TODO(rsoussan): move this to a function....
      if (ros_graph_vio_wrapper_.Initialized() && ros_graph_localizer_wrapper_.Initialized()) {
        ros_graph_localizer_wrapper_.graph_localizer_->pose_node_adder_->node_adder_model_.nodes_ =
          ros_graph_vio_wrapper_.graph_vio()->combined_nav_state_node_adder_->nodes_.get();
        if (ros_graph_vio_wrapper_.graph_vio()->marginals()) {
          ros_graph_localizer_wrapper_.graph_localizer_->pose_node_adder_->node_adder_model_.marginals_ =
            *(ros_graph_vio_wrapper_.graph_vio()->marginals());
        }
      }

      const auto graph_vio_state_msg = ros_graph_vio_wrapper_.GraphVIOStateMsg();
      if (!graph_vio_state_msg) {
        LogDebugEveryN(100, "PublishVIOState: Failed to get vio states msg.");
      } else {
        graph_vio_state_pub_.publish(*graph_vio_state_msg);
        ros_graph_localizer_wrapper_.GraphVIOStateCallback(*graph_vio_state_msg);
      }
      ros_graph_localizer_wrapper_.Update();
      PublishGraphLocalizerMessages();
    }
    PublishHeartbeat();
    rate.sleep();
  }
}
}  // namespace ros_graph_localizer

PLUGINLIB_EXPORT_CLASS(ros_graph_localizer::RosGraphLocalizerNodelet, nodelet::Nodelet);
