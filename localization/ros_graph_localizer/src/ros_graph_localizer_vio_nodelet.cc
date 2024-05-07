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
#include <ros_graph_localizer/ros_graph_localizer_vio_nodelet.h>
#include <ros_graph_vio/parameter_reader.h>

#include <std_msgs/Empty.h>

namespace ros_graph_localizer {
namespace lc = localization_common;
namespace mc = msg_conversions;
namespace rv = ros_graph_vio;

RosGraphLocalizerVIONodelet::RosGraphLocalizerVIONodelet() : ff_util::FreeFlyerNodelet(NODE_GRAPH_LOC, true) {
  private_nh_.setCallbackQueue(&private_queue_);
  heartbeat_.node = GetName();
  heartbeat_.nodelet_manager = ros::this_node::getName();

  config_reader::ConfigReader config;
  config.AddFile("localization/ros_graph_vio.config");
  lc::LoadGraphLocalizerConfig(config);
  LoadRosGraphLocalizerNodeletParams(config, params_);
  rv::LoadRosGraphVIONodeletParams(config, vio_params_);
  last_heartbeat_time_ = ros::Time::now();
}

void RosGraphLocalizerVIONodelet::Initialize(ros::NodeHandle* nh) {
  // Setup the platform name
  platform_name_ = GetPlatform();
  platform_name_ = (platform_name_.empty() ? "" : platform_name_ + "/");

  ff_common::InitFreeFlyerApplication(getMyArgv());
  SubscribeAndAdvertise(nh);
  Run();
}

void RosGraphLocalizerVIONodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  graph_vio_state_pub_ = nh->advertise<ff_msgs::GraphVIOState>(TOPIC_GRAPH_VIO_STATE, 10);
  graph_loc_pub_ = nh->advertise<ff_msgs::GraphLocState>(TOPIC_GRAPH_LOC_STATE, 10);
  reset_pub_ = nh->advertise<std_msgs::Empty>(TOPIC_GNC_EKF_RESET, 10);
  heartbeat_pub_ = nh->advertise<ff_msgs::Heartbeat>(TOPIC_HEARTBEAT, 5, true);
  imu_sub_ = private_nh_.subscribe(TOPIC_HARDWARE_IMU, vio_params_.max_imu_buffer_size,
                                   &RosGraphLocalizerVIONodelet::ImuCallback, this, ros::TransportHints().tcpNoDelay());
  fp_sub_ = private_nh_.subscribe(TOPIC_LOCALIZATION_OF_FEATURES, vio_params_.max_feature_point_buffer_size,
                                  &RosGraphLocalizerVIONodelet::FeaturePointsCallback, this,
                                  ros::TransportHints().tcpNoDelay());
  flight_mode_sub_ =
    private_nh_.subscribe(TOPIC_MOBILITY_FLIGHT_MODE, 10, &RosGraphLocalizerVIONodelet::FlightModeCallback, this);

  // graph_vio_sub_ =
  //   private_nh_.subscribe(TOPIC_GRAPH_VIO_STATE, params_.max_graph_vio_state_buffer_size,
  //                        &RosGraphLocalizerVIONodelet::GraphVIOStateCallback, this,
  //                        ros::TransportHints().tcpNoDelay());
  sparse_map_vl_sub_ = private_nh_.subscribe(
    TOPIC_LOCALIZATION_ML_FEATURES, params_.max_vl_matched_projections_buffer_size,
    &RosGraphLocalizerVIONodelet::SparseMapVisualLandmarksCallback, this, ros::TransportHints().tcpNoDelay());
  bias_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_INIT_BIAS,
                                           &RosGraphLocalizerVIONodelet::ResetBiasesAndLocalizerVIO, this);
  bias_from_file_srv_ = private_nh_.advertiseService(
    SERVICE_GNC_EKF_INIT_BIAS_FROM_FILE, &RosGraphLocalizerVIONodelet::ResetBiasesFromFileAndResetLocalizerVIO, this);
  reset_map_srv_ =
    private_nh_.advertiseService(SERVICE_LOCALIZATION_RESET_MAP, &RosGraphLocalizerVIONodelet::ResetMap, this);
  // TODO(rsoussan): Reset biases from file here?
  reset_srv_ = private_nh_.advertiseService(
    SERVICE_GNC_EKF_RESET, &RosGraphLocalizerVIONodelet::ResetBiasesFromFileAndResetLocalizerVIO, this);
  input_mode_srv_ =
    private_nh_.advertiseService(SERVICE_GNC_EKF_SET_INPUT, &RosGraphLocalizerVIONodelet::SetMode, this);
}

bool RosGraphLocalizerVIONodelet::SetMode(ff_msgs::SetEkfInput::Request& req, ff_msgs::SetEkfInput::Response& res) {
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

void RosGraphLocalizerVIONodelet::DisableLocalizer() { localizer_enabled_ = false; }

void RosGraphLocalizerVIONodelet::EnableLocalizer() { localizer_enabled_ = true; }

bool RosGraphLocalizerVIONodelet::localizer_enabled() const { return localizer_enabled_; }

bool RosGraphLocalizerVIONodelet::ResetBiasesAndLocalizerVIO(std_srvs::Empty::Request& req,
                                                             std_srvs::Empty::Response& res) {
  DisableLocalizer();
  ros_graph_vio_wrapper_.ResetBiasesAndVIO();
  ros_graph_localizer_wrapper_.ResetLocalizer();
  PublishReset();
  EnableLocalizer();
  return true;
}

bool RosGraphLocalizerVIONodelet::ResetBiasesFromFileAndResetLocalizerVIO(std_srvs::Empty::Request& req,
                                                                          std_srvs::Empty::Response& res) {
  return ResetBiasesFromFileAndResetLocalizerVIO();
}

bool RosGraphLocalizerVIONodelet::ResetBiasesFromFileAndResetLocalizerVIO() {
  DisableLocalizer();
  ros_graph_vio_wrapper_.ResetBiasesFromFileAndResetVIO();
  ros_graph_localizer_wrapper_.ResetLocalizer();
  PublishReset();
  EnableLocalizer();
  return true;
}

bool RosGraphLocalizerVIONodelet::ResetLocalizer(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  ResetAndEnableLocalizer();
  return true;
}

void RosGraphLocalizerVIONodelet::ResetAndEnableLocalizer() {
  // TODO(rsoussan): avoid resetting vio?
  DisableLocalizer();
  ros_graph_localizer_wrapper_.ResetLocalizer();
  PublishReset();
  EnableLocalizer();
}

bool RosGraphLocalizerVIONodelet::ResetMap(ff_msgs::ResetMap::Request& req, ff_msgs::ResetMap::Response& res) {
  // TODO(rsoussan): Better way to clear buffer?
  sparse_map_vl_sub_ = private_nh_.subscribe(
    TOPIC_LOCALIZATION_ML_FEATURES, params_.max_vl_matched_projections_buffer_size,
    &RosGraphLocalizerVIONodelet::SparseMapVisualLandmarksCallback, this, ros::TransportHints().tcpNoDelay());
  ResetAndEnableLocalizer();
  return true;
}

void RosGraphLocalizerVIONodelet::PublishGraphVIOState() {
  auto msg = ros_graph_vio_wrapper_.GraphVIOStateMsg();
  if (!msg) {
    LogDebugEveryN(100, "PublishVIOState: Failed to get vio states msg.");
    return;
  }
  graph_vio_state_pub_.publish(*msg);
}

void RosGraphLocalizerVIONodelet::PublishGraphLocalizerState() {
  const auto msg = ros_graph_localizer_wrapper_.GraphLocStateMsg();
  if (msg) graph_loc_pub_.publish(*msg);
}

/*void RosGraphLocalizerVIONodelet::PublishLocalizerGraph() {
  const auto latest_localizer_graph_msg = ros_graph_localizer_wrapper_.LatestGraphMsg();
  if (!latest_localizer_graph_msg) {
    LogDebugEveryN(100, "PublishLocalizerGraph: Failed to get latest localizer graph msg.");
    return;
  }
  graph_pub_.publish(*latest_localizer_graph_msg);
}*/

void RosGraphLocalizerVIONodelet::PublishGraphVIOMessages() {
  if (!localizer_enabled()) return;

  // TODO(rsoussan): Only publish if things have changed?
  PublishGraphVIOState();
  // if (ros_graph_vio_wrapper_.publish_graph()) PublishVIOGraph();
  // if (ros_graph_vio_wrapper_.save_graph_dot_file()) ros_graph_vio_wrapper_.SaveGraphDotFile();
}

void RosGraphLocalizerVIONodelet::FeaturePointsCallback(const ff_msgs::Feature2dArray::ConstPtr& feature_array_msg) {
  if (!localizer_enabled()) return;
  ros_graph_vio_wrapper_.FeaturePointsCallback(*feature_array_msg);
}

void RosGraphLocalizerVIONodelet::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  if (!localizer_enabled()) return;
  ros_graph_vio_wrapper_.ImuCallback(*imu_msg);
  ros_graph_localizer_wrapper_.ImuCallback(*imu_msg);
}

void RosGraphLocalizerVIONodelet::FlightModeCallback(ff_msgs::FlightMode::ConstPtr const& mode) {
  ros_graph_vio_wrapper_.FlightModeCallback(*mode);
  ros_graph_localizer_wrapper_.FlightModeCallback(*mode);
}

void RosGraphLocalizerVIONodelet::PublishReset() const {
  std_msgs::Empty msg;
  reset_pub_.publish(msg);
}

void RosGraphLocalizerVIONodelet::PublishHeartbeat() {
  heartbeat_.header.stamp = ros::Time::now();
  if ((heartbeat_.header.stamp - last_heartbeat_time_).toSec() < 1.0) return;
  heartbeat_pub_.publish(heartbeat_);
  last_heartbeat_time_ = heartbeat_.header.stamp;
}

void RosGraphLocalizerVIONodelet::PublishGraphLocalizerMessages() {
  if (!localizer_enabled()) return;

  // TODO(rsoussan): Only publish if things have changed?
  PublishGraphLocalizerState();
  PublishWorldTBodyTF();
  // if (ros_graph_localizer_wrapper_.publish_graph()) PublishLocalizerGraph();
  // if (ros_graph_localizer_wrapper_.save_graph_dot_file()) ros_graph_localizer_wrapper_.SaveGraphDotFile();
}

void RosGraphLocalizerVIONodelet::PublishWorldTBodyTF() {
  const auto latest_pose = ros_graph_localizer_wrapper_.LatestPose();
  const auto latest_timestamp = ros_graph_localizer_wrapper_.LatestTimestamp();
  if (!latest_pose || !latest_timestamp) {
    LogErrorEveryN(100, "PublishWorldTBodyTF: Failed to get latest pose and timestamp.");
    return;
  }

  const auto world_T_body_tf = lc::PoseToTF(*latest_pose, "world", "body",
                                            *latest_timestamp, platform_name_);
  if (world_T_body_tf.header.stamp == last_tf_body_time_) return;
  last_tf_body_time_ = world_T_body_tf.header.stamp;
  transform_pub_.sendTransform(world_T_body_tf);
}

void RosGraphLocalizerVIONodelet::PublishWorldTDockTF() {
  const auto world_T_dock = ros_graph_localizer_wrapper_.WorldTDock();
  if (!world_T_dock) return;
  const auto world_T_dock_tf =
    lc::PoseToTF(*world_T_dock, "world", "dock/body", lc::TimeFromRosTime(ros::Time::now()), platform_name_);
  // If the rate is higher than the sim time, prevent sending repeat tfs
  if (world_T_dock_tf.header.stamp == last_tf_dock_time_) return;
  last_tf_dock_time_ = world_T_dock_tf.header.stamp;
  transform_pub_.sendTransform(world_T_dock_tf);
}

void RosGraphLocalizerVIONodelet::ARVisualLandmarksCallback(
  const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg) {
  if (!localizer_enabled()) return;
  ros_graph_localizer_wrapper_.ARVisualLandmarksCallback(*visual_landmarks_msg);
  PublishWorldTDockTF();
}

void RosGraphLocalizerVIONodelet::SparseMapVisualLandmarksCallback(
  const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg) {
  if (!localizer_enabled()) return;
  ros_graph_localizer_wrapper_.SparseMapVisualLandmarksCallback(*visual_landmarks_msg);
}

void RosGraphLocalizerVIONodelet::GraphVIOStateCallback(const ff_msgs::GraphVIOState::ConstPtr& graph_vio_state_msg) {
  if (!localizer_enabled()) return;
  ros_graph_localizer_wrapper_.GraphVIOStateCallback(*graph_vio_state_msg);
}

void RosGraphLocalizerVIONodelet::Run() {
  ros::Rate rate(100);
  // ResetAndEnableLocalizer();
  // Load Biases from file by default
  // Biases reestimated if a intialize bias service call is received
  ResetBiasesFromFileAndResetLocalizerVIO();
  while (ros::ok()) {
    private_queue_.callAvailable();
    if (localizer_enabled()) {
      ros_graph_vio_wrapper_.Update();
      PublishGraphVIOMessages();
      // TODO(rsoussan): clean this up
      if (ros_graph_vio_wrapper_.Initialized() && ros_graph_localizer_wrapper_.Initialized()) {
        ros_graph_localizer_wrapper_.graph_localizer_->pose_node_adder_->node_adder_model_.nodes_ =
         ros_graph_vio_wrapper_.graph_vio()->combined_nav_state_node_adder_->nodes_.get();
        if (ros_graph_vio_wrapper_.graph_vio()->marginals()) {
          ros_graph_localizer_wrapper_.graph_localizer_->pose_node_adder_->node_adder_model_.marginals_ =
            *(ros_graph_vio_wrapper_.graph_vio()->marginals());
        }
        // TODO(rsoussan): check for latest graph vio state msg! pass to localizer if it exists! (and not the same as
        // the last one...)
      }
      ros_graph_localizer_wrapper_.Update();
      PublishGraphLocalizerMessages();
    }
    PublishHeartbeat();
    rate.sleep();
  }
}
}  // namespace ros_graph_localizer

PLUGINLIB_EXPORT_CLASS(ros_graph_localizer::RosGraphLocalizerVIONodelet, nodelet::Nodelet);
