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

#include <ff_msgs/EkfState.h>
#include <ff_msgs/LocalizationGraph.h>
#include <ff_util/ff_names.h>
#include <graph_localizer/graph_localizer_nodelet.h>
#include <graph_localizer/utilities.h>
#include <localization_common/logger.h>
#include <localization_common/utilities.h>

#include <std_msgs/Empty.h>

namespace graph_localizer {
namespace lc = localization_common;
GraphLocalizerNodelet::GraphLocalizerNodelet()
    : ff_util::FreeFlyerNodelet(NODE_GRAPH_LOC, true), platform_name_(GetPlatform()) {
  private_nh_.setCallbackQueue(&private_queue_);
  heartbeat_.node = GetName();
  heartbeat_.nodelet_manager = ros::this_node::getName();

  config_reader::ConfigReader config;
  lc::LoadGraphLocalizerConfig(config);
  if (!config.ReadFiles()) {
    LOG(FATAL) << "Failed to read config files.";
  }
  sparse_mapping_min_num_landmarks_ = lc::LoadInt(config, "loc_adder_min_num_matches");
  ar_min_num_landmarks_ = lc::LoadInt(config, "ar_tag_loc_adder_min_num_matches");
}

void GraphLocalizerNodelet::Initialize(ros::NodeHandle* nh) {
  ff_common::InitFreeFlyerApplication(getMyArgv());
  SubscribeAndAdvertise(nh);
  Run();
}

void GraphLocalizerNodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  state_pub_ = nh->advertise<ff_msgs::EkfState>(TOPIC_GRAPH_LOC_STATE, 10);
  sparse_mapping_pose_pub_ = nh->advertise<geometry_msgs::PoseStamped>(TOPIC_SPARSE_MAPPING_POSE, 10);
  ar_tag_pose_pub_ = nh->advertise<geometry_msgs::PoseStamped>(TOPIC_AR_TAG_POSE, 10);
  graph_pub_ = nh->advertise<ff_msgs::LocalizationGraph>(TOPIC_GRAPH_LOC, 10);
  reset_pub_ = nh->advertise<std_msgs::Empty>(TOPIC_GNC_EKF_RESET, 10);
  heartbeat_pub_ = nh->advertise<ff_msgs::Heartbeat>(TOPIC_HEARTBEAT, 5, true);

  // Buffer max should be at least 10 and a max of 2 seconds of data
  // Imu freq: ~62.5
  constexpr int kMaxNumImuMsgs = 125;
  // AR freq: ~1
  constexpr int kMaxNumARMsgs = 10;
  // VL freq: ~1
  constexpr int kMaxNumVLMsgs = 10;
  // OF freq: ~10
  constexpr int kMaxNumOFMsgs = 20;

  imu_sub_ = private_nh_.subscribe(TOPIC_HARDWARE_IMU, kMaxNumImuMsgs, &GraphLocalizerNodelet::ImuCallback, this,
                                   ros::TransportHints().tcpNoDelay());
  ar_sub_ =
    private_nh_.subscribe(TOPIC_LOCALIZATION_AR_FEATURES, kMaxNumARMsgs,
                          &GraphLocalizerNodelet::ARVisualLandmarksCallback, this, ros::TransportHints().tcpNoDelay());
  of_sub_ =
    private_nh_.subscribe(TOPIC_LOCALIZATION_OF_FEATURES, kMaxNumOFMsgs, &GraphLocalizerNodelet::OpticalFlowCallback,
                          this, ros::TransportHints().tcpNoDelay());
  vl_sub_ =
    private_nh_.subscribe(TOPIC_LOCALIZATION_ML_FEATURES, kMaxNumVLMsgs,
                          &GraphLocalizerNodelet::VLVisualLandmarksCallback, this, ros::TransportHints().tcpNoDelay());
  bias_srv_ =
    private_nh_.advertiseService(SERVICE_GNC_EKF_INIT_BIAS, &GraphLocalizerNodelet::ResetBiasesAndLocalizer, this);
  reset_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_RESET, &GraphLocalizerNodelet::ResetLocalizer, this);
  input_mode_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_SET_INPUT, &GraphLocalizerNodelet::SetMode, this);
}

bool GraphLocalizerNodelet::SetMode(ff_msgs::SetEkfInput::Request& req, ff_msgs::SetEkfInput::Response& res) {
  const auto input_mode = req.mode;
  static int last_mode = -1;
  if (input_mode == ff_msgs::SetEkfInputRequest::MODE_NONE) {
    LogInfo("Received Mode None request, turning off localizer.");
    DisableLocalizer();
  } else if (last_mode == ff_msgs::SetEkfInputRequest::MODE_NONE) {
    LogInfo(
      "Received Mode request that is not None and current mode is "
      "None, resetting localizer.");
    ResetAndEnableLocalizer();
  }

  // Might need to resestimate world_T_dock on ar mode switch
  if (input_mode == ff_msgs::SetEkfInputRequest::MODE_AR_TAGS &&
      last_mode != ff_msgs::SetEkfInputRequest::MODE_AR_TAGS) {
    LogInfo("SetMode: Switching to AR_TAG mode.");
    graph_localizer_wrapper_.MarkWorldTDockForResettingIfNecessary();
  }
  return true;
}

void GraphLocalizerNodelet::DisableLocalizer() { localizer_enabled_ = false; }

void GraphLocalizerNodelet::EnableLocalizer() { localizer_enabled_ = true; }

bool GraphLocalizerNodelet::localizer_enabled() const { return localizer_enabled_; }

bool GraphLocalizerNodelet::ResetBiasesAndLocalizer(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  graph_localizer_wrapper_.ResetBiasesAndLocalizer();
  PublishReset();
  EnableLocalizer();
  return true;
}

bool GraphLocalizerNodelet::ResetLocalizer(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  ResetAndEnableLocalizer();
  return true;
}

void GraphLocalizerNodelet::ResetAndEnableLocalizer() {
  graph_localizer_wrapper_.ResetLocalizer();
  PublishReset();
  EnableLocalizer();
}

void GraphLocalizerNodelet::OpticalFlowCallback(const ff_msgs::Feature2dArray::ConstPtr& feature_array_msg) {
  of_timer_.HeaderDiff(feature_array_msg->header);
  of_timer_.LogEveryN(100);

  if (!localizer_enabled()) return;
  graph_localizer_wrapper_.OpticalFlowCallback(*feature_array_msg);
}

void GraphLocalizerNodelet::VLVisualLandmarksCallback(const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg) {
  vl_timer_.HeaderDiff(visual_landmarks_msg->header);
  vl_timer_.LogEveryN(100);

  if (!localizer_enabled()) return;
  graph_localizer_wrapper_.VLVisualLandmarksCallback(*visual_landmarks_msg);
  if (ValidVLMsg(*visual_landmarks_msg, sparse_mapping_min_num_landmarks_)) PublishSparseMappingPose();
}

void GraphLocalizerNodelet::ARVisualLandmarksCallback(const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg) {
  ar_timer_.HeaderDiff(visual_landmarks_msg->header);
  ar_timer_.LogEveryN(100);

  if (!localizer_enabled()) return;
  graph_localizer_wrapper_.ARVisualLandmarksCallback(*visual_landmarks_msg);
  PublishWorldTDockTF();
  if (ValidVLMsg(*visual_landmarks_msg, ar_min_num_landmarks_)) PublishARTagPose();
}

void GraphLocalizerNodelet::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  imu_timer_.HeaderDiff(imu_msg->header);
  imu_timer_.LogEveryN(100);

  if (!localizer_enabled()) return;
  graph_localizer_wrapper_.ImuCallback(*imu_msg);
}

void GraphLocalizerNodelet::PublishLocalizationState() {
  const auto latest_localization_state_msg = graph_localizer_wrapper_.LatestLocalizationStateMsg();
  if (!latest_localization_state_msg) {
    LogWarningEveryN(100, "PublishLocalizationState: Failed to get latest localization state msg.");
    return;
  }
  state_pub_.publish(*latest_localization_state_msg);
}

void GraphLocalizerNodelet::PublishLocalizationGraph() {
  const auto latest_localization_graph_msg = graph_localizer_wrapper_.LatestLocalizationGraphMsg();
  if (!latest_localization_graph_msg) {
    LogWarningEveryN(100, "PublishLocalizationGraph: Failed to get latest localization graph msg.");
    return;
  }
  graph_pub_.publish(*latest_localization_graph_msg);
}

void GraphLocalizerNodelet::PublishSparseMappingPose() const {
  const auto latest_sparse_mapping_pose_msg = graph_localizer_wrapper_.LatestSparseMappingPoseMsg();
  if (!latest_sparse_mapping_pose_msg) {
    LogWarning("PublishSparseMappingPose: Failed to get latest sparse mapping pose msg.");
    return;
  }
  sparse_mapping_pose_pub_.publish(*latest_sparse_mapping_pose_msg);
}

void GraphLocalizerNodelet::PublishARTagPose() const {
  const auto latest_ar_tag_pose_msg = graph_localizer_wrapper_.LatestARTagPoseMsg();
  if (!latest_ar_tag_pose_msg) {
    LogWarning("PublishARTagPose: Failed to get latest ar tag pose msg.");
    return;
  }
  ar_tag_pose_pub_.publish(*latest_ar_tag_pose_msg);
}

void GraphLocalizerNodelet::PublishWorldTBodyTF() {
  const auto latest_combined_nav_state = graph_localizer_wrapper_.LatestCombinedNavState();
  if (!latest_combined_nav_state) {
    LogErrorEveryN(100, "PublishWorldTBodyTF: Failed to get world_T_body.");
    return;
  }

  const auto world_T_body_tf = lc::PoseToTF(latest_combined_nav_state->pose(), "world", "body",
                                            latest_combined_nav_state->timestamp(), platform_name_);
  transform_pub_.sendTransform(world_T_body_tf);
}

void GraphLocalizerNodelet::PublishWorldTDockTF() {
  const auto world_T_dock = graph_localizer_wrapper_.estimated_world_T_dock();
  const auto world_T_dock_tf =
    lc::PoseToTF(world_T_dock, "world", "dock/body", lc::TimeFromRosTime(ros::Time::now()), platform_name_);
  transform_pub_.sendTransform(world_T_dock_tf);
}

void GraphLocalizerNodelet::PublishReset() const {
  std_msgs::Empty msg;
  reset_pub_.publish(msg);
}

void GraphLocalizerNodelet::PublishHeartbeat() {
  heartbeat_.header.stamp = ros::Time::now();
  heartbeat_pub_.publish(heartbeat_);
}

void GraphLocalizerNodelet::PublishGraphMessages() {
  if (!localizer_enabled()) return;

  // Publish loc information here since graph updates occur on optical flow updates
  PublishLocalizationState();
  if (graph_localizer_wrapper_.publish_localization_graph()) PublishLocalizationGraph();
  if (graph_localizer_wrapper_.save_localization_graph_dot_file())
    graph_localizer_wrapper_.SaveLocalizationGraphDotFile();
}

void GraphLocalizerNodelet::Run() {
  ros::Rate rate(100);
  while (ros::ok()) {
    callbacks_timer_.Start();
    private_queue_.callAvailable();
    callbacks_timer_.StopAndLogEveryN(100);
    graph_localizer_wrapper_.Update();
    PublishGraphMessages();
    PublishHeartbeat();
    rate.sleep();
  }
}
}  // namespace graph_localizer

PLUGINLIB_EXPORT_CLASS(graph_localizer::GraphLocalizerNodelet, nodelet::Nodelet);
