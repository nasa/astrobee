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
#include <localization_common/utilities.h>

#include <std_msgs/Empty.h>

#include <glog/logging.h>

namespace graph_localizer {
namespace lc = localization_common;
GraphLocalizerNodelet::GraphLocalizerNodelet()
    : ff_util::FreeFlyerNodelet(NODE_GRAPH_LOC, true), platform_name_(GetPlatform()) {
  private_nh_.setCallbackQueue(&private_queue_);
}

void GraphLocalizerNodelet::Initialize(ros::NodeHandle* nh) {
  ff_common::InitFreeFlyerApplication(getMyArgv());
  SubscribeAndAdvertise(nh);
  Run();
}

void GraphLocalizerNodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  // TODO(Rsoussan): should these use private nh as well??
  state_pub_ = nh->advertise<ff_msgs::EkfState>(TOPIC_GRAPH_LOC_STATE, 10);
  sparse_mapping_pose_pub_ = nh->advertise<geometry_msgs::PoseStamped>(TOPIC_SPARSE_MAPPING_POSE, 10);
  graph_pub_ = nh->advertise<ff_msgs::LocalizationGraph>(TOPIC_GRAPH_LOC, 10);
  reset_pub_ = nh->advertise<std_msgs::Empty>(TOPIC_GNC_EKF_RESET, 10);

  imu_sub_ = private_nh_.subscribe(TOPIC_HARDWARE_IMU, 0, &GraphLocalizerNodelet::ImuCallback, this,
                                   ros::TransportHints().tcpNoDelay());
  ar_sub_ = private_nh_.subscribe(TOPIC_LOCALIZATION_AR_FEATURES, 0, &GraphLocalizerNodelet::ARVisualLandmarksCallback,
                                  this, ros::TransportHints().tcpNoDelay());
  of_sub_ = private_nh_.subscribe(TOPIC_LOCALIZATION_OF_FEATURES, 0, &GraphLocalizerNodelet::OpticalFlowCallback, this,
                                  ros::TransportHints().tcpNoDelay());
  vl_sub_ = private_nh_.subscribe(TOPIC_LOCALIZATION_ML_FEATURES, 0, &GraphLocalizerNodelet::VLVisualLandmarksCallback,
                                  this, ros::TransportHints().tcpNoDelay());
  bias_srv_ =
    private_nh_.advertiseService(SERVICE_GNC_EKF_INIT_BIAS, &GraphLocalizerNodelet::ResetBiasesAndLocalizer, this);
  reset_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_RESET, &GraphLocalizerNodelet::ResetLocalizer, this);
  input_mode_srv_ = private_nh_.advertiseService(SERVICE_GNC_EKF_SET_INPUT, &GraphLocalizerNodelet::SetMode, this);
}

bool GraphLocalizerNodelet::SetMode(ff_msgs::SetEkfInput::Request& req, ff_msgs::SetEkfInput::Response& res) {
  const auto input_mode = req.mode;
  static int last_mode = -1;
  if (input_mode == ff_msgs::SetEkfInputRequest::MODE_NONE) {
    LOG(INFO) << "Received Mode None request, turning off localizer.";
    DisableLocalizer();
  } else if (last_mode == ff_msgs::SetEkfInputRequest::MODE_NONE) {
    LOG(INFO) << "Received Mode request that is not None and current mode is "
                 "None, resetting localizer.";
    ResetAndEnableLocalizer();
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

  // Publish loc information here since graph updates occur on optical flow updates
  PublishLocalizationState();
  PublishWorldTDockTF();
  if (graph_localizer_wrapper_.publish_localization_graph()) PublishLocalizationGraph();
  if (graph_localizer_wrapper_.save_localization_graph_dot_file())
    graph_localizer_wrapper_.SaveLocalizationGraphDotFile();
}

void GraphLocalizerNodelet::VLVisualLandmarksCallback(const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg) {
  vl_timer_.HeaderDiff(visual_landmarks_msg->header);
  vl_timer_.LogEveryN(100);

  if (!localizer_enabled()) return;
  graph_localizer_wrapper_.VLVisualLandmarksCallback(*visual_landmarks_msg);
  if (ValidVLMsg(*visual_landmarks_msg)) PublishSparseMappingPose();
}

void GraphLocalizerNodelet::ARVisualLandmarksCallback(const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg) {
  ar_timer_.HeaderDiff(visual_landmarks_msg->header);
  ar_timer_.LogEveryN(100);

  if (!localizer_enabled()) return;
  graph_localizer_wrapper_.ARVisualLandmarksCallback(*visual_landmarks_msg);
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
    LOG_EVERY_N(WARNING, 100) << "PublishLocalizationState: Failed to get latest localization state msg.";
    return;
  }
  state_pub_.publish(*latest_localization_state_msg);
}

void GraphLocalizerNodelet::PublishLocalizationGraph() {
  const auto latest_localization_graph_msg = graph_localizer_wrapper_.LatestLocalizationGraphMsg();
  if (!latest_localization_graph_msg) {
    LOG_EVERY_N(WARNING, 100) << "PublishLocalizationGraph: Failed to get latest localization graph msg.";
    return;
  }
  graph_pub_.publish(*latest_localization_graph_msg);
}

void GraphLocalizerNodelet::PublishSparseMappingPose() const {
  const auto latest_sparse_mapping_pose_msg = graph_localizer_wrapper_.LatestSparseMappingPoseMsg();
  if (!latest_sparse_mapping_pose_msg) {
    LOG(WARNING) << "PublishSparseMappingPose: Failed to get latest sparse mapping pose msg.";
    return;
  }
  sparse_mapping_pose_pub_.publish(*latest_sparse_mapping_pose_msg);
}

void GraphLocalizerNodelet::PublishWorldTBodyTF() {
  const auto latest_combined_nav_state = graph_localizer_wrapper_.LatestCombinedNavState();
  if (!latest_combined_nav_state) {
    LOG_EVERY_N(ERROR, 100) << "PublishWorldTBodyTF: Failed to get world_T_body.";
    return;
  }

  const auto world_T_body_tf = lc::PoseToTF(latest_combined_nav_state->pose(), "world", "body",
                                            latest_combined_nav_state->timestamp(), platform_name_);
  transform_pub_.sendTransform(world_T_body_tf);
}

void GraphLocalizerNodelet::PublishWorldTDockTF() {
  const auto world_T_dock = graph_localizer_wrapper_.estimated_world_T_dock();
  if (!world_T_dock) {
    LOG_EVERY_N(WARNING, 100) << "PublishWorldTDockTF: Failed to get world_T_dock.";
    return;
  }

  const auto world_T_dock_tf =
    lc::PoseToTF(world_T_dock->first, "world", "dock/body", world_T_dock->second, platform_name_);
  transform_pub_.sendTransform(world_T_dock_tf);
}

void GraphLocalizerNodelet::PublishReset() const {
  std_msgs::Empty msg;
  reset_pub_.publish(msg);
}

void GraphLocalizerNodelet::Run() {
  ros::Rate rate(100);
  while (ros::ok()) {
    callbacks_timer_.Start();
    private_queue_.callAvailable();
    callbacks_timer_.StopAndLog();
    graph_localizer_wrapper_.Update();
    rate.sleep();
  }
}
}  // namespace graph_localizer

PLUGINLIB_EXPORT_CLASS(graph_localizer::GraphLocalizerNodelet, nodelet::Nodelet);
