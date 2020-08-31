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
GraphLocalizerNodelet::GraphLocalizerNodelet() : ff_util::FreeFlyerNodelet(NODE_GRAPH_LOC, true) {}

void GraphLocalizerNodelet::Initialize(ros::NodeHandle* nh) {
  ff_common::InitFreeFlyerApplication(getMyArgv());
  SubscribeAndAdvertise(nh);
}

void GraphLocalizerNodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  state_pub_ = nh->advertise<ff_msgs::EkfState>(TOPIC_GRAPH_LOC_STATE, 1);
  sparse_mapping_pose_pub_ = nh->advertise<geometry_msgs::PoseStamped>(TOPIC_SPARSE_MAPPING_POSE, 1);
  graph_pub_ = nh->advertise<ff_msgs::LocalizationGraph>(TOPIC_GRAPH_LOC, 1);
  reset_pub_ = nh->advertise<std_msgs::Empty>(TOPIC_GNC_EKF_RESET, 1);

  imu_sub_ = nh->subscribe(TOPIC_HARDWARE_IMU, 1, &GraphLocalizerNodelet::ImuCallback, this,
                           ros::TransportHints().tcpNoDelay());
  ar_sub_ = nh->subscribe(TOPIC_LOCALIZATION_AR_FEATURES, 1, &GraphLocalizerNodelet::ARVisualLandmarksCallback, this,
                          ros::TransportHints().tcpNoDelay());
  of_sub_ = nh->subscribe(TOPIC_LOCALIZATION_OF_FEATURES, 1, &GraphLocalizerNodelet::OpticalFlowCallback, this,
                          ros::TransportHints().tcpNoDelay());
  vl_sub_ = nh->subscribe(TOPIC_LOCALIZATION_ML_FEATURES, 1, &GraphLocalizerNodelet::VLVisualLandmarksCallback, this,
                          ros::TransportHints().tcpNoDelay());
  bias_srv_ = nh->advertiseService(SERVICE_GNC_EKF_INIT_BIAS, &GraphLocalizerNodelet::ResetBiasesAndLocalizer, this);
  reset_srv_ = nh->advertiseService(SERVICE_GNC_EKF_RESET, &GraphLocalizerNodelet::ResetLocalizer, this);
  input_mode_srv_ = nh->advertiseService(SERVICE_GNC_EKF_SET_INPUT, &GraphLocalizerNodelet::SetMode, this);
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
  if (!localizer_enabled()) return;
  graph_localizer_wrapper_.OpticalFlowCallback(*feature_array_msg);

  // Publish loc information here since graph updates occur on optical flow updates
  PublishLocalizationState();
  PublishLocalizationGraph();
}

void GraphLocalizerNodelet::VLVisualLandmarksCallback(const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg) {
  if (!localizer_enabled()) return;
  graph_localizer_wrapper_.VLVisualLandmarksCallback(*visual_landmarks_msg);
  if (ValidVLMsg(*visual_landmarks_msg)) PublishSparseMappingPose();
}

void GraphLocalizerNodelet::ARVisualLandmarksCallback(const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg) {
  if (!localizer_enabled()) return;
  graph_localizer_wrapper_.ARVisualLandmarksCallback(*visual_landmarks_msg);
}

void GraphLocalizerNodelet::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  if (!localizer_enabled()) return;
  graph_localizer_wrapper_.ImuCallback(*imu_msg);
}

void GraphLocalizerNodelet::PublishLocalizationState() {
  const auto latest_localization_state_msg = graph_localizer_wrapper_.LatestLocalizationStateMsg();
  if (!latest_localization_state_msg) {
    LOG(WARNING) << "PublishLocalizationState: Failed to get latest localization state msg.";
    return;
  }
  state_pub_.publish(*latest_localization_state_msg);
}

void GraphLocalizerNodelet::PublishLocalizationGraph() {
  const auto latest_localization_graph_msg = graph_localizer_wrapper_.LatestLocalizationGraphMsg();
  if (!latest_localization_graph_msg) {
    LOG(WARNING) << "PublishLocalizationGraph: Failed to get latest localization graph msg.";
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

void GraphLocalizerNodelet::PublishReset() const {
  std_msgs::Empty msg;
  reset_pub_.publish(msg);
}
}  // namespace graph_localizer

PLUGINLIB_EXPORT_CLASS(graph_localizer::GraphLocalizerNodelet, nodelet::Nodelet);
