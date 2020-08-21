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
#include <ff_util/ff_names.h>
#include <graph_localizer/graph_localizer_nodelet.h>
#include <graph_localizer/utilities.h>

#include <glog/logging.h>

namespace graph_localizer {

GraphLocalizerNodelet::GraphLocalizerNodelet() {}

void GraphLocalizerNodelet::Initialize(ros::NodeHandle* nh) {
  // Bootstrap our environment
  // TODO(rsoussan): are these needed?
  // ff_common::InitFreeFlyerApplication(getMyArgv());
  SubscribeAndAdvertise(nh);
  Run();
}

void GraphLocalizerNodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  state_pub_ = nh->advertise<ff_msgs::EkfState>(TOPIC_GNC_EKF, 1);
  pose_pub_ = nh->advertise<geometry_msgs::PoseWithCovarianceStamped>(TOPIC_LOCALIZATION_POSE, 1);
  // TODO(rsoussan): is this needed?
  // twist_pub_   =
  // nh->advertise<geometry_msgs::TwistStamped>(TOPIC_LOCALIZATION_TWIST, 1);

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

// TODO(rsoussan): This is stupid and not a service.  Remove if we don't have to
// use loc manager anymore
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

// TODO(rsoussan): This is stupid and not a service.  Remove if we don't have to
// use loc manager anymore
bool GraphLocalizerNodelet::ResetBiasesAndLocalizer(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  graph_localizer_wrapper_.ResetBiasesAndLocalizer();
  EnableLocalizer();
  return true;
}

// TODO(rsoussan): This is stupid and not a service.  Remove if we don't have to
// use loc manager anymore
bool GraphLocalizerNodelet::ResetLocalizer(std_srvs::Empty::Request& req, std_srvs::Empty::Response& res) {
  ResetAndEnableLocalizer();
  return true;
}

void GraphLocalizerNodelet::ResetAndEnableLocalizer() {
  graph_localizer_wrapper_.ResetLocalizer();
  EnableLocalizer();
}

void GraphLocalizerNodelet::OpticalFlowCallback(const ff_msgs::Feature2dArray::ConstPtr& feature_array_msg) {
  if (!localizer_enabled()) return;
  graph_localizer_wrapper_.OpticalFlowCallback(*feature_array_msg);
  // TODO(rsoussan): move these somwhere else?
  PublishPose();
  PublishLocalizationState();
}

void GraphLocalizerNodelet::VLVisualLandmarksCallback(const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg) {
  if (!localizer_enabled()) return;
  graph_localizer_wrapper_.VLVisualLandmarksCallback(*visual_landmarks_msg);
}

void GraphLocalizerNodelet::ARVisualLandmarksCallback(const ff_msgs::VisualLandmarks::ConstPtr& visual_landmarks_msg) {
  if (!localizer_enabled()) return;
  graph_localizer_wrapper_.ARVisualLandmarksCallback(*visual_landmarks_msg);
}

void GraphLocalizerNodelet::ImuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
  if (!localizer_enabled()) return;
  graph_localizer_wrapper_.ImuCallback(*imu_msg);
}

void GraphLocalizerNodelet::PublishLocalizationState() const {
  ff_msgs::EkfState latest_localization_msg;
  if (!graph_localizer_wrapper_.LatestLocalizationMsg(latest_localization_msg)) return;
  state_pub_.publish(latest_localization_msg);
}

void GraphLocalizerNodelet::PublishPose() const {
  geometry_msgs::PoseWithCovarianceStamped latest_pose_msg;
  if (!graph_localizer_wrapper_.LatestPoseMsg(latest_pose_msg)) return;
  pose_pub_.publish(latest_pose_msg);
}

void GraphLocalizerNodelet::Run() {
  // TODO(rsoussan): handle reset? handle mode change?
  ros::Rate rate(100);  // 100 Hz
  while (ros::ok()) {
    ros::spinOnce();
    rate.sleep();
  }
}
}  // namespace graph_localizer

PLUGINLIB_EXPORT_CLASS(graph_localizer::GraphLocalizerNodelet, nodelet::Nodelet);
