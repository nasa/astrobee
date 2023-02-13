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

#include <ground_truth_localizer/ground_truth_localizer_nodelet.h>
#include <ground_truth_localizer/utilities.h>
#include <ff_common/ff_names.h>
#include <localization_common/utilities.h>

#include <glog/logging.h>

namespace ground_truth_localizer {

FF_DEFINE_LOGGER("ground_truth_localizer");

namespace lc = localization_common;
GroundTruthLocalizerNodelet::GroundTruthLocalizerNodelet(const rclcpp::NodeOptions& options)
    : ff_util::FreeFlyerComponent(options, NODE_SIM_LOC, true) {}

void GroundTruthLocalizerNodelet::Initialize(NodeHandle &nh) {
  platform_name_ = GetPlatform();
  platform_name_ = (platform_name_.empty() ? "" : platform_name_ + "/");
  SubscribeAndAdvertise(nh);
  // Initialize the transform broadcaster
  transform_pub_ = std::make_unique<tf2_ros::TransformBroadcaster>(*nh);
}

void GroundTruthLocalizerNodelet::SubscribeAndAdvertise(NodeHandle &nh) {
  node_ = nh;
  pose_pub_ = FF_CREATE_PUBLISHER(nh, geometry_msgs::PoseStamped, TOPIC_LOCALIZATION_POSE, 1);
  twist_pub_ = FF_CREATE_PUBLISHER(nh, geometry_msgs::TwistStamped, TOPIC_LOCALIZATION_TWIST, 1);
  state_pub_ = FF_CREATE_PUBLISHER(nh, ff_msgs::EkfState, TOPIC_GNC_EKF, 1);
  heartbeat_pub_ = FF_CREATE_PUBLISHER(nh, ff_msgs::Heartbeat, TOPIC_HEARTBEAT, 5);
  reset_pub_ = FF_CREATE_PUBLISHER(nh, std_msgs::Empty, TOPIC_GNC_EKF_RESET, 10);

  pose_sub_ = FF_CREATE_SUBSCRIBER(nh,
                                   geometry_msgs::PoseStamped,
                                   TOPIC_LOCALIZATION_TRUTH,
                                   1,
                                   &GroundTruthLocalizerNodelet::PoseCallback);
  twist_sub_ = FF_CREATE_SUBSCRIBER(nh,
                                  geometry_msgs::TwistStamped,
                                  TOPIC_LOCALIZATION_TRUTH_TWIST,
                                  1,
                                  &GroundTruthLocalizerNodelet::TwistCallback);

  input_mode_srv_ = nh->create_service<ff_msgs::SetEkfInput>(
    SERVICE_GNC_EKF_SET_INPUT,
    std::bind(&GroundTruthLocalizerNodelet::SetMode, this, std::placeholders::_1, std::placeholders::_2));
  bias_srv_ = nh->create_service<std_srvs::Empty>(SERVICE_GNC_EKF_INIT_BIAS,
      std::bind(&GroundTruthLocalizerNodelet::DefaultServiceResponse,
                              this, std::placeholders::_1, std::placeholders::_2));
  bias_from_file_srv_ = nh->create_service<std_srvs::Empty>(SERVICE_GNC_EKF_INIT_BIAS_FROM_FILE,
      std::bind(&GroundTruthLocalizerNodelet::DefaultServiceResponse,
                              this, std::placeholders::_1, std::placeholders::_2));
  reset_srv_ = nh->create_service<std_srvs::Empty>(SERVICE_GNC_EKF_RESET,
      std::bind(&GroundTruthLocalizerNodelet::DefaultServiceResponse,
                              this, std::placeholders::_1, std::placeholders::_2));
}

bool GroundTruthLocalizerNodelet::SetMode(const std::shared_ptr<ff_msgs::SetEkfInput::Request> req,
                                          std::shared_ptr<ff_msgs::SetEkfInput::Response> res) {
  input_mode_ = req->mode;
  return true;
}

bool GroundTruthLocalizerNodelet::DefaultServiceResponse(const std::shared_ptr<std_srvs::Empty::Request> req,
                                                         std::shared_ptr<std_srvs::Empty::Response> res) {
  return true;
}

void GroundTruthLocalizerNodelet::PoseCallback(geometry_msgs::PoseStamped::ConstPtr const& pose) {
  assert(pose->header.frame_id == "world");
  pose_ = PoseFromMsg(*pose);
  pose_pub_->publish(*pose);
  const lc::Time timestamp = lc::TimeFromHeader(pose->header);
  PublishLocState(timestamp);
  heartbeat_.header.stamp = GetTimeNow();
  // Publish heartbeat for graph localizer and imu augmentor since flight software expects this
  // and this runs in place of them
  heartbeat_.node = NODE_GRAPH_LOC;
  heartbeat_pub_->publish(heartbeat_);
  heartbeat_.node = NODE_IMU_AUG;
  heartbeat_pub_->publish(heartbeat_);
}

void GroundTruthLocalizerNodelet::TwistCallback(geometry_msgs::TwistStamped::ConstPtr const& twist) {
  assert(twist->header.frame_id == "world");
  twist_ = TwistFromMsg(*twist);
  twist_pub_->publish(*twist);
  const lc::Time timestamp = lc::TimeFromHeader(twist->header);
  PublishLocState(timestamp);
}

void GroundTruthLocalizerNodelet::PublishLocState(const lc::Time& timestamp) {
  if (!twist_ || !pose_) return;
  const auto loc_state_msg = LocStateMsg(*pose_, *twist_, timestamp);
  state_pub_->publish(loc_state_msg);

  // Also publish world_T_body TF
  const auto world_T_body_tf = lc::PoseToTF(*pose_, "world", "body", timestamp, platform_name_);

  // If the rate is higher than the sim time, prevent repeated timestamps
  if (world_T_body_tf.header.stamp == last_time_) return;
  last_time_ = world_T_body_tf.header.stamp;

  transform_pub_->sendTransform(world_T_body_tf);
}
}  // namespace ground_truth_localizer

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(ground_truth_localizer::GroundTruthLocalizerNodelet)
