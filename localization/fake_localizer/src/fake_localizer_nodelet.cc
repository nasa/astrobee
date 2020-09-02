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

#include <fake_localizer/fake_localizer_nodelet.h>
#include <fake_localizer/utilities.h>
#include <ff_msgs/EkfState.h>
#include <ff_util/ff_names.h>
#include <localization_common/utilities.h>

#include <glog/logging.h>

namespace fake_localizer {
namespace lc = localization_common;
FakeLocalizerNodelet::FakeLocalizerNodelet()
    : ff_util::FreeFlyerNodelet(NODE_SIM_LOC, true), platform_name_(GetPlatform()) {}

void FakeLocalizerNodelet::Initialize(ros::NodeHandle* nh) {
  ff_common::InitFreeFlyerApplication(getMyArgv());
  SubscribeAndAdvertise(nh);
}

void FakeLocalizerNodelet::SubscribeAndAdvertise(ros::NodeHandle* nh) {
  pose_pub_ = nh->advertise<geometry_msgs::PoseStamped>(TOPIC_LOCALIZATION_POSE, 1);
  twist_pub_ = nh->advertise<geometry_msgs::TwistStamped>(TOPIC_LOCALIZATION_TWIST, 1);

  pose_sub_ = nh->subscribe(TOPIC_LOCALIZATION_TRUTH, 1, &FakeLocalizerNodelet::PoseCallback, this,
                            ros::TransportHints().tcpNoDelay());
  twist_sub_ = nh->subscribe(TOPIC_LOCALIZATION_TRUTH_TWIST, 1, &FakeLocalizerNodelet::TwistCallback, this,
                             ros::TransportHints().tcpNoDelay());

  input_mode_srv_ = nh->advertiseService(SERVICE_GNC_EKF_SET_INPUT, &FakeLocalizerNodelet::SetMode, this);
}

bool FakeLocalizerNodelet::SetMode(ff_msgs::SetEkfInput::Request& req, ff_msgs::SetEkfInput::Response& res) {
  input_mode_ = req.mode;
  return true;
}

void FakeLocalizerNodelet::PoseCallback(geometry_msgs::PoseStamped::ConstPtr const& pose) {
  assert(pose->header.frame_id == "world");
  if (input_mode_ == ff_msgs::SetEkfInputRequest::MODE_TRUTH) {
    pose_ = PoseFromMsg(*pose);
    pose_pub_.publish(pose);
    const lc::Time timestamp = lc::TimeFromHeader(pose->header);
    PublishLocState(timestamp);
  }
}

void FakeLocalizerNodelet::TwistCallback(geometry_msgs::TwistStamped::ConstPtr const& twist) {
  assert(twist->header.frame_id == "world");
  if (input_mode_ == ff_msgs::SetEkfInputRequest::MODE_TRUTH) {
    twist_ = TwistFromMsg(*twist);
    twist_pub_.publish(twist);
    const lc::Time timestamp = lc::TimeFromHeader(twist->header);
    PublishLocState(timestamp);
  }
}

void FakeLocalizerNodelet::PublishLocState(const lc::Time& timestamp) {
  if (!twist_ || !pose_) return;
  const auto loc_state_msg = LocStateMsg(*pose_, *twist_, timestamp);
  state_pub_.publish(loc_state_msg);

  // Also publish world_T_body TF
  const auto world_T_body_tf = lc::PoseToTF(*pose_, "world", "body", timestamp, platform_name_);
  transform_pub_.sendTransform(world_T_body_tf);
}
}  // namespace fake_localizer

PLUGINLIB_EXPORT_CLASS(fake_localizer::FakeLocalizerNodelet, nodelet::Nodelet);
