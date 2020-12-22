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

#include "dds_ros_bridge/ros_pose_rapid_position.h"

namespace ff {

RosPoseRapidPosition::RosPoseRapidPosition(const std::string& subscribe_topic,
                                           const std::string& pub_topic,
                                           bool use_rate,
                                           const ros::NodeHandle& nh,
                                           const unsigned int queue_size) :
    RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size) {
  params_.config.poseEncoding = rapid::RAPID_ROT_QUAT;

  params_.topicSuffix += pub_topic;

  // instantiate provider
  provider_.reset(
    new rapid::PositionProviderRosPoseHelper(params_, "RosPoseRapidPosition"));

  // start subscriber
  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosPoseRapidPosition::DataCallback,
                       this);

  // Setup timers for publishing the position but don't start them until the
  // rate is set. The bridge will set the rate at the end of its init if needed
  position_timer_ = nh_.createTimer(ros::Rate(1.0),
                                    &RosPoseRapidPosition::PubPosition,
                                    this,
                                    false,
                                    false);

  // This is used for ground testing and in those cases, we don't care about
  // limiting the rate of the messge.
  use_rate_ = use_rate;
}

void RosPoseRapidPosition::DataCallback(
                              geometry_msgs::PoseStampedConstPtr const& data) {
  if (use_rate_) {
    pose_msg_ = data;
  } else {
    provider_->Publish(data);
  }
}

void RosPoseRapidPosition::PubPosition(const ros::TimerEvent& event) {
  // Make sure we have received a pose message before trying to send it
  if (pose_msg_ != NULL) {
    provider_->Publish(pose_msg_);
  }
}

void RosPoseRapidPosition::SetPositionPublishRate(float rate) {
  if (rate == 0) {
    position_timer_.stop();
  } else {
    position_timer_.setPeriod(ros::Duration(ros::Rate(rate)));
    position_timer_.start();  // Start in case it was never started
  }
}
}  // end namespace ff
