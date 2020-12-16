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
}

void RosPoseRapidPosition::DataCallback(
                              geometry_msgs::PoseStampedConstPtr const& data) {
  provider_->Publish(data);
}

}  // end namespace ff
