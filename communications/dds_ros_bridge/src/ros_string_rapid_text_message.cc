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

#include "dds_ros_bridge/ros_string_rapid_text_message.h"

namespace ff {

RosStringRapidTextMessage::RosStringRapidTextMessage(
    const std::string& subscribe_topic,
    const std::string& pub_topic,
    const ros::NodeHandle &nh,
    const unsigned int queue_size,
    const std::string& category)
  : RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size),
    category_(category) {
  // setup parameters
  params_.topic += pub_topic;

  // instantiate provider
  provider_.reset(new rapid::TextMessager(params_));

  // start subscriber
  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosStringRapidTextMessage::CallBack,
                       this);
}

void RosStringRapidTextMessage::CallBack(
                                        const std_msgs::String::ConstPtr& msg) {
  provider_->sendText(category_.c_str(), rapid::MSG_DEBUG, msg->data.c_str());
}

}  // end namespace ff
