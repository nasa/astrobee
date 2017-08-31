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

#include <string>

#include "dds_ros_bridge/ros_string_rapid_text_message.h"

namespace ff {

RosStringRapidTextMessage::RosStringRapidTextMessage(
    const std::string& subscribeTopic, const std::string& pubTopic,
    const ros::NodeHandle &nh, const unsigned int queueSize,
    const std::string& category)
  : RosSubRapidPub(subscribeTopic, pubTopic, nh, queueSize),
    m_category_(category) {
  // setup parameters
  m_params_.topic += pubTopic;

  // instantiate provider
  m_provider_.reset(new rapid::TextMessager(m_params_));

  // start subscriber
  m_sub_ = m_nh_.subscribe(subscribeTopic, queueSize,
    &RosStringRapidTextMessage::CallBack, this);
}

void RosStringRapidTextMessage::CallBack(
  const std_msgs::String::ConstPtr& msg) {
  m_provider_->sendText(m_category_.c_str(),
    rapid::MSG_DEBUG, msg->data.c_str());
}

}  // end namespace ff
