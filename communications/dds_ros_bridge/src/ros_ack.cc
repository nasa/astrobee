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
#include <cstring>
#include <memory>

#include "dds_ros_bridge/ros_ack.h"
#include "dds_ros_bridge/enum_helper.h"
#include "dds_ros_bridge/util.h"

#include "rapidUtil/RapidHelper.h"

#include "ff_msgs/AgentStateStamped.h"
#include "RapidConstants.h"
#include "AckSupport.h"

ff::RosAckToRapid::RosAckToRapid(
    const std::string& subscribeTopic,
    const std::string& pubTopic,
    const ros::NodeHandle &nh,
    const unsigned int queueSize)
  : RosSubRapidPub(subscribeTopic, pubTopic, nh, queueSize) {
  m_supplier_.reset(
    new ff::RosAckToRapid::StateSupplier(rapid::ACK_TOPIC + pubTopic,
      "", "RapidAckProfile", ""));

  m_sub_ = m_nh_.subscribe(subscribeTopic, queueSize,
    &RosAckToRapid::Callback, this);

  rapid::RapidHelper::initHeader(m_supplier_->event().hdr);
}

void ff::RosAckToRapid::Callback(const ff_msgs::AckStamped::ConstPtr& ack) {
  if (ack->cmd_origin == "ground") {
    rapid::Ack &msg = m_supplier_->event();
    msg.hdr.timeStamp = util::RosTime2RapidTime(ack->header.stamp);

    msg.status = util::ConvertAckStatus(ack->status);
    msg.completedStatus = util::ConvertAckCompletedStatus(ack->completed_status);
    std::strncpy(msg.cmdId, ack->cmd_id.data(), 64);
    std::strncpy(msg.message, ack->message.data(), 128);

    m_supplier_->sendEvent();
  }
}

