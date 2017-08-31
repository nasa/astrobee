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

#include "dds_ros_bridge/ros_compressed_file_ack.h"
#include "dds_ros_bridge/util.h"

#include "rapidUtil/RapidHelper.h"

#include "ff_msgs/CompressedFileAck.h"
#include "CompressedFileAckSupport.h"

namespace rea = rapid::ext::astrobee;

ff::RosCompressedFileAckToRapid::RosCompressedFileAckToRapid(
    const std::string& subscribeTopic,
    const std::string& pubTopic,
    const ros::NodeHandle &nh,
    const unsigned int queueSize)
  : RosSubRapidPub(subscribeTopic, pubTopic, nh, queueSize) {
  m_supplier_.reset(
    new ff::RosCompressedFileAckToRapid::Supplier(
      "astrobee_compressed_file_ack" + pubTopic,
      "", "AstrobeeCompressedFileAck", ""));

  m_sub_ = m_nh_.subscribe(subscribeTopic, queueSize,
    &RosCompressedFileAckToRapid::Callback, this);

  rapid::RapidHelper::initHeader(m_supplier_->event().hdr);
}

void ff::RosCompressedFileAckToRapid::Callback(
  const ff_msgs::CompressedFileAck::ConstPtr& ack) {
  rea::CompressedFileAck &msg = m_supplier_->event();
  msg.hdr.timeStamp = util::RosTime2RapidTime(ack->header.stamp);
  msg.id = ack->id;

  m_supplier_->sendEvent();
}
