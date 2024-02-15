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

#include "comms_bridge/rapid_pub_request.h"

#include <string>

namespace ff {

RapidPubRequest::RapidPubRequest(std::string const& robot_name) :
    request_seq_(0) {
  std::string dds_topic_name = robot_name + "-" +
                            rapid::ext::astrobee::GENERIC_COMMS_REQUEST_TOPIC;
  ROS_INFO("Comms Bridge: DDS Publisher DDS request topic name: %s\n",
           dds_topic_name.c_str());
  request_supplier_.reset(new RapidPubRequest::RequestSupplier(
    dds_topic_name, "", "AstrobeeGenericCommsRequestProfile", ""));

  rapid::RapidHelper::initHeader(request_supplier_->event().hdr);
}

RapidPubRequest::~RapidPubRequest() {}

void RapidPubRequest::SendRequest(std::string const& output_topic) {
  rapid::ext::astrobee::GenericCommsRequest &msg = request_supplier_->event();

  msg.hdr.timeStamp = comms_util::RosTime2RapidTime(ros::Time::now());
  msg.hdr.serial = ++request_seq_;

  // Currently the output topic can only be 128 characters long
  unsigned int size = output_topic.size();
  if (size > 127) {
    ROS_ERROR("Comms Bridge: Out topic for topic %s is greater than 128 characters. Please change idl.",
              output_topic.c_str());
    size = 127;
  }
  memset(msg.outputTopic, 0, 128);
  strncpy(msg.outputTopic, output_topic.data(), size);
  msg.outputTopic[size] = '\0';

  // Send message
  request_supplier_->sendEvent();
}


}  // end namespace ff
