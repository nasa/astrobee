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

#include "dds_ros_bridge/ros_log_sample.h"

namespace rea = rapid::ext::astrobee;

using rea::SeverityLevel;
using rosgraph_msgs::Log;

namespace {

#define GENERATE_SEVERITY_CASE(NAME) \
  case Log::NAME: return rea::SEVERITY_LEVEL_##NAME

#define GENERATE_SEVERITY_DEFAULT(NAME) \
  default: \
    ROS_FATAL("unknown %s: %d", level_name, level); \
    return rea::SEVERITY_LEVEL_##NAME

rea::SeverityLevel ConvertSeverityLevel(const uint8_t level) {
  static const char* level_name = "severity_level";
  switch (level) {
    GENERATE_SEVERITY_CASE(DEBUG);
    GENERATE_SEVERITY_CASE(INFO);
    GENERATE_SEVERITY_CASE(WARN);
    GENERATE_SEVERITY_CASE(ERROR);
    GENERATE_SEVERITY_CASE(FATAL);
    GENERATE_SEVERITY_DEFAULT(DEBUG);
  }
}

}  // end namespace

ff::RosLogSampleToRapid::RosLogSampleToRapid(
                                            const std::string& subscribe_topic,
                                            const std::string& pub_topic,
                                            const ros::NodeHandle &nh,
                                            const unsigned int queue_size)
  : RosSubRapidPub(subscribe_topic, pub_topic, nh, queue_size) {
  state_supplier_.reset(
    new ff::RosLogSampleToRapid::StateSupplier(
        rapid::ext::astrobee::LOG_SAMPLE_TOPIC + pub_topic,
        "", "AstrobeeLogSampleProfile", ""));

  sub_ = nh_.subscribe(subscribe_topic,
                       queue_size,
                       &RosLogSampleToRapid::MsgCallback,
                       this);

  rapid::RapidHelper::initHeader(state_supplier_->event().hdr);
}

void ff::RosLogSampleToRapid::MsgCallback(const rosgraph_msgs::LogConstPtr&
                                                                        log_msg) {
  rapid::ext::astrobee::LogSample &msg = state_supplier_->event();
  msg.hdr.timeStamp = util::RosTime2RapidTime(log_msg->header.stamp);

  msg.level = ConvertSeverityLevel(log_msg->level);

  if (log_msg->name.size() > 64) {
    ROS_ERROR("DDS: Log Name is longer than 64 characteres!");
  }
  std::strncpy(msg.name, log_msg->name.data(), 64);
  msg.name[63] = '\0';

  if (log_msg->msg.size() > 1000) {
      ROS_ERROR("DDS: Log Msg is longer than 1000 characteres!");
  }
  std::strncpy(msg.msg, log_msg->msg.data(), 1000);
  msg.msg[999] = '\0';

  state_supplier_->sendEvent();
}

