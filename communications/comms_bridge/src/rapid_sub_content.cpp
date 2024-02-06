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

#include "comms_bridge/rapid_sub_content.h"

#include <string>

namespace ff {

RapidSubContent::RapidSubContent(const std::string& entity_name,
                                 const std::string& subscribe_topic,
                                 const std::string& subscriber_partition,
                                 GenericRapidMsgRosPub* rapid_msg_ros_pub)
    : GenericRapidSub(entity_name, subscribe_topic, subscriber_partition),
      ros_pub_(rapid_msg_ros_pub) {
  // connect to ddsEventLoop
  try {
    dds_event_loop_.connect<rapid::ext::astrobee::GenericCommsContent>(this,
                                              subscribe_topic,       // topic
                                              subscriber_partition,  // name
                                              entity_name,           // profile
                                              "");                   // library
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("RapidSubContent exception: " << e.what());
    throw;
  } catch (...) {
    ROS_ERROR("RapidSubContent exception unknown");
    throw;
  }

  // Start thread
  StartThread();
}

void RapidSubContent::operator() (
                    rapid::ext::astrobee::GenericCommsContent const* content) {
  ros_pub_->ConvertContent(content, subscriber_partition_);
}

}  // end namespace ff
