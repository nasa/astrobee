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

#include "comms_bridge/rapid_advertisement_info.h"

#include <string>

namespace ff {

RapidAdvertisementInfo::RapidAdvertisementInfo(const std::string& subscribe_topic,
                                               const std::string& subscriber_partition,
                                               GenericRapidMsgRosPub* rapid_msg_ros_pub)
    : GenericRapidSub("AstrobeeGenericCommsAdvertisementInfoProfile", subscribe_topic, rapid_msg_ros_pub),
      subscriber_partition_(subscriber_partition) {
  // connect to ddsEventLoop
  try {
    dds_event_loop_.connect<rapid::ext::astrobee::GenericCommsAdvertisementInfo>(
        this,
        subscribe_topic,                          // topic
        subscriber_partition,                      // name
        "AstrobeeGenericCommsAdvertisementInfo",  // profile
        "");                                      // library
  } catch (std::exception& e) {
    ROS_ERROR_STREAM("RapidAdvertisementInfo exception: " << e.what());
    throw;
  } catch (...) {
    ROS_ERROR("RapidAdvertisementInfo exception unknown");
    throw;
  }

  // start thread
  StartThread();
}

void RapidAdvertisementInfo::operator() (
            rapid::ext::astrobee::GenericCommsAdvertisementInfo const* data) {
  ros_pub_->ConvertAdvertisementInfo(data);
}

}  // end namespace ff
