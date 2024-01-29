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

#ifndef COMMS_BRIDGE_GENERIC_RAPID_MSG_ROS_PUB_H_
#define COMMS_BRIDGE_GENERIC_RAPID_MSG_ROS_PUB_H_

#include <comms_bridge/bridge_publisher.h>
#include <comms_bridge/util.h>

#include <string>
#include <map>

#include "dds_msgs/GenericCommsAdvertisementInfoSupport.h"
#include "dds_msgs/GenericCommsContentSupport.h"

// default time to delay between advertisement and publishing on that topic [sec]
#define DEFAULT_ADVERTISE_TO_PUB_DELAY 3.0

namespace ff {

class GenericRapidMsgRosPub : public BridgePublisher {
 public:
  explicit GenericRapidMsgRosPub(double ad2pub_delay = DEFAULT_ADVERTISE_TO_PUB_DELAY);
  virtual ~GenericRapidMsgRosPub();

  void ConvertAdvertisementInfo(
              rapid::ext::astrobee::GenericCommsAdvertisementInfo const* data);
  void ConvertContent(rapid::ext::astrobee::GenericCommsContent const* data);
};

}  // end namespace ff

#endif  // COMMS_BRIDGE_GENERIC_RAPID_MSG_ROS_PUB_H_
