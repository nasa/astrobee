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

#ifndef COMMS_BRIDGE_DDS_ROS_BRIDGE_PUBLISHER_H_
#define COMMS_BRIDGE_DDS_ROS_BRIDGE_PUBLISHER_H_

/* This is a specialization of ROSBridgePublisher using a DDS conduit
*/

#include <ff_msgs/RelayAdvertisement.h>
#include <ff_msgs/RelayContent.h>
#include <ff_msgs/RelayReset.h>

#include <comms_bridge/bridge_publisher.h>

// default time to delay between advertising and publishing on that topic [sec]
#define DEFAULT_DDSROSBRIDGE_PUB_ADVERTISE_DELAY 3.0

class DDSROSBridgePublisher : public BridgePublisher {
 public:
  explicit DDSROSBridgePublisher(double ad2pub_delay = DEFAULT_DDSROSBRIDGE_PUB_ADVERTISE_DELAY);
  virtual ~DDSROSBridgePublisher();

 protected:
  // prohibit shallow copy or assignment
  DDSROSBridgePublisher(const DDSROSBridgePublisher&) : BridgePublisher(m_ad2pub_delay) {}
  void operator=(const DDSROSBridgePublisher&) {}
};

#endif  // COMMS_BRIDGE_DDS_ROS_BRIDGE_PUBLISHER_H_
