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

#ifndef COMMS_BRIDGE_ROS_DDS_BRIDGE_SUBSCRIBER_H_
#define COMMS_BRIDGE_ROS_DDS_BRIDGE_SUBSCRIBER_H_

/* This is a specialization of ROSBridgeSubscriber using a DDS conduit
*/

#include <ff_msgs/RelayAdvertisement.h>
#include <ff_msgs/RelayContent.h>
#include <ff_msgs/RelayReset.h>

#include <comms_bridge/bridge_subscriber.h>

// // SoraCore Includes
// #include "knDds/DdsSupport.h"
// #include "knDds/DdsEntitiesFactory.h"
// #include "knDds/DdsEntitiesFactorySvc.h"

// miro includes
#include <miro/Configuration.h>
#include <miro/Robot.h>
#include <miro/Log.h>

#include <string>

class ROSDDSBridgeSubscriber : public BridgeSubscriber {
 public:
  explicit ROSDDSBridgeSubscriber(std::string agent_name);
  virtual ~ROSDDSBridgeSubscriber();

  // Called with the mutex held
  virtual void subscribeTopic(std::string const& in_topic, const RelayTopicInfo& info);

  // Called with the mutex held
  virtual void advertiseTopic(const RelayTopicInfo& info);

  // Called with the mutex held
  virtual void relayMessage(const RelayTopicInfo& topic_info, ContentInfo const& content_info);

 protected:
  // prohibit shallow copy or assignment
  ROSDDSBridgeSubscriber(const ROSDDSBridgeSubscriber&) {}
  void operator=(const ROSDDSBridgeSubscriber&) {}
 private:
  std::string agent_name_, participant_name_;
};

#endif  // COMMS_BRIDGE_ROS_DDS_BRIDGE_SUBSCRIBER_H_
