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

#ifndef COMMS_BRIDGE_GENERIC_ROS_SUB_RAPID_PUB_H_
#define COMMS_BRIDGE_GENERIC_ROS_SUB_RAPID_PUB_H_

/* This is a specialization of ROSBridgeSubscriber using a DDS conduit
*/

#include <comms_bridge/bridge_subscriber.h>
#include <comms_bridge/generic_rapid_pub.h>

#include <ff_msgs/GenericCommsAdvertisementInfo.h>
#include <ff_msgs/GenericCommsContent.h>
#include <ff_msgs/GenericCommsReset.h>

#include <map>
#include <string>
#include <utility>
#include <vector>

#include "dds_msgs/GenericCommsRequestSupport.h"

namespace ff {

class GenericROSSubRapidPub : public BridgeSubscriber {
 public:
  GenericROSSubRapidPub();
  ~GenericROSSubRapidPub();

  void AddTopics(std::map<std::string,
       std::vector<std::pair<std::string, std::string>>> const& link_entries);

  void InitializeDDS(std::vector<std::string> const& connections);

  // Called with the mutex held
  virtual void subscribeTopic(std::string const& in_topic,
                              const RelayTopicInfo& info);

  // Called with the mutex held
  virtual void advertiseTopic(const RelayTopicInfo& info);

  // Called with the mutex held
  virtual void relayMessage(const RelayTopicInfo& topic_info,
                            ContentInfo const& content_info);

  void ConvertRequest(rapid::ext::astrobee::GenericCommsRequest const* data,
                      std::string const& connecting_robot);

 private:
  bool dds_initialized_;

  std::map<std::string, std::vector<std::pair<std::string, std::string>>> topic_mapping_;
  std::map<std::string, GenericRapidPubPtr> robot_connections_;
};

}  // end namespace ff

#endif  // COMMS_BRIDGE_GENERIC_ROS_SUB_RAPID_PUB_H_
