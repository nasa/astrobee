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

#include "comms_bridge/ros_dds_bridge_subscriber.h"

ROSDDSBridgeSubscriber::ROSDDSBridgeSubscriber() {
  // FIXME: any needed setup
}

ROSDDSBridgeSubscriber::~ROSDDSBridgeSubscriber() {
  // FIXME: any needed cleanup
}

// Called with the mutex held
void ROSDDSBridgeSubscriber::subscribeTopic(std::string const& in_topic, const RelayTopicInfo& info) {
  // FIXME: DDS stuff here
}

// Called with the mutex held
void ROSDDSBridgeSubscriber::advertiseTopic(const RelayTopicInfo& info) {
  // FIXME: DDS stuff here
}

// Called with the mutex held
void ROSDDSBridgeSubscriber::relayMessage(const RelayTopicInfo& topic_info, ContentInfo const& content_info) {
  // FIXME: DDS stuff here
}
