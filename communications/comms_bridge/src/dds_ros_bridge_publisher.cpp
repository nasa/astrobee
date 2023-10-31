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

#include "comms_bridge/dds_ros_bridge_publisher.h"

DDSROSBridgePublisher::DDSROSBridgePublisher(double ad2pub_delay) : BridgePublisher(ad2pub_delay) {}

DDSROSBridgePublisher::~DDSROSBridgePublisher() {}

// When the peered subscriber transmits advertisement information, call:
// advertiseTopic(const std::string &output_topic, const AdvertisementInfo &ad_info)
// while holding m_mutex
// to setup the output ROS advertisement

// When the peered subscriber transmits a serialized message, call:
// relayMessage(RelayTopicInfo &topic_info, const ContentInfo &content_info)
// to publish the output ROS message
