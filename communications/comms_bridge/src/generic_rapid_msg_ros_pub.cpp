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

#include <comms_bridge/generic_rapid_msg_ros_pub.h>

#include <map>
#include <string>
#include <vector>

namespace ff {

GenericRapidMsgRosPub::GenericRapidMsgRosPub(double ad2pub_delay) :
    BridgePublisher(ad2pub_delay),
    dds_initialized_(false),
    enable_advertisement_info_request_(false) {
}

GenericRapidMsgRosPub::~GenericRapidMsgRosPub() {}

void GenericRapidMsgRosPub::InitializeDDS(std::vector<std::string> const& connections,
                                          bool enable_advertisement_info_request) {
  std::string robot_name;
  for (size_t i = 0; i < connections.size(); ++i) {
    robot_name = connections[i];
    RapidPubRequestPtr rapid_pub = std::make_shared<RapidPubRequest>(robot_name);
    robot_connections_[robot_name] = rapid_pub;
  }

  enable_advertisement_info_request_ = enable_advertisement_info_request;

  dds_initialized_ = true;
}

// Handle Ad Message
void GenericRapidMsgRosPub::ConvertAdvertisementInfo(
            rapid::ext::astrobee::GenericCommsAdvertisementInfo const* data) {
  const std::lock_guard<std::mutex> lock(m_mutex_);

  const std::string output_topic = data->outputTopic;

  ROS_INFO("Comms Bridge Nodelet: Received advertisement message for topic %s\n",
           output_topic.c_str());

  AdvertisementInfo ad_info;
  ad_info.latching = data->latching;
  ad_info.data_type = data->dataType;
  ad_info.md5_sum = data->md5Sum;
  ad_info.definition = data->msgDefinition;
  ROS_INFO_STREAM("ad_info latching: " << data->latching);
  ROS_INFO_STREAM("ad_info dataType: " << data->dataType);
  ROS_INFO_STREAM("ad_info md5Sum: " << data->md5Sum);
  ROS_INFO_STREAM("ad_info msgDefinition: " << data->msgDefinition);

  if (!advertiseTopic(output_topic, ad_info)) {
    ROS_INFO("Comms Bridge Nodelet: Received more than one advertisement info message for topic: %s\n",
             output_topic.c_str());
  }
}

// Handle content message
void GenericRapidMsgRosPub::ConvertContent(
                      rapid::ext::astrobee::GenericCommsContent const* data,
                      std::string const& connecting_robot) {
  const std::lock_guard<std::mutex> lock(m_mutex_);

  const std::string output_topic = data->outputTopic;

  ROS_INFO("Comms Bridge Nodelet: Received content message for topic %s\n",
           output_topic.c_str());

  std::map<std::string, RelayTopicInfo>::iterator iter = m_relay_topics_.find(output_topic);
  if (iter == m_relay_topics_.end()) {
    ROS_ERROR("Comms Bridge Nodelet: Received content for topic %s but never received advertisement info.\n",
              output_topic.c_str());

    RelayTopicInfo topic_info;
    topic_info.out_topic = output_topic;
    topic_info.ad_info.md5_sum = data->md5Sum;
    iter = m_relay_topics_.emplace(output_topic, topic_info).first;

    RequestAdvertisementInfo(output_topic, connecting_robot);
  }

  RelayTopicInfo &topic_info = iter->second;

  ContentInfo content_info;
  content_info.type_md5_sum = data->md5Sum;

  unsigned char* buf = data->data.get_contiguous_buffer();
  for (size_t i = 0; i < data->data.length(); i++) {
    content_info.data.push_back(buf[i]);
  }

  ROS_INFO("Comms Bridge Nodelet: Calling relay message for topic %s\n",
           output_topic.c_str());

  if (!relayMessage(topic_info, content_info)) {
    ROS_ERROR("Comms Bridge Nodelet: Error relaying message for topic %s\n",
              output_topic.c_str());
  }
}

void GenericRapidMsgRosPub::RequestAdvertisementInfo(
                                          std::string const& output_topic,
                                          std::string const& connecting_robot) {
  if (enable_advertisement_info_request_) {
    if (dds_initialized_) {
      // Check robot connection exists
      if (robot_connections_.find(connecting_robot) !=
                                                    robot_connections_.end()) {
        ROS_INFO("Sending request for advertisement info for topic %s!\n",
                 output_topic.c_str());
        robot_connections_[connecting_robot]->SendRequest(output_topic);
      } else {
        ROS_ERROR("Comms Bridge: No connection for %s in rapid msg ros pub.\n",
                  connecting_robot.c_str());
      }
    } else {
      ROS_ERROR("Comms Bridge: DDS is not initialized for sending advertisement info request.\n");
    }
  } else {
    ROS_INFO("Comms Bridge: Advertisement info request not enabled.\n");
  }
}

}  // end namespace ff
