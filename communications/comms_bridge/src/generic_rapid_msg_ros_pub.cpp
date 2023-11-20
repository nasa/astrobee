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

namespace ff {

GenericRapidMsgRosPub::GenericRapidMsgRosPub(double ad2pub_delay) :
    BridgePublisher(ad2pub_delay) {
}

GenericRapidMsgRosPub::~GenericRapidMsgRosPub() {}

void GenericRapidMsgRosPub::ConvertData(
            rapid::ext::astrobee::GenericCommsAdvertisementInfo const* data) {
  const std::lock_guard<std::mutex> lock(m_mutex_);

  const std::string output_topic = data->outputTopic;

  ROS_DEBUG("Comms Bridge Nodelet: Received advertisement message for topic %s\n",
            output_topic.c_str());

  AdvertisementInfo ad_info;
  ad_info.latching = data->latching;
  ad_info.data_type = data->dataType;
  ad_info.md5_sum = data->md5Sum;
  ad_info.definition = data->msgDefinition;

  if (!advertiseTopic(output_topic, ad_info)) {
    ROS_ERROR("Comms Bridge Nodelet: Error advertising topic: %s\n",
              output_topic.c_str());
  }
}

void GenericRapidMsgRosPub::ConvertData(
                      rapid::ext::astrobee::GenericCommsContent const* data) {
  const std::lock_guard<std::mutex> lock(m_mutex_);

  const std::string output_topic = data->outputTopic;

  ROS_DEBUG("Comms Bridge Nodelet: Received content message for topic %s\n",
            output_topic.c_str());

  std::map<std::string, RelayTopicInfo>::iterator iter = m_relay_topics_.find(output_topic);
  if (iter == m_relay_topics_.end()) {
    ROS_ERROR("Comms Bridge Nodelet: Received content for topic %s but never received advertisement info.\n",
              output_topic);
  } else {
    RelayTopicInfo &topic_info = iter->second;

    ContentInfo content_info;
    content_info.type_md5_sum = data->md5Sum;

    unsigned char* buf = data->data.get_contiguous_buffer();
    for (size_t i = 0; i < data->data.length(); i++) {
      content_info.data.push_back(buf[i]);
    }

    if (!relayMessage(topic_info, content_info)) {
      ROS_ERROR("Comms Bridge Nodelet: Error relaying message for topic %s\n",
                output_topic.c_str());
    }
  }
}

}  // end namespace ff
