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
#include <map>

#include "comms_bridge/ros_ros_bridge_subscriber.h"

// ROS internal subscriber message queue size
#define SUBSCRIBER_QUEUE_SIZE 10
// ROS internal publisher message queue size
#define PUBLISHER_QUEUE_SIZE 10

ROSROSBridgeSubscriber::ROSROSBridgeSubscriber(const std::string& meta_topic_prefix)
    : m_meta_topic_prefix(meta_topic_prefix) {
  m_n_advertised = 0;

  setupMetaChannels();
  setupReverseMetaChannels();
}

ROSROSBridgeSubscriber::~ROSROSBridgeSubscriber() {}

void ROSROSBridgeSubscriber::setupMetaChannels() {
  ros::NodeHandle nh;

  std::string advertisement_topic = m_meta_topic_prefix + "/advert";
  m_advertiser_pub = nh.advertise<ros_bridge::RelayAdvertisement>(advertisement_topic, PUBLISHER_QUEUE_SIZE);

  std::string content_topic = m_meta_topic_prefix + "/content";
  m_relayer_pub = nh.advertise<ros_bridge::RelayContent>(content_topic, PUBLISHER_QUEUE_SIZE);
}

void ROSROSBridgeSubscriber::setupReverseMetaChannels() {
  ros::NodeHandle nh;

  std::string reset_topic = m_meta_topic_prefix + "/reset";
  m_reset_sub = nh.subscribe(reset_topic, SUBSCRIBER_QUEUE_SIZE, &ROSROSBridgeSubscriber::handleResetMessage, this);
}

void ROSROSBridgeSubscriber::handleResetMessage(const ros_bridge::RelayReset::ConstPtr& msg) {
  const std::string out_topic = msg->topic;

  const std::lock_guard<std::mutex> lock(m_mutex);

  std::map<std::string, RelayTopicInfo>::iterator iter = m_relay_topics.begin();
  while (iter != m_relay_topics.end()) {
    if (iter->second.out_topic == out_topic)
      break;
    iter++;
  }

  if (iter == m_relay_topics.end()) {
    if (m_verbose)
      printf("Received reset on unknown topic %s\n", out_topic.c_str());
    return;
  }

  if (m_verbose)
    printf("Received reset for topic %s\n", out_topic.c_str());

  requested_resets[out_topic] = true;
}

void ROSROSBridgeSubscriber::subscribeTopic(std::string const& in_topic, const RelayTopicInfo& info) {
  // this is just the base subscriber letting us know it's adding a topic
  // nothing more we need to do
}

void ROSROSBridgeSubscriber::advertiseTopic(const RelayTopicInfo& info) {
  const AdvertisementInfo &ad_info = info.ad_info;

  ros_bridge::RelayAdvertisement ad;

  ad.header.seq = ++m_n_advertised;
  ad.header.stamp = ros::Time::now();

  ad.output_topic = info.out_topic;

  ad.latching = ad_info.latching;

  ad.data_type = ad_info.data_type;
  ad.md5_sum = ad_info.md5_sum;
  ad.definition = ad_info.definition;

  m_advertiser_pub.publish(ad);
}

void ROSROSBridgeSubscriber::relayMessage(const RelayTopicInfo& topic_info, ContentInfo const& content_info) {
  std::map<std::string, bool>::iterator iter = requested_resets.find(topic_info.out_topic);
  if (iter != requested_resets.end()) {
    advertiseTopic(topic_info);
    requested_resets.erase(iter);
  }

  ros_bridge::RelayContent content;

  content.header.seq = topic_info.relay_seqnum;
  content.header.stamp = ros::Time::now();

  content.output_topic = topic_info.out_topic;

  content.type_md5_sum = content_info.type_md5_sum;

  content.msg.resize(content_info.data_size);
  std::memcpy(static_cast<void*>(content.msg.data()), static_cast<const void*>(content_info.data), content.msg.size());

  m_relayer_pub.publish(content);
}
