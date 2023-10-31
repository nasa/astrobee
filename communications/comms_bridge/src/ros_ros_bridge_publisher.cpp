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

#include <sstream>
#include <string>
#include <map>

#include "comms_bridge/ros_ros_bridge_publisher.h"

// ROS internal subscriber message queue size
#define SUBSCRIBER_QUEUE_SIZE 10
// ROS internal publisher message queue size
#define PUBLISHER_QUEUE_SIZE 10

ROSROSBridgePublisher::ROSROSBridgePublisher(const std::string& meta_topic_prefix, double ad2pub_delay)
    : BridgePublisher(ad2pub_delay), m_meta_topic_prefix(meta_topic_prefix) {
  setupMetaChannels();
  setupReverseMetaChannels();
}

ROSROSBridgePublisher::~ROSROSBridgePublisher() {}

void ROSROSBridgePublisher::setupMetaChannels() {
  ros::NodeHandle nh;

  std::string advertisement_topic = m_meta_topic_prefix + "/advert";
  if (m_verbose > 0)
    printf("advert topic = %s\n", advertisement_topic.c_str());
  m_advert_sub =
    nh.subscribe(advertisement_topic, SUBSCRIBER_QUEUE_SIZE, &ROSROSBridgePublisher::handleAdMessage, this);

  std::string content_topic = m_meta_topic_prefix + "/content";
  if (m_verbose > 0)
    printf("content topic = %s\n", content_topic.c_str());
  m_content_sub =
    nh.subscribe(content_topic, SUBSCRIBER_QUEUE_SIZE, &ROSROSBridgePublisher::handleContentMessage, this);
}

void ROSROSBridgePublisher::setupReverseMetaChannels() {
  ros::NodeHandle nh;

  std::string reset_topic = m_meta_topic_prefix + "/reset";
  m_reset_pub = nh.advertise<ff_msgs::RelayReset>(reset_topic, PUBLISHER_QUEUE_SIZE);
}

void ROSROSBridgePublisher::requestTopicInfo(std::string const& output_topic) {
  ff_msgs::RelayReset reset_msg;

  reset_msg.header.seq = 0;
  reset_msg.header.stamp = ros::Time::now();

  reset_msg.topic = output_topic;

  m_reset_pub.publish(reset_msg);
}

void ROSROSBridgePublisher::handleContentMessage(const ff_msgs::RelayContent::ConstPtr& msg) {
  const std::lock_guard<std::mutex> lock(m_mutex);

  const std::string output_topic = msg->output_topic;

  if (m_verbose)
    printf("Received content message for topic %s\n", output_topic.c_str());

  std::map<std::string, RelayTopicInfo>::iterator iter = m_relay_topics.find(output_topic);
  if (iter == m_relay_topics.end()) {
    // first time seeing this topic
    // The advertisement that preceded this was either lost, delayed out of
    //  order, or we the publisher were restarted after it was sent
    if (m_verbose)
      printf("  first seeing topic\n");

    RelayTopicInfo topic_info;
    topic_info.out_topic = output_topic;
    topic_info.ad_info.md5_sum = msg->type_md5_sum;
    iter = m_relay_topics.emplace(output_topic, topic_info).first;

    requestTopicInfo(output_topic);
  }

  RelayTopicInfo &topic_info = iter->second;

  ContentInfo content_info;
  content_info.type_md5_sum = msg->type_md5_sum;
  content_info.data = msg->msg;

  if (!relayMessage(topic_info, content_info))
    printf("  error relaying message\n");
}

void ROSROSBridgePublisher::handleAdMessage(const ff_msgs::RelayAdvertisement::ConstPtr& msg) {
  const std::lock_guard<std::mutex> lock(m_mutex);

  const std::string output_topic = msg->output_topic;

  if (m_verbose)
    printf("Received advertisement message for topic %s\n", output_topic.c_str());

  AdvertisementInfo ad_info;
  ad_info.md5_sum = msg->md5_sum;
  ad_info.data_type = msg->data_type;
  ad_info.definition = msg->definition;

  if (!advertiseTopic(msg->output_topic, ad_info))
    printf("  error advertising topic\n");
}
