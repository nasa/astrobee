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

#include "comms_bridge/bridge_subscriber.h"

// ROS internal subscriber message queue size
#define SUBSCRIBER_QUEUE_SIZE 10
// max serialized message size
#define ROS_BRIDGE_MAX_MSG_SIZE (10*1024*1024)

using ros::Subscriber;
using ros::Publisher;
using topic_tools::ShapeShifter;

BridgeSubscriber::BridgeSubscriber() {
  m_n_relayed_ = 0;
  m_verbose_ = 0;

  m_msgbuffer_ = new uint8_t[ROS_BRIDGE_MAX_MSG_SIZE];
}

BridgeSubscriber::~BridgeSubscriber() { delete[] m_msgbuffer_; }

void BridgeSubscriber::setVerbosity(unsigned int verbosity) { m_verbose_ = verbosity; }

void BridgeSubscriber::handleRelayedMessage(const ros::MessageEvent<ShapeShifter const>& msg_event,
                                               std::string const& topic, SubscriberPtr sub) {
  ShapeShifter::ConstPtr ptr = msg_event.getConstMessage();

  boost::shared_ptr<const ros::M_string> const& connection_header =
    msg_event.getConnectionHeaderPtr();

  if (m_verbose_) {
    ROS_INFO("got data on %s:\n", topic.c_str());
    if (m_verbose_ > 1) {
      ROS_INFO("  datatype: \"%s\"\n", ptr->getDataType().c_str());
      ROS_INFO("  md5: \"%s\"\n", ptr->getMD5Sum().c_str());
    }
    if (m_verbose_ > 2)
      ROS_INFO("  def: \"%s\"\n", ptr->getMessageDefinition().c_str());

    if (m_verbose_ > 2) {
      ROS_INFO("  conn header:\n");
      for (ros::M_string::const_iterator iter = connection_header->begin();
           iter != connection_header->end();
           iter++)
        ROS_INFO("    %s: %s\n", iter->first.c_str(), iter->second.c_str());
    }
  }

  std::unique_lock<std::mutex> lock(m_mutex_);

  std::map<std::string, RelayTopicInfo>::iterator iter = m_relay_topics_.find(topic);
  if (iter == m_relay_topics_.end()) {
    ROS_INFO("Received message on non-relayed topic %s\n", topic.c_str());
    return;
  }

  RelayTopicInfo &topic_info = iter->second;

  if (!topic_info.advertised) {
    if (m_verbose_)
      ROS_INFO("  sending advertisement\n");

    bool latching = false;
    if (connection_header) {
      ros::M_string::const_iterator iter = connection_header->find("latching");
      if (iter != connection_header->end() && iter->second == "1")
        latching = true;
    }

    AdvertisementInfo ad_info;
    ad_info.latching = latching;
    ad_info.data_type = ptr->getDataType();
    ad_info.md5_sum = ptr->getMD5Sum();
    ad_info.definition = ptr->getMessageDefinition();
    topic_info.ad_info = ad_info;

    advertiseTopic(topic_info);
    topic_info.advertised = true;

    // ROS "latches" the msg type md5sum for us, only accepting same in future
    // so no need for us to save and check that it doesn't change
  }

  ros::serialization::OStream stream(m_msgbuffer_, ROS_BRIDGE_MAX_MSG_SIZE);
  stream.next(*ptr);  // serializes just the message contents
  ssize_t serialized_size = ROS_BRIDGE_MAX_MSG_SIZE - stream.getLength();
  if (m_verbose_ > 2)
    ROS_INFO("  serialized size = %zd\n", serialized_size);
  if (serialized_size <= 0) {
    ROS_ERROR("Serialization buffer size deficient, discarding message");
    return;
  }

  ContentInfo content_info;
  content_info.type_md5_sum = ptr->getMD5Sum();
  content_info.data_size = (size_t)serialized_size;
  content_info.data = m_msgbuffer_;
  relayMessage(topic_info, content_info);
  topic_info.relay_seqnum++;
  m_n_relayed_++;

  lock.release()->unlock();
  // now done with any access to topic info

  if (m_verbose_)
    fflush(stdout);
}

// called with mutex held
SubscriberPtr BridgeSubscriber::rosSubscribe(std::string const& topic) {
  ros::NodeHandle nh;

  SubscriberPtr ptr = std::make_shared<Subscriber>();

  ros::SubscribeOptions opts;
  opts.topic = topic;
  opts.queue_size = SUBSCRIBER_QUEUE_SIZE;
  opts.md5sum = ros::message_traits::md5sum<ShapeShifter>();
  opts.datatype = ros::message_traits::datatype<ShapeShifter>();
  opts.helper = boost::make_shared<ros::SubscriptionCallbackHelperT<
    const ros::MessageEvent<ShapeShifter const> &> >(
      std::bind(&BridgeSubscriber::handleRelayedMessage, this, std::placeholders::_1, topic, ptr));
  *ptr = nh.subscribe(opts);

  if (m_verbose_)
    ROS_INFO("Subscribed to topic %s\n", topic.c_str());

  return ptr;
}

bool BridgeSubscriber::addTopic(std::string const& in_topic, std::string const& out_topic) {
  const std::lock_guard<std::mutex> lock(m_mutex_);

  // Enforce that all relays have a unique input topic
  if (m_relay_topics_.find(in_topic) != m_relay_topics_.end()) {
    if (m_verbose_)
      ROS_ERROR("Already subscribed to relay from topic %s\n", in_topic.c_str());
    return false;
  }

  // Enforce that all relays have a unique output topic
  // The republishing side will already be checking for this but may have no way
  // to communicate that to us, so we check too
  std::map<std::string, RelayTopicInfo>::iterator iter = m_relay_topics_.begin();
  while (iter != m_relay_topics_.end()) {
    if (iter->second.out_topic == out_topic) {
      if (m_verbose_)
        ROS_ERROR("Already relaying to topic %s\n", out_topic.c_str());
      return false;
    }
    iter++;
  }

  RelayTopicInfo topic_info;
  topic_info.out_topic = out_topic;

  topic_info.sub = rosSubscribe(in_topic);
  // handleRelayedMessage() may immediately start getting called

  m_relay_topics_[in_topic] = topic_info;

  // let implementation know
  subscribeTopic(in_topic, topic_info);

  return true;
}
