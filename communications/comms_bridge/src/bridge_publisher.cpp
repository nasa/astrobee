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

#include "comms_bridge/bridge_publisher.h"

#include <sstream>
#include <future>
#include <string>
#include <map>
#include <utility>

// ROS internal publisher message queue size
#define PUBLISHER_QUEUE_SIZE 10
// max number of messages to store because we're not ready to republish them
#define MAX_WAITING_QUEUE_SIZE 100
// default time to delay between advertising and publishing on that topic [sec]
#define DEFAULT_ADVERTISE_DELAY 3.0

using ros::Publisher;
using ros::Publisher;
using topic_tools::ShapeShifter;

BridgePublisher::BridgePublisher(double ad2pub_delay) : m_ad2pub_delay_(ad2pub_delay) {
  m_n_relayed_ = 0;
  m_verbose_ = 0;

  worker_thread_.reset(new std::thread(&BridgePublisher::drainThread, this));
}

BridgePublisher::~BridgePublisher() {
  worker_thread_->join();
}

void BridgePublisher::setVerbosity(unsigned int verbosity) { m_verbose_ = verbosity; }

bool BridgePublisher::advertiseTopic(const std::string& output_topic, const AdvertisementInfo& ad_info) {
  if (m_verbose_)
    ROS_INFO("Advertising for topic %s\n", output_topic.c_str());

  std::map<std::string, RelayTopicInfo>::iterator iter = m_relay_topics_.find(output_topic);
  if (iter == m_relay_topics_.end()) {
    if (m_verbose_)
      ROS_INFO("  topic is yet-unknown\n");
    RelayTopicInfo topic_info;
    topic_info.out_topic = output_topic;
    iter = m_relay_topics_.emplace(output_topic, topic_info).first;
  }

  RelayTopicInfo &topic_info = iter->second;

  if (topic_info.advertised) {
    // we've already advertised, nothing to do
    if (topic_info.ad_info.md5_sum != ad_info.md5_sum ||
        topic_info.ad_info.data_type != ad_info.data_type ||
        topic_info.ad_info.definition != ad_info.definition)
      ROS_ERROR("Received advertisement with differing type info from previous advertisement, messages will be lost");
    return false;
  } else {
    if (topic_info.ad_info.md5_sum != "" && topic_info.ad_info.md5_sum != ad_info.md5_sum)
      ROS_WARN(
        "Advertisement received for topic %s with differing md5sum from prior content, prior messages will be lost",
        output_topic.c_str());

    // we haven't seen an ad for this topic yet, populate everything
    topic_info.ad_info = ad_info;

    ros::NodeHandle nh;
    ShapeShifter shifter;
    // will always succeed, even if inconsistent type info
    shifter.morph(ad_info.md5_sum, ad_info.data_type, ad_info.definition, "");
    topic_info.publisher = shifter.advertise(nh, output_topic, PUBLISHER_QUEUE_SIZE, ad_info.latching);

    topic_info.advertised = true;

    const timepoint_t now = std::chrono::steady_clock::now();
    topic_info.delay_until = now + std::chrono::microseconds((unsigned)(m_ad2pub_delay_ * 1e6));

    if (m_ad2pub_delay_ > 0) {
      m_drain_queue_.push(std::make_pair(topic_info.delay_until, output_topic));
      m_drain_cv_.notify_all();
    } else {
      // no post-advertise delay

      // send out any old messages first
      drainWaitingQueue(topic_info);
    }
  }

  return true;
}

bool BridgePublisher::relayContent(RelayTopicInfo& topic_info, const ContentInfo& content_info) {
  ShapeShifter shifter;

  // IStreams don't really modify their input but the implementation is
  // lazily general so we have to be ugly
  ros::serialization::IStream stream(const_cast<unsigned char*>(content_info.data.data()), content_info.data.size());
  try {
    stream.next(shifter);
  }
  catch (ros::Exception &e) {
    ROS_ERROR("Deserialization error (%s), losing message", e.what());
    return false;
  }

  // Output error if inconsistent type info
  if (topic_info.ad_info.md5_sum != content_info.type_md5_sum) {
    ROS_ERROR("Content md5 sum doesn't match the advertiment info md5 sum. This could cause issues!");
  }

  // will always succeed, even if inconsistent type info
  shifter.morph(topic_info.ad_info.md5_sum, topic_info.ad_info.data_type, topic_info.ad_info.definition, "");

  try {
    topic_info.publisher.publish(shifter);
  }
  catch (ros::Exception &e) {
    ROS_ERROR("Error publishing morphed message (%s), losing message", e.what());
    return false;
  }

  topic_info.relay_seqnum++;
  m_n_relayed_++;

  if (m_verbose_)
    ROS_INFO("Sent message on topic %s\n", topic_info.out_topic.c_str());

  return true;
}

bool BridgePublisher::relayMessage(RelayTopicInfo& topic_info, const ContentInfo& content_info) {
  if (m_verbose_)
    ROS_INFO("Relaying content message for topic %s\n", topic_info.out_topic.c_str());

  if (!topic_info.advertised) {
    if (m_verbose_)
      ROS_INFO("  topic is yet-unknown, enqueuing message for now\n");

    topic_info.waiting_msgs.emplace(content_info);

    return true;
  }

  if (topic_info.ad_info.md5_sum != content_info.type_md5_sum)
    ROS_WARN(
      "Message received for topic %s with differing md5sum from prior advertiesment, future messages will be lost",
      topic_info.out_topic.c_str());

  const timepoint_t now = std::chrono::steady_clock::now();

  if (!topic_info.advertised || now < topic_info.delay_until) {
    // we still don't have type information for this message, or
    // we're still delaying after advertising
    // so just enqueue messages

    if (m_verbose_)
      ROS_INFO("  topic is not %s, enqueuing message for now\n",
             (!topic_info.advertised ? "yet advertised" : "yet ready"));

    // limit the number of waiting messages, tossing the oldest ones
    while (topic_info.waiting_msgs.size() >= MAX_WAITING_QUEUE_SIZE)
      topic_info.waiting_msgs.pop();

    topic_info.waiting_msgs.emplace(content_info);

    if (m_verbose_ > 1)
      ROS_INFO("  %zu messages now waiting\n", topic_info.waiting_msgs.size());

    return true;
  }

  // send out any old messages first
  drainWaitingQueue(topic_info);

  // send out this message
  return relayContent(topic_info, content_info);
}

void BridgePublisher::drainThread() {
  std::unique_lock<std::mutex> lk(m_mutex_);

  if (m_verbose_)
    ROS_INFO("Started drain thread\n");

  while (1) {
    while (m_drain_queue_.empty())
      m_drain_cv_.wait(lk);

    std::pair<timepoint_t, std::string> entry = m_drain_queue_.top();
    m_drain_queue_.pop();

    lk.unlock();

    timepoint_t t = entry.first;

    if (m_verbose_ > 1)
      ROS_INFO(
        "draining %s in %0.3f seconds\n", entry.second.c_str(),
        std::chrono::duration<double, std::chrono::seconds::period>((t - std::chrono::steady_clock::now())).count());
    std::this_thread::sleep_until(t);

    if (m_verbose_ > 1)
      ROS_INFO("draining %s now\n", entry.second.c_str());

    lk.lock();

    drainWaitingQueue(entry.second);
  }
}

// called with mutex held
void BridgePublisher::drainWaitingQueue(RelayTopicInfo& topic_info) {
  if (m_verbose_ && topic_info.waiting_msgs.size() > 0)
    ROS_INFO("Draining queue of %zu waiting messages\n", topic_info.waiting_msgs.size());
  while (topic_info.waiting_msgs.size() > 0) {
    const ContentInfo old_msg = topic_info.waiting_msgs.front();
    relayContent(topic_info, old_msg);
    topic_info.waiting_msgs.pop();
  }
}

// called with mutex held
void BridgePublisher::drainWaitingQueue(std::string const& output_topic) {
  std::map<std::string, RelayTopicInfo>::iterator iter = m_relay_topics_.find(output_topic);
  assert(iter != m_relay_topics_.end());

  RelayTopicInfo &topic_info = iter->second;

  drainWaitingQueue(topic_info);
}
