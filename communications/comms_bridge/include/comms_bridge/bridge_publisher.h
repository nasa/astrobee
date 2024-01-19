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

#ifndef COMMS_BRIDGE_BRIDGE_PUBLISHER_H_
#define COMMS_BRIDGE_BRIDGE_PUBLISHER_H_

/*
 Virtual base class for the subscriber (input side) of a generic ROS bridge
 Actual implementation depends on an inheriting class
 */

#include <stdint.h>

#include <ros/ros.h>
#include <topic_tools/shape_shifter.h>

#include <mutex>
#include <memory>
#include <map>
#include <queue>
#include <chrono>
#include <thread>
#include <condition_variable>
#include <string>
#include <vector>
#include <utility>

typedef std::shared_ptr<ros::Subscriber> SubscriberPtr;
typedef std::shared_ptr<ros::Publisher> PublisherPtr;
typedef std::chrono::time_point<std::chrono::steady_clock> timepoint_t;

class BridgePublisher {
 public:
  virtual ~BridgePublisher();

  // 0 => none, > 0 => progressively more
  void setVerbosity(unsigned int verbosity);

 protected:
  // a yet-unknown message type will have null string values
  class AdvertisementInfo {
   public:
    // whehter republisher should advertise this as a latching ROS topic
    bool latching;

    // ROS message data type name, eg "std_msgs/Int32"
    std::string data_type;

    // ROS message md5sum of type, eg "da5909fbe378aeaf85e547e830cc1bb7"
    std::string md5_sum;

    // ROS message definition, eg as one finds in a .msg definition file
    std::string definition;
  };

  class ContentInfo {
   public:
    // the md5sum from the original advertisement
    std::string type_md5_sum;

    // opaque serialized message data
    std::vector<uint8_t> data;
  };

  class RelayTopicInfo {
   public:
    RelayTopicInfo() : out_topic(""), advertised(false), relay_seqnum(0) {}

    ros::Publisher publisher;   // our subscriber on this input topic
    std::string out_topic;      // topic name to republish on
    bool advertised;            // whether we have sent type info to other side
    unsigned int relay_seqnum;  // counter of messages relayed on this topic

    // Will be filled in only on receipt of advertisement from our
    // bridger subscriber counterpart
    AdvertisementInfo ad_info;

    std::queue<ContentInfo>
      waiting_msgs;           // unpublished messages because type yet unknown or we're still delaying after advertising
    timepoint_t delay_until;  // time to wait until relaying any data, after having advertised
  };

  /**************************************************************************/

  // This base class should only be extended, not itself instantiated
  explicit BridgePublisher(double ad2pub_delay);

  // Should be called by implementing class when it has received a topic's
  //  AdvertisementInfo
  // Must be called with mutex held
  bool advertiseTopic(const std::string& output_topic, const AdvertisementInfo& ad_info);

  // Should be called by an implementing class when it receives a serialized
  //  message over its particular conduit
  // Must be called with mutex held
  bool relayMessage(RelayTopicInfo& topic_info, const ContentInfo& content_info);

  // worker thread spawned at construction
  void drainThread();

  // does actual work of publishing a message (eg called by relayMessage)
  // Must be called with mutex held
  bool relayContent(RelayTopicInfo& topic_info, const ContentInfo& content_info);

  // Publishes messages that were enqueued for a given topic
  // Must be called with mutex held
  void drainWaitingQueue(RelayTopicInfo& topic_info);
  void drainWaitingQueue(std::string const& output_topic);

  unsigned int m_verbose_;

  double m_ad2pub_delay_;  // seconds

  std::mutex m_mutex_;  // serializes access to below data structures
  std::shared_ptr<std::thread> worker_thread_;
  std::map<std::string, RelayTopicInfo> m_relay_topics_;  // keyed by input topic
  std::priority_queue<std::pair<timepoint_t, std::string>> m_drain_queue_;
  std::condition_variable m_drain_cv_;
  // stats:
  unsigned int m_n_relayed_;
};

#endif  // COMMS_BRIDGE_BRIDGE_PUBLISHER_H_
