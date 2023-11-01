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

#ifndef COMMS_BRIDGE_BRIDGE_SUBSCRIBER_H_
#define COMMS_BRIDGE_BRIDGE_SUBSCRIBER_H_

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
#include <string>

typedef std::shared_ptr<ros::Subscriber> SubscriberPtr;
typedef std::shared_ptr<ros::Publisher> PublisherPtr;

class BridgeSubscriber {
 public:
  BridgeSubscriber();
  virtual ~BridgeSubscriber();

  // Add a relay between this input topic on the subscriber side and this
  //  output topic on the republisher side
  // All relays must have a unique input topic and unique output topic
  // Other topologies (merging topics or duplicating topics) should be
  //  implemented by some other node on top of this one
  bool addTopic(std::string const& in_topic, std::string const& out_topic);

  // 0 => none, > 0 => progressively more
  void setVerbosity(unsigned int verbosity);

 protected:
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

  class RelayTopicInfo {
   public:
    RelayTopicInfo() : out_topic(""), advertised(false), relay_seqnum(0) {}

    SubscriberPtr sub;          // our subscriber on this input topic
    std::string out_topic;      // topic name to republish on
    bool advertised;            // whether we have sent type info to other side
    unsigned int relay_seqnum;  // counter of messages relayed on this topic

    // not filled in until advertiseTopic() is called
    AdvertisementInfo ad_info;
  };

  class ContentInfo {
   public:
    // the md5sum from the original advertisement
    std::string type_md5_sum;

    // length in bytes of the serialized data
    size_t data_size;

    // opaque serialized message data
    const uint8_t* data;
  };

  // Called via addTopic()
  // Notifies implementation that a bridge between these two topics is needed
  // No information about the message type etc. is yet available
  // Called with the mutex held
  virtual void subscribeTopic(std::string const& in_topic, const RelayTopicInfo& info) = 0;

  // Called upon receipt of the first message on a subscribed topic
  // Notifies implementation to advertise the topic with this type info
  // Called with the mutex held
  virtual void advertiseTopic(const RelayTopicInfo& info) = 0;

  // Called on receipt of any message on a subscribed topic
  // Notifies implementation to relay this message content to the output topic
  // Note any pointers within info struct are valid only during this call
  // Called with the mutex held
  virtual void relayMessage(const RelayTopicInfo& topic_info, ContentInfo const& content_info) = 0;

  /**************************************************************************/

  SubscriberPtr rosSubscribe(std::string const& topic);

  void handleRelayedMessage(const ros::MessageEvent<topic_tools::ShapeShifter const>& msg_event,
                            std::string const& topic, SubscriberPtr sub);

  unsigned int m_verbose_;

  std::mutex m_mutex_;                                    // serializes access to below data structures
  uint8_t* m_msgbuffer_;                                  // data serialization scratch
  std::map<std::string, RelayTopicInfo> m_relay_topics_;  // keyed by input topic
  // stats:
  unsigned int m_n_relayed_;
};

#endif  // COMMS_BRIDGE_BRIDGE_SUBSCRIBER_H_
