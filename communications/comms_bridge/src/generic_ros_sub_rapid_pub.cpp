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

#include "comms_bridge/generic_ros_sub_rapid_pub.h"

#include <map>
#include <string>
#include <utility>
#include <vector>

namespace ff {

GenericROSSubRapidPub::GenericROSSubRapidPub(ros::NodeHandle const* nh) :
    dds_initialized_(false), pub_rate_msgs_(false) {
  // Setup timer for checking when to publish messages that need to be published
  // at a certain rate. The timer will trigger at 10Hz
  rate_msg_pub_timer_ = nh->createTimer(ros::Rate(10.0),
                                       &GenericROSSubRapidPub::CheckPubMsg,
                                       this,
                                       false,
                                       false);
}

GenericROSSubRapidPub::~GenericROSSubRapidPub() {}

void GenericROSSubRapidPub::AddTopics(std::map<std::string,
                std::vector<std::shared_ptr<TopicEntry>>> const& link_entries) {
  std::string in_topic, primary_out_topic;
  // Make sure dds is initialized before adding topics
  if (dds_initialized_) {
    for (auto it = link_entries.begin(); it != link_entries.end(); ++it) {
      in_topic = it->first;
      // Use the first out_topic we read in as the out topic the base class uses
      primary_out_topic = it->second[0]->out_topic_;
      // Save all robot/out topic pairs so that the bridge can pass the correct
      // advertisement info and content message to each roboot that needs it
      topic_mapping_[primary_out_topic] = it->second;

      // Check if there is a valid rate that a message needs to be published at
      for (size_t i = 0; i < it->second.size(); i++) {
        if (it->second[i]->rate_seconds_ > 0) {
          pub_rate_msgs_ = true;
          rate_topic_entries_.push_back(it->second[i]);
          if (FloatMod(it->second[i]->rate_seconds_, 0.1) != 0) {
            ROS_ERROR("Rate for %s is not divisible by 0.1 and will not be published at the desired rate.\n",
                      primary_out_topic.c_str());
          }
        }
      }

      // Add topic to base class
      ROS_INFO("Adding topic %s to base class.", in_topic.c_str());
      addTopic(in_topic, primary_out_topic);
    }
  } else {
    ROS_ERROR("Comms Bridge: Cannot add topics until dds is initialized.\n");
  }

  // If we have messages that get published at a certain rate, start the timer
  // that checks when to publish the rate messages
  if (pub_rate_msgs_) {
    rate_msg_pub_timer_.start();
  }
}

float GenericROSSubRapidPub::FloatMod(float x, float y) {
  int n = static_cast<int>(x/y);
  float result = x - n * y;
  return result;
}

void GenericROSSubRapidPub::InitializeDDS(std::vector<std::string> const& connections) {
  std::string robot_name;
  for (size_t i = 0; i < connections.size(); ++i) {
    robot_name = connections[i];
    GenericRapidPubPtr rapid_pub = std::make_shared<GenericRapidPub>(robot_name);
    robot_connections_[robot_name] = rapid_pub;
  }

  dds_initialized_ = true;
}

// Called with the mutex held
void GenericROSSubRapidPub::subscribeTopic(std::string const& in_topic,
                                           const RelayTopicInfo& info) {
  // this is just the base subscriber letting us know it's adding a topic
  // nothing more we need to do
}

// Called with the mutex held
void GenericROSSubRapidPub::advertiseTopic(const RelayTopicInfo& relay_info) {
  const AdvertisementInfo &info = relay_info.ad_info;
  std::string out_topic = relay_info.out_topic, robot_name, robot_out_topic;

  ROS_INFO("Received ros advertise topic for topic %s\n", out_topic.c_str());

  // Make sure we recognize the topic
  if (topic_mapping_.find(out_topic) == topic_mapping_.end()) {
    ROS_ERROR("Comms Bridge: Output topic %s unknown in advertise topic.\n",
              out_topic.c_str());
    return;
  }

  for (size_t i = 0; i < topic_mapping_[out_topic].size(); ++i) {
    robot_name = topic_mapping_[out_topic][i]->connecting_robot_;
    robot_out_topic = topic_mapping_[out_topic][i]->out_topic_;

    ROS_INFO("Robot name: %s Robot out topic: %s\n",
             robot_name.c_str(),
             robot_out_topic.c_str());

    // Check robot connection exists
    if (robot_connections_.find(robot_name) == robot_connections_.end()) {
      ROS_ERROR("Comms Bridge: No connection for %s.\n", robot_name.c_str());
      continue;
    }

    robot_connections_[robot_name]->SendAdvertisementInfo(robot_out_topic,
                                                          info.latching,
                                                          info.data_type,
                                                          info.md5_sum,
                                                          info.definition);
  }
}

// Called with the mutex held
void GenericROSSubRapidPub::relayMessage(const RelayTopicInfo& topic_info,
                                         ContentInfo const& content_info) {
  std::string out_topic = topic_info.out_topic, robot_name, robot_out_topic;
  unsigned int size;
  ROS_INFO("Received ros content message for topic %s\n", out_topic.c_str());

  // Make sure we recognize the topic
  if (topic_mapping_.find(out_topic) == topic_mapping_.end()) {
    ROS_ERROR("Comms Bridge: Output topic %s unknown in relay message.\n",
              out_topic.c_str());
    return;
  }

  for (size_t i = 0; i < topic_mapping_[out_topic].size(); ++i) {
    robot_name = topic_mapping_[out_topic][i]->connecting_robot_;
    robot_out_topic = topic_mapping_[out_topic][i]->out_topic_;

    ROS_INFO("Robot name: %s Robot out topic: %s\n",
             robot_name.c_str(),
             robot_out_topic.c_str());

    // Check the message is a rate message. If so, don't publish the message.
    // Save the data so it can be sent when it needs to be sent.
    if (topic_mapping_[out_topic][i]->rate_seconds_ > 0) {
      topic_mapping_[out_topic][i]->SetDataToSend(topic_info.relay_seqnum,
                                                  content_info.type_md5_sum,
                                                  content_info.data_size,
                                                  content_info.data);
    } else {
      // Check robot connection exists
      if (robot_connections_.find(robot_name) == robot_connections_.end()) {
        ROS_ERROR("Comms Bridge: No connection for %s.\n", robot_name.c_str());
        continue;
      }

      robot_connections_[robot_name]->SendContent(robot_out_topic,
                                                  content_info.type_md5_sum,
                                                  content_info.data,
                                                  content_info.data_size,
                                                  topic_info.relay_seqnum);
    }
  }
}

void GenericROSSubRapidPub::ConvertRequest(
                        rapid::ext::astrobee::GenericCommsRequest const* data,
                        std::string const& connecting_robot) {
  const std::lock_guard<std::mutex> lock(m_mutex_);

  std::string out_topic, robot_out_topic = data->outputTopic;
  bool found = false;

  // This is the output topic on the robot and may not match the keyed output
  // topic so we need to find the keyed one
  // First check if it is the keyed topic
  auto search = topic_mapping_.find(robot_out_topic);
  if (search != topic_mapping_.end()) {
    out_topic = robot_out_topic;
    found = true;
  } else {
    // If it is not the keyed topic, try to find it.
    for (auto it = topic_mapping_.begin(); it != topic_mapping_.end() && !found; it++) {
      for (size_t i = 0; it->second.size() && !found; i++) {
        if (robot_out_topic == it->second[i]->out_topic_) {
          out_topic = it->first;
          found = true;
        }
      }
    }
  }

  // Make sure we found the keyed topic
  if (!found) {
    ROS_ERROR("Received request for topic %s but it wasn't added to the ros sub rapid pub.\n",
              robot_out_topic.c_str());
    return;
  }

  std::map<std::string, RelayTopicInfo>::iterator iter = m_relay_topics_.begin();
  while (iter != m_relay_topics_.end()) {
    if (iter->second.out_topic == out_topic)
      break;
    iter++;
  }

  if (iter == m_relay_topics_.end()) {
    ROS_ERROR("Received request for topic %s but it wasn't added to the bridge subscriber.\n",
              out_topic.c_str());
    return;
  }

  ROS_INFO("Received reset for topic %s\n", out_topic.c_str());

  // Check robot connection exists
  if (robot_connections_.find(connecting_robot) == robot_connections_.end()) {
    ROS_ERROR("Comms Bridge: No connection for %s\n", connecting_robot.c_str());
    return;
  }

  const AdvertisementInfo &info = iter->second.ad_info;
  robot_connections_[connecting_robot]->SendAdvertisementInfo(robot_out_topic,
                                                              info.latching,
                                                              info.data_type,
                                                              info.md5_sum,
                                                              info.definition);
}

void GenericROSSubRapidPub::CheckPubMsg(const ros::TimerEvent& event) {
  double time_now = ros::Time::now().toSec();
  std::string robot_name, robot_out_topic;
  std::shared_ptr<TopicEntry> topic_entry;
  ROS_ERROR_STREAM("Time now in seconds is: " << time_now);
  // Go through all rate messages and publish them if it is time to
  for (size_t i = 0; i < rate_topic_entries_.size(); i++) {
    topic_entry = rate_topic_entries_[i];
    double time_diff = time_now - topic_entry->last_time_pub_;
    // Check if it is time to publish the rate nmessage.
    // Add a little time buffer since this timer isn't tailored specifically to
    // the message
    if ((time_diff + 0.05) > topic_entry->rate_seconds_) {
      robot_name = topic_entry->connecting_robot_;
      robot_out_topic = topic_entry->out_topic_;

      ROS_ERROR("Rate message for topic %s is being published.",
                robot_out_topic.c_str());

      // Check robot connection exists
      if (robot_connections_.find(robot_name) == robot_connections_.end()) {
        ROS_ERROR("Comms bridge: No connection for %s.\n", robot_name.c_str());
        continue;
      }

      robot_connections_[robot_name]->SendContent(robot_out_topic,
                                                  topic_entry->type_md5_sum_,
                                                  topic_entry->data_,
                                                  topic_entry->data_size_,
                                                  topic_entry->seq_num_);

      // Set time published for message
      topic_entry->last_time_pub_ = time_now;
    }
  }
}

}  // end namespace ff
