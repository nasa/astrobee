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

#include "dds_ros_bridge/ros_data_to_disk.h"

namespace {

#define GENERATE_DATA_TO_DISK_CASE(NAME) \
  case ff_msgs::SaveSettings::NAME: return rapid::ext::astrobee::DATA_##NAME

#define GENERATE_DATA_TO_DISK_DEFAULT(NAME) \
  default: \
    ROS_FATAL("DDS: Unknown data to disk downlink option: %d", state); \
    return rapid::ext::astrobee::DATA_##NAME

rapid::ext::astrobee::DownlinkOption ConvertOption(uint8_t state) {
  switch (state) {
    GENERATE_DATA_TO_DISK_CASE(IMMEDIATE);
    GENERATE_DATA_TO_DISK_CASE(DELAYED);
    GENERATE_DATA_TO_DISK_DEFAULT(IMMEDIATE);
  }
}

}  // end namespace

ff::RosDataToDiskToRapid::RosDataToDiskToRapid(
                                      const std::string& state_subscribe_topic,
                                      const std::string& topics_subscribe_topic,
                                      const std::string& pub_topic,
                                      const ros::NodeHandle& nh,
                                      const unsigned int queue_size) :
    RosSubRapidPub(state_subscribe_topic, pub_topic, nh, queue_size),
    state_subscribe_topic_(state_subscribe_topic),
    topics_subscribe_topic_(topics_subscribe_topic) {
  state_supplier_.reset(new ff::RosDataToDiskToRapid::StateSupplier(
      rapid::ext::astrobee::DATA_TO_DISK_STATE_TOPIC + pub_topic,
      "",
      "AstrobeeDataToDiskStateProfile",
      ""));

  topics_supplier_.reset(new ff::RosDataToDiskToRapid::TopicsSupplier(
      rapid::ext::astrobee::DATA_TOPICS_LIST_TOPIC + pub_topic,
      "",
      "AstrobeeDataTopicsListProfile",
      ""));

    sub_ = nh_.subscribe(state_subscribe_topic,
                         queue_size,
                         &RosDataToDiskToRapid::StateCallback,
                         this);

    topics_sub_ = nh_.subscribe(topics_subscribe_topic,
                                queue_size,
                                &RosDataToDiskToRapid::TopicsCallback,
                                this);

  rapid::RapidHelper::initHeader(state_supplier_->event().hdr);
  rapid::RapidHelper::initHeader(topics_supplier_->event().hdr);
}

void ff::RosDataToDiskToRapid::StateCallback(
                                ff_msgs::DataToDiskStateConstPtr const& state) {
  rapid::ext::astrobee::DataToDiskState &msg = state_supplier_->event();
  msg.hdr.timeStamp = util::RosTime2RapidTime(state->header.stamp);

  // Currently the data to disk name can only be 32 characters long
  if (state->name.size() > 32) {
    ROS_ERROR("DDS: Data to disk name %s is longer than 32 characters!",
              state->name.c_str());
  }
  std::strncpy(msg.name, state->name.data(), 32);
  msg.name[31] = '\0';

  // Copy over the topic save settings
  // Currently the code only supports 64 topics
  unsigned int size = state->topic_save_settings.size();
  if (size > 64) {
    ROS_WARN("DDS: There are %i topics but only 64 can be sent to the ground.",
              size);
    size = 64;
  }

  msg.topicSaveSettings.length(size);
  for (unsigned int i = 0; i < size; i++) {
    // Currently the topic name can only be 128 characters long
    if (state->topic_save_settings[i].topic_name.size() > 128) {
      ROS_ERROR("DDS: The topic named %s is longer than 128 characters!",
                state->topic_save_settings[i].topic_name.c_str());
    }
    std::strncpy(msg.topicSaveSettings[i].topicName,
                 state->topic_save_settings[i].topic_name.data(),
                 128);
    msg.topicSaveSettings[i].topicName[127] = '\0';

    msg.topicSaveSettings[i].downlinkOption =
                    ConvertOption(state->topic_save_settings[i].downlinkOption);

    msg.topicSaveSettings[i].frequency =
                                        state->topic_save_settings[i].frequency;
  }

  state_supplier_->sendEvent();
}

void ff::RosDataToDiskToRapid::TopicsCallback(
                                ff_msgs::DataTopicsListConstPtr const& topics) {
  rapid::ext::astrobee::DataTopicsList &msg = topics_supplier_->event();
  msg.hdr.timeStamp = util::RosTime2RapidTime(topics->header.stamp);

  // Currently the code only supports sending 256 topics
  unsigned int size = topics->topic_names.size();
  if (size > 256) {
    ROS_WARN("DDS: There are %i topics but only 256 can be sent to the ground",
              size);
    size = 256;
  }

  msg.topics.length(size);
  for (unsigned int i = 0; i < size; i++) {
    // Currently the topic name can only be 128 characters long
    if (topics->topic_names[i].size() > 128) {
      ROS_ERROR("DDS: The topic named %s is longer than 128 characters!",
                topics->topic_names[i].c_str());
    }
    std::strncpy(msg.topics[i], topics->topic_names[i].data(), 128);
    msg.topics[i][127] = '\0';
  }

  topics_supplier_->sendEvent();
}
