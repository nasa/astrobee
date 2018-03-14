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
 * License for the specific language governiing permissions and limitations
 * under the License.
 */

#include <data_bagger/data_bagger.h>

namespace data_bagger {
DataBagger::DataBagger() :
  ff_util::FreeFlyerNodelet(),
  pub_queue_size_(10),
  startup_time_secs_(20) {
}

DataBagger::~DataBagger() {
}

void DataBagger::Initialize(ros::NodeHandle *nh) {
  config_params_.AddFile("management/data_bagger.config");
  if (!ReadParams()) {
    return;
  }

  // Setup the publishers
  // All states should be latched
  pub_data_state_ = nh->advertise<ff_msgs::DataToDiskState>(
                                            TOPIC_MANAGEMENT_DATA_BAGGER_STATE,
                                            pub_queue_size_,
                                            true);

  pub_data_topics_ = nh->advertise<ff_msgs::DataTopicsList>(
                                            TOPIC_MANAGEMENT_DATA_BAGGER_TOPICS,
                                            pub_queue_size_);

  // Timer used to determine when to query ros for the topic list. Timer is one
  // shot since it is only used at start up and it is started right away
  startup_timer_ = nh->createTimer(ros::Duration(startup_time_secs_),
                                   &DataBagger::OnStartupTimer,
                                   this,
                                   true,
                                   true);
}

bool DataBagger::ReadParams() {
  std::string err_msg;

  // Read config files into lua
  if (!config_params_.ReadFiles()) {
    err_msg = "Data bagger: Unable to read configuration files.";
    NODELET_ERROR("%s", err_msg.c_str());
    // TODO(Katie) assert fault
    return false;
  }

  // Get strtup time. Used to determine when to query ros for topic names
  if (!config_params_.GetUInt("startup_time_secs", &startup_time_secs_)) {
    NODELET_WARN("Unable to read startup time.");
    startup_time_secs_ = 20;
  }

  return true;
}

void DataBagger::OnStartupTimer(ros::TimerEvent const& event) {
  GetTopicNames();
}

void DataBagger::GetTopicNames() {
  ff_msgs::DataTopicsList data_topics_msg;

  ros::master::V_TopicInfo master_topics;
  ros::master::getTopics(master_topics);

  for (ros::master::V_TopicInfo::iterator it = master_topics.begin();
                                              it != master_topics.end(); it++) {
    data_topics_msg.topic_names.push_back(it->name);
  }

  data_topics_msg.header.stamp = ros::Time::now();
  pub_data_topics_.publish(data_topics_msg);
}
}  // namespace data_bagger

PLUGINLIB_EXPORT_CLASS(data_bagger::DataBagger, nodelet::Nodelet)
