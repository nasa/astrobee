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

#include <data_bagger/astrobee_recorder.h>
#include <data_bagger/data_bagger.h>

#include <thread>

namespace data_bagger {

DataBagger::DataBagger() :
  ff_util::FreeFlyerNodelet(),
  delayed_recorder_(nullptr),
  immediate_recorder_(nullptr),
  recording_delayed_bag_(false),
  recording_immediate_bag_(false),
  pub_queue_size_(10),
  startup_time_secs_(20) {
}

DataBagger::~DataBagger() {
  ResetRecorders();
}

void DataBagger::Initialize(ros::NodeHandle *nh) {
  std::string err_msg = "";

  config_params_.AddFile("management/data_bagger.config");
  if (!ReadParams()) {
    return;
  }

  if (!MakeDataDirs()) {
    return;
  }

  // Setup the publishers
  // All states should be latched
  pub_data_state_ = nh->advertise<ff_msgs::DataToDiskState>(
                                            TOPIC_MANAGEMENT_DATA_BAGGER_STATE,
                                            pub_queue_size_,
                                            true);

  // Check to see if there were default topics to start recording. Have to do
  // this after the state publisher is initialized.
  if (default_data_state_.topic_save_settings.size() > 0) {
    if (!SetDataToDisk(default_data_state_, err_msg)) {
      this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
      return;
    }
  }

  pub_data_topics_ = nh->advertise<ff_msgs::DataTopicsList>(
                                            TOPIC_MANAGEMENT_DATA_BAGGER_TOPICS,
                                            pub_queue_size_,
                                            true);

  service_ = nh->advertiseService(SERVICE_MANAGEMENT_DATA_BAGGER_SET_DATA_TO_DISK,
                                   &DataBagger::SetDataToDiskService,
                                   this);


  // Timer used to determine when to query ros for the topic list. Timer is one
  // shot since it is only used at start up and it is started right away
  startup_timer_ = nh->createTimer(ros::Duration(startup_time_secs_),
                                   &DataBagger::OnStartupTimer,
                                   this,
                                   true);
}

bool DataBagger::ReadParams() {
  // Read config files into lua
  if (!config_params_.ReadFiles()) {
    this->AssertFault(ff_util::INITIALIZATION_FAILED,
                      "Unable to read configuration files.");
    return false;
  }

  // Get strtup time. Used to determine when to query ros for topic names
  if (!config_params_.GetUInt("startup_time_secs", &startup_time_secs_)) {
    NODELET_WARN("Unable to read startup time.");
    startup_time_secs_ = 20;
  }

  if (!config_params_.GetStr("bags_save_directory", &save_dir_)) {
    this->AssertFault(ff_util::INITIALIZATION_FAILED,
                      "Unable to read save directory from config.");
    return false;
  }

  // Force directory
  if (save_dir_.back() != '/') {
    save_dir_ = save_dir_.append("/");
  }

  if (config_params_.CheckValExists("default_topics")) {
    config_reader::ConfigReader::Table topics_table(&config_params_,
                                                    "default_topics");
    default_data_state_.name = "ars_default";
    std::string downlink;
    ff_msgs::SaveSettings save_settings;
    // Lua indices start at one
    for (int i = 1; i < (topics_table.GetSize() + 1); i++) {
      config_reader::ConfigReader::Table topic_entry(&topics_table, i);
      if (!topic_entry.GetStr("topic", &save_settings.topic_name)) {
        this->AssertFault(ff_util::INITIALIZATION_FAILED,
                          "Unable to read topic for default topics table.");
        return false;
      }

      if (!topic_entry.GetStr("downlink", &downlink)) {
        this->AssertFault(ff_util::INITIALIZATION_FAILED,
                          "Unable to read downlink for default topics table.");
        return false;
      }

      if (downlink == "immediate") {
        save_settings.downlinkOption = ff_msgs::SaveSettings::IMMEDIATE;
      } else if (downlink == "delayed") {
        save_settings.downlinkOption = ff_msgs::SaveSettings::DELAYED;
      } else {
        this->AssertFault(ff_util::INITIALIZATION_FAILED,
                          "Downlink option invalid for default topics table.");
        return false;
      }

      if (!topic_entry.GetReal("frequency", &save_settings.frequency)) {
        this->AssertFault(ff_util::INITIALIZATION_FAILED,
                          "Unable to read frequency for default topics table.");
        return false;
      }

      default_data_state_.topic_save_settings.push_back(save_settings);
    }
  } else {
    NODELET_WARN("Default topics table doesn't exist so no bag started.");
  }

  return true;
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

bool DataBagger::MakeDataDirs() {
  std::string err_msg;

  if (!MakeDir(save_dir_, true, err_msg)) {
    return false;
  }

  if (!MakeDir((save_dir_ + "immediate/"), true, err_msg)) {
    return false;
  }

  if (!MakeDir((save_dir_ + "delayed/"), true, err_msg)) {
    return false;
  }

  return true;
}

bool DataBagger::MakeDir(std::string dir,
                         bool assert_init_fault,
                         std::string &err_msg) {
  struct stat dir_info;

  // Check if directory exists and if it doesn't, create it
  if (stat(dir.c_str(), &dir_info) != 0) {
    if (mkdir(dir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) == -1) {
      err_msg = dir + " does not exist and creating it returned the error: ";
      err_msg += strerror(errno);
      // The text below is to make this error message less confusing.
      err_msg += ". Check data_bagger.config.";

      if (assert_init_fault) {
        this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
      }
      return false;
    }
  }

  return true;
}

void DataBagger::OnStartupTimer(ros::TimerEvent const& event) {
  GetTopicNames();
}

bool DataBagger::SetDataToDiskService(ff_msgs::SetDataToDisk::Request &req,
                                      ff_msgs::SetDataToDisk::Response &res) {
  if (!SetDataToDisk(req.state, res.status)) {
    res.success = false;
    return true;
  }

  res.success = true;
  return true;
}

bool DataBagger::SetDataToDisk(ff_msgs::DataToDiskState &state,
                               std::string &err_msg) {
  ResetRecorders();

  for (auto & setting : state.topic_save_settings) {
    if (setting.downlinkOption == setting.DELAYED) {
      recorder_options_delayed_.topics.push_back(setting.topic_name);
      recording_delayed_bag_ = true;
    } else if (setting.downlinkOption == setting.IMMEDIATE) {
      recorder_options_immediate_.topics.push_back(setting.topic_name);
      recording_immediate_bag_ = true;
    }

    // TODO(Someone) Need to figure out how to record topics at different
    // frequencies. For now, report error if frequency is valid to let the
    // operator know that this functionality isn't implemented yet
    if (setting.frequency != -1) {
      err_msg = "Frequency for every topic must be -1. Different frequencies ";
      err_msg = "for each topic is not supported yet!";
      return false;
    }
  }

  // Create dated folder for data
  boost::posix_time::ptime posix_time = ros::WallTime::now().toBoost();
  std::string time_str = boost::posix_time::to_iso_extended_string(posix_time);

  // Time string format is YYYY-MM-DDThh::mm::ss. Need to extract date.
  std::size_t t_pos = time_str.find("T");
  std::string date_str = time_str.substr(0, t_pos);

  std::string dated_delayed_dir = save_dir_ + "delayed/" + date_str + "/";
  std::string dated_immediate_dir = save_dir_ + "immediate/" + date_str + "/";

  if (!MakeDir(dated_delayed_dir, false, err_msg)) {
    return false;
  }

  if (!MakeDir(dated_immediate_dir, false, err_msg)) {
    return false;
  }

  // Publish the data to disk state so that the bridge can send it to the
  // ground
  state.header.stamp = ros::Time::now();
  pub_data_state_.publish(state);

  recorder_options_delayed_.prefix = dated_delayed_dir + state.name;
  recorder_options_immediate_.prefix = dated_immediate_dir + state.name;

  if (recording_delayed_bag_) {
    delayed_thread_ = std::thread(&DataBagger::StartDelayedRecording, this);
  }

  if (recording_immediate_bag_) {
    immediate_thread_ = std::thread(&DataBagger::StartImmediateRecording, this);
  }

  return true;
}

void DataBagger::StartDelayedRecording() {
  delayed_recorder_ = new astrobee_rosbag::Recorder(recorder_options_delayed_);
  delayed_recorder_->run();
}

void DataBagger::StartImmediateRecording() {
  immediate_recorder_ = new astrobee_rosbag::Recorder(recorder_options_immediate_);
  immediate_recorder_->run();
}

// clear topics, stop recording, detach recording threads
void DataBagger::ResetRecorders() {
  recorder_options_delayed_.topics.clear();
  recorder_options_immediate_.topics.clear();
  recording_delayed_bag_ = false;
  recording_immediate_bag_ = false;

  if (delayed_recorder_ != nullptr) {
    delayed_recorder_->stop();
    delayed_thread_.join();
    delete delayed_recorder_;
    delayed_recorder_ = nullptr;
  }
  if (immediate_recorder_ != nullptr) {
    immediate_recorder_->stop();
    immediate_thread_.join();
    delete immediate_recorder_;
    immediate_recorder_ = nullptr;
  }
}
}  // namespace data_bagger

PLUGINLIB_EXPORT_CLASS(data_bagger::DataBagger, nodelet::Nodelet)
