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
  ff_util::FreeFlyerNodelet(NODE_DATA_BAGGER, true),
  delayed_recorder_(nullptr),
  immediate_recorder_(nullptr),
  pub_queue_size_(10),
  startup_time_secs_(20),
  bag_size_bytes_(96000000),
  delayed_profile_name_("") {
}

DataBagger::~DataBagger() {
  ResetRecorders(true);
  ResetRecorders(false);
}

// Service that defines delayed recording settings
bool DataBagger::SetDelayedDataToDiskService(ff_msgs::SetDataToDisk::Request &req,
                                             ff_msgs::SetDataToDisk::Response &res) {
  // Don't allow set data to disk when we are recording
  if (combined_data_state_.recording) {
    res.status = "Can't set data to disk while recording data. Please stop ";
    res.status += "recording and try again.";
    res.success = false;
    return true;
  }

  // Clear delayed topics if we get new delayed topics to record
  recorder_options_delayed_.topics.clear();

  // Also clear current profile name
  delayed_profile_name_ = "";

  // Check for empty topic size
  if (req.state.topic_save_settings.size() == 0) {
    GenerateCombinedState(NULL);
    PublishState();
    res.success = true;
    return true;
  }

  for (auto & setting : req.state.topic_save_settings) {
    // Check to see if a ground user is trying to bag an immediate topic. This
    // is currently not allowed. Only internal fsw data topics can be immediate
    if (setting.downlinkOption == setting.IMMEDIATE) {
      res.status = "Please don't try to record immediate data. Immediate ";
      res.status += "data is for internal fsw only.";
      res.success = false;
      return true;
    }

    AddTopicNamespace(setting.topic_name);
    recorder_options_delayed_.topics.push_back(setting.topic_name);

    // TODO(Someone) Need to figure out how to record topics at different
    // frequencies. For now, report error if frequency is valid to let the
    // operator know that this functionality isn't implemented yet
    if (setting.frequency != -1) {
      res.status = "Frequency for every topic must be -1. Different ";
      res.status += "frequencies for each topic is not suported yet!";
      res.success = false;
      return true;
    }
  }

  delayed_profile_name_ = req.state.name;

  GenerateCombinedState(&req.state);
  PublishState();

  res.success = true;
  return true;
}

// This service enables and disables the delayed recording
bool DataBagger::EnableDelayedRecordingService(ff_msgs::EnableRecording::Request &req,
                                               ff_msgs::EnableRecording::Response &res) {
  // Check if we are starting a recording or stopping a recording
  if (req.enable) {
    // Check to make sure a delayed profile is loaded
    if (delayed_profile_name_ == "" ||
                                recorder_options_delayed_.topics.size() == 0) {
      res.status = "Delayed profile not uploaded or no topics in last ";
      res.status += "uploaded delayed profile. Please upload a valid profile";
      res.status += " before recording.";
      res.success = false;
      return true;
    }

    std::string dated_dir = save_dir_ + GetDate(false) + "/" + robot_name_ +
                                                                    "/delayed/";
    if (!MakeDir(dated_dir, false, res.status)) {
      res.success = false;
      return true;
    }

    if (req.bag_description == "") {
      recorder_options_delayed_.prefix = dated_dir + GetDate(true) + "_" +
                                         delayed_profile_name_;
    } else {
      recorder_options_delayed_.prefix = dated_dir + GetDate(true) + "_" +
                              delayed_profile_name_ + "_" + req.bag_description;
    }

    recorder_options_delayed_.split = true;
    recorder_options_delayed_.max_size = bag_size_bytes_;
    recorder_options_delayed_.append_date = false;

    StartDelayedRecording();

    combined_data_state_.recording = true;

    PublishState();
  } else {
    // Send false to reset the delayed recorder
    ResetRecorders(false);
    combined_data_state_.recording = false;
    PublishState();
  }

  res.success = true;
  return true;
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

  // Publish empty state
  PublishState();

  pub_data_topics_ = nh->advertise<ff_msgs::DataTopicsList>(
                                            TOPIC_MANAGEMENT_DATA_BAGGER_TOPICS,
                                            pub_queue_size_,
                                            true);

  set_service_ =
          nh->advertiseService(SERVICE_MANAGEMENT_DATA_BAGGER_SET_DATA_TO_DISK,
                               &DataBagger::SetDelayedDataToDiskService,
                               this);

  record_service_ =
          nh->advertiseService(SERVICE_MANAGEMENT_DATA_BAGGER_ENABLE_RECORDING,
                               &DataBagger::EnableDelayedRecordingService,
                               this);

  // Timer used to determine when to query ros for the topic list. Timer is one
  // shot since it is only used at start up and it is started right away
  startup_timer_ = nh->createTimer(ros::Duration(startup_time_secs_),
                                   &DataBagger::OnStartupTimer,
                                   this,
                                   true);
}

bool DataBagger::ReadParams() {
      ROS_ERROR_STREAM("ReadParams");
  // Read config files into lua
  if (!config_params_.ReadFiles()) {
    this->AssertFault(ff_util::INITIALIZATION_FAILED,
                      "Unable to read configuration files.");
    return false;
  }

  // Get robot name for directory name
  if (!config_params_.GetStr("robot_name", &robot_name_)) {
    this->AssertFault(ff_util::INITIALIZATION_FAILED,
                      "Unable to read robot name.");
    return false;
  }

  // Get startup time. Used to determine when to query ros for topic names
  if (!config_params_.GetUInt("startup_time_secs", &startup_time_secs_)) {
    NODELET_WARN("Unable to read startup time.");
    startup_time_secs_ = 20;
  }

  // Get max bag size in bytes.
  if (!config_params_.GetLongLong("bag_size_bytes", &bag_size_bytes_)) {
    NODELET_WARN("Unable to read bag size bytes. Setting to 96 MB.");
    bag_size_bytes_ = 96000000;
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
      ROS_ERROR_STREAM("default_topics ");
    // If there isn't anything in the table, don't set the profile name
    if (topics_table.GetSize() > 0) {
      default_data_state_.name = "ars_default";
    }

    std::string downlink;
    ff_msgs::SaveSettings save_settings;
    // Lua indices start at one
      ROS_ERROR_STREAM("default_topics " << topics_table.GetSize());
    for (int i = 1; i < (topics_table.GetSize() + 1); i++) {
      ROS_ERROR_STREAM("topics_table ");

      config_reader::ConfigReader::Table topic_entry(&topics_table, i);
      if (!topic_entry.GetStr("topic", &save_settings.topic_name)) {
        this->AssertFault(ff_util::INITIALIZATION_FAILED,
                          "Unable to read topic for default topics table.");
        return false;
      }

      ROS_ERROR_STREAM("topic immediate " << save_settings.topic_name);
      AddTopicNamespace(save_settings.topic_name);

      if (!topic_entry.GetStr("downlink", &downlink)) {
        this->AssertFault(ff_util::INITIALIZATION_FAILED,
                          "Unable to read downlink for default topics table.");
        return false;
      }

      if (downlink == "immediate" || downlink == "Immediate") {
        save_settings.downlinkOption = ff_msgs::SaveSettings::IMMEDIATE;
      } else {
        this->AssertFault(ff_util::INITIALIZATION_FAILED,
                          "Downlink option invalid! Must be immediate!");
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

// Reads all topic names published by the robot
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

// Recursively created bag destionation folder
bool DataBagger::MakeDir(std::string dir,
                         bool assert_init_fault,
                         std::string &err_msg) {
  ROS_ERROR_STREAM("MakeDir" << dir);
  struct stat dir_info;

  // Check if directory exists and if it doesn't, create it
  if (stat(dir.c_str(), &dir_info) != 0) {
    // Make dir doesn't make the parent so we have to do it ourselves
    std::string running_dir = "";
    char *current_dir, *saveptr;
    char *char_dir = new char[dir.length()+1];
    // Copy string to character array so we can manipulate it
    strncpy(char_dir, dir.c_str(), dir.length());
    char_dir[dir.length()] = '\0';

    // Split string
    current_dir = strtok_r(char_dir, "/", &saveptr);
    while (current_dir != NULL) {
      // Convert name to string
      running_dir += "/" + std::string(current_dir);
      // Check if directory exists and if it doesn't, create it
      if (stat(running_dir.c_str(), &dir_info) != 0) {
        if (mkdir(running_dir.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) == -1) {
          err_msg = running_dir;
          err_msg += " does not exist and creating it returned the error ";
          err_msg += strerror(errno);
          // The text below is to make this error message less confusing.
          err_msg += ". Check data_bagger.config.";

          if (assert_init_fault) {
            this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
          }

          // Clean up memory
          if (char_dir != NULL) {
            delete [] char_dir;
          }
          return false;
        }
      }
      current_dir = strtok_r(NULL, "/", &saveptr);
    }

    // Clean up memory
    if (char_dir != NULL) {
      delete [] char_dir;
    }
  }

  return true;
}

// This function returns the date in a string. If the argument is true, the
// string will contain the time as well.
// This is used to set the name of the bag files.
std::string DataBagger::GetDate(bool with_time) {
  std::string time_str;
  time_t rawtime;
  struct tm * time_info = new struct tm;
  char cur_time[100];

  time(&rawtime);
  time_info = localtime_r(&rawtime, time_info);

  if (time_info == NULL) {
    ROS_ERROR_STREAM("Unable to get local time. Errno is " <<
                                                          std::strerror(errno));
    return "invalid_time";
  }

  if (with_time) {
    strftime(cur_time, 100, "%Y%m%d_%H%M", time_info);
  } else {
    strftime(cur_time, 100, "%Y-%m-%d", time_info);
  }

  time_str = cur_time;

  // time_info shouldn't be null, but let's check for kicks and giggles
  if (time_info != NULL) {
    delete time_info;
  }
  return time_str;
}


// ROS namespacing sucks! In simulation, we use namespaces. So the data bagger
// has to be robust to namespacing. The weird thing about the ros bagger that we
// are using, is if you specify a relate topic, it will keep that relative name
// in the bag. For instance, let's say we want to record the gnc ekf topic. If
// we give the ros bagger the topic gnc/ekf and there is no namespace, it will
// subscribe to topic /gnc/ekf but the topic in the bag it records will be
// gnc/ekf. This breaks our tools so we must fix it in the code. Also if we
// give the ros bagger the topic gnc/ekf and there is a namespace of bumble, it
// will subscribe to /bumble/gnc/ekf but the topic in the bag it records will be
// gnc/ekf.
void DataBagger::AddTopicNamespace(std::string &topic) {
  // This function assumes a user doesn't put the namespace in the topic. If
  // they do, this won't work
  // Check topic not empty
  if (topic.size() == 0) {
    return;
  }

  // Make sure there is a leading slash
  if (topic[0] != '/') {
    topic = topic.insert(0, "/");
  }

  // Check if there is a namespace
  if (GetPlatform() != "") {
    topic = topic.insert(0, ("/" + GetPlatform()));
  }
}

// The startup timer is trigger when the robot first starts
// Starts the immediate recording
void DataBagger::OnStartupTimer(ros::TimerEvent const& event) {
  std::string err_msg = "";
  ROS_ERROR_STREAM("OnStartupTimer");

  // Get name of topics being published
  GetTopicNames();
  ROS_ERROR_STREAM("OnStartupTimer1");

  // Check to see if there were default topics to start recording.
  if (default_data_state_.topic_save_settings.size() > 0) {
  ROS_ERROR_STREAM("OnStartupTimer2");
    if (!SetImmediateDataToDisk(err_msg)) {
      this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
      return;
    }
  }
}

bool DataBagger::SetImmediateDataToDisk(std::string &err_msg) {
  ROS_ERROR_STREAM("SetImmediateDataToDisk");
  ResetRecorders(true);

  // Clear record options if we get new immediate data
  recorder_options_immediate_.topics.clear();

  for (auto & setting : default_data_state_.topic_save_settings) {
    // Can assume all downlink options are immediate since this comes from the
    // config file and was already checked when the config file was read in
    recorder_options_immediate_.topics.push_back(setting.topic_name);

    // TODO(Someone) Need to figure out how to record topics at different
    // frequencies. For now, report error if frequency is valid to let the
    // operator know that this functionality isn't implemented yet
    if (setting.frequency != -1) {
      err_msg = "Frequency for every topic must be -1. Different frequencies ";
      err_msg = "for each topic is not supported yet!";
      return false;
    }
  }

  std::string dated_dir = save_dir_ + GetDate(false) + "/" + robot_name_;
  dated_dir += "/immediate/";

  if (!MakeDir(dated_dir, false, err_msg)) {
    return false;
  }

  recorder_options_immediate_.prefix = dated_dir + GetDate(true) + "_" +
                                                      default_data_state_.name;
  recorder_options_immediate_.split = true;
  recorder_options_immediate_.max_size = bag_size_bytes_;
  recorder_options_immediate_.append_date = false;

  StartImmediateRecording();

  GenerateCombinedState(NULL);
  PublishState();

  return true;
}

void DataBagger::StartDelayedRecording() {
  delayed_recorder_ = new astrobee_rosbag::Recorder(recorder_options_delayed_);
  delayed_recorder_->run();
}

void DataBagger::StartImmediateRecording() {
  ROS_ERROR_STREAM("StartImmediateRecording");
  immediate_recorder_ = new astrobee_rosbag::Recorder(recorder_options_immediate_);
  immediate_recorder_->run();
}

// Clear topics, stop recording
void DataBagger::ResetRecorders(bool immediate) {
  if (immediate) {
    if (immediate_recorder_ != nullptr) {
      immediate_recorder_->stop();
      delete immediate_recorder_;
      immediate_recorder_ = nullptr;
    }
  } else {
    if (delayed_recorder_ != nullptr) {
      delayed_recorder_->stop();
      delete delayed_recorder_;
      delayed_recorder_ = nullptr;
    }
  }
}

void DataBagger::GenerateCombinedState(ff_msgs::DataToDiskState *ground_state) {
  combined_data_state_.recording = false;

  // Equal is implemented for vectors !! :)
  combined_data_state_.topic_save_settings =
                                        default_data_state_.topic_save_settings;

  combined_data_state_.name = default_data_state_.name;

  if (ground_state != NULL) {
    if (default_data_state_.name == "") {
      combined_data_state_.name = ground_state->name;
    } else {
      combined_data_state_.name += " + " + ground_state->name;
    }

    for (auto & setting : ground_state->topic_save_settings) {
      combined_data_state_.topic_save_settings.push_back(setting);
    }
  }
}

// Publishes recording state when changes are made
void DataBagger::PublishState() {
  combined_data_state_.header.stamp = ros::Time::now();
  pub_data_state_.publish(combined_data_state_);
}

}  // namespace data_bagger

PLUGINLIB_EXPORT_CLASS(data_bagger::DataBagger, nodelet::Nodelet)
