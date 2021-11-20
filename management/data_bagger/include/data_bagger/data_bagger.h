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

#ifndef DATA_BAGGER_DATA_BAGGER_H_
#define DATA_BAGGER_DATA_BAGGER_H_

#include <boost/date_time/posix_time/posix_time.hpp>

#include <config_reader/config_reader.h>

#include <data_bagger/astrobee_recorder.h>

#include <errno.h>

#include <ff_msgs/DataToDiskState.h>
#include <ff_msgs/DataTopicsList.h>
#include <ff_msgs/EnableRecording.h>
#include <ff_msgs/SetDataToDisk.h>

#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>

#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <sys/stat.h>
#include <sys/types.h>

#include <string>
#include <vector>


namespace data_bagger {

class DataBagger : public ff_util::FreeFlyerNodelet {
 public:
  DataBagger();
  ~DataBagger();

  // Service that sets delayed recording settings
  bool SetDelayedDataToDiskService(ff_msgs::SetDataToDisk::Request &req,
                            ff_msgs::SetDataToDisk::Response &res);

  // This service enables and disables the delayed recording
  bool EnableDelayedRecordingService(ff_msgs::EnableRecording::Request &req,
                              ff_msgs::EnableRecording::Response &res);

 protected:
  virtual void Initialize(ros::NodeHandle *nh);
  bool ReadParams();  // Reads data bagger parameters and default profile

 private:
  // Reads all topic names published by the robot
  void GetTopicNames();

  // Recursively created bag destionation folder
  bool MakeDir(std::string dir, bool assert_init_fault, std::string &err_msg);

  // Gets the date for name generation
  std::string GetDate(bool with_time);

  // Adds the namespace to topic names
  void AddTopicNamespace(std::string &topic);

  // Timer for when the robot finishes startup, starts immediate recording
  void OnStartupTimer(ros::TimerEvent const& event);

  // Sets immediate recording settings
  bool SetImmediateDataToDisk(std::string &err_msg);

  // Start Recordings
  void StartDelayedRecording();
  void StartImmediateRecording();

  // Clear topics, stop recording
  void ResetRecorders(bool immediate);

  // Update data bagger profile state
  void GenerateCombinedState(ff_msgs::DataToDiskState *ground_state);

  // Publish data bagger state
  void PublishState();

  ros::NodeHandle nh_;

  astrobee_rosbag::Recorder *delayed_recorder_, *immediate_recorder_;

  config_reader::ConfigReader config_params_;

  ff_msgs::DataToDiskState default_data_state_, combined_data_state_;

  int pub_queue_size_;
  unsigned int startup_time_secs_;
  int64_t bag_size_bytes_;

  ros::Publisher pub_data_state_, pub_data_topics_;
  ros::Timer startup_timer_;

  ros::ServiceServer set_service_, record_service_;

  rosbag::RecorderOptions recorder_options_delayed_;
  rosbag::RecorderOptions recorder_options_immediate_;

  std::string save_dir_, robot_name_, delayed_profile_name_;
};

}  //  namespace data_bagger

#endif  // DATA_BAGGER_DATA_BAGGER_H_
