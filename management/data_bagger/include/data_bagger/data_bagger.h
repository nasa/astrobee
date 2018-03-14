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

#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <config_reader/config_reader.h>

#include <ff_msgs/DataToDiskState.h>
#include <ff_msgs/DataTopicsList.h>

#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>

#include <string>

namespace data_bagger {

class DataBagger : public ff_util::FreeFlyerNodelet {
 public:
  DataBagger();
  ~DataBagger();

 protected:
  virtual void Initialize(ros::NodeHandle *nh);
  bool ReadParams();

 private:
  void OnStartupTimer(ros::TimerEvent const& event);
  void GetTopicNames();

  config_reader::ConfigReader config_params_;

  ff_msgs::DataToDiskState data_state_msg_;

  int pub_queue_size_;
  unsigned int startup_time_secs_;

  ros::Publisher pub_data_state_, pub_data_topics_;
  ros::Timer startup_timer_;
};

}  //  namespace data_bagger

#endif  // DATA_BAGGER_DATA_BAGGER_H_
