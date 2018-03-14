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

#ifndef DISK_MONITOR_DISK_MONITOR_H_
#define DISK_MONITOR_DISK_MONITOR_H_

#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <errno.h>
#include <stdint.h>

#include <sys/stat.h>
#include <sys/types.h>
#include <sys/statvfs.h>

#include <config_reader/config_reader.h>
#include <ff_msgs/DiskState.h>
#include <ff_msgs/DiskStateStamped.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>

#include <memory>
#include <string>
#include <vector>

namespace disk_monitor {

class DiskMonitor : public ff_util::FreeFlyerNodelet {
 public:
  DiskMonitor();
  ~DiskMonitor();
  bool PathExists(std::string path);

 protected:
  virtual void Initialize(ros::NodeHandle *nh);
  bool ReadParams();

 private:
  void OnCheckTimer(ros::TimerEvent const& event);
  bool CheckAndPublish();

  config_reader::ConfigReader config_params_;

  ff_msgs::DiskStateStamped disk_stats_msg_;

  int pub_queue_size_, update_freq_;

  ros::Publisher pub_disk_stats_;
  ros::Timer stats_timer_;

  std::string pub_topic_disk_stats_, processor_name_;
  std::vector<std::string> paths_;
};

}  // namespace disk_monitor

#endif  // DISK_MONITOR_DISK_MONITOR_H_
