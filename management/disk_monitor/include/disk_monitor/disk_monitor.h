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
  bool ReadParams(ff_util::FaultKeys fault_key);

  struct PathInfo {
    PathInfo();
    PathInfo(std::string const& path_name_in,
             bool const check_load_high_fault_in,
             float const high_thres_percent_in,
             float const too_high_thres_percent_in);
    std::string path_name;
    bool check_load_high_fault;
    float high_thres_percent;
    float critical_thres_percent;
  };

 private:
  void ReloadParams();
  bool LoadParams(ff_util::FaultKeys fault_key);
  void OnCheckTimer(ros::TimerEvent const& event);
  bool CheckAndPublish();

  bool usage_high_fault_triggered_, usage_too_high_fault_triggered_;

  config_reader::ConfigReader config_params_;

  ff_msgs::DiskStateStamped disk_stats_msg_;

  int pub_queue_size_, update_freq_;

  ros::Publisher pub_disk_stats_;
  ros::Timer reload_params_timer_, stats_timer_;

  std::shared_ptr<std::vector<PathInfo>> paths_info_;
  std::string pub_topic_disk_stats_, processor_name_;
  std::string disks_high_, disks_too_high_;
};

}  // namespace disk_monitor

#endif  // DISK_MONITOR_DISK_MONITOR_H_
