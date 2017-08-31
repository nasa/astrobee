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

// A node to monitor a disk or two, publishing on a given interval

#include <disk_monitor/disk_monitor.h>

namespace disk_monitor {
DiskMonitor::DiskMonitor() :
  ff_util::FreeFlyerNodelet(),
  pub_queue_size_(10) {
  // don't specify node name so we use the name from the launch file
}

DiskMonitor::~DiskMonitor() {
}

bool DiskMonitor::PathExists(std::string path) {
  struct stat status;
  return !(stat(path.data(), &status) < 0);
}

void DiskMonitor::Initialize(ros::NodeHandle * nh) {
  // First three letters of the node name specifies processor
  processor_name_ = GetName().substr(0, 3);

  config_params_.AddFile("management/disk_monitor.config");
  if (!ReadParams()) {
    exit(EXIT_FAILURE);
    return;
  }

  // Don't set up param reload timer since it doesn't make sense to reload the
  // disk monitor params during run time

  // Setup the publisher
  pub_disk_stats_ = nh->advertise<ff_msgs::DiskStateStamped>(
                    TOPIC_MANAGEMENT_DISK_MONITOR_STATE, pub_queue_size_, true);

  // There is no point starting if there are no disks registered
  if (paths_.size() == 0) {
    ROS_ERROR("Disk monitor: config was empty? no disks to monitor. bailing.");
    return;
  }

  // Setup disk state message
  disk_stats_msg_.header.frame_id = "world";
  disk_stats_msg_.processor_name = processor_name_;

  // Check that all the paths are valid
  for (unsigned int i = 0; i < paths_.size(); i++) {
    if (!PathExists(paths_[i])) {
      ROS_ERROR("Disk monitor: %s[%s] does not exist!", processor_name_.c_str(),
                paths_[i].data());
      return;
    }
    ff_msgs::DiskState temp;
    temp.path = paths_[i];

    disk_stats_msg_.disks.push_back(temp);
  }

  CheckAndPublish();

  stats_timer_ = nh->createTimer(ros::Duration(update_freq_),
                              &DiskMonitor::OnCheckTimer, this);
}

bool DiskMonitor::ReadParams() {
  if (!config_params_.ReadFiles()) {
    ROS_FATAL("Disk monitor: Unable to read configuration files.");
    return false;
  }

  // Get table for this processor
  config_reader::ConfigReader::Table processor_config(&config_params_,
                                                      processor_name_.c_str());

  if (!processor_config.GetInt("update_freq_hz", &update_freq_)) {
    ROS_FATAL("Disk monitor: Update frequency not specified for %s",
              processor_name_.c_str());
    return false;
  }

  // Get the names of the filesystems we are monitoring
  config_reader::ConfigReader::Table filesys(&processor_config, "filesys");
  std::string temp_path;
  // Lua indices start at one
  for (int i = 1; i < (filesys.GetSize() + 1); i++) {
    if (!filesys.GetStr(i, &temp_path)) {
      ROS_FATAL("Disk monitor: One of the filesystems wasn't a string!");
      return false;
    }
    paths_.push_back(temp_path);
  }

  return true;
}

void DiskMonitor::OnCheckTimer(ros::TimerEvent const& ev) {
  CheckAndPublish();
}

bool DiskMonitor::CheckAndPublish() {
  struct statvfs stats;
  uint64_t total, avail;

  for (unsigned int i = 0; i < paths_.size(); i++) {
    if (statvfs(paths_[i].data(), &stats) < 0) {
      ROS_ERROR("statvfs error: %s", strerror(errno));
      return false;
    }

    total = (uint64_t) stats.f_bsize * stats.f_blocks;
    avail = (uint64_t) stats.f_bsize * stats.f_bavail;

    // have to cast to avoid warning, uint64_t is not long long unsigned int on
    // 64-bit
    // ROS_INFO("%s[%s]: %llu / %llu", processor_name_.data(), path.data(),
        // static_cast<long long unsigned int>(avail),
        // static_cast<long long unsigned int>(total));  // NOLINT
    if (paths_[i] != disk_stats_msg_.disks[i].path) {
      ROS_FATAL("Disk monitor: Paths don't match in message! Fix!!!");
      return false;
    }

    disk_stats_msg_.disks[i].capacity = total;
    disk_stats_msg_.disks[i].used = total - avail;
  }

  disk_stats_msg_.header.stamp = ros::Time::now();
  pub_disk_stats_.publish(disk_stats_msg_);

  return true;
}

}  // namespace disk_monitor

PLUGINLIB_EXPORT_CLASS(disk_monitor::DiskMonitor, nodelet::Nodelet)
