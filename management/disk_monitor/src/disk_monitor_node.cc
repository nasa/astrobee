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
  usage_high_fault_triggered_(false),
  usage_too_high_fault_triggered_(false),
  pub_queue_size_(10),
  disks_high_(""),
  disks_too_high_("") {
  // don't specify node name so we use the name from the launch file
}

DiskMonitor::~DiskMonitor() {
}

bool DiskMonitor::PathExists(std::string path) {
  struct stat status;
  return !(stat(path.data(), &status) < 0);
}

void DiskMonitor::Initialize(ros::NodeHandle * nh) {
  std::string err_msg;

  // First three letters of the node name specifies processor
  processor_name_ = GetName().substr(0, 3);

  config_params_.AddFile("management/disk_monitor.config");
  if (!LoadParams(ff_util::INITIALIZATION_FAILED)) {
    return;
  }

  reload_params_timer_ = nh->createTimer(ros::Duration(1),
      [this](ros::TimerEvent e) {
      config_params_.CheckFilesUpdated(std::bind(&DiskMonitor::ReloadParams, this));},
      false,
      true);

  // Setup disk state message
  disk_stats_msg_.header.frame_id = "world";
  disk_stats_msg_.processor_name = processor_name_;

  // Setup the publisher
  pub_disk_stats_ = nh->advertise<ff_msgs::DiskStateStamped>(
                    TOPIC_MANAGEMENT_DISK_MONITOR_STATE, pub_queue_size_);

  stats_timer_ = nh->createTimer(ros::Duration(update_freq_),
                              &DiskMonitor::OnCheckTimer, this);
}

void DiskMonitor::ReloadParams() {
  if (LoadParams(ff_util::DISK_CONFIG_INVALID)) {
    // If load parameters succeeded, clear fault. If fault not asserted, the
    // clear won't do anything
    this->ClearFault(ff_util::DISK_CONFIG_INVALID);
    CheckAndPublish();
  }
}

bool DiskMonitor::LoadParams(ff_util::FaultKeys fault_key) {
  std::string err_msg;
  unsigned int i;

  // Save paths in case the new paths we try to read are invalid
  std::shared_ptr<std::vector<PathInfo>> old_paths = paths_info_;

  // Make a new path pointer so we can read new paths
  paths_info_ = std::make_shared<std::vector<PathInfo>>();

  if (!ReadParams(fault_key)) {
    // Unable to read in the parameters. Copy old known good paths back in if
    // they weren't null to begin with.
    if (old_paths != NULL) {
      paths_info_ = old_paths;
    }
    return false;
  }

  // If the new paths were read in correctly, make sure there were paths read in
  if (paths_info_->size() == 0) {
    err_msg = "Disk monitor: config was empty? no disks to monitor. ";
    err_msg += "Falling back to old paths if they exist.";
    ROS_ERROR("%s", err_msg.c_str());
    this->AssertFault(fault_key, err_msg);
    if (old_paths != NULL && old_paths->size() != 0) {
      paths_info_ = old_paths;
    }
    return false;
  }

  // Check that all the paths are valid, don't craft the diskk monitor state yet
  // because we may have to fall back to old paths if one of paths doesn't exist
  for (i = 0; i < paths_info_->size(); i++) {
    if (!PathExists(paths_info_->at(i).path_name)) {
      err_msg = "Disk monitor: " + processor_name_ + "[" +
                paths_info_->at(i).path_name +
                "] does not exist! Falling back to old paths if the exist.";
      ROS_ERROR("%s", err_msg.c_str());
      this->AssertFault(fault_key, err_msg);
      if (old_paths != NULL) {
        paths_info_ = old_paths;
      }
      return false;
    }
  }

  // If the new paths were all valid, we need to redo the state message
  disk_stats_msg_.disks.clear();
  for (i = 0; i < paths_info_->size(); i++) {
    ff_msgs::DiskState temp;
    temp.path = paths_info_->at(i).path_name;
    disk_stats_msg_.disks.push_back(temp);
  }

  return true;
}


bool DiskMonitor::ReadParams(ff_util::FaultKeys fault_key) {
  std::string err_msg;
  if (!config_params_.ReadFiles()) {
    err_msg = "Disk monitor: Unable to read configuration files.";
    ROS_ERROR("%s", err_msg.c_str());
    this->AssertFault(fault_key, err_msg);
    return false;
  }

  // Check if the config table for this processor exists
  if (!config_params_.CheckValExists(processor_name_.c_str())) {
    err_msg = "Disk monitor: Config table doesn't exist for processor " +
                                                                processor_name_;
    ROS_ERROR("%s", err_msg.c_str());
    this->AssertFault(fault_key, err_msg);
    return false;
  }

  // Get table for this processor
  config_reader::ConfigReader::Table processor_config(&config_params_,
                                                      processor_name_.c_str());

  if (!processor_config.GetInt("update_freq_hz", &update_freq_)) {
    err_msg = "Disk monitor: Update frequency not specified for " +
                                                                processor_name_;
    ROS_ERROR("%s", err_msg.c_str());
    this->AssertFault(fault_key, err_msg);
    return false;
  }

  // Check if the file system table exists
  if (!processor_config.CheckValExists("filesys")) {
    err_msg = "Disk monitor: filesys table doesn't exist for processor " +
                                                                processor_name_;
    ROS_ERROR("%s", err_msg.c_str());
    this->AssertFault(fault_key, err_msg);
    return false;
  }

  // Get the names of the filesystems we are monitoring
  config_reader::ConfigReader::Table filesys(&processor_config, "filesys");

  std::string name;
  bool check_fault;
  // Lua indices start at one
  for (int i = 1; i < (filesys.GetSize() + 1); i++) {
    config_reader::ConfigReader::Table filesys_entry(&filesys, i);

    if (!filesys_entry.GetStr("name", &name)) {
      err_msg = "Disk monitor: The filesystem name located at " +
                                                              std::to_string(i);
      err_msg += " either wasn't specified or wasn't a string!";
      ROS_ERROR("%s", err_msg.c_str());
      this->AssertFault(fault_key, err_msg);
      return false;
    }

    if (!filesys_entry.GetBool("check_usage_high_fault", &check_fault)) {
      err_msg = "Disk monitor: The check usage high fault located at " +
                                                              std::to_string(i);
      err_msg += " either wasn't specified or wasn't a boolean.";
      ROS_ERROR("%s", err_msg.c_str());
      this->AssertFault(fault_key, err_msg);
      return false;
    }

    PathInfo new_path;
    new_path.path_name = name;
    new_path.check_load_high_fault = check_fault;

    if (check_fault) {
      float high_thres, critical_thres;
      if (!filesys_entry.GetPosReal("high_thres_percent", &high_thres)) {
        err_msg = "Disk monitor: The high threshold percent located at " +
                                                              std::to_string(i);
        err_msg += " either wasn't specified or wasn't a float.";
        ROS_ERROR("%s", err_msg.c_str());
        this->AssertFault(fault_key, err_msg);
        return false;
      }

      if (!filesys_entry.GetPosReal("critical_thres_percent",
                                    &critical_thres)) {
        err_msg = "Disk monitor: The critical threshold percent located at " +
                                                              std::to_string(i);
        err_msg += " either wasn't specifed or wasn't a float.";
        ROS_ERROR("%s", err_msg.c_str());
        this->AssertFault(fault_key, err_msg);
        return false;
      }
      new_path.high_thres_percent = high_thres;
      new_path.critical_thres_percent = critical_thres;
    }

    paths_info_->push_back(new_path);
  }

  return true;
}

void DiskMonitor::OnCheckTimer(ros::TimerEvent const& ev) {
  CheckAndPublish();
}

bool DiskMonitor::CheckAndPublish() {
  struct statvfs stats;
  uint64_t total, avail, used;

  // Create 2 strings to record which disks are running out of memory
  std::string disks_high = "";
  std::string disks_too_high = "";
  std::string fault_msg = "";

  for (unsigned int i = 0; i < paths_info_->size(); i++) {
    if (statvfs(paths_info_->at(i).path_name.data(), &stats) < 0) {
      ROS_ERROR("statvfs error: %s", strerror(errno));
      return false;
    }

    total = (uint64_t) stats.f_bsize * stats.f_blocks;
    avail = (uint64_t) stats.f_bsize * stats.f_bavail;
    used = total - avail;

    // have to cast to avoid warning, uint64_t is not long long unsigned int on
    // 64-bit
    // ROS_INFO("%s[%s]: %llu / %llu", processor_name_.data(), path.data(),
        // static_cast<long long unsigned int>(avail),
        // static_cast<long long unsigned int>(total));  // NOLINT
    if (paths_info_->at(i).path_name != disk_stats_msg_.disks[i].path) {
      ROS_FATAL("Disk monitor: Paths don't match in message! Fix!!!");
      return false;
    }

    disk_stats_msg_.disks[i].capacity = total;
    disk_stats_msg_.disks[i].used = used;

    if (paths_info_->at(i).check_load_high_fault) {
      // Check too high first
      if (used > (total * paths_info_->at(i).critical_thres_percent)) {
        if (disks_too_high.size() > 0) {
          disks_too_high.append(", ");
        }
        disks_too_high.append(paths_info_->at(i).path_name);
      } else if (used > (total * paths_info_->at(i).high_thres_percent)) {
        if (disks_high.size() > 0) {
          disks_high.append(", ");
        }
        disks_high.append(paths_info_->at(i).path_name);
      }
    }
  }

  if (disks_high.size() > 0) {
    if (!usage_high_fault_triggered_ || disks_high_ != disks_high) {
      fault_msg = "Disk usage high for " + disks_high + ".";
      disks_high_ = disks_high;
      usage_high_fault_triggered_ = true;
      this->AssertFault(ff_util::DISK_USAGE_HIGH, fault_msg);
    }
  } else {
    if (usage_high_fault_triggered_) {
      disks_high_ = "";
      usage_high_fault_triggered_ = false;
      this->ClearFault(ff_util::DISK_USAGE_HIGH);
    }
  }

  if (disks_too_high.size() > 0) {
    if (!usage_too_high_fault_triggered_ || disks_too_high_ != disks_too_high) {
      fault_msg = "Disk usage too high for " + disks_too_high + ".";
      disks_too_high_ = disks_too_high;
      usage_too_high_fault_triggered_ = true;
      this->AssertFault(ff_util::DISK_USAGE_TOO_HIGH, fault_msg);
    }
  } else {
    if (usage_too_high_fault_triggered_) {
      disks_too_high_ = "";
      usage_too_high_fault_triggered_ = false;
      this->ClearFault(ff_util::DISK_USAGE_TOO_HIGH);
    }
  }

  disk_stats_msg_.header.stamp = ros::Time::now();
  pub_disk_stats_.publish(disk_stats_msg_);

  return true;
}

/************************** Path Info Functions *******************************/
DiskMonitor::PathInfo::PathInfo() :
  path_name(""),
  check_load_high_fault(false),
  high_thres_percent(0.0),
  critical_thres_percent(0.0) {
}

DiskMonitor::PathInfo::PathInfo(std::string const& path_name_in,
                                bool const check_load_high_fault_in,
                                float const high_thres_percent_in,
                                float const critical_thres_percent_in) :
  path_name(path_name_in),
  check_load_high_fault(check_load_high_fault_in),
  high_thres_percent(high_thres_percent_in),
  critical_thres_percent(critical_thres_percent_in) {
}
}  // namespace disk_monitor

PLUGINLIB_EXPORT_CLASS(disk_monitor::DiskMonitor, nodelet::Nodelet)
