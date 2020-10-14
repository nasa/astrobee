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

#include <mem_monitor/mem_monitor.h>

namespace mem_monitor {

MemMonitor::MemMonitor() :
  ff_util::FreeFlyerNodelet(),
  load_fault_state_(CLEARED),
  pub_queue_size_(10),
  update_freq_hz_(1),
  mem_load_limit_(100) {
}

MemMonitor::~MemMonitor() {
}

void MemMonitor::Initialize(ros::NodeHandle *nh) {
  std::string err_msg;
  // First three letters of the node name specifies processor
  processor_name_ = GetName().substr(0, 3);

  config_params_.AddFile("management/mem_monitor.config");
  if (!ReadParams()) {
    return;
  }

  reload_params_timer_ = nh->createTimer(ros::Duration(1),
      [this](ros::TimerEvent e) {
      config_params_.CheckFilesUpdated(std::bind(&MemMonitor::ReadParams, this));},
      false,
      true);

  // All state messages are latching
  mem_state_pub_ = nh->advertise<ff_msgs::MemStateStamped>(
                                            TOPIC_MANAGEMENT_MEM_MONITOR_STATE,
                                            pub_queue_size_,
                                            true);

  // Timer for asserting the memory load too high fault
  assert_load_fault_timer_ = nh->createTimer(
                            ros::Duration(assert_load_high_fault_timeout_sec_),
                            &MemMonitor::AssertLoadHighFaultCallback,
                            this,
                            true,
                            false);

  // Timer for clearing the cpu load too high fault
  clear_load_fault_timer_ = nh->createTimer(
                              ros::Duration(clear_load_high_fault_timeout_sec_),
                              &MemMonitor::ClearLoadHighFaultCallback,
                              this,
                              true,
                              false);

  // Timer for checking cpu stats. Timer is not one shot and start it right away
  stats_timer_ = nh->createTimer(ros::Duration(update_freq_hz_),
                                 &MemMonitor::PublishStatsCallback,
                                 this,
                                 false,
                                 true);
}

bool MemMonitor::ReadParams() {
  std::string err_msg;
  // Read config files into lua
  if (!config_params_.ReadFiles()) {
    err_msg = "Memory monitor: Unable to read configuration files.";
    FF_ERROR(err_msg);
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }

  // Get table for this processor
  config_reader::ConfigReader::Table processor_config(&config_params_,
                                                      processor_name_.c_str());

  // get udpate stats frequency
  if (!processor_config.GetInt("update_freq_hz", &update_freq_hz_)) {
    err_msg = "Memory monitor: Update frequency not specified for " +
                                                                processor_name_;
    FF_ERROR(err_msg);
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }
  // get memory load limit
  if (!processor_config.GetReal("mem_load_limit", &mem_load_limit_)) {
    err_msg = "Memory monitor: Memory percentage high load not specified for " +
                                                                processor_name_;
    FF_ERROR(err_msg);
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }
  // get memory assert load high fault timeout secs
  if (!processor_config.GetInt("assert_load_high_fault_timeout_sec", &assert_load_high_fault_timeout_sec_)) {
    err_msg = "Memory monitor: Memory assert load high fault timeout seconds";
    err_msg +=  "not specified for " + processor_name_;
    FF_ERROR(err_msg);
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }
  // get udpate stats frequency
  if (!processor_config.GetInt("clear_load_high_fault_timeout_sec", &clear_load_high_fault_timeout_sec_)) {
    err_msg = "Memory monitor: Memory clear load high fault timeout seconds not specified for " +
                                                                processor_name_;
    FF_ERROR(err_msg);
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }

  return true;
}

void MemMonitor::AssertLoadHighFaultCallback(ros::TimerEvent const& te) {
  // Stop timer so we don't trigger the fault over and over again
  assert_load_fault_timer_.stop();
  std::string err_msg = "Memory average load is " +
                        std::to_string(mem_load_value_) +
                        " which is greater than " +
                        std::to_string(mem_load_limit_) + ".";
  FF_ERROR(err_msg);
  this->AssertFault(ff_util::MEMORY_USAGE_TOO_HIGH, err_msg);
  load_fault_state_ = ASSERTED;
}

void MemMonitor::ClearLoadHighFaultCallback(ros::TimerEvent const& te) {
  // Stop timer so we don't try to clear the fault over and over again
  clear_load_fault_timer_.stop();
  this->ClearFault(ff_util::MEMORY_USAGE_TOO_HIGH);
  load_fault_state_ = CLEARED;
}

int MemMonitor::ParseLine(char* line) {
  // This assumes that a digit will be found and the line ends in " Kb".
  int i = strlen(line);
  const char* p = line;
  while (*p < '0' || *p > '9') p++;
  line[i - 3] = '\0';
  i = atoi(p);
  return i;
}

void MemMonitor::AssertStats() {
// Check to see if the total percentage is greater than the fault threshold
  if (mem_load_value_ > mem_load_limit_) {
    // Check to see if the fault is in the process of being cleared or is
    // cleared
    if (load_fault_state_ == CLEARING) {
      // Just need to stop the timer put in place to clear the fault. Don't need
      // to start the timer to assert the fault since it is already triggered.
      clear_load_fault_timer_.stop();
      // Switch load fault state back to triggered
      load_fault_state_ = ASSERTED;
    } else if (load_fault_state_ == CLEARED) {
      // If fault isn't triggered, start the process of triggering it
      assert_load_fault_timer_.start();
      load_fault_state_ = ASSERTING;
      // Keep an average total load value to report to the ground what is going
      // on
      avg_load_high_value_ = mem_load_value_;
    } else if (load_fault_state_ == ASSERTING) {
      avg_load_high_value_ += mem_load_value_;
      avg_load_high_value_ /= 2;
    }
  } else {
    // Check to see if the fault is in the process of being triggered or is
    // triggered
    if (load_fault_state_ == ASSERTING) {
      // Just need to stop the timer put in place to trigger the fault. Don't
      // need to start the timer to clear the fault since it is already cleared.
      assert_load_fault_timer_.stop();
      // Switch load fault state back to cleared
      load_fault_state_ = CLEARED;
    } else if (load_fault_state_ == ASSERTED) {
      // If fault is triggered, start the process of clearing it
      clear_load_fault_timer_.start();
      load_fault_state_ = CLEARING;
    }
  }
}

std::string getHostfromURI(std::string uri) {
  std::size_t uri_begin = uri.find_first_of("/");
  std::size_t uri_end = uri.find_last_of(":");
  if (std::string::npos != uri_begin && std::string::npos != uri_end &&
      uri_begin <= uri_end) {
    uri.erase(uri.begin() + uri_end, uri.end());
    uri.erase(uri.begin(), uri.begin() + uri_begin + 2);
    return uri;
  } else {
    ROS_ERROR_STREAM("Invalid URI, returning ");
    return {};
  }
}

void MemMonitor::PublishStatsCallback(ros::TimerEvent const &te) {
  // Stop the timer
  stats_timer_.stop();
  // Declare the message
  ff_msgs::MemStateStamped mem_state_msg;

  // Update memory info
  // In the mem_info_ structure, sizes of the memory and swap
  // fields  are  given  as  multiples  of mem_unit bytes.
  sysinfo(&mem_info_);
  // Total Physical Memory (RAM)
  mem_state_msg.ram_total = (mem_info_.totalram * 1e-06) * mem_info_.mem_unit;
  // Total Virtual Memory
  mem_state_msg.virt_total = ((mem_info_.totalswap * 1e-06)
                            + (mem_info_.totalram * 1e-06))
                            * mem_info_.mem_unit;

  // Physical Memory currently used
  mem_state_msg.ram_used = (mem_info_.totalram - mem_info_.freeram) * 1e-06
                          * mem_info_.mem_unit;
  mem_load_value_ = mem_state_msg.ram_used;

  // Virtual Memory Currently Used
  mem_state_msg.virt_used =  (mem_info_.totalswap - mem_info_.freeswap) * 1e-06
                            * mem_info_.mem_unit
                            + mem_state_msg.ram_used;

  mem_load_value_ = mem_state_msg.ram_used / mem_state_msg.ram_total * 1e+2;
  // Get ROS nodes memory useage
  std::vector<std::string> nodes;
  ros::master::getNodes(nodes);

  // Get own URI
  // Check if the node is being executed in this computer
  // Get URI of the node
  XmlRpc::XmlRpcValue args, result, payload;
  args.setSize(2);
  args[0] = ros::this_node::getName();
  args[1] = ros::this_node::getName();
  ros::master::execute("lookupNode", args, result, payload, true);
  std::string monitor_host = getHostfromURI(result[2]);
  if (monitor_host.empty()) {
    ROS_ERROR_STREAM("URI of the memory monitor not valid");
    return;
  }
  mem_state_msg.name = monitor_host;

  // Go through all the node list and
  for (uint i = 0; i < nodes.size(); ++i) {
    ROS_ERROR_STREAM(monitor_host << " reading " << nodes[i] << " node:" << i << "/" << nodes.size());
    // Look for PID if not already on the list
    if (nodes_pid_.find(nodes[i]) == nodes_pid_.end()) {
      // Check if the node is being executed in this computer
      // Get URI of the node
      args.setSize(2);
      args[0] = ros::this_node::getName();
      args[1] = nodes[i];
      ros::master::execute("lookupNode", args, result, payload, true);
      std::string node_host = getHostfromURI(result[2]);
      if (node_host.empty()) {
        nodes_pid_.insert(std::pair<std::string, int>(nodes[i], -1));
        continue;
      }

      // If it is in the same cpu
      if (node_host != monitor_host) {
        // Insert it on the list
        nodes_pid_.insert(std::pair<std::string, int>(nodes[i], -1));
      }

      // Get the node PID
      std::array<char, 128> buffer;
      std::string pid;
      FILE* pipe = popen(("rosnode info " + nodes[i] + " 2>/dev/null | grep Pid| cut -d' ' -f2").c_str(), "r");
      if (!pipe) {
        // Node not found
        ROS_ERROR_STREAM("rosnode info failed for node " << nodes[i]);
        nodes_pid_.insert(std::pair<std::string, int>(nodes[i], -1));
        continue;
      }
      while (fgets(buffer.data(), 128, pipe) != NULL) {
        pid += buffer.data();
      }
      ROS_ERROR_STREAM("pid found " << pid);

      if (pid.empty()) {
        nodes_pid_.insert(std::pair<std::string, int>(nodes[i], -1));
        continue;
      }
      pclose(pipe);
      // Insert it on the list
      nodes_pid_.insert(std::pair<std::string, int>(nodes[i], std::stoi(pid)));
    }
    // Check that the process is in this computer
    if (nodes_pid_.find(nodes[i])->second <= 0)
      continue;

    // Get Memory useage
    ff_msgs::MemState mem_node;
    mem_node.name = nodes[i];
    FILE* file = fopen(("/proc/" + std::to_string(nodes_pid_.find(nodes[i])->second) + "/status").c_str(), "r");
    if (!file) {
      // File not found
      ROS_ERROR_STREAM("failed reading the PID file for " << nodes[i]);
      continue;
    }
    char line[128];
    ROS_ERROR_STREAM("start reading stats for " << nodes[i]);
    while (fgets(line, 128, file) != NULL) {
      // Get virtual memory in Mb
      if (strncmp(line, "VmSize:", 7) == 0) {
        mem_node.virt = ParseLine(line) * 1e-03;       // Convert from Kb to Mb
      }
      // Get peak virtual memory in Mb
      if (strncmp(line, "VmPeak:", 7) == 0) {
        mem_node.virt_peak = ParseLine(line) * 1e-03;  // Convert from Kb to Mb
      }
      // Get physical memory in Mb
      if (strncmp(line, "VmRSS:", 6) == 0) {
        mem_node.ram = ParseLine(line) * 1e-03;        // Convert from Kb to Mb
        mem_node.ram_perc = static_cast<float>(mem_node.ram) / static_cast<float>(mem_state_msg.ram_total) * 1e+02;
      }
      // Get physical memory in Mb
      if (strncmp(line, "VmHWM:", 6) == 0) {
        mem_node.ram_peak = ParseLine(line) * 1e-03;  // Convert from Kb to Mb
      }
    }
    fclose(file);
    ROS_ERROR_STREAM("finished reading for " << nodes[i]);
    mem_state_msg.nodes.push_back(mem_node);
  }

  ROS_ERROR_STREAM("pub");
  // Send mem stats
  mem_state_msg.header.stamp = ros::Time::now();
  mem_state_pub_.publish(mem_state_msg);

  // Assert stats
  AssertStats();

  // Restart the timer
  stats_timer_.start();
}

}  // namespace mem_monitor

PLUGINLIB_EXPORT_CLASS(mem_monitor::MemMonitor, nodelet::Nodelet)
