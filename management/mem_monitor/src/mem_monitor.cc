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

namespace {
constexpr char kProcStat[] = "/proc/self/stat";
}  // namespace

MemMonitor::MemMonitor() :
  ff_util::FreeFlyerNodelet(),
  load_fault_state_(CLEARED),
  avg_load_high_value_(0.0),
  pub_queue_size_(10),
  update_freq_hz_(1),
  mem_avg_load_limit_(95) {
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

  // Update memory info
  sysinfo(&mem_info_);
  // Total Virtual Memory
  mem_state_msg_.total_ram = mem_info_.totalram * mem_info_.mem_unit;
  // Total Physical Memory (RAM)
  mem_state_msg_.total_swap = mem_info_.totalram * mem_info_.mem_unit;
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
  // get memory average load limit
  if (!processor_config.GetInt("mem_avg_load_limit", &mem_avg_load_limit_)) {
    err_msg = "Memory monitor: Memory average load limit not specified for " +
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
                        std::to_string(avg_load_high_value_) +
                        " which is greater than " +
                        std::to_string(mem_avg_load_limit_) + ".";
  FF_ERROR(err_msg);
  this->AssertFault(ff_util::LOAD_TOO_HIGH, err_msg);
  load_fault_state_ = ASSERTED;
}

void MemMonitor::ClearLoadHighFaultCallback(ros::TimerEvent const& te) {
  // Stop timer so we don't try to clear the fault over and over again
  clear_load_fault_timer_.stop();
  this->ClearFault(ff_util::LOAD_TOO_HIGH);
  load_fault_state_ = CLEARED;
}

int MemMonitor::CollectLoadStats() {
  // Update memory info
  sysinfo(&mem_info_);

  // Virtual Memory Currently Used
  mem_state_msg_.used_swap = mem_info_.totalram - mem_info_.freeram;
  // Add other values in next statement to avoid int overflow on right hand side...
  mem_state_msg_.used_swap += mem_info_.totalswap - mem_info_.freeswap;
  mem_state_msg_.used_swap *= mem_info_.mem_unit;

  // Physical Memory currently used
  mem_state_msg_.used_ram = mem_info_.totalram - mem_info_.freeram;
  // Multiply in next statement to avoid int overflow on right hand side...
  mem_state_msg_.used_ram *= mem_info_.mem_unit;

  // Get ROS nodes memory useage
  std::vector<std::string> nodes;
  ros::master::getNodes(nodes);

  for (uint i = 0; i < nodes.size(); ++i) {
    // Get URI of the node
    XmlRpc::XmlRpcValue args, result, payload;
    args.setSize(2);
    args[0] = "/llp_monitors";
    args[1] = nodes[i];
    ros::master::execute("lookupNode", args, result, payload, true);
    // ROS_ERROR_STREAM(nodes[i] <<" URI: " << result[2]);
    std::string uri = result[2];

    // Make new client of node
    XmlRpc::XmlRpcClient* client =
      ros::XMLRPCManager::instance()->getXMLRPCClient(ros::master::getHost(), ros::master::getPort(), uri.c_str());
    ROS_ERROR_STREAM("client: " << nodes[i] << " Host: " << client->getHost() << " Port: " << client->getPort()
                                << " Uri: " << client->getUri());

    // Get PID of the node
    std::string method = "getPid";
    XmlRpc::XmlRpcValue request, response;
    request[0] = "/llp_monitors";
    if (client->execute(method.c_str(), request, response))
      ROS_ERROR_STREAM("getPid code: " << response[0] << " status Message: " << response[1] << " PID: " << response[2]);

    // Gives out own PID
    // args[1] = nodes[0];
    // ros::master::execute("getPid", args, result, payload, true);
    // ROS_ERROR_STREAM("lookupNode code: " << result[0] << "status Message: " << result[1] <<"PID: " << result[2]);
  }

  return 0;
}

void MemMonitor::PublishStatsCallback(ros::TimerEvent const &te) {
  // Get mem load stats first
  if (CollectLoadStats() < 0) {
    ROS_FATAL("Memory node unable to get load stats!");
    return;
  }

  // Send mem stats
  mem_state_msg_.header.stamp = ros::Time::now();
  mem_state_pub_.publish(mem_state_msg_);
}

}  // namespace mem_monitor

PLUGINLIB_EXPORT_CLASS(mem_monitor::MemMonitor, nodelet::Nodelet)
