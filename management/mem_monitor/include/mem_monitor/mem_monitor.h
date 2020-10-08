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


#ifndef MEM_MONITOR_MEM_MONITOR_H_
#define MEM_MONITOR_MEM_MONITOR_H_

#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <ros/xmlrpc_manager.h>
#include <XmlRpcValue.h>
#include <XmlRpcClient.h>

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <sys/types.h>
#include <sys/sysinfo.h>

#include <config_reader/config_reader.h>
#include <ff_msgs/MemState.h>
#include <ff_msgs/MemStateStamped.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>

#include <cstdint>
#include <string>
#include <vector>
#include <map>

namespace mem_monitor {

enum LoadFaultState {
  ASSERTING,
  ASSERTED,
  CLEARING,
  CLEARED
};

class MemMonitor : public ff_util::FreeFlyerNodelet {
 public:
  MemMonitor();
  ~MemMonitor();

 protected:
  virtual void Initialize(ros::NodeHandle *nh);
  bool ReadParams();

 private:
  void AssertLoadHighFaultCallback(ros::TimerEvent const& te);

  void ClearLoadHighFaultCallback(ros::TimerEvent const& te);

  /** Collect usage stats about memory useage, calculate percentages
    * based on the last time this was called. You should call this in
    * regular intervals for the numbers to make sense over time. */
  int ParseLine(char* line);
  void AssertStats();
  void PublishStatsCallback(ros::TimerEvent const &te);

  // Relevant variables


  // Scope memory info from sysinfo
  struct sysinfo mem_info_;

  LoadFaultState load_fault_state_;
  int pub_queue_size_, update_freq_hz_, mem_avg_load_limit_;
  float mem_load_value_, mem_load_limit_, avg_load_high_value_;
  int assert_load_high_fault_timeout_sec_, clear_load_high_fault_timeout_sec_;

  config_reader::ConfigReader config_params_;

  ros::Publisher mem_state_pub_;
  ros::Timer reload_params_timer_, stats_timer_;
  ros::Timer assert_load_fault_timer_, clear_load_fault_timer_;
  std::string processor_name_;

  // Store PID values
  std::map<std::string, int> nodes_pid_;
};

}  // namespace mem_monitor

#endif  // MEM_MONITOR_MEM_MONITOR_H_
