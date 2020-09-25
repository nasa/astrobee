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
  struct Load {
    std::uint64_t total_time,
                  user_time,
                  system_time,
                  idle_time,
                  nice_time,
                  io_time,
                  irq_time,
                  soft_irq_time,
                  steal_time,
                  guest_time,
                  system_all_time,
                  idle_all_time;

    double total_percentage,
           nice_percentage,
           user_percentage,
           system_percentage,
           virt_percentage;
  };

  // Scope memory info from sysinfo
  struct sysinfo mem_info_;

  bool temp_fault_triggered_;

  LoadFaultState load_fault_state_;

  config_reader::ConfigReader config_params_;

  double temperature_scale_;

  float avg_load_high_value_;

  ff_msgs::MemStateStamped mem_state_msg_;

  int pub_queue_size_, update_freq_hz_, mem_avg_load_limit_;
  int assert_load_high_fault_timeout_sec_, clear_load_high_fault_timeout_sec_;

  ros::Publisher mem_state_pub_;
  ros::Timer reload_params_timer_, stats_timer_;
  ros::Timer assert_load_fault_timer_, clear_load_fault_timer_;
  std::string processor_name_;

  void AssertLoadHighFaultCallback(ros::TimerEvent const& te);

  void ClearLoadHighFaultCallback(ros::TimerEvent const& te);

  /** Collect usage stats about memory useage, calculate percentages
    * based on the last time this was called. You should call this in
    * regular intervals for the numbers to make sense over time. */
  int CollectLoadStats();

  void PublishStatsCallback(ros::TimerEvent const &te);
};

}  // namespace mem_monitor

#endif  // MEM_MONITOR_MEM_MONITOR_H_
