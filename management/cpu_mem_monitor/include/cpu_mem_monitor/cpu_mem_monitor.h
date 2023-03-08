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


#ifndef CPU_MEM_MONITOR_CPU_MEM_MONITOR_H_
#define CPU_MEM_MONITOR_CPU_MEM_MONITOR_H_

#include <pluginlib/class_list_macros.h>

#include <ros/ros.h>
#include <ros/package.h>

#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include <sys/types.h>
#include <sys/sysinfo.h>

#include <config_reader/config_reader.h>
#include <cpu_mem_monitor/cpu.h>
#include <ff_msgs/CpuState.h>
#include <ff_msgs/CpuStateStamped.h>
#include <ff_msgs/MemState.h>
#include <ff_msgs/MemStateStamped.h>
#include <ff_common/ff_names.h>
#include <ff_util/ff_nodelet.h>

#include <cstdint>
#include <string>
#include <vector>
#include <map>

namespace cpu_mem_monitor {

enum LoadFaultState {
  ASSERTING,
  ASSERTED,
  CLEARING,
  CLEARED
};

class CpuMemMonitor : public ff_util::FreeFlyerNodelet {
 public:
  /**
   * CPU Memory monitor allocate, register, initialize model
   */
  CpuMemMonitor();
  /**
   * destruct model
   */
  ~CpuMemMonitor();

 protected:
  virtual void Initialize(ros::NodeHandle *nh);
  bool ReadParams();

 private:
  // Get the PIDs of the nodes to monitor
  void GetPIDs(ros::TimerEvent const &te);

  // Assert CPU loads and report if too high
  void AssertCPULoadHighFaultCallback(ros::TimerEvent const& te);
  void ClearCPULoadHighFaultCallback(ros::TimerEvent const& te);

  // Assert Memory loads and report if too high
  void AssertMemLoadHighFaultCallback(ros::TimerEvent const& te);
  void ClearMemLoadHighFaultCallback(ros::TimerEvent const& te);

  // Assert the collected CPU and Memory status
  void AssertCpuStats();
  void AssertMemStats();

  // Collect usage stats about all the CPUs and memory, calculate
  // percentages based on the last time this was called. You should
  // call this in regular intervals for the numbers to make sense over time.
  int CollectCPUStats();
  int CollectMemStats();

  // Callback to scan and publish the CPU and Memory stats
  void PublishStatsCallback(ros::TimerEvent const &te);

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

  config_reader::ConfigReader config_params_;
  ros::Publisher cpu_state_pub_;            // Cpu stats publisher
  ros::Publisher mem_state_pub_;            // Memory stats publisher
  ros::Timer reload_params_timer_;          // Ckeck if parameters were updated
  ros::Timer pid_timer_;                    // Update PIDs
  ros::Timer stats_timer_;                  // Update stats
  ros::Timer assert_cpu_load_fault_timer_;  // Check cpu load limits
  ros::Timer clear_cpu_load_fault_timer_;   // Clear cpu fault timer
  ros::Timer assert_mem_load_fault_timer_;  // Check memory load limits
  ros::Timer clear_mem_load_fault_timer_;   // Clear memory fault timer
  int pub_queue_size_;                      // Monitor publishing queue size
  double update_freq_hz_, update_pid_hz_;   // Publishing and PID update frequency
  struct sysinfo mem_info_;                 // Scope memory info from sysinfo

  unsigned int ncpus_;          // Number of cpu's
  std::string processor_name_;  // Processor running this monitor (mlp,llp)

  std::string monitor_host_;     // Either mlp, llp, hlp

  std::map<std::string, int> nodes_pid_;             // Map PID values
  std::map<std::string, uint64_t> nodes_proc_time_;  // Map proc time values

  // CPU Monitor Parameters
  bool temp_fault_triggered_;                // Temperature fault triggered
  Cpu freq_cpus_;
  double temperature_scale_;                 // Scale of temperature
  float avg_cpu_load_high_value_;
  std::vector<Load> load_cpus_;
  int cpu_avg_load_limit_, cpu_temp_limit_;

  // Memory Monitor Parameters
  float mem_load_value_, avg_mem_load_high_value_, mem_load_limit_;


  // Status messages to publish
  ff_msgs::CpuStateStamped cpu_state_msg_;
  ff_msgs::MemStateStamped mem_state_msg_;

  // Asserting faults
  LoadFaultState load_fault_state_;
  int assert_load_high_fault_timeout_sec_, clear_load_high_fault_timeout_sec_;
};


int ParseLine(char* line) {
  // This assumes that a digit will be found and the line ends in " Kb".
  int i = strlen(line);
  const char* p = line;
  for (; (*p < '0' || *p > '9') && *p != '\0'; ++p) {}
  line[i - 3] = '\0';
  i = atoi(p);
  return i;
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

}  // namespace cpu_mem_monitor

#endif  // CPU_MEM_MONITOR_CPU_MEM_MONITOR_H_
