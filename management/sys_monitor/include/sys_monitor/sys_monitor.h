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

#ifndef SYS_MONITOR_SYS_MONITOR_H_
#define SYS_MONITOR_SYS_MONITOR_H_

#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <ros/package.h>

#include <config_reader/config_reader.h>
#include <ff_msgs/CommandConstants.h>
#include <ff_msgs/CommandStamped.h>
#include <ff_msgs/Heartbeat.h>
#include <ff_msgs/Fault.h>
#include <ff_msgs/FaultConfig.h>
#include <ff_msgs/FaultInfo.h>
#include <ff_msgs/FaultState.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>

#include <map>
#include <string>
#include <vector>

using ff_msgs::CommandArg;

#define DEBUG false

namespace sys_monitor {
class SysMonitor : public ff_util::FreeFlyerNodelet {
 public:
  SysMonitor();
  ~SysMonitor();

  /** 
   * Add fault to fault state and publish fault state.
   * @param fault_id        unique fault id
   * @param fault_msg       message containing fault information   
   * @param time_occurred   time fault occurred
   */
  void AddFault(unsigned int fault_id, std::string const& fault_msg = "",
                ros::Time time_occurred = ros::Time::now());

  void AddFault(ff_msgs::Fault const& fault, bool check_added = false);

  void RemoveFault(unsigned int fault_id);

 protected:
  class Watchdog {
   public:
    Watchdog(SysMonitor *const sys_monitor, ros::Duration const& timeout,
             uint const allowed_misses, uint const fault_id);
    uint GetFaultId();
    ff_msgs::HeartbeatConstPtr GetPreviousHeartbeat();
    bool IsFaultOccurring();
    void OutputHeartbeatFault(std::string node_name);
    void ResetFaultOccurring();
    void ResetTimer();
    void SetPreviousHeartbeat(ff_msgs::HeartbeatConstPtr hb);
    void TimerCallBack(ros::TimerEvent const& te);
   private:
    ros::Timer timer_;
    SysMonitor *const monitor_;
    uint missed_count_;
    uint const missed_allowed_;
    uint const fault_id_;
    bool hb_fault_occurring_;
    ff_msgs::HeartbeatConstPtr previous_hb_;
  };
  typedef std::shared_ptr<Watchdog> WatchdogPtr;

  struct Fault {
    Fault(std::string const& node_name, bool const blocking,
                                          ff_msgs::CommandStampedPtr response);
    std::string const node_name_;
    bool const blocking_;
    ff_msgs::CommandStampedConstPtr response_;
  };

  /**
   * Add a watchdog for node_name if one does not exist
   * @param node_name       unique node name, use ros::this_node::getName()
   * @param timeout         watchdog timer timeout
   * @param allowed_misses  allowable missed watchdog timeouts
   * @param fault_id        unique fault id
   */
  void AddWatchDog(ros::Duration const& timeout, std::string const& node_name,
                   uint const allowed_misses, uint const fault_id);

  void SetFaultState(unsigned int fault_id, bool adding_fault);

  /**
   * Heartbeat callback will reset watchdog for the sender's node
   * @param heartbeat received message
   */
  void HeartbeatCallback(ff_msgs::HeartbeatConstPtr const& heartbeat);

  virtual void Initialize(ros::NodeHandle *nh);

  void OutputFaultTables();

  void PublishCmd(ff_msgs::CommandStampedConstPtr cmd);

  void PublishFaultConfig();

  void PublishFaultState();

  void PublishFaultResponse(unsigned int fault_id);

  /**
   * Read params will read in all the parameters from the lua config files.
   * When reloading parameters with this function, the watch dog will be cleared
   * and restarted. False is returned if unable to read in config files.
   */
  bool ReadParams();

  /**
   * Read command will read in the command name and arguments to a command.
   * If no command is specified, false is returned.
   */
  bool ReadCommand(config_reader::ConfigReader::Table *entry,
                   ff_msgs::CommandStampedPtr cmd);

  ff_msgs::FaultState fault_state_;
  ff_msgs::FaultConfig fault_config_;

  ros::NodeHandle nh_;
  ros::Publisher pub_cmd_;
  ros::Publisher pub_fault_config_, pub_fault_state_;
  ros::Timer reload_params_timer_;
  ros::Subscriber sub_hb_;

  std::map<unsigned int, std::shared_ptr<Fault>> all_faults_;
  std::map<std::string, WatchdogPtr> watch_dogs_;

  // TODO(Katie) possibly remove this
  std::vector<std::string> unwatched_heartbeats_;

  config_reader::ConfigReader config_params_;

  int pub_queue_size_;
  int sub_queue_size_;
  int num_current_blocking_fault_;
};
}  // namespace sys_monitor

#endif  // SYS_MONITOR_SYS_MONITOR_H_
