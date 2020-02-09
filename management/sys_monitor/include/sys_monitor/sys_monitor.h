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

#include <nodelet/NodeletLoad.h>
#include <nodelet/NodeletUnload.h>
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
#include <ff_msgs/TimeDiffStamped.h>
#include <ff_msgs/UnloadLoadNodelet.h>
#include <ff_util/ff_faults.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_nodelet.h>

#include <cmath>
#include <map>
#include <string>
#include <vector>

using ff_msgs::CommandArg;

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

  void ChangeFaultErrMsg(unsigned int fault_id, std::string err_msg);

  void RemoveFault(unsigned int fault_id);

 protected:
  class Watchdog {
   public:
    Watchdog(SysMonitor *const sys_monitor,
             std::string const& node_name,
             ros::Duration const& timeout,
             uint const allowed_misses,
               uint const fault_id);
    uint fault_id();
    uint misses_allowed();
    ff_msgs::HeartbeatConstPtr previous_hb();
    bool hb_fault_occurring();
    bool heartbeat_started();
    bool unloaded();
    std::string nodelet_manager();
    std::string nodelet_name();
    std::string nodelet_type();
    void hb_fault_occurring(bool occurring);
    void nodelet_manager(std::string manager_name);
    void nodelet_name(std::string name);
    void nodelet_type(std::string type);
    void unloaded(bool is_unloaded);
    void ResetTimer();
    void StopTimer();
    void previous_hb(ff_msgs::HeartbeatConstPtr hb);
    void TimerCallBack(ros::TimerEvent const& te);

   private:
    ros::Timer timer_;
    SysMonitor *const monitor_;
    uint missed_count_;
    uint const misses_allowed_;
    uint const fault_id_;
    bool hb_fault_occurring_;
    bool heartbeat_started_;
    bool unloaded_;
    std::string nodelet_manager_;
    std::string nodelet_name_;
    std::string nodelet_type_;
    ff_msgs::HeartbeatConstPtr previous_hb_;
  };
  typedef std::shared_ptr<Watchdog> WatchdogPtr;

  struct Fault {
    Fault(std::string const& node_name_in,
          bool const blocking_in,
          bool const warning_in,
          ff_msgs::CommandStampedPtr response_in);
    std::string const node_name;
    bool const blocking;
    bool const warning;
    ff_msgs::CommandStampedConstPtr response;
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

  void AssertFault(ff_util::FaultKeys enum_key,
                   std::string const& message,
                   ros::Time time_fault_occurred = ros::Time::now());

  void ClearFault(ff_util::FaultKeys enum_key);

  virtual void Initialize(ros::NodeHandle *nh);

  void OutputFaultTables();

  void PublishCmd(ff_msgs::CommandStampedConstPtr cmd);

  void PublishFaultConfig();

  void PublishFaultState();

  void PublishFaultResponse(unsigned int fault_id);

  void PublishHeartbeatCallback(ros::TimerEvent const& te);

  void PublishHeartbeat(bool initialization_fault = false);

  void PublishTimeDiff(float time_diff_sec);

  void StartupTimerCallback(ros::TimerEvent const& te);

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

  bool NodeletService(ff_msgs::UnloadLoadNodelet::Request &req,
                      ff_msgs::UnloadLoadNodelet::Response &res);
  int LoadNodelet(ff_msgs::UnloadLoadNodelet::Request &req);
  int UnloadNodelet(std::string const& nodelet, std::string const& manager);

  ff_msgs::FaultState fault_state_;
  ff_msgs::FaultConfig fault_config_;
  ff_msgs::Heartbeat heartbeat_;

  nodelet::NodeletLoad load_service_;
  nodelet::NodeletUnload unload_service_;

  ros::NodeHandle nh_;
  ros::Publisher pub_cmd_, pub_heartbeat_;
  ros::Publisher pub_fault_config_, pub_fault_state_;
  ros::Publisher pub_time_diff_;
  ros::Timer reload_params_timer_, startup_timer_, heartbeat_timer_;
  ros::ServiceServer unload_load_nodelet_service_;
  ros::Subscriber sub_hb_;

  std::map<unsigned int, std::shared_ptr<Fault>> all_faults_;
  std::map<std::string, WatchdogPtr> watch_dogs_;

  // TODO(Katie) possibly remove this
  std::vector<std::string> unwatched_heartbeats_;

  std::string time_diff_node_;

  config_reader::ConfigReader config_params_;

  bool time_diff_fault_triggered_;
  int pub_queue_size_, sub_queue_size_;
  int num_current_blocking_fault_;
  unsigned int startup_time_, heartbeat_pub_rate_;
  float time_drift_thres_sec_;
};
}  // namespace sys_monitor

#endif  // SYS_MONITOR_SYS_MONITOR_H_
