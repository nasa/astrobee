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

#ifndef FF_UTIL_FF_NODELET_H_
#define FF_UTIL_FF_NODELET_H_

// ROS includes
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>
#include <diagnostic_msgs/KeyValue.h>

#include <config_reader/config_reader.h>

#include <ff_msgs/Fault.h>
#include <ff_msgs/Heartbeat.h>
#include <ff_msgs/Trigger.h>

#include <ff_util/ff_faults.h>
#include <ff_util/ff_names.h>

#include <map>
#include <string>
#include <vector>
#include <thread>

// Constants
#define DEFAULT_ACTION_WAIT_TIME    30.0
#define DEFAULT_SERVICE_WAIT_TIME   30.0
#define DEFAULT_DIAGNOSTICS_RATE    1.0

// Logging wrappers
#define FF_DEBUG(msg) NODELET_DEBUG_STREAM(msg)
#define FF_INFO(msg) NODELET_INFO_STREAM(msg)
#define FF_WARN(msg) NODELET_WARN_STREAM(msg)
#define FF_ERROR(msg) NODELET_ERROR_STREAM(msg)
#define FF_FATAL(msg) NODELET_FATAL_STREAM(msg)

namespace ff_util {

class FreeFlyerNodelet : public nodelet::Nodelet {
 public:
  enum ResolveType : uint8_t {
    NAMESPACE = 0,
    TRANSFORM = 1,
    RESOURCE  = 1
  };

  // Use default name from freeflyer
  explicit FreeFlyerNodelet(bool autostart_hb_timer = true);
  // Explicitly specift the name
  explicit FreeFlyerNodelet(std::string const& name, bool autostart_hb_timer = true);
  virtual ~FreeFlyerNodelet();

  void AssertFault(FaultKeys enum_key,
                   std::string const& message,
                   ros::Time time_fault_occurred = ros::Time::now());
  void ClearAllFaults();
  void ClearFault(FaultKeys enum_key);
  void PrintFaults();

  // NodeHandle management
  ros::NodeHandle* GetPlatformHandle(bool multithreaded = false);
  ros::NodeHandle* GetPrivateHandle(bool multithreaded = false);

  // Get the name of this node (mainly useful for drivers)
  std::string GetName();
  std::string GetPlatform();
  std::string GetTransform(std::string const& child);

 protected:
  // Virtual methods that *can* be implemented by FF nodes. We don't make
  // these mandatory, as there is already a load callback in Gazebo.
  virtual void Initialize(ros::NodeHandle *nh) {}
  virtual void Reset() {}
  virtual void Sleep() {}
  virtual void Wakeup() {}

  // Stop the heartbeat messages
  void StopHeartbeat();

  // Diagnostic management
  void SendDiagnostics(const std::vector<diagnostic_msgs::KeyValue> &keyval);

  // The set function does all of the internal work. We have moved this out
  // of the onInit() call, so that it can be invoked when a nodelet is not used
  // for example, in simulation, where the dynamic loading is within gazebo...
  void Setup(ros::NodeHandle & nh, ros::NodeHandle & nh_mt);

 private:
  // Called on a heartbeat event
  void HeartbeatCallback(ros::TimerEvent const& ev);

  // Called when nodelet should be initialized
  void InitCallback(ros::TimerEvent const& ev);

  // Called when a trigger action is called
  bool TriggerCallback(ff_msgs::Trigger::Request &req, ff_msgs::Trigger::Response &res);

  // Called in heartbeat callback or by nodes that do not to use the hb timer
  void PublishHeartbeat();

  // We capture the init function and start up heartbeats, etc, then call Initialize()
  void onInit();

  // Called in onInit to read in the config values associated with the node
  void ReadConfig();

  // Heartbeat autostart
  bool autostart_hb_timer_;
  bool initialized_;
  bool sleeping_;

  unsigned int heartbeat_queue_size_;

  config_reader::ConfigReader param_config_;

  // Heartbeat message, also used to report faults
  ff_msgs::Heartbeat heartbeat_;

  // Node handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_mt_;
  ros::NodeHandle nh_private_;
  ros::NodeHandle nh_private_mt_;

  // Timers
  ros::Timer timer_heartbeat_;
  ros::Timer timer_deferred_init_;

  // Publishers
  ros::Publisher pub_heartbeat_;
  ros::Publisher pub_diagnostics_;

  // Reset service
  ros::ServiceServer srv_trigger_;

  std::map<std::string, int> faults_;

  // Name and subsystem
  std::string platform_;
  std::string node_;
};

}  // namespace ff_util

#endif  // FF_UTIL_FF_NODELET_H_
