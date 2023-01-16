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

#ifndef FF_UTIL_FF_COMPONENT_H_
#define FF_UTIL_FF_COMPONENT_H_

#include <ff_common/ff_ros.h>

#include <ff_util/ff_faults.h>
#include <ff_util/ff_names.h>
#include <ff_util/ff_timer.h>

#include <boost/filesystem.hpp>

#include <config_reader/config_reader.h>

#include <diagnostic_msgs/msg/key_value.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
namespace diagnostic_msgs {
typedef msg::KeyValue KeyValue;
typedef msg::DiagnosticStatus DiagnosticStatus;
typedef msg::DiagnosticArray DiagnosticArray;
}  // namespace diagnostic_msgs

#include <cstdint>
#include <map>
#include <string>
#include <vector>
#include <thread>
#include <chrono>

#include "ff_msgs/msg/fault.hpp"
#include "ff_msgs/msg/heartbeat.hpp"
#include "ff_msgs/srv/trigger.hpp"
namespace ff_msgs {
typedef msg::Fault Fault;
typedef msg::Heartbeat Heartbeat;
typedef srv::Trigger Trigger;
}  // namespace ff_msgs


// Constants
#define DEFAULT_ACTION_WAIT_TIME    30.0
#define DEFAULT_SERVICE_WAIT_TIME   30.0
#define DEFAULT_DIAGNOSTICS_RATE    1.0

namespace ff_util {

class FreeFlyerComponent {
 public:
  enum ResolveType : uint8_t {
    NAMESPACE = 0,
    TRANSFORM = 1,
    RESOURCE  = 1
  };

  // Use default name from freeflyer - crashes in ROS2
  // explicit FreeFlyerComponent(const rclcpp::NodeOptions & options, bool autostart_hb_timer = true);
  // Explicitly specift the name
  explicit FreeFlyerComponent(const rclcpp::NodeOptions& options, std::string const& name,
                            bool autostart_hb_timer = true);
  // Explicitly specify the node from gazebo
  explicit FreeFlyerComponent(std::string const& name, bool autostart_hb_timer = true);
  void FreeFlyerComponentGazeboInit(rclcpp::Node::SharedPtr node);


  // Necessary ROS2 function for components
  rclcpp::node_interfaces::NodeBaseInterface::SharedPtr get_node_base_interface() const {
    return node_->get_node_base_interface();
  }

  virtual ~FreeFlyerComponent();

  void AssertFault(FaultKeys enum_key,
                   std::string const& message,
                   ros::Time time_fault_occurred = rclcpp::Clock().now());
  void ClearAllFaults();
  void ClearFault(FaultKeys enum_key);
  void PrintFaults();

  // Get the name of this node (mainly useful for drivers)
  std::string GetName();
  std::string GetPlatform();
  std::string GetTransform(std::string const& child);

 protected:
  // Virtual methods that *can* be implemented by FF nodes. We don't make
  // these mandatory, as there is already a load callback in Gazebo.
  virtual void Initialize(NodeHandle node) {}
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
  void Setup(std::string plugin_name);

  std::map<std::string, int> faults_;

 private:
  void HeartbeatCallback();
  void InitCallback();
  void TriggerCallback(const std::shared_ptr<ff_msgs::Trigger::Request> req,
                       std::shared_ptr<ff_msgs::Trigger::Response> res);

  // Called in heartbeat callback or by nodes that do not to use the hb timer
  void PublishHeartbeat();

  // Called in onInit to read in the config values associated with the node
  void ReadConfig();

  // Node
  rclcpp::Node::SharedPtr node_;
  rclcpp::Clock::SharedPtr clock_;

  // Heartbeat autostart
  bool autostart_hb_timer_;
  bool initialized_;
  bool sleeping_;

  unsigned int heartbeat_queue_size_;

  config_reader::ConfigReader param_config_;

  // Heartbeat message, also used to report faults
  ff_msgs::Heartbeat heartbeat_;


  ff_util::FreeFlyerTimer timer_heartbeat_;
  ff_util::FreeFlyerTimer timer_deferred_init_;

  // Publishers
  Publisher<ff_msgs::Heartbeat> pub_heartbeat_;
  Publisher<diagnostic_msgs::DiagnosticArray> pub_diagnostics_;

  // Reset service
  Service<ff_msgs::Trigger> srv_trigger_;

  // Name and subsystem
  std::string platform_;
  std::string node_name_;
};

}  // namespace ff_util

#endif  // FF_UTIL_FF_COMPONENT_H_
