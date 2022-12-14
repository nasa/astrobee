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


#include <ff_util/ff_nodelet.h>

#include <boost/filesystem.hpp>

#if ROS1

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <diagnostic_msgs/DiagnosticStatus.h>

#else
#include "diagnostic_msgs/msg/diagnostic_status.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("ff_nodelet");
#endif

#include <cstdint>
#include <string>

namespace ff_util {

namespace fs = boost::filesystem;


#if ROS1
FreeFlyerNodelet::FreeFlyerNodelet(std::string const& node, bool autostart_hb_timer) :
  nodelet::Nodelet(),
  autostart_hb_timer_(autostart_hb_timer),
  initialized_(false),
  sleeping_(false),
  heartbeat_queue_size_(5),
  node_name_(node) {
}

FreeFlyerNodelet::FreeFlyerNodelet(bool autostart_hb_timer) :
  nodelet::Nodelet(),
  autostart_hb_timer_(autostart_hb_timer),
  initialized_(false),
  node_name_("") {
}
#else
FreeFlyerNodelet::FreeFlyerNodelet(
  const rclcpp::NodeOptions & options, std::string const& node, bool autostart_hb_timer) :
  node_(std::make_shared<rclcpp::Node>(node, options)),
  autostart_hb_timer_(autostart_hb_timer),
  initialized_(false),
  sleeping_(false),
  heartbeat_queue_size_(5),
  node_name_(node) {
}

FreeFlyerNodelet::FreeFlyerNodelet(const rclcpp::NodeOptions & options, bool autostart_hb_timer) :
  node_(std::make_shared<rclcpp::Node>("", options)),
  autostart_hb_timer_(autostart_hb_timer),
  initialized_(false),
  node_name_("") {
}
#endif

FreeFlyerNodelet::~FreeFlyerNodelet() {
}

#if ROS1
// Called directly by Gazebo and indirectly through onInit() by nodelet
void FreeFlyerNodelet::Setup(ros::NodeHandle & nh, ros::NodeHandle & nh_mt, std::string plugin_name) {
  // Copy the node handles
  nh_ = nh;
  nh_mt_ = nh_mt;

  // Get the platform name from the node handle (roslaunch group name attribute)
  if (nh_.getNamespace().length() > 1)
    platform_ = nh_.getNamespace().substr(1);

  // If not set, try and grab the node name from the launch file
  if (node_name_.empty()) {
    if (plugin_name != "") {
      node_name_ = plugin_name;
    } else if (getName().empty()) {
      FF_FATAL_STREAM("Node name not specified.");
    } else {
      // If a robot name has been specified, remove it from the node name
      if (platform_.size() > 1) {
        // Remove robot name from node name
        node_name_ = getName().substr((platform_.size() + 2), node_name_.size() - 1);
      } else {
        node_name_ = getName().substr(1, node_name_.size() - 1);
      }
    }
  }

  // Read in faults for this node
  param_config_.AddFile("faults.config");
  param_config_.AddFile("context.config");
  ReadConfig();

  // Setup the private node handles
  nh_private_ =
    ros::NodeHandle(ros::NodeHandle(nh_, PRIVATE_PREFIX), node_name_);
  nh_private_mt_ =
    ros::NodeHandle(ros::NodeHandle(nh_mt_, PRIVATE_PREFIX), node_name_);

  // Set node name and manager in heartbeat message since it won't change
  heartbeat_.node = node_name_;
  heartbeat_.nodelet_manager = ros::this_node::getName();

  // Immediately, setup a publisher for faults coming from this node
  // Topic needs to be latched for initialization faults
  {
  // ROS_CREATE_PUBLISHER(pub_heartbeat_, ff_msgs::Heartbeat, TOPIC_HEARTBEAT, heartbeat_queue_size_);
  pub_heartbeat_ = nh_.advertise<ff_msgs::Heartbeat>(
    TOPIC_HEARTBEAT, heartbeat_queue_size_, true);
  }
  {
  ROS_CREATE_PUBLISHER(pub_diagnostics_, diagnostic_msgs::DiagnosticArray, TOPIC_DIAGNOSTICS, 5);
  }

  // Defer the initialization of the node to prevent a race condition with
  // nodelet registration. See this issue for more details:
  // > https://github.com/ros/nodelet_core/issues/46
    timer_deferred_init_ = nh_.createTimer(ros::Duration(0.1),
    &FreeFlyerNodelet::InitCallback, this, true, true);
}

#else  // ROS2

// Called directly by Gazebo and indirectly through onInit() by nodelet
void FreeFlyerNodelet::Setup(std::string plugin_name) {
  // Get the platform name from the node handle (roslaunch group name attribute)
  if (std::string(node_->get_namespace()).size() > 1)
    platform_ = std::string(node_->get_namespace()).substr(1);

  // If not set, try and grab the node name from the launch file
  if (node_name_.empty()) {
    if (plugin_name != "") {
      node_name_ = plugin_name;
    } else if (std::string(node_->get_name()).empty()) {
      FF_FATAL_STREAM("Node name not specified.");
    } else {
      // If a robot name has been specified, remove it from the node name
      if (platform_.size() > 1) {
        // Remove robot name from node name
        node_name_ = std::string(node_->get_name()).substr((platform_.size() + 2), node_name_.size() - 1);
      } else {
        node_name_ = std::string(node_->get_name()).substr(1, node_name_.size() - 1);
      }
    }
  }

  // Read in faults for this node
  param_config_.AddFile("faults.config");
  param_config_.AddFile("context.config");
  ReadConfig();

  // Set node name and manager in heartbeat message since it won't change
  heartbeat_.node = node_name_;
  // heartbeat_.nodelet_manager = ros::this_node::getName();

  // Immediately, setup a publisher for faults coming from this node
  // Topic needs to be latched!!!! for initialization faults
  ROS_CREATE_PUBLISHER(pub_heartbeat_, ff_msgs::Heartbeat, TOPIC_HEARTBEAT, heartbeat_queue_size_);
  // pub_heartbeat_ = nh_.advertise<ff_msgs::Heartbeat>(
  //   TOPIC_HEARTBEAT, heartbeat_queue_size_, true);
  ROS_CREATE_PUBLISHER(pub_diagnostics_, diagnostic_msgs::DiagnosticArray, TOPIC_DIAGNOSTICS, 5);

  // Defer the initialization of the node to prevent a race condition with
  // nodelet registration. See this issue for more details:
  // > https://github.com/ros/nodelet_core/issues/46
  ROS_CREATE_TIMER(timer_deferred_init_, 0.1,
      std::bind(&FreeFlyerNodelet::InitCallback, this), true, true);
}
#endif

#if ROS1
// Called by the nodelet framework to initialize the nodelet. Note that this is
// *NOT* called by the Gazebo plugins They enter directly via a Setup(...) call.
void FreeFlyerNodelet::onInit() {
  Setup(getNodeHandle(), getMTNodeHandle(), "");
}
#endif
void FreeFlyerNodelet::ReadConfig() {
  // Read fault config file into lua
  if (!param_config_.ReadFiles()) {
    FF_ERROR_STREAM(node_name_ << ": Couldn't open faults.config! Make sure it " <<
             "is in the astrobee/config folder!");
    return;
  }

  // Read in the heartbeat topic queue size
  if (!param_config_.GetUInt("heartbeat_queue_size", &heartbeat_queue_size_)) {
    heartbeat_queue_size_ = 5;
  }

  // Check if there is a fault table for this node, some nodes may not have
  // faults
  if (param_config_.CheckValExists(node_name_.c_str())) {
    config_reader::ConfigReader::Table fault_table(&param_config_,
                                                   node_name_.c_str());
    int fault_id;
    std::string fault_key;
    // Lua indices start at one
    for (int i = 1; i < (fault_table.GetSize() + 1); i++) {
      config_reader::ConfigReader::Table fault_entry(&fault_table, i);

      // Get the fault id
      if (!fault_entry.GetInt("id", &fault_id)) {
        FF_ERROR_STREAM(node_name_ << ": Fault id at index " << i <<
                 " not specified in the node's fault table.");
        return;
      }

      // Get fault key
      if (!fault_entry.GetStr("key", &fault_key)) {
        FF_ERROR_STREAM(node_name_ << ": Fault key at index " << i <<
                 " not specified in the node's fault table.");
        return;
      }

      // don't need description, only specified so developers know what the
      // fault refers to
      faults_[fault_key] = fault_id;
    }
  }

  // Check if there is a fault table for all nodes, this table contains faults
  // for all nodes
  if (param_config_.CheckValExists("all")) {
    config_reader::ConfigReader::Table all_fault_table(&param_config_, "all");
    int fault_id;
    std::string fault_key;
    // Lua indices start at one
    for (int i = 1; i < (all_fault_table.GetSize() + 1); i++) {
      config_reader::ConfigReader::Table all_fault_entry(&all_fault_table, i);

      // Get fault id
      if (!all_fault_entry.GetInt("id", &fault_id)) {
        FF_ERROR_STREAM(node_name_ << ": Fault id at index " << i <<
                 " not specified in the all fault table.");
        return;
      }

      // Get fault key
      if (!all_fault_entry.GetStr("key", &fault_key)) {
        FF_ERROR_STREAM(node_name_ <<": Fault key at index " << i <<
                 " not specified in the all fault table.");
        return;
      }

      // don't need description, only specified so developers know what the
      // fault refers to
      faults_[fault_key] = fault_id;
    }
  }

  return;
}

void FreeFlyerNodelet::AssertFault(FaultKeys enum_key,
                                   std::string const& message,
                                   ros::Time time_fault_occurred) {
  std::string key = fault_keys[enum_key];
  std::string err_msg;
  bool found = false;
  unsigned int id;

  // Check to make sure the fault key is valid for this nodelet
  if (faults_.count(key) > 0) {
    id = faults_[key];
    err_msg = message;
  } else {
    FF_ERROR_STREAM(node_name_ << ": Asserting fault " << key << " failed! " <<
             "Fault doesn't exist for this node. Trying to assert unknown " <<
             "key fault ...");
    // Check to make sure the fault key is valid
    if (faults_.count("UNKNOWN_FAULT_KEY") > 0) {
      id = faults_["UNKNOWN_FAULT_KEY"];
      err_msg = node_name_ + ": Fault key " + key + " wasn't recognized. " + message;
    } else {
      FF_ERROR_STREAM(node_name_ << ": Unable to assert unknown key fault!");
      return;
    }
  }

  // Check to see if the fault is already being reported
  for (unsigned int i = 0; i < heartbeat_.faults.size() && !found; i++) {
    if (id == heartbeat_.faults[i].id) {
      if (message != heartbeat_.faults[i].msg) {
        heartbeat_.faults[i].msg.assign(message, 0, 128);
      }
      found = true;
    }
  }

  // If fault not found, add it
  if (!found) {
    FF_WARN_STREAM(node_name_ << ": Fault (" << key << ") :: " << message);
    ff_msgs::Fault fault;
    fault.id = id;
    fault.time_of_fault = time_fault_occurred;
    fault.msg.assign(err_msg, 0, 128);
    heartbeat_.faults.push_back(fault);
    PublishHeartbeat();
  }
}

void FreeFlyerNodelet::ClearAllFaults() {
  heartbeat_.faults.clear();
  // Publish heartbeat so that the system monitor is notified immediately
  PublishHeartbeat();
}

void FreeFlyerNodelet::ClearFault(FaultKeys enum_key) {
  std::string key = fault_keys[enum_key];
  unsigned int id;
  // Check to make sure the fault key is valid for this nodelet
  if (faults_.count(key) > 0) {
    id = faults_[key];
  } else {
    FF_ERROR_STREAM(node_name_ << ": Clearing fault " << key << " failed! " <<
             "Fault doesn't exist for this node. Trying to clear unknown " <<
             "key fault ...");
    // Check to make sure the fault key is valid
    if (faults_.count("UNKNOWN_FAULT_KEY") > 0) {
      id = faults_["UNKNOWN_FAULT_KEY"];
    } else {
      FF_ERROR_STREAM(node_name_ << ": Unable to clear unknown key fault!");
      return;
    }
  }

  // Find fault and remove from list, It's okay if the fault wasn't in the list
  for (unsigned int i = 0; i < heartbeat_.faults.size(); i++) {
    if (id == heartbeat_.faults[i].id) {
      heartbeat_.faults.erase(heartbeat_.faults.begin() + i);
      // Publish heartbeat so that the system monitor is notified immediately
      PublishHeartbeat();
      break;
    }
  }
  FF_WARN_STREAM(node_name_ << ": Fault (" << key << ") has been cleared!");
}

void FreeFlyerNodelet::PrintFaults() {
  FF_INFO_STREAM(node_name_ << "'s faults:");
  for (std::map<std::string, int>::iterator it = faults_.begin();
                                                    it != faults_.end(); ++it) {
    FF_INFO_STREAM("  id: " << it->second << " key: " << it->first);
  }
}

// Not sure if we need this functionality but I added it just in case
void FreeFlyerNodelet::StopHeartbeat() {
  // Stop heartbeat timer
  #if ROS1
  timer_heartbeat_.STOP_TIMER();
  #else
  timer_heartbeat_->STOP_TIMER();

  #endif
}



#if ROS1
void FreeFlyerNodelet::HeartbeatCallback(ros::TimerEvent const& ev) {
  double s = (ev.last_real - ev.last_expected).toSec();
  if (s > 1.0)
    FF_INFO_STREAM(node_name_.c_str() << ": " << s);
  PublishHeartbeat();
}
void FreeFlyerNodelet::InitCallback(ros::TimerEvent const& ev) {
  ROS_ERROR("InitCallback");
  // Return a single threaded nodehandle by default
  initialized_ = false;
  Initialize(&nh_);
  initialized_ = true;

  // Check if there was an initialization fault and send the heartbeat if there
  // was
  if (heartbeat_.faults.size() > 0) {
    PublishHeartbeat();
    return;
  }

  // Start timer that was setup earlier
  if (autostart_hb_timer_) {
    // timer_heartbeat_ = nh_.createTimer(ros::Rate(1.0),
    //   &FreeFlyerNodelet::HeartbeatCallback, this, false, true);
    // ROS_CREATE_TIMER(timer_heartbeat_, 1.0, std::bind(&FreeFlyerNodelet::HeartbeatCallback, this), false, true);
    timer_heartbeat_ = nh_.createTimer(ros::Rate(1.0),
      &FreeFlyerNodelet::HeartbeatCallback, this, false, true);
  }

  Reset();

  // Start a trigger service on the private nodehandle /platform/pvt/name
  // srv_trigger_ = nh_private_.advertiseService(TOPIC_TRIGGER,
  //   &FreeFlyerNodelet::TriggerCallback, this);
  ROS_CREATE_SERVICE(srv_trigger_, ff_msgs::Trigger, TOPIC_TRIGGER,
                     &FreeFlyerNodelet::TriggerCallback);
  ROS_ERROR("InitCallback end");
}
#else
void FreeFlyerNodelet::HeartbeatCallback() {
  // double s = (ev.last_real - ev.last_expected).toSec();
  // if (s > 1.0)
  //   FF_INFO_STREAM(node_name_.c_str() << ": " << s);
  PublishHeartbeat();
}
void FreeFlyerNodelet::InitCallback() {
  // Return a single threaded nodehandle by default
  initialized_ = false;
  Initialize(node_);
  initialized_ = true;

  // Check if there was an initialization fault and send the heartbeat if there
  // was
  if (heartbeat_.faults.size() > 0) {
    PublishHeartbeat();
    return;
  }

  // Start timer that was setup earlier
  if (autostart_hb_timer_) {
    timer_heartbeat_ = rclcpp::create_timer(node_, node_->get_clock(), rclcpp::Duration(1.0),
        std::bind(&FreeFlyerNodelet::HeartbeatCallback, this));
  }

  Reset();

  // Start a trigger service on the private nodehandle /platform/pvt/name
  ROS_CREATE_SERVICE(srv_trigger_, ff_msgs::Trigger, TOPIC_TRIGGER,
                     &FreeFlyerNodelet::TriggerCallback);
}
#endif

#if ROS1
bool FreeFlyerNodelet::TriggerCallback(
  ff_msgs::Trigger::Request &req, ff_msgs::Trigger::Response &res) {
  switch (req.event) {
  // Allow a reset from woken state only
  case ff_msgs::Trigger::Request::RESTART:
    if (!sleeping_) {
      ClearAllFaults();
      Reset();
      return true;
    }
    break;
  // Allow sleep from woken state only
  case ff_msgs::Trigger::Request::SLEEP:
    if (!sleeping_) {
      Sleep();
      sleeping_ = true;
      return true;
    }
    break;
  // Allow wakeup from sleeping state only
  case ff_msgs::Trigger::Request::WAKEUP:
    if (sleeping_) {
      Wakeup();
      sleeping_ = false;
      return true;
    }
    break;
  // For all other events that might be sent incorrectly
  default:
    FF_WARN_STREAM("Unknown trigger event" << req.event);
    return false;
  }
  FF_WARN_STREAM("Invalid state transition for trigger " << req.event);
  return false;
}
#else
void FreeFlyerNodelet::TriggerCallback(const std::shared_ptr<ff_msgs::Trigger::Request> req,
                                             std::shared_ptr<ff_msgs::Trigger::Response> res) {
  switch (req->event) {
  // Allow a reset from woken state only
  case ff_msgs::Trigger::Request::RESTART:
    if (!sleeping_) {
      ClearAllFaults();
      Reset();
      return;
    }
    break;
  // Allow sleep from woken state only
  case ff_msgs::Trigger::Request::SLEEP:
    if (!sleeping_) {
      Sleep();
      sleeping_ = true;
      return;
    }
    break;
  // Allow wakeup from sleeping state only
  case ff_msgs::Trigger::Request::WAKEUP:
    if (sleeping_) {
      Wakeup();
      sleeping_ = false;
      return;
    }
    break;
  // For all other events that might be sent incorrectly
  default:
    FF_WARN_STREAM("Unknown trigger event" << req->event);
    return;
  }
  FF_WARN_STREAM("Invalid state transition for trigger " << req->event);
  return;
}
#endif


void FreeFlyerNodelet::PublishHeartbeat() {
  if (initialized_) {
    heartbeat_.header.stamp = ROS_TIME_NOW();
    #if ROS1
    pub_heartbeat_.publish(heartbeat_);
    #else
    pub_heartbeat_->publish(heartbeat_);
    #endif
  }
}

void FreeFlyerNodelet::SendDiagnostics(
  const std::vector<diagnostic_msgs::KeyValue> &keyval) {
  // Setup the diagnostics skeleton
  diagnostic_msgs::DiagnosticStatus ds;
  if (heartbeat_.faults.size() == 0) {
    ds.level = diagnostic_msgs::DiagnosticStatus::OK;
    ds.message = "Node is operating nominally";
  } else {
    ds.level = diagnostic_msgs::DiagnosticStatus::ERROR;
    ds.message = "Node is in a fault state";
  }
  ds.name = node_name_;
  if (!platform_.empty())
    ds.name = platform_ + "::" + ds.name;
  ds.values = keyval;
  // Append the faults to the KV
  for (unsigned int i = 0; i < heartbeat_.faults.size(); i++) {
    diagnostic_msgs::KeyValue fault;
    fault.key = "FAULT" + std::to_string(i+1);
    // Find message in key value pairs
    for (unsigned int j = 0; j < heartbeat_.faults[i].data.size(); j++) {
      if (heartbeat_.faults[i].data[j].key.compare("message") == 0) {
        fault.value = heartbeat_.faults[i].data[j].s + "(" +
                                  std::to_string(heartbeat_.faults[i].id) + ")";
        break;
      }
    }
    ds.values.push_back(fault);
  }
  // Publish the diagnostics
  diagnostic_msgs::DiagnosticArray da;
  da.header.stamp = ROS_TIME_NOW();
  da.status.push_back(ds);
  pub_diagnostics_->publish(da);
}

// NodeHandle management
#if ROS1
ros::NodeHandle* FreeFlyerNodelet::GetPlatformHandle(bool multithreaded) {
  return (multithreaded ? &nh_mt_ : &nh_);
}

// NodeHandle management
ros::NodeHandle* FreeFlyerNodelet::GetPrivateHandle(bool multithreaded) {
  return (multithreaded ? &nh_private_ : &nh_private_mt_);
}
#endif

std::string FreeFlyerNodelet::GetName() {
  return node_name_;
}

// Get the name of this node (mainly useful for drivers)
std::string FreeFlyerNodelet::GetPlatform() {
  return platform_;
}

// Get a platform prefixed frame name
std::string FreeFlyerNodelet::GetTransform(std::string const& child) {
  std::string frame = child;
  if (!platform_.empty())
    frame = platform_ + "/" + child;
  return frame;
}

}  // namespace ff_util
