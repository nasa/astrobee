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

#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>

#include <cstdint>
#include <string>

namespace ff_util {

namespace fs = boost::filesystem;

FreeFlyerNodelet::FreeFlyerNodelet(
  std::string const& node, bool autostart_hb_timer) :
  nodelet::Nodelet(),
  autostart_hb_timer_(autostart_hb_timer),
  initialized_(false),
  sleeping_(false),
  heartbeat_queue_size_(5),
  node_(node) {
}

FreeFlyerNodelet::FreeFlyerNodelet(bool autostart_hb_timer) :
  nodelet::Nodelet(),
  autostart_hb_timer_(autostart_hb_timer),
  initialized_(false),
  node_("") {
}

FreeFlyerNodelet::~FreeFlyerNodelet() {
}

// Called directly by Gazebo and indirectly through onInit() by nodelet
void FreeFlyerNodelet::Setup(ros::NodeHandle & nh, ros::NodeHandle & nh_mt) {
  // Copy the node handles
  nh_ = nh;
  nh_mt_ = nh_mt;

  // Get the platform name from the node handle (roslaunch group name attribute)
  if (nh_.getNamespace().length() > 1)
    platform_ = nh_.getNamespace().substr(1);

  // If not set, try and grab the node name from the launch file
  if (node_.empty()) {
    if (getName().empty()) {
      FF_FATAL("Node name not specified.");
    } else {
      // If a robot name has been specified, remove it from the node name
      if (platform_.size() > 1) {
        // Remove robot name from node name
        node_ = getName().substr((platform_.size() + 2), node_.size() - 1);
      } else {
        node_ = getName().substr(1, node_.size() - 1);
      }
    }
  }

  // Read in faults for this node
  param_config_.AddFile("faults.config");
  param_config_.AddFile("context.config");
  ReadConfig();

  // Setup the private node handles
  nh_private_ =
    ros::NodeHandle(ros::NodeHandle(nh_, PRIVATE_PREFIX), node_);
  nh_private_mt_ =
    ros::NodeHandle(ros::NodeHandle(nh_mt_, PRIVATE_PREFIX), node_);

  // Set node name and manager in heartbeat message since it won't change
  heartbeat_.node = node_;
  heartbeat_.nodelet_manager = ros::this_node::getName();

  // Immediately, setup a publisher for faults coming from this node
  // Topic needs to be latched for initialization faults
  pub_heartbeat_ = nh_.advertise<ff_msgs::Heartbeat>(
    TOPIC_HEARTBEAT, heartbeat_queue_size_, true);
  pub_diagnostics_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>(
    TOPIC_DIAGNOSTICS, 5);

  // Setup a heartbeat timer for this node if auto start was requested
  if (autostart_hb_timer_) {
    // Don't autostart until nodelet finishes initialization function
    timer_heartbeat_ = nh_.createTimer(ros::Rate(1.0),
      &FreeFlyerNodelet::HeartbeatCallback, this, false, false);
  }

  // Defer the initialization of the node to prevent a race condition with
  // nodelet registration. See this issue for more details:
  // > https://github.com/ros/nodelet_core/issues/46
  timer_deferred_init_ = nh_.createTimer(ros::Duration(0.1),
    &FreeFlyerNodelet::InitCallback, this, true, true);
}

// Called by the nodelet framework to initialize the nodelet. Note that this is
// *NOT* called by the Gazebo plugins They enter directly via a Setup(...) call.
void FreeFlyerNodelet::onInit() {
  Setup(getNodeHandle(), getMTNodeHandle());
}

void FreeFlyerNodelet::ReadConfig() {
  // Read fault config file into lua
  if (!param_config_.ReadFiles()) {
    FF_ERROR(node_ << ": Couldn't open faults.config! Make sure it " <<
             "is in the astrobee/config folder!");
    return;
  }

  // Read in the heartbeat topic queue size
  if (!param_config_.GetUInt("heartbeat_queue_size", &heartbeat_queue_size_)) {
    heartbeat_queue_size_ = 5;
  }

  // Check if there is a fault table for this node, some nodes may not have
  // faults
  if (param_config_.CheckValExists(node_.c_str())) {
      config_reader::ConfigReader::Table fault_table(&param_config_,
                                                     node_.c_str());
      int fault_id;
      std::string fault_key;
      for (int i = 1; i < (fault_table.GetSize() + 1); i++) {
        config_reader::ConfigReader::Table fault_entry(&fault_table, i);

        // Get the fault id
        if (!fault_entry.GetInt("id", &fault_id)) {
          FF_ERROR(node_ << ": Fault id at index " << i <<
                   " not specified in the node's fault table.");
          return;
        }

        // Get fault key
        if (!fault_entry.GetStr("key", &fault_key)) {
          FF_ERROR(node_ << ": Fault key at index " << i <<
                   " not specified in the node's fault table.");
          return;
        }

        // don't need description, only specified so developers know what the
        // fault refers to
        faults_[fault_key] = fault_id;
      }
  }

  return;
}

void FreeFlyerNodelet::AssertFault(std::string const& key,
                                   std::string const& message,
                                   ros::Time time_fault_occurred) {
  FF_WARN(node_ << ": Fault (" << key << ") :: " << message);
  bool found = false;
  unsigned int id;

  // Check to make sure the fault key is valid for this nodelet
  if (faults_.count(key) > 0) {
    id = faults_[key];
  } else {
    FF_ERROR(node_ << ": Asserting fault " << key << " failed! " <<
             "Fault doesn't exist for this node.");
    FF_ERROR("Message: " << message);
    return;
  }

  // Check to see if the fault is already being reported
  for (unsigned int i = 0; i < heartbeat_.faults.size() && !found; i++) {
    if (id == heartbeat_.faults[i].id) {
      // TODO(Katie) Maybe update message and re-publish faults
      found = true;
    }
  }

  // If fault not found, add it
  if (!found) {
    ff_msgs::Fault fault;
    fault.id = id;
    fault.time_of_fault = time_fault_occurred;
    fault.msg.assign(message, 0, 128);
    heartbeat_.faults.push_back(fault);
    PublishHeartbeat();
  }
}

void FreeFlyerNodelet::ClearAllFaults() {
  heartbeat_.faults.clear();
  // Publish heartbeat so that the system monitor is notified immediately
  PublishHeartbeat();
}

void FreeFlyerNodelet::ClearFault(std::string const& key) {
  unsigned int id;
  // Check to make sure the fault key is valid for this nodelet
  if (faults_.count(key) > 0) {
    id = faults_[key];
  } else {
    FF_ERROR(node_ << ": Clearing fault " << key << " failed! " <<
             "Fault doesn't exist for this node.");
    return;
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
}

void FreeFlyerNodelet::PrintFaults() {
  FF_INFO(node_ << "'s faults:");
  for (std::map<std::string, int>::iterator it = faults_.begin();
                                                    it != faults_.end(); ++it) {
    FF_INFO("  id: " << it->second << " key: " << it->first);
  }
}

// Not sure if we need this functionality but I added it just in case
void FreeFlyerNodelet::StopHeartbeat() {
  // Stop heartbeat timer
  timer_heartbeat_.stop();
}

void FreeFlyerNodelet::HeartbeatCallback(ros::TimerEvent const& ev) {
  double s = (ev.last_real - ev.last_expected).toSec();
  if (s > 1.0)
    ROS_INFO_STREAM(node_ << ": " << s);
  PublishHeartbeat();
}

void FreeFlyerNodelet::InitCallback(ros::TimerEvent const& ev) {
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
    timer_heartbeat_.start();
  }

  Reset();

  // Start a trigger service on the private nodehandle /platform/pvt/name
  srv_trigger_ = nh_private_.advertiseService(TOPIC_TRIGGER,
    &FreeFlyerNodelet::TriggerCallback, this);
}

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
    FF_WARN("Unknown trigger event" << req.event);
    return false;
  }
  FF_WARN("Invalid state transition for trigger " << req.event);
  return false;
}

void FreeFlyerNodelet::PublishHeartbeat() {
  if (initialized_) {
    heartbeat_.header.stamp = ros::Time::now();
    pub_heartbeat_.publish(heartbeat_);
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
  ds.name = node_;
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
  da.header.stamp = ros::Time::now();
  da.status.push_back(ds);
  pub_diagnostics_.publish(da);
}

// NodeHandle management
ros::NodeHandle* FreeFlyerNodelet::GetPlatformHandle(bool multithreaded) {
  return (multithreaded ? &nh_mt_ : &nh_);
}

// NodeHandle management
ros::NodeHandle* FreeFlyerNodelet::GetPrivateHandle(bool multithreaded) {
  return (multithreaded ? &nh_private_ : &nh_private_mt_);
}


std::string FreeFlyerNodelet::GetName() {
  return node_;
}

// Get the name of this node (mainly useful for drivers)
std::string FreeFlyerNodelet::GetPlatform() {
  return platform_;
}


}  // namespace ff_util
