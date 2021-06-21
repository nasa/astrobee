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

#include "sys_monitor/sys_monitor.h"

namespace sys_monitor {
SysMonitor::SysMonitor() :
  ff_util::FreeFlyerNodelet(NODE_SYS_MONITOR, false),
  time_diff_node_("imu_aug"),
  time_diff_fault_triggered_(false),
  pub_queue_size_(10),
  sub_queue_size_(100),
  time_drift_thres_sec_(0.25) {
}

SysMonitor::~SysMonitor() {
}



void SysMonitor::AddFault(unsigned int fault_id,
                          std::string const& fault_msg,
                          ros::Time time_occurred) {
  bool found = false;
  // Check to make sure fault is not already in the fault state
  for (uint i = 0; i < fault_state_.faults.size(); i++) {
    if (fault_id == fault_state_.faults[i].id) {
      found = true;
      break;
    }
  }

  // If fault isn't in fault state, add it and publish fault state
  if (!found) {
    ROS_ERROR("Fault with id %i occurred. Fault error message: %s", fault_id,
                                                            fault_msg.c_str());
    ff_msgs::Fault fault;
    fault.id = fault_id;
    fault.time_of_fault = time_occurred;
    fault.msg.assign(fault_msg, 0, 128);

    fault_state_.faults.push_back(fault);
    SetFaultState(fault_id, true);

    PublishFaultState();
  }
}

void SysMonitor::AddFault(ff_msgs::Fault const& fault, bool check_added) {
  unsigned int fault_id = fault.id;
  bool found = false;
  if (check_added) {
    // Check to see if the fault has already been added
    for (uint i = 0; i < fault_state_.faults.size(); i++) {
      if (fault_id == fault_state_.faults[i].id) {
        found = true;
        break;
      }
    }
  }

  if (!found) {
    ROS_ERROR("Fault with id %i occurred. Fault error message: %s", fault_id,
                                                            fault.msg.c_str());
    fault_state_.faults.push_back(fault);
    SetFaultState(fault_id, true);

    PublishFaultState();
  }
}

void SysMonitor::ChangeFaultErrMsg(unsigned int fault_id, std::string err_msg) {
  for (unsigned int i = 0; i < fault_state_.faults.size(); i++) {
    if (fault_id == fault_state_.faults[i].id) {
      fault_state_.faults[i].msg = err_msg;
    }
  }
  PublishFaultState();
}

void SysMonitor::RemoveFault(unsigned int fault_id) {
  for (unsigned int i = 0; i < fault_state_.faults.size(); i++) {
    if (fault_id == fault_state_.faults[i].id) {
      fault_state_.faults.erase(fault_state_.faults.begin() + i);
      break;
    }
  }

  SetFaultState(fault_id, false);

  PublishFaultState();
}

void SysMonitor::AddWatchDog(ros::Duration const& timeout,
                             std::string const& node_name,
                             uint const allowed_misses,
                             uint const fault_id) {
  if (!watch_dogs_.count(node_name)) {
    SysMonitor::WatchdogPtr watchDog(new SysMonitor::Watchdog(this,
                                                              node_name,
                                                              timeout,
                                                              allowed_misses,
                                                              fault_id));
    watch_dogs_.emplace(node_name, watchDog);
  } else {
    NODELET_INFO("AddWatchDog() already exists for %s",
                                                          node_name.c_str());
  }
}

void SysMonitor::SetFaultState(unsigned int fault_id, bool adding_fault) {
  // If adding or removing a blocking fault, increment the counter appropriately
  if (all_faults_.count(fault_id) > 0) {
    if (all_faults_.at(fault_id)->blocking) {
      if (adding_fault) {
        num_current_blocking_fault_++;
      } else {
        num_current_blocking_fault_--;
      }
    }
  } else {
    NODELET_ERROR("Fault %i wasn't found when setting fault state.", fault_id);
  }

  // Set the state appropriately
  if (fault_state_.faults.size() > 0) {
    if (num_current_blocking_fault_ > 0) {
      fault_state_.state = ff_msgs::FaultState::BLOCKED;
    } else if (num_current_blocking_fault_ < 0) {
      // This should never happen but add output just in case
      NODELET_ERROR("Number of blocking faults is negative!!");
    } else {
      fault_state_.state = ff_msgs::FaultState::FAULT;
    }
  } else {  // No faults
    fault_state_.state = ff_msgs::FaultState::FUNCTIONAL;
  }
}

void SysMonitor::HeartbeatCallback(ff_msgs::HeartbeatConstPtr const& hb) {
  uint i = 0, j = 0, tmp_id;
  bool fault_found = true;

  // Check to see if node heartbeat is set up in watchdogs
  if (watch_dogs_.count(hb->node) > 0) {
    WatchdogPtr wd = watch_dogs_.at(hb->node);
    wd->ResetTimer();
    if (wd->nodelet_manager() == "") {
      wd->nodelet_manager(hb->nodelet_manager);
    }

    // Check to see if node restarted publishing its heartbeat
    if (wd->hb_fault_occurring()) {
      wd->hb_fault_occurring(false);
      RemoveFault(wd->fault_id());
    }

    // Check time drift, use time in imu_aug heartbeat
    if (hb->node == time_diff_node_) {
      float time_diff_sec = (ros::Time::now() - hb->header.stamp).toSec();
      PublishTimeDiff(time_diff_sec);
      // Check if time difference is great than threshold. If it is, trigger
      // fault
      if (abs(time_diff_sec) > time_drift_thres_sec_) {
        if (!time_diff_fault_triggered_) {
          std::string key = ff_util::fault_keys[ff_util::TIME_DIFF_TOO_HIGH];
          unsigned int id = faults_[key];
          AddFault(id, ("Time diff is: " +
                        std::to_string(abs(time_diff_sec))));
          PublishFaultResponse(id);
          time_diff_fault_triggered_ = true;
        }
      } else {
        if (time_diff_fault_triggered_) {
          std::string key = ff_util::fault_keys[ff_util::TIME_DIFF_TOO_HIGH];
          unsigned int id = faults_[key];
          RemoveFault(id);
          time_diff_fault_triggered_ = false;
        }
      }
    }

    // Get last heartbeat for fault comparison
    ff_msgs::HeartbeatConstPtr previous_hb = wd->previous_hb();

    // Check if nodelet was unloaded and has restarted. Need to remove faults
    // from previous heartbeat
    if (wd->unloaded()) {
      wd->unloaded(false);
      for (i = 0; i < previous_hb->faults.size(); i++) {
        RemoveFault(previous_hb->faults[i].id);
      }
      previous_hb = NULL;
    }
    // Set previous hb to null so that we don't compare the faults with the last
    // time the nodelet was run

    // Check to see if this is the first heartbeat from the node
    if (!previous_hb) {
      wd->previous_hb(hb);
      // Add all the faults in the heartbeat
      for (i = 0; i < hb->faults.size(); i++) {
        AddFault(hb->faults[i]);
        PublishFaultResponse(hb->faults[i].id);
      }
      return;
    }

    // Check to see if there were and are no faults
    if (previous_hb->faults.size() == 0 && hb->faults.size() == 0) {
      wd->previous_hb(hb);
      return;
    }

    // Check to see if faults were added
    for (i = 0; i < hb->faults.size(); i++) {
      fault_found = false;
      tmp_id = hb->faults[i].id;
      for (j = 0; j < previous_hb->faults.size(); j++) {
        if (tmp_id == previous_hb->faults[j].id) {
          fault_found = true;
          if (previous_hb->faults[j].msg != hb->faults[i].msg) {
            ChangeFaultErrMsg(tmp_id, hb->faults[i].msg);
          }
          break;
        }
      }

      if (!fault_found) {
        // New fault needs to be added to the fault state and response needs
        // to be triggered
        AddFault(hb->faults[i]);
        PublishFaultResponse(tmp_id);
      }
    }

    // Check to see if faults were removed
    for (i = 0; i < previous_hb->faults.size(); i++) {
      fault_found = false;
      tmp_id = previous_hb->faults[i].id;
      for (j = 0; j < hb->faults.size(); j++) {
        if (tmp_id == hb->faults[j].id) {
          fault_found = true;
          break;
        }
      }

      if (!fault_found) {
        // Fault is no longer occurring so it needs to be removed from the
        // fault state
        RemoveFault(tmp_id);
      }
    }

    // Set previous heartbeat for fault comparison on next heartbeat
    wd->previous_hb(hb);
  } else {
    // TODO(Katie) possibly remove this
    bool found = false;
    for (unsigned int i = 0; i < unwatched_heartbeats_.size(); i++) {
      if (unwatched_heartbeats_[i] == hb->node) {
        found = true;
        break;
      }
    }

    if (!found) {
      NODELET_WARN("Heartbeat unrecognized from %s!", hb->node.c_str());
      unwatched_heartbeats_.push_back(hb->node);
    }
  }
}

void SysMonitor::Initialize(ros::NodeHandle *nh) {
  nh_ = *nh;

  // Set node name in heartbeat message
  heartbeat_.node = NODE_SYS_MONITOR;

  // Set nodelet manager name
  heartbeat_.nodelet_manager = ros::this_node::getName();

  // Heartbeat must be latching
  pub_heartbeat_ = nh_.advertise<ff_msgs::Heartbeat>(
                                        TOPIC_MANAGEMENT_SYS_MONITOR_HEARTBEAT,
                                        pub_queue_size_,
                                        true);

  // Add config files to config reader
  config_params_.AddFile("management/sys_monitor.config");
  config_params_.AddFile("management/fault_table.config");

  if (!ReadParams()) {
    return;
  }

  // Create a callback timer which checks to see if the config files have been
  // changed.
  reload_params_timer_ = nh_.createTimer(ros::Duration(1),
      [this](ros::TimerEvent e) {
      config_params_.CheckFilesUpdated(std::bind(&SysMonitor::ReadParams,
      this)); }, false, true);

  // Create a startup timer. Timer will be used to check if all the heartbeats
  // had started in the specified amount of time.
  startup_timer_ = nh_.createTimer(ros::Duration(startup_time_),
                                   &SysMonitor::StartupTimerCallback,
                                   this,
                                   true,
                                   true);

  // Create a timer to publish the heartbeat for the system monitor.
  heartbeat_timer_ = nh_.createTimer(ros::Duration(heartbeat_pub_rate_),
                                     &SysMonitor::PublishHeartbeatCallback,
                                     this,
                                     false,
                                     true);

  sub_hb_ = nh_.subscribe(TOPIC_HEARTBEAT,
                          sub_queue_size_,
                          &SysMonitor::HeartbeatCallback,
                          this,
                          ros::TransportHints().tcpNoDelay());

  pub_cmd_ = nh_.advertise<ff_msgs::CommandStamped>(TOPIC_COMMAND,
                                                    pub_queue_size_,
                                                    false);

  // All configs should be latching
  pub_fault_config_ = nh_.advertise<ff_msgs::FaultConfig>(
                                            TOPIC_MANAGEMENT_SYS_MONITOR_CONFIG,
                                            pub_queue_size_,
                                            true);

  // All states should be latching
  pub_fault_state_ = nh_.advertise<ff_msgs::FaultState>(
                                            TOPIC_MANAGEMENT_SYS_MONITOR_STATE,
                                            pub_queue_size_,
                                            true);

  pub_time_diff_ = nh_.advertise<ff_msgs::TimeDiffStamped>(
                                        TOPIC_MANAGEMENT_SYS_MONITOR_TIME_DIFF,
                                        pub_queue_size_,
                                        false);

  fault_state_.state = ff_msgs::FaultState::FUNCTIONAL;


  // Set up service
  unload_load_nodelet_service_ = nh_.advertiseService(
                            SERVICE_MANAGEMENT_SYS_MONITOR_UNLOAD_LOAD_NODELET,
                            &SysMonitor::NodeletService,
                            this);

  // Publish fault state when first starting up
  PublishFaultState();

  // Publish fault config so the ground knows what faults to expect
  PublishFaultConfig();

  num_current_blocking_fault_ = 0;

  OutputFaultTables();
}

// Function used for debugging purposes only
void SysMonitor::OutputFaultTables() {
  std::string subsys, node, warning, blocking_str;
  ff_msgs::CommandStampedConstPtr tmp_cmd;
  bool blocking;

  // Output all faults first
  NODELET_DEBUG("All faults: ");
  for (unsigned int i = 0; i < fault_config_.faults.size(); i++) {
    subsys = fault_config_.subsystems[fault_config_.faults[i].subsystem];
    node = fault_config_.nodes[fault_config_.faults[i].node];

    if (fault_config_.faults[i].warning) {
      warning = "true";
    } else {
      warning = "false";
    }

    // Get command out of the all faults map
    tmp_cmd = all_faults_.at(fault_config_.faults[i].id)->response;

    // Get blocking flag out of the all faults map
    blocking = all_faults_.at(fault_config_.faults[i].id)->blocking;
    if (blocking) {
      blocking_str = "true";
    } else {
      blocking_str = "false";
    }

    NODELET_DEBUG_STREAM("fault: " <<
                "\nid: " << fault_config_.faults[i].id <<
                "\nsubsys: " <<  subsys.c_str() <<
                "\nnode: " << node.c_str() <<
                "\nwarning: " << warning.c_str() <<
                "\nblocking: " << blocking_str.c_str() <<
                "\ndescr: " << fault_config_.faults[i].description.c_str() <<
                "\nCommand:\n\t" << (*tmp_cmd));
  }

  // Output heartbeat tables second
  NODELET_DEBUG("Heartbeat faults: ");
  typedef std::map<std::string, WatchdogPtr>::iterator it_type;
  for (it_type it = watch_dogs_.begin(); it != watch_dogs_.end(); it++) {
    NODELET_DEBUG_STREAM("hb fault: " <<
                  "\nid: " << it->second->fault_id() <<
                  "\nnode name: " << it->first <<
                  "\nnode type: " << it->second->nodelet_type() <<
                  "\nmisses: " << it->second->misses_allowed());
  }
}

void SysMonitor::PublishCmd(ff_msgs::CommandStampedPtr cmd) {
  cmd->cmd_id = "sys_monitor" + std::to_string(ros::Time::now().sec);
  pub_cmd_.publish(cmd);
}

void SysMonitor::PublishFaultConfig() {
  fault_config_.header.stamp = ros::Time::now();
  pub_fault_config_.publish(fault_config_);
}

void SysMonitor::PublishFaultState() {
  fault_state_.header.stamp = ros::Time::now();
  pub_fault_state_.publish(fault_state_);
}

void SysMonitor::PublishFaultResponse(unsigned int fault_id) {
  if (all_faults_.count(fault_id)) {
    // Don't publish command if it is a warning
    if (!all_faults_.at(fault_id)->warning) {
      ff_msgs::CommandStampedConstPtr cmd = all_faults_.at(fault_id)->response;
      // Check for unload since it doesn't to send the command to the executive
      // just to have the executive call the system monitor service
      if (cmd->cmd_name == ff_msgs::CommandConstants::CMD_NAME_UNLOAD_NODELET) {
        // Need to check the command is formatted correctly
        if (cmd->args.size() != 2 ||
            cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
            cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
          NODELET_ERROR("Malformed unload command for fault %i.", fault_id);
        } else {
          UnloadNodelet(cmd->args[0].s, cmd->args[1].s);
        }
      } else if (cmd->cmd_name != ff_msgs::CommandConstants::CMD_NAME_NO_OP) {
        // Don't publish command if it is a noop
        PublishCmd(all_faults_.at(fault_id)->response);
      }
    }
  } else {
    NODELET_FATAL("Fault id %i unrecognized!", fault_id);
  }
}

void SysMonitor::PublishHeartbeatCallback(ros::TimerEvent const& te) {
  PublishHeartbeat();
}

void SysMonitor::PublishHeartbeat(bool initialization_fault) {
  // If there was an initialization fault, add it to the heartbeat
  if (initialization_fault) {
    // The system monitor only has one fault it asserts and this fault usually
    // occurs before the fault ids are read in from the config files. Also the
    // fault id is not being sent to the ground. Thus we are just going to set
    // the initialization fault id to zero.

    // Check to make sure the fault wasn't already added
    if (heartbeat_.faults.size() == 0) {
      ff_msgs::Fault fault;
      fault.id = 0;
      fault.time_of_fault = ros::Time::now();
      heartbeat_.faults.push_back(fault);
    }

    // Stop heartbeat timer if initialization failed since initialization can
    // fail when the config file changes and the parameters are reloaded
    heartbeat_timer_.stop();
  }

  heartbeat_.header.stamp = ros::Time::now();
  pub_heartbeat_.publish(heartbeat_);
}

void SysMonitor::PublishTimeDiff(float time_diff_sec) {
  ff_msgs::TimeDiffStamped time_diff_msg;
  time_diff_msg.header.stamp = ros::Time::now();
  time_diff_msg.time_diff_sec = time_diff_sec;
  pub_time_diff_.publish(time_diff_msg);
}

void SysMonitor::StartupTimerCallback(ros::TimerEvent const& te) {
  for (auto it = watch_dogs_.begin(); it != watch_dogs_.end(); ++it) {
    if (!it->second->heartbeat_started()) {
      std::string err_msg = "Never received heartbeat from " + it->first;
      AddFault(it->second->fault_id(), err_msg);
      PublishFaultResponse(it->second->fault_id());
      it->second->hb_fault_occurring(true);
    }
  }
}

bool SysMonitor::ReadParams() {
  // Reset/reload watch dogs when config file changes
  if (watch_dogs_.size() > 0) {
    watch_dogs_.clear();
  }

  // Read config files into lua
  if (!config_params_.ReadFiles()) {
    NODELET_ERROR("Error loading system monitor parameters.");
    PublishHeartbeat(true);
    return false;
  }

  // Get startup time. Used to check if all the heartbeats have started within
  // the specified startup time.
  if (!config_params_.GetUInt("startup_time_sec", &startup_time_)) {
    NODELET_WARN("Unable to read startup time.");
    startup_time_ = 90;
  }

  if (!config_params_.GetUInt("heartbeat_pub_rate_sec", &heartbeat_pub_rate_)) {
    NODELET_WARN("Unable to read heartbeat pub rate.");
    heartbeat_pub_rate_ = 5;
  }

  if (!config_params_.GetStr("time_diff_node", &time_diff_node_)) {
    NODELET_WARN("Unable to read time diff node name.");
    time_diff_node_ = "imu_aug";
  }

  if (!config_params_.GetPosReal("time_drift_thres_sec",
                                                      &time_drift_thres_sec_)) {
    NODELET_WARN("Unable to read time drift threshold.");
    time_drift_thres_sec_ = 0.25;
  }

  // Get the list of nodes not running so that we don't monitor or trigger the
  // faults associated with those nodes
  std::vector<std::string> nodes_not_running;
  if (config_params_.CheckValExists("nodes_not_running")) {
    std::string temp_node;
    config_reader::ConfigReader::Table nodes_not_running_tbl(&config_params_,
                                                          "nodes_not_running");
    int tbl_size = nodes_not_running_tbl.GetSize() + 1;
    for (int i = 1; i < tbl_size; i++) {
      nodes_not_running_tbl.GetStr(i, &temp_node);
      nodes_not_running.push_back(temp_node);
    }
  }

  // Read in all faults
  // Check fault table exists
  if (!config_params_.CheckValExists("subsystems")) {
    NODELET_ERROR("Unable to find/read the fault table.");
    PublishHeartbeat(true);
    return false;
  }

  config_reader::ConfigReader::Table subsystems_tbl(&config_params_,
                                                                  "subsystems");
  unsigned int fault_id, misses;
  double timeout;
  int i, j, k, nodes_tbl_size, faults_tbl_size;
  std::string subsys_name, node_name, fault_description;
  bool blocking, warning, found;

  // Go through all the subsystems
  int subsystems_tbl_size = subsystems_tbl.GetSize() + 1;
  // Lua indices start at one
  for (i = 1; i < subsystems_tbl_size; i++) {
    config_reader::ConfigReader::Table subsystem_tbl(&subsystems_tbl, i);
    if (!subsystem_tbl.GetStr("name", &subsys_name)) {
      NODELET_ERROR("Unable to read name at %i in subsys table", i);
      PublishHeartbeat(true);
      return false;
    }

    // Add subsystem name to the config list of subsystem names
    fault_config_.subsystems.emplace_back(subsys_name);

    // Check nodes table exists
    if (!subsystem_tbl.CheckValExists("nodes")) {
      NODELET_ERROR("Unable to find nodes table in %s's table.",
                                                          subsys_name.c_str());
      PublishHeartbeat(true);
      return false;
    }

    config_reader::ConfigReader::Table nodes_tbl(&subsystem_tbl, "nodes");
    // Go through all the nodes
    nodes_tbl_size = nodes_tbl.GetSize() + 1;
    // Lua indices start at one
    for (j = 1; j < nodes_tbl_size; j++) {
      config_reader::ConfigReader::Table node_tbl(&nodes_tbl, j);
      if (!node_tbl.GetStr("name", &node_name)) {
        NODELET_ERROR("Unable to read name at %i in %s's node table.",
                                                        j, subsys_name.c_str());
        PublishHeartbeat(true);
        return false;
      }

      found = false;
      for (unsigned int m = 0; m < nodes_not_running.size(); m++) {
        if (node_name == nodes_not_running[m]) {
          found = true;
          break;
        }
      }

      if (found) {
        continue;
      }

      // Add node name to the config list of node names
      fault_config_.nodes.emplace_back(node_name);

      // Check faults table exists
      if (!node_tbl.CheckValExists("faults")) {
        NODELET_ERROR("Unable to read the fault table for node %s.",
                                                            node_name.c_str());
        PublishHeartbeat(true);
        return false;
      }

      config_reader::ConfigReader::Table faults_tbl(&node_tbl, "faults");
      // Go through all faults
      faults_tbl_size = faults_tbl.GetSize() + 1;
      // Lua indices start at one
      for (k = 1; k < faults_tbl_size; k++) {
        config_reader::ConfigReader::Table fault_entry(&faults_tbl, k);

        ff_msgs::CommandStampedPtr response_cmd(new ff_msgs::CommandStamped());

        if (fault_entry.GetUInt("id", &fault_id) &&
            fault_entry.GetBool("blocking", &blocking) &&
            fault_entry.GetBool("warning", &warning) &&
            fault_entry.GetStr("description", &fault_description) &&
            ReadCommand(&fault_entry, response_cmd)) {
          ff_msgs::FaultInfo fault_info;

          // subsystem was added to vector at back which corresponds to i - 1
          fault_info.subsystem = i - 1;
          // node was added to vector at back which corresponds to size - 1
          fault_info.node = fault_config_.nodes.size() - 1;
          fault_info.id = fault_id;
          fault_info.warning = warning;
          fault_info.description.assign(fault_description, 0, 64);

          // Add fault info to fault config message so it can be sent to ground
          fault_config_.faults.push_back(fault_info);

          auto fault = std::make_shared<SysMonitor::Fault>(node_name,
                                                           blocking,
                                                           warning,
                                                           response_cmd);

          all_faults_.emplace(fault_id, fault);

          // Check if fault is a heartbeat fault and read in the heartbeat info
          // if it is
          if (fault_entry.CheckValExists("heartbeat")) {
            config_reader::ConfigReader::Table heartbeat(&fault_entry,
                                                                  "heartbeat");
            if (heartbeat.GetReal("timeout_sec", &timeout) &&
                heartbeat.GetUInt("misses", &misses)) {
              AddWatchDog(ros::Duration(timeout), node_name, misses, fault_id);
            } else {
              NODELET_ERROR("Unable to add heartbeat for node %s.",
                                                            node_name.c_str());
              PublishHeartbeat(true);
              return false;
            }
          }
        } else {
          NODELET_ERROR("Unable to read fault at %i in %s's table.",
                                                          k, node_name.c_str());
          PublishHeartbeat(true);
          return false;
        }
      }
    }
  }

  // Extract nodelet type and add to watchdog map
  if (config_params_.CheckValExists("nodelet_types")) {
    std::string type = "";
    config_reader::ConfigReader::Table types_tbl(&config_params_,
                                                              "nodelet_types");
    int types_tbl_size = types_tbl.GetSize() + 1;
    for (i = 1; i < types_tbl_size; i++) {
      config_reader::ConfigReader::Table type_entry(&types_tbl, i);
      if (!type_entry.GetStr("name", &node_name)) {
        NODELET_WARN("Name not found at %i in types table.", i);
        continue;
      }

      if (!type_entry.GetStr("type", &type)) {
        NODELET_WARN("Type not found at %i in types table.", i);
        continue;
      }

      found = false;
      for (unsigned int m = 0; m < nodes_not_running.size(); m++) {
        if (node_name == nodes_not_running[m]) {
          found = true;
          break;
        }
      }

      if (found) {
        continue;
      }

      // Check to make sure node got added to watchdog maps
      if (watch_dogs_.count(node_name) > 0) {
        watch_dogs_.at(node_name)->nodelet_type(type);
      } else {
        NODELET_WARN("Couldn't add type, %s wasn't in fault table.",
                                                            node_name.c_str());
      }
    }
  }

  return true;
}

bool SysMonitor::ReadCommand(config_reader::ConfigReader::Table *entry,
                             ff_msgs::CommandStampedPtr cmd) {
  std::string cmd_name;
  config_reader::ConfigReader::Table command(entry, "response");
  if (!command.GetStr("name", &cmd_name)) {
    NODELET_FATAL("Command name not specified.");
    return false;
  }

  cmd->cmd_name = cmd_name;
  cmd->cmd_src = "sys_monitor";
  cmd->cmd_origin = "sys_monitor";
  cmd->subsys_name = "Astrobee";

  if (command.CheckValExists("args")) {
    config_reader::ConfigReader::Table args(&command, "args");
    int num_args = args.GetSize(), i;
    unsigned int type;
    cmd->args.resize(num_args);
    for (i = 0; i < num_args; ++i) {
      // Lua indices start at 1
      config_reader::ConfigReader::Table arg(&args, (i + 1));
      // First element in table is the type
      if (!arg.GetUInt(1, &type)) {
        NODELET_FATAL("First command argument value is not a uint");
        return false;
      }

      // Remaining elements are the parameter values
      switch (type) {
        case CommandArg::DATA_TYPE_BOOL:
          {
            bool val;
            if (!arg.GetBool(2, &val)) {
              NODELET_FATAL("Expected command argument to be a bool!");
              return false;
            }
            cmd->args[i].data_type = CommandArg::DATA_TYPE_BOOL;
            cmd->args[i].b = val;
          }
          break;
        case CommandArg::DATA_TYPE_DOUBLE:
          {
            double val;
            if (!arg.GetReal(2, &val)) {
              NODELET_FATAL("Expected command argument to be a double");
              return false;
            }
            cmd->args[i].data_type = CommandArg::DATA_TYPE_DOUBLE;
            cmd->args[i].d = val;
          }
          break;
        case CommandArg::DATA_TYPE_FLOAT:
          {
            float val;
            if (!arg.GetReal(2, &val)) {
              NODELET_FATAL("Expected command argument to be a float.");
              return false;
            }
            cmd->args[i].data_type = CommandArg::DATA_TYPE_FLOAT;
            cmd->args[i].f = val;
          }
          break;
        case CommandArg::DATA_TYPE_INT:
          {
            int val;
            if (!arg.GetInt(2, &val)) {
              NODELET_FATAL("Expected command argument to be an int.");
              return false;
            }
            cmd->args[i].data_type = CommandArg::DATA_TYPE_INT;
            cmd->args[i].i = val;
          }
          break;
        case CommandArg::DATA_TYPE_LONGLONG:
          {
            int64_t val;
            if (!arg.GetLongLong(2, &val)) {
              NODELET_FATAL("Expected command argument to be an int.");
              return false;
            }
            cmd->args[i].data_type = CommandArg::DATA_TYPE_LONGLONG;
            cmd->args[i].ll = val;
          }
          break;
        case CommandArg::DATA_TYPE_STRING:
          {
            std::string val;
            if (!arg.GetStr(2, &val)) {
              NODELET_FATAL("Expected command argument to be a string");
              return false;
            }
            cmd->args[i].data_type = CommandArg::DATA_TYPE_STRING;
            cmd->args[i].s = val;
          }
          break;
        case CommandArg::DATA_TYPE_VEC3d:
          {
            int j;
            double val;
            cmd->args[i].data_type = CommandArg::DATA_TYPE_VEC3d;
            for (j = 0; j < 3; ++j) {
              // Index to get vector values in table starts at 2
              if (!arg.GetReal((j + 2), &val)) {
                NODELET_FATAL("Expected command argument to be a double.");
                return false;
              }
              cmd->args[i].vec3d[j] = val;
            }
          }
          break;
        case CommandArg::DATA_TYPE_MAT33f:
          {
            int j;
            float val;
            cmd->args[i].data_type = CommandArg::DATA_TYPE_MAT33f;
            for (j = 0; j < 9; ++j) {
              // Index in get matrix values in table starts at 2
              if (!arg.GetReal((j + 2), &val)) {
                NODELET_FATAL("Expected command argument to be a float.");
                return false;
              }
              cmd->args[i].mat33f[j] = val;
            }
          }
          break;
        default:
          NODELET_FATAL("SysMonitor: Type for command argument unrecognized!");
          return false;
      }
    }
  }

  return true;
}

bool SysMonitor::NodeletService(ff_msgs::UnloadLoadNodelet::Request &req,
                                ff_msgs::UnloadLoadNodelet::Response &res) {
  if (req.load) {
    res.result = LoadNodelet(req);
  } else {
    res.result = UnloadNodelet(req.name, req.manager_name);
  }

  return true;
}

int SysMonitor::LoadNodelet(ff_msgs::UnloadLoadNodelet::Request &req) {
  std::string platform = GetPlatform();
  if (platform == "") {
    load_service_.request.name = "/" + req.name;
  } else {
    load_service_.request.name = "/" + platform + "/" + req.name;
  }

  load_service_.request.remap_source_args = req.remap_source_args;
  load_service_.request.remap_target_args = req.remap_target_args;
  load_service_.request.my_argv = req.my_argv;
  load_service_.request.bond_id = req.bond_id;

  // Extract manager from watchdog map if not specified in service
  std::string manager;
  if (req.manager_name == "") {
    // Check if node name was added to the heartbeat map
    if (watch_dogs_.count(req.name) > 0) {
      manager = watch_dogs_.at(req.name)->nodelet_manager();
      if (manager == "") {
        return ff_msgs::UnloadLoadNodelet::Response::MANAGER_NAME_MISSING;
      }
    } else {
      return ff_msgs::UnloadLoadNodelet::Response::NODE_NOT_IN_MAP;
    }
  } else {
    manager = req.manager_name;
  }

  // Extract type from watchdog map if not specified in service
  if (req.type == "") {
    // Check if node name was added to the heartbeat map
    if (watch_dogs_.count(req.name) > 0) {
      load_service_.request.type = watch_dogs_.at(req.name)->nodelet_type();
      if (load_service_.request.type == "") {
        return ff_msgs::UnloadLoadNodelet::Response::TYPE_MISSING;
      }
    } else {
      return ff_msgs::UnloadLoadNodelet::Response::NODE_NOT_IN_MAP;
    }
  } else {
    load_service_.request.type = req.type;
  }

  if (!ros::service::call(manager + "/load_nodelet", load_service_)) {
    NODELET_FATAL("Unable to load nodelet %s in %s manager.",
                                            req.name.c_str(), manager.c_str());
    return ff_msgs::UnloadLoadNodelet::Response::ROS_SERVICE_FAILED;
  }
  return ff_msgs::UnloadLoadNodelet::Response::SUCCESSFUL;
}

int SysMonitor::UnloadNodelet(std::string const& nodelet,
                              std::string const& manager) {
  std::string platform = GetPlatform();
  if (platform == "") {
    unload_service_.request.name = "/" + nodelet;
  } else {
    unload_service_.request.name = "/" + platform + "/" + nodelet;
  }

  std::string manager_name = "";
  // Check if node name was added to the heartbeat map
  if (watch_dogs_.count(nodelet) > 0) {
    // If manager wasn't specified, extract it from the watch dogs map
    if (manager == "") {
      manager_name = watch_dogs_.at(nodelet)->nodelet_manager();
      if (manager_name == "") {
        return ff_msgs::UnloadLoadNodelet::Response::MANAGER_NAME_MISSING;
      }
    } else {
      manager_name = manager;
    }
  } else {
    return ff_msgs::UnloadLoadNodelet::Response::NODE_NOT_IN_MAP;
  }

  if (!ros::service::call(manager_name + "/unload_nodelet", unload_service_)) {
    NODELET_FATAL("Unable to unload nodet %s in %s manager.",
                                              nodelet.c_str(), manager.c_str());
    return ff_msgs::UnloadLoadNodelet::Response::ROS_SERVICE_FAILED;
  }

  // Stop timer so we don't get a heartbeat missing fault
  watch_dogs_.at(nodelet)->StopTimer();
  // Set unloaded to true so that if it gets restarted, we remove the previous
  // faults
  watch_dogs_.at(nodelet)->unloaded(true);

  return ff_msgs::UnloadLoadNodelet::Response::SUCCESSFUL;
}

/**************************** Watchdog Functions *****************************/
SysMonitor::Watchdog::Watchdog(SysMonitor *const sys_monitor,
                               std::string const& nodelet_name,
                               ros::Duration const& timeout,
                               uint const allowed_misses,
                               uint const fault_id) :
  monitor_(sys_monitor),
  missed_count_(0),
  misses_allowed_(allowed_misses),
  fault_id_(fault_id),
  hb_fault_occurring_(false),
  heartbeat_started_(false),
  unloaded_(false),
  nodelet_manager_(""),
  nodelet_name_(nodelet_name),
  nodelet_type_(""),
  previous_hb_() {
  timer_ = monitor_->nh_.createTimer(timeout,
                      &SysMonitor::Watchdog::TimerCallBack, this, false, false);
}

uint SysMonitor::Watchdog::fault_id() {
  return fault_id_;
}

uint SysMonitor::Watchdog::misses_allowed() {
  return misses_allowed_;
}

ff_msgs::HeartbeatConstPtr SysMonitor::Watchdog::previous_hb() {
  return previous_hb_;
}

bool SysMonitor::Watchdog::hb_fault_occurring() {
  return hb_fault_occurring_;
}

bool SysMonitor::Watchdog::heartbeat_started() {
  return heartbeat_started_;
}

bool SysMonitor::Watchdog::unloaded() {
  return unloaded_;
}

std::string SysMonitor::Watchdog::nodelet_manager() {
  return nodelet_manager_;
}

std::string SysMonitor::Watchdog::nodelet_name() {
  return nodelet_name_;
}

std::string SysMonitor::Watchdog::nodelet_type() {
  return nodelet_type_;
}

void SysMonitor::Watchdog::hb_fault_occurring(bool occurring) {
  hb_fault_occurring_ = occurring;
}

void SysMonitor::Watchdog::nodelet_manager(std::string manager_name) {
  nodelet_manager_ = manager_name;
}

void SysMonitor::Watchdog::nodelet_name(std::string name) {
  nodelet_name_ = name;
}

void SysMonitor::Watchdog::nodelet_type(std::string type) {
  nodelet_type_ = type;
}

void SysMonitor::Watchdog::unloaded(bool is_unloaded) {
  unloaded_ = is_unloaded;
}

void SysMonitor::Watchdog::ResetTimer() {
  if (!heartbeat_started_) {
    heartbeat_started_ = true;
  }

  timer_.stop();
  timer_.start();
  missed_count_ = 0;
}

void SysMonitor::Watchdog::StopTimer() {
  timer_.stop();
}

void SysMonitor::Watchdog::previous_hb(ff_msgs::HeartbeatConstPtr hb) {
  previous_hb_ = hb;
}

void SysMonitor::Watchdog::TimerCallBack(ros::TimerEvent const& te) {
  if (missed_count_++ >= misses_allowed_) {
    std::string err_msg = "Didn't receive a heartbeat from " + nodelet_name_;
    monitor_->AddFault(fault_id_, err_msg);
    monitor_->PublishFaultResponse(fault_id_);
    // Stop timer so the fault is only triggered once. Timer will be restarted
    // once a heartbeart from the node is received
    timer_.stop();
    hb_fault_occurring_ = true;
  }
}

SysMonitor::Fault::Fault(std::string const& node_name_in,
                         bool const blocking_in,
                         bool const warning_in,
                         ff_msgs::CommandStampedPtr response_in) :
  node_name(node_name_in),
  blocking(blocking_in),
  warning(warning_in),
  response(response_in) {
}
}  // namespace sys_monitor

PLUGINLIB_EXPORT_CLASS(sys_monitor::SysMonitor, nodelet::Nodelet)
