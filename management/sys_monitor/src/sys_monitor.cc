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
  ff_util::FreeFlyerNodelet(NODE_SYS_MONITOR, true),
  pub_queue_size_(10),
  sub_queue_size_(10) {
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
    fault_state_.faults.push_back(fault);
    SetFaultState(fault_id, true);

    PublishFaultState();
  }
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
    SysMonitor::WatchdogPtr watchDog(new SysMonitor::Watchdog(this, timeout,
                                                    allowed_misses, fault_id));
    watch_dogs_.emplace(node_name, watchDog);
  } else {
    ROS_INFO("SysMonitor::AddWatchDog() already exists for %s",
                                                          node_name.c_str());
  }
}

void SysMonitor::SetFaultState(unsigned int fault_id, bool adding_fault) {
  // If adding or removing a blocking fault, increment the counter appropriately
  if (all_faults_.count(fault_id) > 0) {
    if (all_faults_.at(fault_id)->blocking_) {
      if (adding_fault) {
        num_current_blocking_fault_++;
      } else {
        num_current_blocking_fault_--;
      }
    }
  } else {
    ROS_ERROR("SysMonitor: Fault %i wasn't found when setting fault state",
                                                                      fault_id);
  }

  // Set the state appropriately
  if (fault_state_.faults.size() > 0) {
    if (num_current_blocking_fault_ > 0) {
      fault_state_.state = ff_msgs::FaultState::BLOCKED;
    } else if (num_current_blocking_fault_ < 0) {
      // This should never happen but add output just in case
      ROS_ERROR("SysMonitor: number of blocking faults is negative!!");
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

  // Don't need to check our own heartbeat
  if (hb->node.compare("sys_monitor") == 0) {
    return;
  }

  // Check to see if node heartbeat is set up in watchdogs
  if (watch_dogs_.count(hb->node) > 0) {
    WatchdogPtr wd = watch_dogs_.at(hb->node);
    wd->ResetTimer();

    // Check to see if node restarted publishing its heartbeat
    if (wd->IsFaultOccurring()) {
      wd->ResetFaultOccurring();
      RemoveFault(wd->GetFaultId());
    }

    // Get last heartbeat for fault comparison
    ff_msgs::HeartbeatConstPtr previous_hb = wd->GetPreviousHeartbeat();

    // Check to see if this is the first heartbeat from the node
    if (!previous_hb) {
      wd->SetPreviousHeartbeat(hb);
      return;
    }

    // Check to see if there were and are no faults
    if (previous_hb->faults.size() == 0 && hb->faults.size() == 0) {
      wd->SetPreviousHeartbeat(hb);
      return;
    }

    // Check to see if faults were added
    for (i = 0; i < hb->faults.size(); i++) {
      fault_found = false;
      tmp_id = hb->faults[i].id;
      for (j = 0; j < previous_hb->faults.size(); j++) {
        if (tmp_id == previous_hb->faults[j].id) {
          fault_found = true;
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
    wd->SetPreviousHeartbeat(hb);
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
      ROS_WARN("SysMonitor: heartbeat unrecognized from %s!", hb->node.c_str());
      unwatched_heartbeats_.push_back(hb->node);
    }
  }
}

void SysMonitor::Initialize(ros::NodeHandle *nh) {
  nh_ = *nh;

  // Since the fault table must include the sys_monitor config, we have to set
  // the path to where the sys monitor config file lives
  std::string path = common::GetConfigDir();
  path += "/management/";
  config_params_.SetPath(path.c_str());

  // Add config files to config reader
  config_params_.AddFile("fault_table.config");

  if (!ReadParams()) {
    exit(EXIT_FAILURE);
    return;
  }

  // Create a callback timer which checks to see if the config files have been
  // changed.
  reload_params_timer_ = nh_.createTimer(ros::Duration(1),
      [this](ros::TimerEvent e) {
      config_params_.CheckFilesUpdated(std::bind(&SysMonitor::ReadParams,
      this)); }, false, true);

  sub_hb_ = nh_.subscribe(TOPIC_HEARTBEAT, sub_queue_size_,
                                          &SysMonitor::HeartbeatCallback, this);

  pub_cmd_ = nh_.advertise<ff_msgs::CommandStamped>(TOPIC_COMMAND,
                                                        pub_queue_size_, false);

  // All configs should be latching
  pub_fault_config_ = nh_.advertise<ff_msgs::FaultConfig>(
                    TOPIC_MANAGEMENT_SYS_MONITOR_CONFIG, pub_queue_size_, true);

  // All states should be latching
  pub_fault_state_ = nh_.advertise<ff_msgs::FaultState>(
                    TOPIC_MANAGEMENT_SYS_MONITOR_STATE, pub_queue_size_, true);

  fault_state_.state = ff_msgs::FaultState::FUNCTIONAL;
  // Publish fault state when first starting up
  PublishFaultState();

  // Publish fault config so the ground knows what faults to expect
  PublishFaultConfig();

  num_current_blocking_fault_ = 0;

  if (DEBUG) {
    OutputFaultTables();
  }
}

// Function used for debugging purposes only
void SysMonitor::OutputFaultTables() {
  std::string subsys, node, warning, blocking_str;
  ff_msgs::CommandStampedConstPtr tmp_cmd;
  bool blocking;

  // Output all faults first
  ROS_INFO("SysMonitor: all faults: ");
  for (unsigned int i = 0; i < fault_config_.faults.size(); i++) {
    subsys = fault_config_.subsystems[fault_config_.faults[i].subsystem];
    node = fault_config_.nodes[fault_config_.faults[i].node];

    if (fault_config_.faults[i].warning) {
      warning = "true";
    } else {
      warning = "false";
    }

    // Get command out of the all faults map
    tmp_cmd = all_faults_.at(fault_config_.faults[i].id)->response_;

    // Get blocking flag out of the all faults map
    blocking = all_faults_.at(fault_config_.faults[i].id)->blocking_;
    if (blocking) {
      blocking_str = "true";
    } else {
      blocking_str = "false";
    }

    ROS_INFO_STREAM("fault: " <<
                "\nid: " << fault_config_.faults[i].id <<
                "\nsubsys: " <<  subsys.c_str() <<
                "\nnode: " << node.c_str() <<
                "\nwarning: " << warning.c_str() <<
                "\nblocking: " << blocking_str.c_str() <<
                "\ndescr: " << fault_config_.faults[i].description.c_str() <<
                "\nCommand:\n\t" << (*tmp_cmd));
  }

  // Output heartbeat tables second
  ROS_INFO("SysMonitor: heartbeat faults: ");
  typedef std::map<std::string, WatchdogPtr>::iterator it_type;
  for (it_type it = watch_dogs_.begin(); it != watch_dogs_.end(); it++) {
    it->second->OutputHeartbeatFault(it->first);
  }
}

void SysMonitor::PublishCmd(ff_msgs::CommandStampedConstPtr cmd) {
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
    // Don't publish command if it is a noop
    if (all_faults_.at(fault_id)->response_->cmd_name !=
                                    ff_msgs::CommandConstants::CMD_NAME_NO_OP) {
      PublishCmd(all_faults_.at(fault_id)->response_);
    }
  } else {
    ROS_FATAL("SysMonitor: Fault id %i unrecognized!", fault_id);
  }
}

bool SysMonitor::ReadParams() {
  // Reset/reload watch dogs when config file changes
  if (watch_dogs_.size() > 0) {
    watch_dogs_.clear();
  }

  // Read config files into lua
  if (!config_params_.ReadFiles()) {
    ROS_FATAL("Error loading system monitor parameters.");
    return false;
  }

  // Read in all faults
  config_reader::ConfigReader::Table subsystems_tbl(&config_params_,
                                                                  "subsystems");
  unsigned int fault_id, misses;
  double timeout;
  int i, j, k, nodes_tbl_size, faults_tbl_size;
  std::string subsys_name, node_name, fault_description;
  bool warning, blocking;

  // Go through all the subsystems
  int subsystems_tbl_size = subsystems_tbl.GetSize() + 1;
  // Lua indices start at one
  for (i = 1; i < subsystems_tbl_size; i++) {
    config_reader::ConfigReader::Table subsystem_tbl(&subsystems_tbl, i);
    if (!subsystem_tbl.GetStr("name", &subsys_name)) {
      ROS_FATAL("SysMonitor: Unable to read name at %i in subsys table.", i);
      return false;
    }

    // Add subsystem name to the config list of subsystem names
    fault_config_.subsystems.emplace_back(subsys_name);

    config_reader::ConfigReader::Table nodes_tbl(&subsystem_tbl, "nodes");
    // Go through all the nodes
    nodes_tbl_size = nodes_tbl.GetSize() + 1;
    // Lua indices start at one
    for (j = 1; j < nodes_tbl_size; j++) {
      config_reader::ConfigReader::Table node_tbl(&nodes_tbl, j);
      if (!node_tbl.GetStr("name", &node_name)) {
        ROS_FATAL("SysMonitor: Unable to read name at %i in %s's node table.",
                                                        j, subsys_name.c_str());
        return false;
      }

      // Add node name to the config list of node names
      fault_config_.nodes.emplace_back(node_name);

      config_reader::ConfigReader::Table faults_tbl(&node_tbl, "faults");
      // Go through all faults
      faults_tbl_size = faults_tbl.GetSize() + 1;
      // Lua indices start at one
      for (k = 1; k < faults_tbl_size; k++) {
        config_reader::ConfigReader::Table fault_entry(&faults_tbl, k);

        ff_msgs::CommandStampedPtr response_cmd(new ff_msgs::CommandStamped());

        if (fault_entry.GetUInt("id", &fault_id) &&
            fault_entry.GetBool("warning", &warning) &&
            fault_entry.GetBool("blocking", &blocking) &&
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
              ROS_FATAL("SysMonitor: Unable to add heartbeat for node %s.",
                                                            node_name.c_str());
              return false;
            }
          }
        } else {
          ROS_FATAL("SysMonitor: Unable to read fault at %i in %s's table.",
                                                          k, node_name.c_str());
          return false;
        }
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
        ROS_FATAL("SysMonitor: First command argument value is not a uint!");
        return false;
      }

      // Remaining elements are the parameter values
      switch (type) {
        case CommandArg::DATA_TYPE_BOOL:
          {
            bool val;
            if (!arg.GetBool(2, &val)) {
              ROS_FATAL("SysMonitor: Expected command argument to be a bool!");
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
              ROS_FATAL("SysMonitor: Expected command argument to be a double");
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
              ROS_FATAL("SysMonitor: Expected command argument to be a float.");
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
              ROS_FATAL("SysMonitor: Expected command argument to be an int.");
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
              ROS_FATAL("SysMonitor: Expected command argument to be an int.");
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
              ROS_FATAL("SysMonitor: Expected command argument to be a string");
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
                ROS_FATAL("SysMonitor: Expected cmd argument to be a double.");
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
                ROS_FATAL("SysMonitor: Expected cmd argument to be a float.");
                return false;
              }
              cmd->args[i].mat33f[j] = val;
            }
          }
          break;
        default:
          ROS_FATAL("SysMonitor: Type for command argument unrecognized!");
          return false;
      }
    }
  }

  return true;
}

/**************************** Watchdog Functions *****************************/
SysMonitor::Watchdog::Watchdog(SysMonitor *const sys_monitor,
                                ros::Duration const& timeout,
                                uint const allowed_misses,
                                uint const fault_id) :
  monitor_(sys_monitor),
  missed_count_(0),
  missed_allowed_(allowed_misses),
  fault_id_(fault_id),
  hb_fault_occurring_(false),
  previous_hb_() {
  timer_ = monitor_->nh_.createTimer(timeout,
                      &SysMonitor::Watchdog::TimerCallBack, this, false, true);
}

uint SysMonitor::Watchdog::GetFaultId() {
  return fault_id_;
}

ff_msgs::HeartbeatConstPtr SysMonitor::Watchdog::GetPreviousHeartbeat() {
  return previous_hb_;
}

bool SysMonitor::Watchdog::IsFaultOccurring() {
  return hb_fault_occurring_;
}

void SysMonitor::Watchdog::OutputHeartbeatFault(std::string node_name) {
  ROS_INFO_STREAM("hb fault: " <<
                  "\nid: " << fault_id_ <<
                  "\nnode name: " << node_name <<
                  "\nmisses: " << missed_allowed_);
}

void SysMonitor::Watchdog::ResetFaultOccurring() {
  hb_fault_occurring_ = false;
}

void SysMonitor::Watchdog::ResetTimer() {
  timer_.stop();
  timer_.start();
  missed_count_ = 0;
}

void SysMonitor::Watchdog::SetPreviousHeartbeat(ff_msgs::HeartbeatConstPtr hb) {
  previous_hb_ = hb;
}

void SysMonitor::Watchdog::TimerCallBack(ros::TimerEvent const& te) {
  if (missed_count_++ >= missed_allowed_) {
    monitor_->AddFault(fault_id_);
    monitor_->PublishFaultResponse(fault_id_);
    // Stop timer so the fault is only triggered once. Timer will be restarted
    // once a heartbeart from the node is received
    timer_.stop();
    hb_fault_occurring_ = true;
  }
}

SysMonitor::Fault::Fault(std::string const& node_name, bool const blocking,
                                          ff_msgs::CommandStampedPtr response) :
  node_name_(node_name),
  blocking_(blocking),
  response_(response) {
}
}  // namespace sys_monitor

PLUGINLIB_EXPORT_CLASS(sys_monitor::SysMonitor, nodelet::Nodelet)
