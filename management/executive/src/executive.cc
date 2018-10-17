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

#include <jsonloader/planio.h>

#include "executive/executive.h"
#include "executive/op_state.h"
#include "executive/op_state_repo.h"

namespace executive {

Executive::Executive() :
  ff_util::FreeFlyerNodelet(NODE_EXECUTIVE, true),
  state_(OpStateRepo::Instance()->ready()),
  sys_monitor_init_fault_response_(new ff_msgs::CommandStamped()),
  sys_monitor_heartbeat_fault_response_(new ff_msgs::CommandStamped()),
  dock_state_(NULL),
  motion_state_(NULL),
  action_active_timeout_(1),
  arm_feedback_timeout_(4),
  motion_feedback_timeout_(1),
  dock_result_timeout_(360),
  perch_result_timeout_(360),
  switch_result_timeout_(30),
  pub_queue_size_(10),
  sub_queue_size_(10) {
}

Executive::~Executive() {
}

void Executive::CmdCallback(ff_msgs::CommandStampedPtr const& cmd) {
  SetOpState(state_->HandleCmd(cmd));
}


void Executive::DockStateCallback(ff_msgs::DockStateConstPtr const& state) {
  dock_state_ = state;

  // Check to see if the dock state is docking/docked/undocking. If it is, we
  // can change the mobility state to docking/docked.
  // Docking max state signifies that we are docking
  if (dock_state_->state <= ff_msgs::DockState::DOCKING_MAX_STATE &&
      dock_state_->state > ff_msgs::DockState::UNDOCKED) {
    SetMobilityState(ff_msgs::MobilityState::DOCKING, dock_state_->state);
    // TODO(Katie) Possible check the perching state to make sure it doesn't say
    // perched.
  }

  // If the dock state is undocked, the mobility state needs to be set to
  // whatever the motion state is.
  if (dock_state_->state == ff_msgs::DockState::UNDOCKED) {
    SetMobilityState();
  }
}

void Executive::GuestScienceAckCallback(ff_msgs::AckStampedConstPtr const&
                                                                          ack) {
  // TODO(Katie) Add code to change op state if sucessfully stopped or started
  // a primary guest apk. Probably need to subscribe to the guest science
  // config. Possibly remove this since it will probably be replaced by an
  // action
  if (ack->cmd_id == "plan") {
    SetOpState(state_->HandleGuestScienceAck(ack));
  } else {
    cmd_ack_pub_.publish(ack);
  }
}

void Executive::MotionStateCallback(ff_msgs::MotionStatePtr const& state) {
  if (motion_state_ == NULL) {
    motion_state_ = state;
  }

  // Check the current motion state to see if it maps to our mobility state. If
  // it does, set the mobility state and our stored motion state.
  if (state->state == ff_msgs::MotionState::INITIALIZING) {
    // Set motion state to idle a.k.a mobility state to drifting when the
    // mobility subsystem is initializing
    motion_state_->state = ff_msgs::MotionState::IDLE;
  } else if (state->state == ff_msgs::MotionState::IDLE ||
             state->state == ff_msgs::MotionState::IDLING ||
             state->state == ff_msgs::MotionState::STOPPED ||
             state->state == ff_msgs::MotionState::STOPPING ||
             state->state == ff_msgs::MotionState::CONTROLLING ||
             state->state == ff_msgs::MotionState::BOOTSTRAPPING) {
    motion_state_->state = state->state;
  }

  // Check to see if we are docking or undocking. If we are, don't use the
  // motion state as our mobility state.
  if (dock_state_ != NULL) {
    if (dock_state_->state > ff_msgs::DockState::UNDOCKED) {
      // If dock state is unknown or initializing, use motion state to set
      // mobility state
      if (dock_state_->state != ff_msgs::DockState::UNKNOWN &&
          dock_state_->state != ff_msgs::DockState::INITIALIZING) {
        return;
      }
    }
  }

  // Set mobility state
  SetMobilityState();
}

void Executive::PlanCallback(ff_msgs::CompressedFileConstPtr const& plan) {
  plan_ = plan;

  cf_ack_.header.stamp = ros::Time::now();
  cf_ack_.id = plan_->id;
  cf_ack_pub_.publish(cf_ack_);
}

void Executive::SysMonitorHeartbeatCallback(
                                  ff_msgs::HeartbeatConstPtr const& heartbeat) {
  sys_monitor_heartbeat_timer_.stop();

  // Stop the startup timer everytime since it isn't an expensive operation
  sys_monitor_startup_timer_.stop();

  // System monitor has only one fault and it is an initialization fault. Thus
  // if there is a fault in the fault array, the executive needs to trigger the
  // system monitor initialization fault.
  if (heartbeat->faults.size() > 0) {
    CmdCallback(sys_monitor_init_fault_response_);
    return;
  }
  sys_monitor_heartbeat_timer_.start();
}

void Executive::SysMonitorHeartbeatTimeoutCallback(ros::TimerEvent const& te) {
  // If the executive doesn't receive a heartbeat from the system monitor, it
  // needs to trigger the system monitor heartbeat missed fault.
  CmdCallback(sys_monitor_heartbeat_fault_response_);
  sys_monitor_heartbeat_timer_.stop();
}

void Executive::SysMonitorStartupTimeoutCallback(ros::TimerEvent const& te) {
  // If the system monitor didn't startup in a reasonable amount of time,
  // trigger the system monitor heartbeat missed fault response.
  CmdCallback(sys_monitor_heartbeat_fault_response_);
}

void Executive::ZonesCallback(ff_msgs::CompressedFileConstPtr const& zones) {
  zones_ = zones;

  cf_ack_.header.stamp = ros::Time::now();
  cf_ack_.id = zones_->id;
  cf_ack_pub_.publish(cf_ack_);
}

// TODO(Katie) Add stow check
bool Executive::FillArmGoal(ff_msgs::CommandStampedPtr const& cmd,
                            std::string& err_msg,
                            bool plan) {
  bool successful = true;
  if (cmd->cmd_name == CommandConstants::CMD_NAME_ARM_PAN_AND_TILT) {
    // Pan and tilt command has 3 arguments: one is the pan value, one is the
    // tilt value, and the last is a string specifying whether to pan, tilt or
    // both
    if (cmd->args.size() != 3 ||
        cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT ||
        cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT ||
        cmd->args[2].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
      successful = false;
      err_msg = "Malformed arguments for pan and tilt command!";
    } else {
      // Don't check pan and tilt values since limits may change. The arm node
      // is responsible for checking these values.
      // Convert to radians
      arm_goal_.pan = cmd->args[0].f;
      arm_goal_.tilt = cmd->args[1].f;
      if (cmd->args[2].s == "Pan" || cmd->args[2].s == "pan") {
        arm_goal_.command = ff_msgs::ArmGoal::ARM_PAN;
      } else if (cmd->args[2].s == "Tilt" || cmd->args[2].s == "tilt") {
        arm_goal_.command = ff_msgs::ArmGoal::ARM_TILT;
      } else if (cmd->args[2].s == "Both" || cmd->args[2].s == "both") {
        arm_goal_.command = ff_msgs::ArmGoal::ARM_MOVE;
      } else {
        successful = false;
        err_msg = "Unrecognized which parameter in pan and tilt command. Got: "
          + cmd->args[2].s;
      }
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_GRIPPER_CONTROL) {
    // Gripper control has one argument which is a booleanused to specify
    // whether to open or close the arm
    if (cmd->args.size() != 1 ||
       cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
      successful = false;
      err_msg = "Malformed arguments for gripper control command!";
    } else {
      // True means open
      if (cmd->args[0].b) {
        arm_goal_.command = ff_msgs::ArmGoal::GRIPPER_OPEN;
      } else {
        arm_goal_.command = ff_msgs::ArmGoal::GRIPPER_CLOSE;
      }
    }
  } else {
    successful = false;
    err_msg = "Arm command not recognized in fill arm goal.";
  }

  if (!successful && !plan) {
    NODELET_ERROR("%s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  err_msg);
  }
  return successful;
}

bool Executive::FillDockGoal(ff_msgs::CommandStampedPtr const& cmd,
                             std::string& err_msg,
                             bool plan) {
  bool successful = true;
  if (cmd->cmd_name == CommandConstants::CMD_NAME_DOCK) {
    if (cmd->args.size() != 1 ||
        cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_INT) {
      successful = false;
      err_msg = "Malformed argument for dock command in plan.";
    }

    dock_goal_.command = ff_msgs::DockGoal::DOCK;
    if (cmd->args[0].i == 1) {
      dock_goal_.berth = ff_msgs::DockGoal::BERTH_1;
    } else if (cmd->args[0].i == 2) {
      dock_goal_.berth = ff_msgs::DockGoal::BERTH_2;
    } else {
      successful = false;
      err_msg = "Berth must be 1 or 2 not " +  std::to_string(cmd->args[0].i);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_UNDOCK) {
    dock_goal_.command = ff_msgs::DockGoal::UNDOCK;
    // We don't need a berth to undock
    dock_goal_.berth = ff_msgs::DockGoal::BERTH_UNKNOWN;
  } else {
    successful = false;
    err_msg = "Dock command not recognized in fill dock goal.";
  }

  if (!successful && !plan) {
    NODELET_ERROR("%s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  err_msg);
  }

  return successful;
}

bool Executive::FillMotionGoal(Action action,
                               ff_msgs::CommandStampedPtr const& cmd) {
  jsonloader::Segment segment;
  // Flight mode needs to be set for all motion actions
  motion_goal_.flight_mode = agent_state_.flight_mode;

  switch (action) {
    case EXECUTE:
      segment = sequencer_.CurrentSegment();

      motion_goal_.command = ff_msgs::MotionGoal::EXEC;
      // Convert JSON to a segment type
      motion_goal_.segment =
                    sequencer::Segment2Trajectory(sequencer_.CurrentSegment());

      motion_goal_.states.clear();
      break;
    case IDLE:
      // Need to set flight mode to off so the PMCs shutdown
      motion_goal_.flight_mode = ff_msgs::MotionGoal::OFF;
      motion_goal_.command = ff_msgs::MotionGoal::IDLE;
      break;
    case STOP:
      motion_goal_.command = ff_msgs::MotionGoal::STOP;
      break;
    case MOVE:
      if (cmd == nullptr) {
        ROS_ERROR("Executive: move cmd is null in fill motion goal.");
        return false;
      }

      if (cmd->args.size() != 4 ||
          cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
          cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_VEC3d ||
          cmd->args[2].data_type != ff_msgs::CommandArg::DATA_TYPE_VEC3d ||
          cmd->args[3].data_type != ff_msgs::CommandArg::DATA_TYPE_MAT33f) {
        ROS_ERROR("Executive: Malformed arguments for simple move 6dof cmd!");
        PublishCmdAck(cmd->cmd_id,
                      ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                      "Malformed arguments for simple move 6dof command!");
        return false;
      }

      motion_goal_.command = ff_msgs::MotionGoal::MOVE;
      if (motion_goal_.states.size() != 1) {
        motion_goal_.states.resize(1);
      }

      motion_goal_.states[0].header.stamp = ros::Time::now();
      // Copying the target location to the goal
      motion_goal_.states[0].pose.position.x = cmd->args[1].vec3d[0];
      motion_goal_.states[0].pose.position.y = cmd->args[1].vec3d[1];
      motion_goal_.states[0].pose.position.z = cmd->args[1].vec3d[2];

      // Copying the target attitude to the goal
      motion_goal_.states[0].pose.orientation.x = cmd->args[3].mat33f[0];
      motion_goal_.states[0].pose.orientation.y = cmd->args[3].mat33f[1];
      motion_goal_.states[0].pose.orientation.z = cmd->args[3].mat33f[2];
      motion_goal_.states[0].pose.orientation.w = cmd->args[3].mat33f[3];

      motion_goal_.segment.clear();
      break;
    default:
      ROS_ERROR("Executive: Command isn't a mobility action in fill motion!");
      if (cmd != nullptr) {
        PublishCmdAck(cmd->cmd_id,
                      ff_msgs::AckCompletedStatus::EXEC_FAILED,
                      "Command isn't a mobility action in fill motion goal!");
      }
      return false;
  }
  return true;
}

bool Executive::StartAction(Action action,
                            std::string const& cmd_id,
                            std::string& err_msg,
                            bool plan) {
  bool successful = true;
  switch (action) {
    case ARM:
      if (arm_ac_.IsConnected()) {
        arm_ac_.SetCmdInfo(action, cmd_id);
        arm_ac_.SendGoal(arm_goal_);
        NODELET_DEBUG("Arm action goal sent/started.");
      } else {
        successful = false;
        err_msg = "Arm action server not connected! Arm node may have died!";
      }
      break;
    case DOCK:
    case UNDOCK:
      if (dock_ac_.IsConnected()) {
        dock_ac_.SetCmdInfo(action, cmd_id);
        dock_ac_.SendGoal(dock_goal_);
        NODELET_DEBUG("Dock action goal sent/started.");
      } else {
        successful = false;
        err_msg = "Dock action server not connected! Dock node may have died!";
      }
      break;
    case EXECUTE:
    case IDLE:
    case MOVE:
    case STOP:
      if (motion_ac_.IsConnected()) {
        motion_ac_.SetCmdInfo(action, cmd_id);
        motion_ac_.SendGoal(motion_goal_);
        NODELET_DEBUG("Motion action %i goal sent/started.",
                                                          motion_goal_.command);
      } else {
        successful = false;
        err_msg = "Motion action server not connected! Node may have died!";
      }
      break;
    case PERCH:
    case UNPERCH:
      // TODO(Katie) Add Me
      break;
    case SWITCH:
      if (switch_ac_.IsConnected()) {
        switch_ac_.SetCmdInfo(action, cmd_id);
        switch_ac_.SendGoal(switch_goal_);
        NODELET_DEBUG("Switch action goal sent/started.");
      } else {
         successful = false;
         err_msg = "Switch action server not connected! Node may have died!";
      }
      break;
    default:
      successful = false;
      err_msg = "Action to start not recognized!";
  }

  if (!successful && !plan) {
    NODELET_ERROR("%s", err_msg.c_str());
    PublishCmdAck(cmd_id, ff_msgs::AckCompletedStatus::EXEC_FAILED,
                  err_msg);
  } else if (successful && !plan) {
    PublishCmdAck(cmd_id, ff_msgs::AckCompletedStatus::NOT, "",
                  ff_msgs::AckStatus::EXECUTING);
  }

  if (successful) {
    // Add action to actions running so we know what actions are running
    running_actions_.push_back(action);
  }

  return successful;
}

bool Executive::IsActionRunning(Action action) {
  for (unsigned int i = 0; i < running_actions_.size(); i++) {
    if (action == running_actions_[i]) {
      return true;
    }
  }
  return false;
}

bool Executive::AreActionsRunning() {
  if (running_actions_.size() > 0) {
    return true;
  }
  return false;
}

void Executive::CancelAction(Action action) {
  // We don't receive a result if we cancel an action so we need to ack the
  // command id that started the action as canceled and remove the action
  // from the running actions vector.
  RemoveAction(action);
  switch (action) {
    case ARM:
      arm_ac_.CancelGoal();
      PublishCmdAck(arm_ac_.cmd_id(),
                    ff_msgs::AckCompletedStatus::CANCELED,
                    "Arm action was canceled.");
      arm_ac_.SetCmdInfo(NONE, "");
      break;
    case DOCK:
    case UNDOCK:
      dock_ac_.CancelGoal();
      PublishCmdAck(dock_ac_.cmd_id(),
                    ff_msgs::AckCompletedStatus::CANCELED,
                    "Dock action was canceled.");
      dock_ac_.SetCmdInfo(NONE, "");
      break;
    case EXECUTE:
    case MOVE:
    case STOP:
      motion_ac_.CancelGoal();
      PublishCmdAck(motion_ac_.cmd_id(),
                    ff_msgs::AckCompletedStatus::CANCELED,
                    "Motion action was canceled.");
      motion_ac_.SetCmdInfo(NONE, "");
      break;
    case PERCH:
    case UNPERCH:
      // TODO(Katie) Add Me
      break;
    default:
      NODELET_ERROR("Action to cancel not recognized!");
      return;
  }
}

bool Executive::RemoveAction(Action action) {
  unsigned int i = 0;

  bool found = false;
  for (i = 0; i < running_actions_.size(); i++) {
    if (action == running_actions_[i]) {
      found = true;
      break;
    }
  }

  if (found) {
    running_actions_.erase(running_actions_.begin() + i);
  } else {
    NODELET_ERROR_STREAM("Action " << action <<
                                            " not in running actions vector!");
  }
  return found;
}

void Executive::ArmResultCallback(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::ArmResultConstPtr const& result) {
  std::string response = "";
  // Remove action from the running action vector
  RemoveAction(ARM);

  // Get response for op state functions
  if (result != nullptr) {
    response = std::to_string(result->response);
  }

  SetOpState(state_->HandleResult(state,
                                  response,
                                  arm_ac_.cmd_id(),
                                  ARM));
  // Reset command id so we don't try to ack the same id twice
  arm_ac_.SetCmdInfo(NONE, "");
}

void Executive::DockResultCallback(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::DockResultConstPtr const& result) {
  std::string response = "";
  // Remove action from the running action vector
  RemoveAction(dock_ac_.action());

  // Get response for op state functions
  if (result != nullptr) {
    response = std::to_string(result->response);
  }

  SetOpState(state_->HandleResult(state,
                                  response,
                                  dock_ac_.cmd_id(),
                                  dock_ac_.action()));
  // Reset command id so we don't try to ack the same id twice
  dock_ac_.SetCmdInfo(NONE, "");
}

void Executive::MotionFeedbackCallback(
                              ff_msgs::MotionFeedbackConstPtr const& feedback) {
  // The only feedback used from the motion action is the execute feedback and
  // it goes to the sequencer. Otherwise there isn't much to with the feedback
  if (motion_ac_.action() == EXECUTE) {
    sequencer_.Feedback(feedback->progress);
  }
}

void Executive::MotionResultCallback(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::MotionResultConstPtr const& result) {
  std::string response = "";
  // Remove action from the running action vector
  RemoveAction(motion_ac_.action());

  // Get response for op state functions
  if (result != nullptr) {
    response = std::to_string(result->response);
  }

  std::string cmd_id = motion_ac_.cmd_id();
  Action action = motion_ac_.action();

  // Sometimes the handle motion result starts an action so we need to clear the
  // action info before starting another action
  // Reset command id so we don't try to ack the same id twice
  motion_ac_.SetCmdInfo(NONE, "");

  SetOpState(state_->HandleResult(state,
                                  response,
                                  cmd_id,
                                  action));
}

void Executive::SwitchResultCallback(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::SwitchResultConstPtr const& result) {
  std::string response = "";
  // Remove action from the running action vector
  RemoveAction(SWITCH);

  // Get response for op state functions
  if (result != nullptr) {
    response = std::to_string(result->response);
  }

  SetOpState(state_->HandleResult(state,
                                  response,
                                  switch_ac_.cmd_id(),
                                  SWITCH));
  // Reset command id so we don't try to ack the same id twice
  switch_ac_.SetCmdInfo(NONE, "");
}

void Executive::PublishCmdAck(std::string const& cmd_id,
                              uint8_t completed_status,
                              std::string const& message,
                              uint8_t status) {
  ack_.header.stamp = ros::Time::now();
  ack_.cmd_id = cmd_id;
  ack_.status.status = status;
  ack_.completed_status.status = completed_status;
  ack_.message = message;
  cmd_ack_pub_.publish(ack_);
}

void Executive::PublishPlan() {
  plan_pub_.publish(plan_);
}

void Executive::PublishPlanStatus(uint8_t status) {
  // The sequencer sets the plan status to executing for every plan status but
  // since the executive has knowledge of if the plan has just started,
  // is paused, or is executing, the executive sets the status.
  ff_msgs::PlanStatusStamped plan_status = sequencer_.plan_status();
  plan_status.header.stamp = ros::Time::now();
  plan_status.status.status = status;
  plan_status_pub_.publish(plan_status);
}

ff_msgs::MobilityState Executive::GetMobilityState() {
  return agent_state_.mobility_state;
}

// Set the mobility state based on the stored motion state
void Executive::SetMobilityState() {
  if (motion_state_ == NULL) {
    return;
  }

  if (motion_state_->state == ff_msgs::MotionState::IDLE) {
    agent_state_.mobility_state.state = ff_msgs::MobilityState::DRIFTING;
    agent_state_.mobility_state.sub_state = 0;
  } else if (motion_state_->state == ff_msgs::MotionState::IDLING) {
    agent_state_.mobility_state.state = ff_msgs::MobilityState::DRIFTING;
    agent_state_.mobility_state.sub_state = 1;
  } else if (motion_state_->state == ff_msgs::MotionState::STOPPED) {
    agent_state_.mobility_state.state = ff_msgs::MobilityState::STOPPING;
    agent_state_.mobility_state.sub_state = 0;
  } else if (motion_state_->state == ff_msgs::MotionState::STOPPING) {
    agent_state_.mobility_state.state = ff_msgs::MobilityState::STOPPING;
    agent_state_.mobility_state.sub_state = 1;
  } else if (motion_state_->state == ff_msgs::MotionState::CONTROLLING ||
             motion_state_->state == ff_msgs::MotionState::BOOTSTRAPPING) {
    agent_state_.mobility_state.state = ff_msgs::MobilityState::FLYING;
    agent_state_.mobility_state.sub_state = 0;
  }

  PublishAgentState();
}

void Executive::SetMobilityState(uint8_t state, uint32_t sub_state) {
  agent_state_.mobility_state.state = state;
  agent_state_.mobility_state.sub_state = sub_state;
  PublishAgentState();
}

bool Executive::SetPlan() {
  if (plan_) {
    if (sequencer::LoadPlan(plan_, &sequencer_)) {
      // Set plan execution state to paused, apparently this was the way
      // spheres worked
      SetPlanExecState(ff_msgs::ExecState::PAUSED);
      // Publish plan stuff for ground
      PublishPlan();
      PublishPlanStatus(ff_msgs::AckStatus::QUEUED);
      // Clear plan so that the operator has to upload a new plan after this
      // plan is done running
      plan_.reset();
      return true;
    }
    plan_.reset();
  }
  SetPlanExecState(ff_msgs::ExecState::IDLE);
  return false;
}

void Executive::SetPlanExecState(uint8_t state) {
  agent_state_.plan_execution_state.state = state;
  PublishAgentState();
}

// Return a string with an error message, if string blank, zones were set.
std::string Executive::SetZones() {
  if (zones_) {
    ff_msgs::SetZones zones_srv;
    std::string file_contents;

    // Decompress file into a string
    if (!sequencer::DecompressData(
                          reinterpret_cast<const char*>(zones_->file.data()),
                          zones_->file.size(), zones_->type, &file_contents)) {
      // Reset zones so that the same file isn't reloaded
      zones_.reset();
      return "Unable to decompress zones file.";
    }

    // Reset zones so that the same file isn't reloaded
    zones_.reset();

    // Convert string into a json object
    Json::Value file_obj;
    if (!jsonloader::LoadData(file_contents, &file_obj)) {
      return "Error parsing json.";
    }

    // Check to make sure timestamp exists in the file
    if (!file_obj.isMember("timestamp") || !file_obj["timestamp"].isString()) {
      return "Parser error: file timestamp doesn't exist or is not a string.";
    }

    // Get timestamp in milliseconds and convert it to a number
    std::string timestamp = file_obj["timestamp"].asString();
    zones_srv.request.timestamp = MsToSec(timestamp);

    // Check to make sure zones array exists
    if (!file_obj.isMember("zones") || !file_obj["zones"].isArray()) {
      return "Parser error: zones don't exist or are not an array!";
    }

    std::string err_msg;
    ff_msgs::Zone zone;
    int i = 0;
    for (Json::Value const& zone_obj : file_obj["zones"]) {
      // Check to make sure zone name exists
      if (!zone_obj.isMember("name") || !zone_obj["name"].isString()) {
        return "Parser error: name in zone doesn't exist or is not a string.";
      }
      zone.name = zone_obj["name"].asString();

      // Check to make sure safe exists for zone
      if (!zone_obj.isMember("safe") || !zone_obj["safe"].isBool()) {
        return "Parser error: safe in zone doesn't exist or is not a boolean.";
      }

      if (zone_obj["safe"].asBool()) {
        zone.type = ff_msgs::Zone::KEEPIN;
      } else {
        zone.type = ff_msgs::Zone::KEEPOUT;
      }

      // Check to make sure the sequence exists for the zone
      if (!zone_obj.isMember("sequence") || !zone_obj["sequence"].isArray()) {
        return "Parser error: sequence in zone doesn't exist or isn't an array";
      }

      i = 0;
      for (Json::Value const& box_array : zone_obj["sequence"]) {
        zone.index = i;
        if (!box_array.isArray() || box_array.size() != 6) {
          return "Parser error: box isn't an array or doesn't have 6 points";
        }

        if (!box_array[0].isNumeric() || !box_array[1].isNumeric() ||
            !box_array[2].isNumeric() || !box_array[3].isNumeric() ||
            !box_array[4].isNumeric() || !box_array[5].isNumeric()) {
          return "Parser error: one of the box points is not numeric!";
        }

        // First 3 elements are the x, y, z of one corner and the last three
        // are the x, y, z of the other corner
        zone.min.x = box_array[0].asFloat();
        zone.min.y = box_array[1].asFloat();
        zone.min.z = box_array[2].asFloat();
        zone.max.x = box_array[3].asFloat();
        zone.max.y = box_array[4].asFloat();
        zone.max.z = box_array[5].asFloat();

        zones_srv.request.zones.push_back(zone);
        i++;
      }
    }

    // TODO(Katie) Change this! The mapper may not be running when you get this
    // command, may need to set up a timer if the service doesn't currently
    // exist
    // Check to make sure the service is valid and running
    if (!zones_client_.exists()) {
      return "Set zones service isn't running! Mapper node may have died!";
    }

    // Check to see if the timezones were actually set
    if (!zones_client_.call(zones_srv)) {
      return "Zone timestamp was older than the current zone timestamp.";
    }
    return "";
  }
  return "No zones file found.";
}

ros::Time Executive::MsToSec(std::string timestamp) {
  uint64_t time, secs, nsecs;
  time = std::stoull(timestamp);
  secs = time/1000;
  nsecs = (time % 1000) * 1000000;

  return ros::Time(secs, nsecs);
}

sequencer::ItemType Executive::GetCurrentPlanItemType() {
  return sequencer_.CurrentType();
}

ff_msgs::CommandStampedPtr Executive::GetPlanCommand() {
  return sequencer_.CurrentCommand();
}

// Returns false if there are no more command/segments in the plan
bool Executive::AckCurrentPlanItem() {
  ff_msgs::AckCompletedStatus ack;
  ack.status = ff_msgs::AckCompletedStatus::OK;
  return sequencer_.Feedback(ack);
}

uint8_t Executive::GetPlanExecState() {
  return agent_state_.plan_execution_state.state;
}

// Functions used to set variables that are used to configure mobility before a
// move or execute
bool Executive::ConfigureMobility(std::string const& cmd_id,
                                  std::string& err_msg,
                                  bool plan) {
  bool successful = true;

  // Initialize config clients if they haven't been initialized
  if (!choreographer_cfg_) {
    choreographer_cfg_ =
              std::make_shared<ff_util::ConfigClient>(&nh_, NODE_CHOREOGRAPHER);
  }

  if (!mapper_cfg_) {
    mapper_cfg_ = std::make_shared<ff_util::ConfigClient>(&nh_, NODE_MAPPER);
  }

  // Set values for configuring, these values will persist until changed
  // Choreographer
  choreographer_cfg_->Set<double>("desired_vel",
                                          agent_state_.target_linear_velocity);
  choreographer_cfg_->Set<double>("desired_accel",
                                          agent_state_.target_linear_accel);
  choreographer_cfg_->Set<double>("desired_omega",
                                          agent_state_.target_angular_velocity);
  choreographer_cfg_->Set<double>("desired_alpha",
                                          agent_state_.target_angular_accel);
  choreographer_cfg_->Set<bool>("enable_faceforward",
                                              !agent_state_.holonomic_enabled);
  choreographer_cfg_->Set<bool>("enable_collision_checking",
                                                  agent_state_.check_obstacles);
  choreographer_cfg_->Set<bool>("enable_validation", agent_state_.check_zones);
  choreographer_cfg_->Set<bool>("enable_timesync",
                                                agent_state_.time_sync_enabled);
  choreographer_cfg_->Set<bool>("enable_immediate",
                                                agent_state_.immediate_enabled);
  choreographer_cfg_->Set<std::string>("planner", agent_state_.planner);
  // This function is not used for the first segment of a plan so always disable
  // move to start
  choreographer_cfg_->Set<bool>("enable_bootstrapping", false);
  choreographer_cfg_->Set<bool>("enable_replanning", false);

  // Mapper
  mapper_cfg_->Set<double>("inflate_radius", agent_state_.collision_distance);

  // Clear err_msg
  err_msg = "";

  // Reconfigure choreographer, mapper
  if (!choreographer_cfg_->Reconfigure()) {
    successful = false;
    err_msg = "Couldn't configure the mobilty::choreographer node! ";
  }

  if (!mapper_cfg_->Reconfigure()) {
    successful = false;
    err_msg += "Couldn't configure the mobility::mapper node!";
  }

  // Ack error
  if (!successful && !plan) {
    NODELET_ERROR("%s", err_msg.c_str());
    PublishCmdAck(cmd_id,
                  ff_msgs::AckCompletedStatus::EXEC_FAILED,
                  err_msg);
  }

  return successful;
}

bool Executive::ConfigureMobility(bool move_to_start,
                                  bool enable_holonomic,
                                  std::string& err_msg) {
  bool successful = true;

  // TODO(Katie) Change when Ted changes the sequencer

  // Initialize config clients if they haven't been initialized
  if (!choreographer_cfg_) {
    choreographer_cfg_ =
              std::make_shared<ff_util::ConfigClient>(&nh_, NODE_CHOREOGRAPHER);
  }

  if (!mapper_cfg_) {
    mapper_cfg_ =
                  std::make_shared<ff_util::ConfigClient>(&nh_, NODE_MAPPER);
  }

  // Set values for configuring, these values will persist until changed
  // Choreographer
  choreographer_cfg_->Set<double>("desired_vel",
                                          agent_state_.target_linear_velocity);
  choreographer_cfg_->Set<double>("desired_accel",
                                          agent_state_.target_linear_accel);
  choreographer_cfg_->Set<double>("desired_omega",
                                          agent_state_.target_angular_velocity);
  choreographer_cfg_->Set<double>("desired_alpha",
                                          agent_state_.target_angular_accel);
  choreographer_cfg_->Set<bool>("enable_faceforward", enable_holonomic);
  choreographer_cfg_->Set<bool>("enable_collision_checking",
                                                  agent_state_.check_obstacles);
  choreographer_cfg_->Set<bool>("enable_validation", agent_state_.check_zones);
  choreographer_cfg_->Set<bool>("enable_timesync",
                                                agent_state_.time_sync_enabled);
  choreographer_cfg_->Set<bool>("enable_immediate",
                                                agent_state_.immediate_enabled);
  choreographer_cfg_->Set<std::string>("planner", agent_state_.planner);
  choreographer_cfg_->Set<bool>("enable_bootstrapping", move_to_start);
  choreographer_cfg_->Set<bool>("enable_replanning", false);

  // Mapper
  mapper_cfg_->Set<double>("inflate_radius", agent_state_.collision_distance);

  // Clear err_msg
  err_msg = "";

  // Reconfigure choreographer, planner, mapper
  if (!choreographer_cfg_->Reconfigure()) {
    successful = false;
    err_msg = "Couldn't configure the mobilty::choreographer node! ";
  }

  if (!mapper_cfg_->Reconfigure()) {
    successful = false;
    err_msg += "Couldn't configure the mobility::mapper node!";
  }

  return successful;
}

bool Executive::SetCheckObstacles(ff_msgs::CommandStampedPtr const& cmd) {
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
    NODELET_ERROR("Malformed arguments for set check obstacles!");
    PublishCmdAck(cmd->cmd_id,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  "Malformed arguments for set check obstacles!");
    return false;
  }

  agent_state_.check_obstacles = cmd->args[0].b;
  PublishAgentState();
  PublishCmdAck(cmd->cmd_id);
  return true;
}

bool Executive::SetCheckZones(ff_msgs::CommandStampedPtr const& cmd) {
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
    NODELET_ERROR("Malformed arguments for set check zones!");
    PublishCmdAck(cmd->cmd_id,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  "Malformed arguments for set check zones!");
    return false;
  }

  agent_state_.check_zones = cmd->args[0].b;
  PublishAgentState();
  PublishCmdAck(cmd->cmd_id);
  return true;
}

bool Executive::SetEnableAutoReturn(ff_msgs::CommandStampedPtr const& cmd) {
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
    NODELET_ERROR("Malformed arguments for enable auto return command!");
    PublishCmdAck(cmd->cmd_id,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  "Malformed arguments for enable auto return command!");
    return false;
  }

  agent_state_.auto_return_enabled = cmd->args[0].b;
  PublishAgentState();
  PublishCmdAck(cmd->cmd_id);
  return true;
}

bool Executive::SetEnableImmediate(ff_msgs::CommandStampedPtr const& cmd) {
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
    NODELET_ERROR("Malformed arguments for enable immediate command!");
    PublishCmdAck(cmd->cmd_id,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  "Malformed arguments for enable immediate command!");
    return false;
  }

  agent_state_.immediate_enabled = cmd->args[0].b;
  PublishAgentState();
  PublishCmdAck(cmd->cmd_id);
  return true;
}

bool Executive::SetHolonomicMode(ff_msgs::CommandStampedPtr const& cmd) {
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
    NODELET_ERROR("Malformed arguments for set holonomic mode command!");
    PublishCmdAck(cmd->cmd_id,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  "Malformed arguments for set holonomic mode command!");
    return false;
  }

  agent_state_.holonomic_enabled = cmd->args[0].b;
  PublishAgentState();
  PublishCmdAck(cmd->cmd_id);
  return true;
}

bool Executive::SetOperatingLimits(std::vector<ff_msgs::CommandArg> const&
                                                                conditions,
                                   std::string& err_msg) {
  if (conditions[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
      conditions[1].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
      conditions[2].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT  ||
      conditions[3].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT  ||
      conditions[4].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT  ||
      conditions[5].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT  ||
      conditions[6].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT) {
    err_msg = "Malformed arguments for set operating limits command!";
    return false;
  }

  // Check to make sure the flight mode exists before setting everything
  ff_msgs::FlightMode mode;
  if (!ff_util::FlightUtil::GetFlightMode(mode, conditions[1].s)) {
    err_msg = "Flight mode " + conditions[1].s +" doesn't exist!.";
    return false;
  }

  // string profile name
  agent_state_.profile_name = conditions[0].s;
  // string flight mode
  agent_state_.flight_mode = conditions[1].s;
  // target linear velocity
  agent_state_.target_linear_velocity = conditions[2].f;
  // target linear acceleration
  agent_state_.target_linear_accel = conditions[3].f;
  // target angular velocity
  agent_state_.target_angular_velocity = conditions[4].f;
  // target angular acceleration
  agent_state_.target_angular_accel = conditions[5].f;
  // collision distance
  agent_state_.collision_distance = conditions[6].f;

  PublishAgentState();

  return true;
}

bool Executive::SetPlanner(ff_msgs::CommandStampedPtr const& cmd) {
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
    NODELET_ERROR("Malformed arguments for set planner command!");
    PublishCmdAck(cmd->cmd_id,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  "Malformed arguments for set planner command!");
    return false;
  }

  // Check that the planner string is valid
  if (cmd->args[0].s != CommandConstants::PARAM_NAME_PLANNER_TYPE_TRAPEZOIDAL &&
      cmd->args[0].s !=
                CommandConstants::PARAM_NAME_PLANNER_TYPE_QUADRATIC_PROGRAM) {
    NODELET_ERROR("Planner must be either Trapezoidal or QuadraticProgram");
    PublishCmdAck(cmd->cmd_id,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  "Planner must be either Trapezoidal or QuadraticProgram");
    return false;
  }

  agent_state_.planner = cmd->args[0].s;
  PublishAgentState();
  PublishCmdAck(cmd->cmd_id);
  return true;
}

bool Executive::SetTimeSync(ff_msgs::CommandStampedPtr const& cmd) {
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
    NODELET_ERROR("Malformed arguments for enable time sync command!");
    PublishCmdAck(cmd->cmd_id,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  "Malformed arguments for enable time sync command!");
    return false;
  }

  agent_state_.time_sync_enabled = cmd->args[0].b;
  PublishAgentState();
  PublishCmdAck(cmd->cmd_id);
  return true;
}

void Executive::ResetEkf(std::string const& cmd_id) {
  // Check to make sure the service is valid and running
  if (!reset_ekf_client_.exists()) {
    PublishCmdAck(cmd_id,
                  ff_msgs::AckCompletedStatus::EXEC_FAILED,
                  "Reset ekf service isn't running! Ekf node may have died.");
    return;
  }

  std_srvs::Empty::Request req;
  std_srvs::Empty::Response res;

  // Check to see if the reset succeeded
  if (!reset_ekf_client_.call(req, res)) {
    PublishCmdAck(cmd_id,
                  ff_msgs::AckCompletedStatus::EXEC_FAILED,
                  "Resetting the ekf failed.");
    return;
  }

  // Reset ekf succeeded so ack the reaquire position command as successful
  PublishCmdAck(cmd_id);
}

void Executive::StartWaitTimer(float duration) {
  wait_timer_.setPeriod(ros::Duration(duration));
  wait_timer_.start();
}

void Executive::StopWaitTimer() {
  wait_timer_.stop();
}

void Executive::WaitCallback(ros::TimerEvent const& te) {
  wait_timer_.stop();
  SetOpState(state_->HandleWaitCallback());
}

// Stop all motion is a tricky command since we may have multiple actions
// running at one time. We also use stop to transition from idle to stopped
// so we will wanted to start a stop pretty much all the time. The only time
// that we don't send a stop is when we are already trying to stop, trying to
// idle, docked, perched, docking and deactiviting the pmcs, or undocking and
// haven't activated the pmcs yet.
// This function is also as pause for a plan so if the plan flag is set, we
// need to check if we are downloading data and if so, stop it.
bool Executive::StopAllMotion(bool &stop_started,
                              std::string const& cmd_id,
                              std::string const& cmd_src,
                              bool plan) {
  // We pretty much always start stop action even if stopped. See cases below
  // for situations we don't want to stop in
  bool successful = true, start_stop = true;
  std::string err_msg;

  // The first thing we need to check is if the stop was a fault response and if
  // it is, we need to check if mobility is idle. If mobility is idle, we don't
  // want to spin up the pmcs by excuting a stop
  if (cmd_src == "sys_monitor") {
    if (agent_state_.mobility_state.state == ff_msgs::MobilityState::DRIFTING) {
      // Ack command as failed and don't stop
      PublishCmdAck(cmd_id,
                    ff_msgs::AckCompletedStatus::EXEC_FAILED,
                    "PMCs are idle so don't want spin them up due to a fault.");
      stop_started = false;
      return false;
    }
  }

  // If an action is executing, we want to cancel it. We will not wait for a
  // result as not waiting simplifies the code so start a stop after canceling
  // the running actions.

  // If we are perched or docked, we will be idle and don't want to
  // spin up the motors to try to stop so we need to fail the stop command in
  // this case
  if (agent_state_.mobility_state.state == ff_msgs::MobilityState::DOCKING &&
      agent_state_.mobility_state.sub_state == 0) {
    err_msg = "Astrobee cannot stop while docked.";
    start_stop = false;
    successful = false;
    // Will check if we started the undock action but haven't received any
    // feedback in main for loop
  } else if (agent_state_.mobility_state.state ==
                                            ff_msgs::MobilityState::PERCHING &&
             agent_state_.mobility_state.sub_state == 0) {
    err_msg = "Astrobee cannot stop while perched.";
    start_stop = false;
    successful = false;
    // Will check if we started the unperch action but haven't received any
    // feedback in main for loop. Will also check if we are executing arm action
    // in main loop
  }

  // Stop all motion stops both mobility and the arm so we need to check if
  // both are in use
  unsigned int i = 0;
  for (i = 0; i < running_actions_.size(); i++) {
    if (running_actions_[i] == ARM) {
      // Arm node doesn't have a stop, cancelling the current goal will stop the
      // arm
      CancelAction(ARM);
      i--;
      // Set successful to true since it may have been set to false in perch
      // check
      successful = true;
    } else if (running_actions_[i] == DOCK) {
      // Don't stop if we are deactivating PMC. Docker doesn't count down so
      // need to do some math to check this
      if (agent_state_.mobility_state.sub_state <=
                        ff_msgs::DockState::DOCKING_WAITING_FOR_SPIN_DOWN) {
        err_msg = "Already deactivating pmcs. Cannot stop.";
        start_stop = false;
        successful = false;
      } else {
        CancelAction(DOCK);
        i--;
      }
    } else if (running_actions_[i] == EXECUTE) {
      CancelAction(EXECUTE);
      i--;
    } else if (running_actions_[i] == IDLE) {
      err_msg = "Cannot stop while trying to idle.";
      successful = false;
      start_stop = false;
    } else if (running_actions_[i] == MOVE) {
      CancelAction(MOVE);
      i--;
    } else if (running_actions_[i] == PERCH) {
      // TODO(Katie) Fill when perch implemented! Be sure to follow what did
      // for dock
    } else if (running_actions_[i] == STOP) {
      err_msg = "Already stopping.";
      successful = false;
      start_stop = false;
    } else if (running_actions_[i] == UNDOCK) {
      CancelAction(UNDOCK);
      i--;
      // Figure out where we are in the undock process and only send a stop if
      // we are in or have completed the egressing state
      // Invert sub state since that is what we do when we set the substate for
      // undocking
      if (agent_state_.mobility_state.sub_state >
                    ff_msgs::DockState::UNDOCKING_MOVING_TO_APPROACH_POSE) {
        start_stop = false;
        // Set successful to true since it may have been set to false in the
       // if docked statement
        successful = true;
        // Set mobility state to docked
        agent_state_.mobility_state.sub_state = 0;
      } else {  // Off dock so we need to send a stop
        // Set successful and start stop to true since it may have been set to
        // false in the if docked checked
        successful = true;
        start_stop = true;
      }
    } else if (running_actions_[i] == UNPERCH) {
      // TODO(Katie) Fill when unperch implemented! Be sure to follow what you
      // did for undock
    }
  }

  if (plan) {
    // TODO(Katie) Check to see if we are downloading data and cancel the
    // action. Only check this for the plan because a pause and stop
    // command are the same and if a operator has started the download in
    // the teleop tab then they can find the stop download button in the
    // teleop tab
  }

  // Ack before start action since start action will ack for us if it fails
  if (!successful) {
    NODELET_ERROR("%s", err_msg.c_str());
    PublishCmdAck(cmd_id,
                  ff_msgs::AckCompletedStatus::EXEC_FAILED,
                  err_msg);
  } else if (successful && !start_stop) {
    // Ack successful since we cancelled an action that hadn't moved the robot
    PublishCmdAck(cmd_id);
  }

  stop_started = false;
  if (start_stop) {
    if (FillMotionGoal(STOP)) {
      successful = StartAction(STOP, cmd_id, err_msg);
      if (successful) {
        stop_started = true;
      }
    }
  }

  return successful;
}

bool Executive::Dock(ff_msgs::CommandStampedPtr const& cmd,
                     std::string& err_msg,
                     uint8_t& completed_status,
                     bool plan) {
  bool stopped = false;

  // Make sure robot is stopped before attempting to dock. Only accept dock in
  // ready op state so perched, docked, or drifting are the only mobility states
  // we need to check for
  if (agent_state_.mobility_state.state == ff_msgs::MobilityState::DOCKING) {
    err_msg = "Already docked.";
    completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
  } else if (agent_state_.mobility_state.state ==
                                            ff_msgs::MobilityState::PERCHING) {
    err_msg = "Astrobee cannot attempt to dock while it is perched.";
    completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
  } else {
    stopped = true;
    if (!FillDockGoal(cmd, err_msg, plan)) {
      return false;
    }

    if (!StartAction(DOCK, cmd->cmd_id, err_msg, plan)) {
      return false;
    }
  }

  // Fill dock goal and start action publish failed command acks so we only
  // need to fail an ack if we aren't stopped and thus cannot dock
  if (!stopped && !plan) {
    NODELET_ERROR("%s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id, completed_status, err_msg);
  }
  return stopped;
}

bool Executive::Undock(ff_msgs::CommandStampedPtr const& cmd,
                       std::string& err_msg,
                       bool plan) {
  bool docked = false;

  // Make sure robot is docked before attempted to undock. Only accept undock
  // ready op state so only need to check perched, stopped, or drifting
  if (agent_state_.mobility_state.state == ff_msgs::MobilityState::DRIFTING) {
    err_msg = "Can't undock when not docked. Astrobee is currently drifting.";
  } else if (agent_state_.mobility_state.state ==
                                            ff_msgs::MobilityState::PERCHING) {
    err_msg = "Can't undock when not docked. Astrobee is currently perched.";
  } else if (agent_state_.mobility_state.state ==
                                            ff_msgs::MobilityState::STOPPING) {
    err_msg = "Can't undock when not docked. Astrobee is currently stopped.";
  } else {
    docked = true;
    if (!FillDockGoal(cmd, err_msg, plan)) {
      return false;
    }

    if (!StartAction(UNDOCK, cmd->cmd_id, err_msg, plan)) {
      return false;
    }
  }

  // Start action publish failed command acks so we only need to fail an ack if
  // we aren't docked and thus cannot undock
  if (!docked && !plan) {
    NODELET_ERROR("%s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id,
                  ff_msgs::AckCompletedStatus::EXEC_FAILED,
                  err_msg);
  }
  return docked;
}

void Executive::StopArm(std::string const& cmd_id) {
  // TODO(Katie) stub, add actual code
  NODELET_WARN("Stop arm not implemented yet! Stay tuned!");
  PublishCmdAck(cmd_id,
                ff_msgs::AckCompletedStatus::EXEC_FAILED,
                "Stop arm not implemented yet! Stay tuned!");
}

void Executive::StowArm(std::string const& cmd_id) {
  // TODO(Katie) stub, add actual code
  NODELET_WARN("Stow arm not implemented yet! Stay tuned!");
  PublishCmdAck(cmd_id,
                ff_msgs::AckCompletedStatus::EXEC_FAILED,
                "Stow arm not implemented yet!");
}

void Executive::SkipPlanStep(std::string const& cmd_id) {
  // Make sure plan execution state is paused
  if (GetPlanExecState() != ff_msgs::ExecState::PAUSED) {
    NODELET_ERROR("Got command to skip plan step but plan not paused.");
    PublishCmdAck(cmd_id,
                  ff_msgs::AckCompletedStatus::EXEC_FAILED,
                  "Got command to skip a plan step but plan isn't paused.");
    return;
  }

  ff_msgs::AckCompletedStatus ack;
  ack.status = ff_msgs::AckCompletedStatus::CANCELED;
  // Check to see if we are skipping the last step in the plan
  if (sequencer_.Feedback(ack)) {
    PublishPlanStatus(ff_msgs::AckStatus::QUEUED);
  } else {
    PublishPlanStatus(ff_msgs::AckStatus::COMPLETED);
    SetPlanExecState(ff_msgs::ExecState::IDLE);
  }
  PublishCmdAck(cmd_id);
}

bool Executive::DownloadData(ff_msgs::CommandStampedPtr const& cmd,
                             std::string& err_msg,
                             uint8_t& completed_status,
                             bool plan) {
  bool successful = true;
  // Check to make sure command is formatted as expected
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
    successful = false;
    err_msg = "Malformed arguments for download data command!";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else if (cmd->args[0].s !=
                          CommandConstants::PARAM_NAME_DOWNLOAD_METHOD_IMMEDIATE
      && cmd->args[0].s !=
                        CommandConstants::PARAM_NAME_DOWNLOAD_METHOD_DELAYED) {
    successful = false;
    err_msg = "Download method not recognized. Needs to be immediate or delay.";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else if (agent_state_.mobility_state.state !=
                                              ff_msgs::MobilityState::DOCKING ||
            (agent_state_.mobility_state.state ==
                                              ff_msgs::MobilityState::DOCKING &&
             agent_state_.mobility_state.sub_state != 0)) {
    // Can only download data when docked
    successful = false;
    err_msg = "Not docked! Needed to be docked in order to download data.";
    completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
  } else {
    // TODO(Katie) Stub, change to be actual code, including setting a class
    // variable to tell if we are downloading data and what kind of data
    successful = true;
    NODELET_ERROR("Download data not implemented yet!");
    completed_status = ff_msgs::AckCompletedStatus::OK;
  }

  if (!successful && !plan) {
    NODELET_ERROR("%s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id, completed_status, err_msg);
  } else if (successful && !plan) {
    PublishCmdAck(cmd->cmd_id);
  }
  return successful;
}

void Executive::StopDownload(ff_msgs::CommandStampedPtr const& cmd) {
  std::string err_msg;
  // Check to make sure command is formatted as expected
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
    err_msg = "Malformed arguments for stop download command!";
    NODELET_ERROR("%s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  err_msg);
  }

  if (cmd->args[0].s != CommandConstants::PARAM_NAME_DOWNLOAD_METHOD_IMMEDIATE
      && cmd->args[0].s !=
                        CommandConstants::PARAM_NAME_DOWNLOAD_METHOD_DELAYED) {
    err_msg = "Download method not recognized. Needs to be immediate or delay.";
    NODELET_ERROR("%s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  err_msg);
  }

  // TODO(Katie) Can only stop download if download occurring, check class
  // variables to see if a download is in progress and what kind of data
  // TODO(Katie) Stub, change to be actual code
  err_msg = "Stop download not implemented yet!";
  NODELET_ERROR("%s", err_msg.c_str());
  PublishCmdAck(cmd->cmd_id,
                ff_msgs::AckCompletedStatus::EXEC_FAILED,
                err_msg);
  // err_msg = "Not downloading data! No download to stop.";
}

bool Executive::ClearData(ff_msgs::CommandStampedPtr const& cmd,
                          std::string& err_msg,
                          uint8_t& completed_status,
                          bool plan) {
  bool successful = true;
  // Check to make sure command is formatted as expected
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
    successful = false;
    err_msg = "Malformed arguments for clear data command!";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else if (cmd->args[0].s !=
                          CommandConstants::PARAM_NAME_DOWNLOAD_METHOD_IMMEDIATE
      && cmd->args[0].s !=
                        CommandConstants::PARAM_NAME_DOWNLOAD_METHOD_DELAYED) {
    successful = false;
    err_msg = "Data method not recognized. Needs to be immediate or delay.";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else {
    // TODO(Katie) Stub, change to be actual code, including setting a class
    // variable to tell if we are downloading data, cannot clear data if
    // downloading data
    successful = true;
    NODELET_ERROR("Clear data not implemented yet!");
    completed_status = ff_msgs::AckCompletedStatus::OK;
  }

  if (!successful && !plan) {
    NODELET_ERROR("%s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id, completed_status, err_msg);
  } else if (successful && !plan) {
    PublishCmdAck(cmd->cmd_id);
  }
  return successful;
}

bool Executive::PowerItem(ff_msgs::CommandStampedPtr const& cmd,
                          std::string& err_msg,
                          uint8_t& completed_status,
                          bool on,
                          bool plan) {
  // Check to make sure command is formatted as expected
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
    err_msg = "Malformed arguments for power item command!";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
    NODELET_ERROR("%s", err_msg.c_str());
    if (!plan) {
      PublishCmdAck(cmd->cmd_id, completed_status, err_msg);
    }
    return false;
  }

  NODELET_INFO("Item %s is being powered on/off!", cmd->args[0].s.c_str());

  // Handle pmcs and laser the same ince they use the same service message
  if (cmd->args[0].s ==
        CommandConstants::PARAM_NAME_POWERED_COMPONENT_LASER_POINTER ||
      cmd->args[0].s ==
        CommandConstants::PARAM_NAME_POWERED_COMPONENT_PMCS_AND_SIGNAL_LIGHTS) {
    ff_hw_msgs::SetEnabled enable_srv;
    enable_srv.request.enabled = on;

    if (cmd->args[0].s ==
                CommandConstants::PARAM_NAME_POWERED_COMPONENT_LASER_POINTER) {
      // Check to make sure the laser service is valid and running
      if (!CheckServiceExists(laser_enable_client_,
                              "Laser",
                              cmd->cmd_id,
                              err_msg,
                              completed_status)) {
        return false;
      }
      laser_enable_client_.call(enable_srv);
    } else {  // PMCS
      // Check to make sure the pmc service is valid and running
      if (!CheckServiceExists(pmc_enable_client_,
                              "PMC",
                              cmd->cmd_id,
                              err_msg,
                              completed_status)) {
        return false;
      }
      pmc_enable_client_.call(enable_srv);
    }

    // Check to see if the service was successfully enabled/disabled
    if (!enable_srv.response.success) {
      err_msg = enable_srv.response.status_message;
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      NODELET_ERROR("%s", err_msg.c_str());
      if (!plan) {
        PublishCmdAck(cmd->cmd_id, completed_status, err_msg);
      }
    }
  } else {  // Item is probably a payload
    ff_hw_msgs::ConfigurePayloadPower config_srv;
    config_srv.request.top_front = config_srv.request.PERSIST;
    config_srv.request.bottom_front = config_srv.request.PERSIST;
    config_srv.request.top_aft = config_srv.request.PERSIST;
    config_srv.request.bottom_aft = config_srv.request.PERSIST;

    uint8_t power;
    if (on) {
      power = config_srv.request.ON;
    } else {
      power = config_srv.request.OFF;
    }

    if (cmd->args[0].s ==
              CommandConstants::PARAM_NAME_POWERED_COMPONENT_PAYLOAD_TOP_AFT) {
      config_srv.request.top_aft = power;
    } else if (cmd->args[0].s ==
            CommandConstants::PARAM_NAME_POWERED_COMPONENT_PAYLOAD_BOTTOM_AFT) {
      config_srv.request.bottom_aft = power;
    } else if (cmd->args[0].s ==
          CommandConstants::PARAM_NAME_POWERED_COMPONENT_PAYLOAD_BOTTOM_FRONT) {
      config_srv.request.bottom_front = power;
    } else {  // Item wasn't recognized
      err_msg = "Item " + cmd->args[0].s + " not recognized in power item.";
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      NODELET_ERROR("%s", err_msg.c_str());
      if (!plan) {
        PublishCmdAck(cmd->cmd_id, completed_status, err_msg);
      }
      return false;
    }

    if (!CheckServiceExists(payload_power_client_,
                            "Power payload",
                            cmd->cmd_id,
                            err_msg,
                            completed_status)) {
      return false;
    }

    payload_power_client_.call(config_srv);
    // Check to see if the payload was successfully enabled/disabled
    if (!config_srv.response.success) {
      err_msg = config_srv.response.status;
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      NODELET_ERROR("%s", err_msg.c_str());
      if (!plan) {
        PublishCmdAck(cmd->cmd_id, completed_status, err_msg);
      }
      return false;
    }
  }

  if (!plan) {
    PublishCmdAck(cmd->cmd_id);
  }
  return true;
}

bool Executive::CheckServiceExists(ros::ServiceClient& serviceIn,
                                   std::string const& serviceName,
                                   std::string const& cmd_id,
                                   std::string& err_msg,
                                   uint8_t& completed_status,
                                   bool plan) {
  if (!serviceIn.exists()) {
    err_msg = serviceName + " enable service isn't running! Node may have died";
    completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
    NODELET_ERROR("%s", err_msg.c_str());
    if (!plan) {
      PublishCmdAck(cmd_id, completed_status, err_msg);
    }
    return false;
  }
  return true;
}

bool Executive::SetFlashlightBrightness(ff_msgs::CommandStampedPtr const& cmd,
                                        std::string& err_msg,
                                        uint8_t& completed_status,
                                        bool plan) {
  bool successful = true;

  // Check to make sure command is formatted as expected
  if (cmd->args.size() != 2 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
      cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT) {
    successful = false;
    err_msg = "Malformed arguments for set flashlight brightness command!";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else if (cmd->args[0].s !=
             CommandConstants::PARAM_NAME_FLASHLIGHT_LOCATION_BACK &&
             cmd->args[0].s !=
             CommandConstants::PARAM_NAME_FLASHLIGHT_LOCATION_FRONT) {
    successful = false;
    err_msg = "Flashlight location not recognized. Must be Front or Back.";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else if (cmd->args[1].f < 0 || cmd->args[1].f > 1) {
    successful = false;
    err_msg = "Flashlight brightness must be a value between 0 and 1.";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else {
    ff_hw_msgs::SetFlashlight flashlight_srv;
    // Flashlight brightness needs to be a value between 0 and 200
    flashlight_srv.request.brightness = 200 * cmd->args[1].f;
    if (cmd->args[0].s ==
                        CommandConstants::PARAM_NAME_FLASHLIGHT_LOCATION_BACK) {
      if (!back_flashlight_client_.exists()) {
        successful = false;
        err_msg = "Back flashlight control service isn't running.";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        back_flashlight_client_.call(flashlight_srv);
        // Check to see if flashlight brightness was successfully set
        if (!flashlight_srv.response.success) {
          successful = false;
          err_msg = flashlight_srv.response.status_message;
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    } else {
      if (!front_flashlight_client_.exists()) {
        successful = false;
        err_msg = "Front flashlight control service isn't running.";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        front_flashlight_client_.call(flashlight_srv);
        // Check to see if flashlight brightness was successfully set
        if (!flashlight_srv.response.success) {
          successful = false;
          err_msg = flashlight_srv.response.status_message;
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    }
  }

  if (!successful && !plan) {
    NODELET_ERROR("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id, completed_status, err_msg);
  } else if (successful && !plan) {
    PublishCmdAck(cmd->cmd_id);
  }
  return successful;
}

// TODO(Katie) Need to add sci, perch, and haz cams
bool Executive::SetCamera(ff_msgs::CommandStampedPtr const& cmd,
                          std::string& err_msg,
                          uint8_t& completed_status,
                          bool plan) {
  bool successful = true;
  // Check to make sure command is formatted as expected
  if (cmd->args.size() != 4 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
      cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
      cmd->args[2].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT ||
      cmd->args[3].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT) {
    successful = false;
    err_msg = "Malformed arguments for set camera command!";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else {
    // Third argument is a string specifing the width and height, need to
    // parse it
    std::string width, height;
    std::size_t pos;
    if (cmd->args[1].s.find("_") != std::string::npos) {
      pos = cmd->args[1].s.find("_");
    } else if (cmd->args[1].s.find("X") != std::string::npos) {
      pos = cmd->args[1].s.find("X");
    } else if (cmd->args[1].s.find("x") != std::string::npos) {
      pos = cmd->args[1].s.find("x");
    } else {
      successful = false;
      err_msg = "Camera resolution needs format w_h, wXh, or wxh!";
      completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
    }

    if (successful) {
      width = cmd->args[1].s.substr(0, pos);
      height = cmd->args[1].s.substr((pos + 1));

      if (cmd->args[0].s == CommandConstants::PARAM_NAME_CAMERA_NAME_DOCK) {
        // Check to make sure the dock cam service is valid
        if (!dock_cam_config_client_.exists()) {
          successful = false;
          err_msg = "Dock cam config service not running! Node may have died";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        } else {
          ff_msgs::ConfigureCamera config_img_srv;
          // TODO(Katie) set camera parameters for both streaming and recording?
          config_img_srv.request.mode = ff_msgs::ConfigureCamera::Request::BOTH;
          config_img_srv.request.rate = cmd->args[2].f;
          config_img_srv.request.width = std::stoi(width);
          config_img_srv.request.height = std::stoi(height);

          // Check to see if the dock cam was successfully configured
          if (!dock_cam_config_client_.call(config_img_srv)) {
            // Service only fails if height, width or rate are less than or
            // equal to 0
            successful = false;
            err_msg = "Height, width, and/or rate was invalid for dock camera.";
            completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
          }
        }
      } else if (cmd->args[0].s ==
                                CommandConstants::PARAM_NAME_CAMERA_NAME_NAV) {
        // Check to make sure the nav cam service is valid
        if (!nav_cam_config_client_.exists()) {
          successful = false;
          err_msg = "Nav cam configure service not running! Node may have died";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        } else {
          ff_msgs::ConfigureCamera config_img_srv;
          // TODO(Katie) set camera parameters for both streaming and recording?
          config_img_srv.request.mode = ff_msgs::ConfigureCamera::Request::BOTH;
          config_img_srv.request.rate = cmd->args[2].f;
          config_img_srv.request.width = std::stoi(width);
          config_img_srv.request.height = std::stoi(height);

          // Check to see if the nav cam was successfully configured
          if (!nav_cam_config_client_.call(config_img_srv)) {
            // Service only fails if height, width or rate are less than or
            // equal to 0
            successful = false;
            err_msg = "Height, width, and/or rate was invalid for nav camera.";
            completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
          }
        }
      } else if (cmd->args[0].s ==
                                CommandConstants::PARAM_NAME_CAMERA_NAME_SCI) {
        // TODO(Katie) Don't forget to set the bitrate
      } else {
        successful = false;
        err_msg = "Camera " + cmd->args[0].s + "not recognized.";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      }
    }
  }

  if (!successful && !plan) {
    NODELET_WARN("%s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id, completed_status, err_msg);
  } else if (successful && !plan) {
    PublishCmdAck(cmd->cmd_id);
  }
  return successful;
}

// TODO(Katie) Need to add sci, perch, and haz cams
bool Executive::SetCameraRecording(ff_msgs::CommandStampedPtr const& cmd,
                                      std::string& err_msg,
                                      uint8_t& completed_status,
                                      bool plan) {
  bool successful = true;
  // Check to make sure command is formatted as expected
  if (cmd->args.size() != 2 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
      cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
    successful = false;
    err_msg = "Malformed arguments for set camera recording command!";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else {
    if (cmd->args[0].s == CommandConstants::PARAM_NAME_CAMERA_NAME_DOCK) {
      // Check to make sure the dock cam enable service exists
      if (!dock_cam_enable_client_.exists()) {
        successful = false;
        err_msg = "Dock cam enable service not running! Node may have died!";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        ff_msgs::EnableCamera enable_img_srv;
        enable_img_srv.request.mode = ff_msgs::EnableCamera::Request::RECORDING;
        enable_img_srv.request.enable = cmd->args[1].b;

        // Check to see if recording was set successfully
        if (!dock_cam_enable_client_.call(enable_img_srv)) {
          successful = false;
          // Service call should never fail but check just for kicks and giggles
          err_msg = "Enable recording failed for dock cam.";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    } else if (cmd->args[0].s == CommandConstants::PARAM_NAME_CAMERA_NAME_NAV) {
      // Check to make sure the nav cam enable service exists
      if (!nav_cam_enable_client_.exists()) {
        successful = false;
        err_msg = "Nav cam enable service not running! Node may have died!";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        ff_msgs::EnableCamera enable_img_srv;
        enable_img_srv.request.mode = ff_msgs::EnableCamera::Request::RECORDING;
        enable_img_srv.request.enable = cmd->args[1].b;

        // Check to see if recording was set successfully
        if (!nav_cam_enable_client_.call(enable_img_srv)) {
          successful = false;
          // Service call should never fail but check just for kicks and giggles
          err_msg = "Enable recording failed for nav cam.";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    }
  }

  // TODO(Katie) Is this all we need to do? Do we also need to set the topics
  // to be recorded

  if (!successful && !plan) {
    NODELET_WARN("%s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id, completed_status, err_msg);
  } else if (successful && !plan) {
    PublishCmdAck(cmd->cmd_id);
  }
  return successful;
}

// TODO(Katie) Need to add sci, perch, and haz cams
bool Executive::SetCameraStreaming(ff_msgs::CommandStampedPtr const& cmd,
                                   std::string& err_msg,
                                   uint8_t& completed_status,
                                   bool plan) {
  bool successful = true;
  // Check to make sure command is formatted as expected
  if (cmd->args.size() != 2 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
      cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
    successful = false;
    err_msg = "Malformed arguments for set camera streaming!";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else {
    if (cmd->args[0].s == CommandConstants::PARAM_NAME_CAMERA_NAME_DOCK) {
      // Check to make sure the dock cam service is valid
      if (!dock_cam_enable_client_.exists()) {
          successful = false;
          err_msg = "Dock cam enable service not running! Node may have died!";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        ff_msgs::EnableCamera enable_img_srv;
        enable_img_srv.request.mode = ff_msgs::EnableCamera::Request::STREAMING;
        enable_img_srv.request.enable = cmd->args[1].b;

        // Check to see if streaming was successfully set
        if (!dock_cam_enable_client_.call(enable_img_srv)) {
          successful = false;
          // Service call should never fail but check just for kicks and giggles
          err_msg = "Enable streaming failed for dock cam.";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    } else if (cmd->args[0].s == CommandConstants::PARAM_NAME_CAMERA_NAME_NAV) {
      // Check to make sure the nav cam service is valid
      if (!nav_cam_enable_client_.exists()) {
        successful = false;
        err_msg = "Nav cam enable service not running! Node may have died!";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        ff_msgs::EnableCamera enable_img_srv;
        enable_img_srv.request.mode = ff_msgs::EnableCamera::Request::STREAMING;
        enable_img_srv.request.enable = cmd->args[1].b;

        // Check to see if streaming was successfully set
        if (!nav_cam_enable_client_.call(enable_img_srv)) {
          successful = false;
          // Service call should never fail but check just for kicks and giggles
          err_msg = "Enable streaming failed for nav cam.";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    } else {
      successful = false;
      err_msg = "Camera " + cmd->args[0].s + " not recognized.";
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
    }
  }

  if (!successful && !plan) {
    NODELET_WARN("%s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id, completed_status, err_msg);
  } else if (successful && !plan) {
    PublishCmdAck(cmd->cmd_id);
  }
  return successful;
}

bool Executive::SendGuestScienceCommand(ff_msgs::CommandStampedPtr const& cmd,
                                        std::string& err_msg,
                                        uint8_t& completed_status,
                                        bool plan) {
  bool successful = true;

  // Three guest science commands get passed through this function, need to make
  // sure the parameters are correct. Start and stop have the same parameters
  if (cmd->cmd_name == CommandConstants::CMD_NAME_CUSTOM_GUEST_SCIENCE) {
    if (cmd->args.size() != 2 ||
        cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
        cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
      successful = false;
      err_msg = "Malformed arguments for custom guest science command.";
      completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_START_GUEST_SCIENCE ||
             cmd->cmd_name == CommandConstants::CMD_NAME_STOP_GUEST_SCIENCE) {
    if (cmd->args.size() != 1 ||
        cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
      successful = false;
      err_msg = "Malformed arguments for start/stop guest science command.";
      completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
    }
  } else {
    successful = false;
    err_msg = "Command " + cmd->cmd_name + " is not a guest science command.";
    completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
  }

  // If command syntax was valid, send to guest science manager
  if (successful) {
    gs_cmd_pub_.publish(cmd);
  }

  // Ack if command had a bad syntax, otherwise the guest science manager will
  // ack the command
  if (!successful && !plan) {
    NODELET_ERROR("%s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id, completed_status, err_msg);
  }
  return successful;
}

void Executive::SetOpState(OpState* state) {
  if (state_->id() != state->id()) {
    NODELET_INFO("Executive state changing from [%s(%i)] to [%s(%i)].",
             state_->name().c_str(), state_->id(),
             state->name().c_str(), state->id());
    agent_state_.operating_state.state = state->id();
    state_ = state;
    PublishAgentState();
  }
}

void Executive::Shutdown(std::string const& cmd_id) {
  // TODO(Katie) Stub, change to be actual code, ack complete immediately
  // TODO(Katie) Add code to shutdown the robot
  PublishCmdAck(cmd_id);
  ros::shutdown();
}

void Executive::Initialize(ros::NodeHandle *nh) {
  std::string err_msg;
  // Set executive in op state repo so the op_states can call this executive
  OpStateRepo::Instance()->SetExec(this);

  nh_ = *nh;

  // Read in all the action timeouts. They are in config files so that they can
  // be changed on the fly.
  config_params_.AddFile("management/executive.config");
  config_params_.AddFile("management/sys_monitor_fault_info.config");
  if (!ReadParams()) {
    return;
  }

  // Set up a timer to check and reload timeouts if they are changed.
  reload_params_timer_ = nh_.createTimer(ros::Duration(1),
      [this](ros::TimerEvent e) {
          config_params_.CheckFilesUpdated(std::bind(&Executive::ReadParams,
                                                                      this));},
      false, true);

  // initialize actions
  arm_ac_.SetActiveTimeout(action_active_timeout_);
  arm_ac_.SetResponseTimeout(arm_feedback_timeout_);
  arm_ac_.SetResultCallback(std::bind(&Executive::ArmResultCallback,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2));
  arm_ac_.Create(nh, ACTION_BEHAVIORS_ARM);

  dock_ac_.SetActiveTimeout(action_active_timeout_);
  dock_ac_.SetDeadlineTimeout(dock_result_timeout_);
  dock_ac_.SetResultCallback(std::bind(&Executive::DockResultCallback,
                             this,
                             std::placeholders::_1,
                             std::placeholders::_2));
  dock_ac_.Create(nh, ACTION_BEHAVIORS_DOCK);

  switch_ac_.SetActiveTimeout(action_active_timeout_);
  switch_ac_.SetDeadlineTimeout(switch_result_timeout_);
  switch_ac_.SetResultCallback(std::bind(&Executive::SwitchResultCallback,
                               this,
                               std::placeholders::_1,
                               std::placeholders::_2));
  switch_ac_.Create(nh, ACTION_LOCALIZATION_MANAGER_SWITCH);

  motion_ac_.SetActiveTimeout(action_active_timeout_);
  motion_ac_.SetResponseTimeout(motion_feedback_timeout_);
  motion_ac_.SetFeedbackCallback(std::bind(&Executive::MotionFeedbackCallback,
                                 this,
                                 std::placeholders::_1));
  motion_ac_.SetResultCallback(std::bind(&Executive::MotionResultCallback,
                               this,
                               std::placeholders::_1,
                               std::placeholders::_2));
  motion_ac_.Create(nh, ACTION_MOBILITY_MOTION);

  // initialize subs
  cmd_sub_ = nh_.subscribe(TOPIC_COMMAND,
                           sub_queue_size_,
                           &Executive::CmdCallback,
                           this);

  dock_state_sub_ = nh_.subscribe(TOPIC_BEHAVIORS_DOCKING_STATE,
                                  sub_queue_size_,
                                  &Executive::DockStateCallback,
                                  this);

  gs_ack_sub_ = nh_.subscribe(TOPIC_GUEST_SCIENCE_MANAGER_ACK,
                              sub_queue_size_,
                              &Executive::GuestScienceAckCallback,
                              this);

  heartbeat_sub_ = nh_.subscribe(TOPIC_MANAGEMENT_SYS_MONITOR_HEARTBEAT,
                                 sub_queue_size_,
                                 &Executive::SysMonitorHeartbeatCallback,
                                 this);

  motion_sub_ = nh_.subscribe(TOPIC_MOBILITY_MOTION_STATE,
                              sub_queue_size_,
                              &Executive::MotionStateCallback,
                              this);

  plan_sub_ = nh_.subscribe(TOPIC_COMMUNICATIONS_DDS_PLAN,
                            sub_queue_size_,
                            &Executive::PlanCallback,
                            this);

  zones_sub_ = nh_.subscribe(TOPIC_COMMUNICATIONS_DDS_ZONES,
                             sub_queue_size_,
                             &Executive::ZonesCallback,
                             this);

  // initialize pubs
  agent_state_pub_ = nh_.advertise<ff_msgs::AgentStateStamped>(
                                              TOPIC_MANAGEMENT_EXEC_AGENT_STATE,
                                              pub_queue_size_,
                                              true);

  cf_ack_pub_ = nh_.advertise<ff_msgs::CompressedFileAck>(
                                                  TOPIC_MANAGEMENT_EXEC_CF_ACK,
                                                  pub_queue_size_,
                                                  false);

  cmd_ack_pub_ = nh_.advertise<ff_msgs::AckStamped>(TOPIC_MANAGEMENT_ACK,
                                                    pub_queue_size_,
                                                    false);

  gs_cmd_pub_ = nh_.advertise<ff_msgs::CommandStamped>(
                                                  TOPIC_MANAGEMENT_EXEC_COMMAND,
                                                  pub_queue_size_,
                                                  false);

  plan_pub_ = nh_.advertise<ff_msgs::CompressedFile>(TOPIC_MANAGEMENT_EXEC_PLAN,
                                                     pub_queue_size_,
                                                     false);

  plan_status_pub_ = nh_.advertise<ff_msgs::PlanStatusStamped>(
                                              TOPIC_MANAGEMENT_EXEC_PLAN_STATUS,
                                              pub_queue_size_,
                                              true);

  // initialize services
  zones_client_ = nh_.serviceClient<ff_msgs::SetZones>(
                                                    SERVICE_MOBILITY_SET_ZONES);

  laser_enable_client_ = nh_.serviceClient<ff_hw_msgs::SetEnabled>(
                                                SERVICE_HARDWARE_LASER_ENABLE);

  payload_power_client_ = nh_.serviceClient<ff_hw_msgs::ConfigurePayloadPower>(
                                      SERVICE_HARDWARE_EPS_CONF_PAYLOAD_POWER);

  pmc_enable_client_ = nh_.serviceClient<ff_hw_msgs::SetEnabled>(
                                              SERVICE_HARDWARE_EPS_ENABLE_PMCS);

  reset_ekf_client_ = nh_.serviceClient<std_srvs::Empty>(SERVICE_GNC_EKF_RESET);

  front_flashlight_client_ = nh_.serviceClient<ff_hw_msgs::SetFlashlight>(
                                          SERVICE_HARDWARE_LIGHT_FRONT_CONTROL);

  back_flashlight_client_ = nh_.serviceClient<ff_hw_msgs::SetFlashlight>(
                                          SERVICE_HARDWARE_LIGHT_AFT_CONTROL);

  dock_cam_config_client_ = nh_.serviceClient<ff_msgs::ConfigureCamera>(
                                    SERVICE_MANAGEMENT_IMG_SAMPLER_CONFIG_DOCK);

  dock_cam_enable_client_ = nh_.serviceClient<ff_msgs::EnableCamera>(
                                    SERVICE_MANAGEMENT_IMG_SAMPLER_ENABLE_DOCK);

  nav_cam_config_client_ = nh_.serviceClient<ff_msgs::ConfigureCamera>(
                                    SERVICE_MANAGEMENT_IMG_SAMPLER_CONFIG_NAV);

  nav_cam_enable_client_ = nh_.serviceClient<ff_msgs::EnableCamera>(
                                    SERVICE_MANAGEMENT_IMG_SAMPLER_ENABLE_NAV);

  // initialize configure clients later, when initialize here, the service is
  // invalid when we try to use it. Must have something to do with startup order
  // of executive, choreographer, planner, or mapper

  // initialize agent state
  agent_state_.operating_state.state = ff_msgs::OpState::READY;
  SetPlanExecState(ff_msgs::ExecState::IDLE);
  agent_state_.mobility_state.state = ff_msgs::MobilityState::DRIFTING;
  agent_state_.mobility_state.sub_state = 0;
  agent_state_.guest_science_state.state = ff_msgs::ExecState::IDLE;
  agent_state_.proximity = 0;
  agent_state_.profile_name = "";
  agent_state_.flight_mode = "nominal";

  // Get nominal limits
  ff_msgs::FlightMode flight_mode;
  if (!ff_util::FlightUtil::GetFlightMode(flight_mode, "nominal")) {
    err_msg = "Couldn't get flight mode nominal.";
    NODELET_ERROR("%s", err_msg.c_str());
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return;
  } else {
    agent_state_.target_linear_velocity = flight_mode.hard_limit_vel;
    agent_state_.target_linear_accel = flight_mode.hard_limit_accel;
    agent_state_.target_angular_velocity = flight_mode.hard_limit_omega;
    agent_state_.target_angular_accel = flight_mode.hard_limit_alpha;
  }

  // TODO(Katie) Figure out what to set the collision distance to

  agent_state_.holonomic_enabled = false;
  agent_state_.check_obstacles = true;
  agent_state_.check_zones = true;
  agent_state_.auto_return_enabled = true;
  agent_state_.immediate_enabled = true;
  agent_state_.time_sync_enabled = false;
  agent_state_.boot_time = ros::Time::now().sec;

  PublishAgentState();

  // Publish blank plan status so that the GDS displays the correct plan info
  ff_msgs::PlanStatusStamped plan_status;
  plan_status.header.stamp = ros::Time::now();
  plan_status.name = "";
  plan_status.command = -1;
  plan_status_pub_.publish(plan_status);

  // Create timer for wait command with a dummy duration since it will be
  // changed everytime it is started. Make it one shot and don't start until
  // wait command received
  wait_timer_ = nh_.createTimer(ros::Duration(1),
                                &Executive::WaitCallback,
                                this,
                                true,
                                false);


  // Create timer for monitoring the system monitor heartbeat. Don't start it
  // until we receive the first heartbeat from the system monitor
  sys_monitor_heartbeat_timer_ = nh_.createTimer(
                                ros::Duration(sys_monitor_heartbeat_timeout_),
                                &Executive::SysMonitorHeartbeatTimeoutCallback,
                                this,
                                false,
                                false);

  // Create timer to make sure the system monitor was started
  sys_monitor_startup_timer_ = nh_.createTimer(
                                  ros::Duration(sys_monitor_startup_time_secs_),
                                  &Executive::SysMonitorStartupTimeoutCallback,
                                  this,
                                  true,
                                  true);

  // Initialize switch goal because the executive is only ever going to switch
  // to the mapped landmarks ("ml") pipeline...
  switch_goal_.pipeline = "ml";
}

bool Executive::ReadParams() {
  std::string err_msg;
  // Read config files into lua
  if (!config_params_.ReadFiles()) {
    err_msg = "Error loading executive parameters. Couldn't read config files.";
    NODELET_ERROR("%s", err_msg.c_str());
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }

  // get action active timeout
  if (!config_params_.GetPosReal("action_active_timeout",
                                                    &action_active_timeout_)) {
    NODELET_WARN("Action active timeout not specified.");
    action_active_timeout_ = 1;
  }

  // get action feedback timeouts
  if (!config_params_.GetPosReal("motion_feedback_timeout",
                                                &motion_feedback_timeout_)) {
    NODELET_WARN("Motion feedback timeout not specified.");
    motion_feedback_timeout_ = 1;
  }

  if (!config_params_.GetPosReal("arm_feedback_timeout",
                                                      &arm_feedback_timeout_)) {
    NODELET_WARN("Arm feedback timeout not specified.");
    arm_feedback_timeout_ = 4;
  }

  // get action results timeouts
  if (!config_params_.GetPosReal("dock_result_timeout",
                                                      &dock_result_timeout_)) {
    NODELET_WARN("Dock result timeout not specified.");
    dock_result_timeout_ = 360;
  }

  if (!config_params_.GetPosReal("perch_result_timeout",
                                                      &perch_result_timeout_)) {
    NODELET_WARN("Perch result timeout not specified.");
    perch_result_timeout_ = 360;
  }

  if (!config_params_.GetPosReal("switch_result_timeout",
                                                    &switch_result_timeout_)) {
    NODELET_WARN("Switch result timeout not specified.");
    switch_result_timeout_ = 10;
  }

  if (!config_params_.GetPosReal("sys_monitor_startup_time_secs",
                                            &sys_monitor_startup_time_secs_)) {
    NODELET_WARN("System monitor startup time not specified.");
    sys_monitor_startup_time_secs_ = 30;
  }

  // get planner
  if (!config_params_.GetStr("planner", &agent_state_.planner)) {
    NODELET_WARN("System monitor planner not specified.");
    agent_state_.planner = "QuadraticProgram";
  }

  if (!config_params_.GetPosReal("sys_monitor_heartbeat_timeout",
                                            &sys_monitor_heartbeat_timeout_)) {
    err_msg = "System monitor heartbeat timeout not specified.";
    NODELET_ERROR("%s", err_msg.c_str());
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }

  if (!config_params_.CheckValExists("sys_monitor_heartbeat_fault_response")) {
    err_msg = "Sys monitor heartbeat fault response not specified.";
    NODELET_ERROR("%s", err_msg.c_str());
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }

  config_reader::ConfigReader::Table hb_response(&config_params_,
                                        "sys_monitor_heartbeat_fault_response");

  if (!ReadCommand(&hb_response, sys_monitor_heartbeat_fault_response_)) {
    err_msg = "Unable to read sys monitor heartbeat fault response.";
    NODELET_ERROR("%s", err_msg.c_str());
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }

  if (!config_params_.CheckValExists("sys_monitor_init_fault_response")) {
    err_msg = "System monitor init fault response not specified.";
    NODELET_ERROR("%s", err_msg.c_str());
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }

  config_reader::ConfigReader::Table init_response(&config_params_,
                                            "sys_monitor_init_fault_response");

  if (!ReadCommand(&init_response, sys_monitor_init_fault_response_)) {
    err_msg = "Unable to read sys monitor init fault response.";
    NODELET_ERROR("%s", err_msg.c_str());
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }

  return true;
}

bool Executive::ReadCommand(config_reader::ConfigReader::Table *response,
                            ff_msgs::CommandStampedPtr cmd) {
  std::string cmd_name;
  if (!response->GetStr("name", &cmd_name)) {
    NODELET_ERROR("Fault response command name not specified.");
    return false;
  }

  cmd->cmd_name = cmd_name;
  cmd->cmd_src = "executive";
  cmd->subsys_name = "Astrobee";

  if (response->CheckValExists("args")) {
    config_reader::ConfigReader::Table args(response, "args");
    int num_args = args.GetSize(), i;
    unsigned int type;
    cmd->args.resize(num_args);
    for (i = 0; i < num_args; ++i) {
      // Lua indices start at 1
      config_reader::ConfigReader::Table arg(&args, (i + 1));
      // First element in table is the type
      if (!arg.GetUInt(1, &type)) {
        NODELET_ERROR("First command argument value is not a uint");
        return false;
      }

      // Remaining elements are the parameter values
      switch (type) {
        case ff_msgs::CommandArg::DATA_TYPE_BOOL:
          {
            bool val;
            if (!arg.GetBool(2, &val)) {
              NODELET_ERROR("Expected command argument to be a bool!");
              return false;
            }
            cmd->args[i].data_type = ff_msgs::CommandArg::DATA_TYPE_BOOL;
            cmd->args[i].b = val;
          }
          break;
        case ff_msgs::CommandArg::DATA_TYPE_DOUBLE:
          {
            double val;
            if (!arg.GetReal(2, &val)) {
              NODELET_ERROR("Expected command argument to be a double");
              return false;
            }
            cmd->args[i].data_type = ff_msgs::CommandArg::DATA_TYPE_DOUBLE;
            cmd->args[i].d = val;
          }
          break;
        case ff_msgs::CommandArg::DATA_TYPE_FLOAT:
          {
            float val;
            if (!arg.GetReal(2, &val)) {
              NODELET_ERROR("Expected command argument to be a float.");
              return false;
            }
            cmd->args[i].data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
            cmd->args[i].f = val;
          }
          break;
        case ff_msgs::CommandArg::DATA_TYPE_INT:
          {
            int val;
            if (!arg.GetInt(2, &val)) {
              NODELET_ERROR("Expected command argument to be an int.");
              return false;
            }
            cmd->args[i].data_type = ff_msgs::CommandArg::DATA_TYPE_INT;
            cmd->args[i].i = val;
          }
          break;
        case ff_msgs::CommandArg::DATA_TYPE_LONGLONG:
          {
            int64_t val;
            if (!arg.GetLongLong(2, &val)) {
              NODELET_ERROR("Expected command argument to be an int.");
              return false;
            }
            cmd->args[i].data_type = ff_msgs::CommandArg::DATA_TYPE_LONGLONG;
            cmd->args[i].ll = val;
          }
          break;
        case ff_msgs::CommandArg::DATA_TYPE_STRING:
          {
            std::string val;
            if (!arg.GetStr(2, &val)) {
              NODELET_ERROR("Expected command argument to be a string");
              return false;
            }
            cmd->args[i].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
            cmd->args[i].s = val;
          }
          break;
        case ff_msgs::CommandArg::DATA_TYPE_VEC3d:
          {
            int j;
            double val;
            cmd->args[i].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
            for (j = 0; j < 3; ++j) {
              // Index to get vector values in table starts at 2
              if (!arg.GetReal((j + 2), &val)) {
                NODELET_ERROR("Expected command argument to be double.");
                return false;
              }
              cmd->args[i].vec3d[j] = val;
            }
          }
          break;
        case ff_msgs::CommandArg::DATA_TYPE_MAT33f:
          {
            int j;
            float val;
            cmd->args[i].data_type = ff_msgs::CommandArg::DATA_TYPE_MAT33f;
            for (j = 0; j < 9; ++j) {
              // Index in get matrix values in table starts at 2
              if (!arg.GetReal((j + 2), &val)) {
              NODELET_ERROR("Expected command argument to be a float.");
                return false;
              }
              cmd->args[i].mat33f[j] = val;
            }
          }
          break;
        default:
          NODELET_ERROR("Type for command argument unrecognized!");
          return false;
      }
    }
  }

  return true;
}

void Executive::PublishAgentState() {
  agent_state_.header.stamp = ros::Time::now();
  agent_state_pub_.publish(agent_state_);
}

}  // namespace executive

PLUGINLIB_EXPORT_CLASS(executive::Executive, nodelet::Nodelet)
