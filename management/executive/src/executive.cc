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


// TODO(Katie) Add more here. Stop using feedback and result to set mobility
// state. Instead use Dock state, Choreographer state, and Perch state
void Executive::DockStateCallback(ff_msgs::DockStatePtr const& state) {
  // If the operating state is ready and the dock state changes, this probably
  // means that an astronaut manually docked or undocked the Astrobee
  if (state_->id() == ff_msgs::OpState::READY) {
    if (state->state == ff_msgs::DockState::DOCKED) {
      SetMobilityState(ff_msgs::MobilityState::DOCKING);
    } else if (state->state == ff_msgs::DockState::UNDOCKED) {
      // If we were docked, need to set mobility state to idle until we receive
      // GN&C state. If we aren't docked, don't change the mobility state since
      // an undocked mobility state can also mean flying, idle, ...
      if (GetMobilityState().state == ff_msgs::MobilityState::DOCKING) {
        SetMobilityState(ff_msgs::MobilityState::DRIFTING);
      }
    }
    // Don't worry about any of the other dock states since we transition
    // through them when we are autonomously docking and undocking using the
    // action feedback to change the mobility state.
  }
  // If we aren't in operating state ready, we don't want to change the mobility
  // state since again we use action feedback to set the it. We may need to
  // re-visit this if an astronaut can manually dock in any operating state.
  // The current thought is, if an astronaut tries to dock us in any operating
  // state other than ready, we will fail what we are doing and go back into
  // operating state ready before receiving a docked or undocked state.
}

void Executive::GuestScienceAckCallback(ff_msgs::AckStampedConstPtr const&
                                                                          ack) {
  // TODO(Katie) Add code to change op state if sucessfully stopped or started
  // a primary guest apk. Probably need to subscribe to the guest science
  // config. Possibly remove this since it will probably be replaced by an
  // action
  if (ack->cmd_origin == "plan") {
    SetOpState(state_->HandleGuestScienceAck(ack));
  } else {
    cmd_ack_pub_.publish(ack);
  }
}

void Executive::PlanCallback(ff_msgs::CompressedFileConstPtr const& plan) {
  plan_ = plan;

  cf_ack_.header.stamp = ros::Time::now();
  cf_ack_.id = plan_->id;
  cf_ack_pub_.publish(cf_ack_);
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
      arm_goal_.pan = (cmd->args[0].f * M_PI/180);
      arm_goal_.tilt = (cmd->args[1].f * M_PI/180);
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
    ROS_ERROR("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id,
                  cmd->cmd_origin,
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
      dock_goal_.berth = ff_msgs::DockGoal::BERTH_LEFT;
    } else if (cmd->args[0].i == 2) {
      dock_goal_.berth = ff_msgs::DockGoal::BERTH_RIGHT;
    } else {
      successful = false;
      err_msg = "Berth must be 1 or 2 not " +  std::to_string(cmd->args[0].i);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_UNDOCK) {
    dock_goal_.command = ff_msgs::DockGoal::UNDOCK;
    // We don't need a berth to undock
    dock_goal_.berth = ff_msgs::DockGoal::UNKNOWN;
  } else {
    successful = false;
    err_msg = "Dock command not recognized in fill dock goal.";
  }

  if (!successful && !plan) {
    ROS_ERROR("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id,
                  cmd->cmd_origin,
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
                      cmd->cmd_origin,
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
                      cmd->cmd_origin,
                      ff_msgs::AckCompletedStatus::EXEC_FAILED,
                      "Command isn't a mobility action in fill motion goal!");
      }
      return false;
  }
  return true;
}

bool Executive::StartAction(Action action,
                            std::string const& cmd_id,
                            std::string const& cmd_origin,
                            std::string& err_msg,
                            bool plan) {
  bool successful = true;
  switch (action) {
    case ARM:
      if (arm_ac_.IsConnected()) {
        arm_ac_.SetCmdInfo(action, cmd_id, cmd_origin);
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
        dock_ac_.SetCmdInfo(action, cmd_id, cmd_origin);
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
        motion_ac_.SetCmdInfo(action, cmd_id, cmd_origin);
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
        switch_ac_.SetCmdInfo(action, cmd_id, cmd_origin);
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
    ROS_ERROR("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd_id, cmd_origin, ff_msgs::AckCompletedStatus::EXEC_FAILED,
                  err_msg);
  } else if (successful && !plan) {
    PublishCmdAck(cmd_id, cmd_origin, ff_msgs::AckCompletedStatus::NOT, "",
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
                    arm_ac_.cmd_origin(),
                    ff_msgs::AckCompletedStatus::CANCELED,
                    "Arm action was canceled.");
      arm_ac_.SetCmdInfo(NONE, "", "");
      break;
    case DOCK:
    case UNDOCK:
      dock_ac_.CancelGoal();
      PublishCmdAck(dock_ac_.cmd_id(),
                    dock_ac_.cmd_origin(),
                    ff_msgs::AckCompletedStatus::CANCELED,
                    "Dock action was canceled.");
      dock_ac_.SetCmdInfo(NONE, "", "");
      break;
    case EXECUTE:
    case MOVE:
    case STOP:
      motion_ac_.CancelGoal();
      PublishCmdAck(motion_ac_.cmd_id(),
                    motion_ac_.cmd_origin(),
                    ff_msgs::AckCompletedStatus::CANCELED,
                    "Motion action was canceled.");
      motion_ac_.SetCmdInfo(NONE, "", "");
      break;
    case PERCH:
    case UNPERCH:
      // TODO(Katie) Add Me
      break;
    default:
      ROS_ERROR("Executive: Action to cancel not recognized!");
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
    ROS_ERROR_STREAM("Executive: Action " << action <<
                                            " not in running actions vector!");
  }
  return found;
}

void Executive::ArmResultCallback(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::ArmResultConstPtr const& result) {
  // Remove action from the running action vector
  RemoveAction(ARM);

  SetOpState(state_->HandleArmResult(state,
                                     result,
                                     arm_ac_.cmd_id(),
                                     arm_ac_.cmd_origin()));
  // Reset command id so we don't try to ack the same id twice
  arm_ac_.SetCmdInfo(NONE, "", "");
}

void Executive::DockActiveCallback() {
  SetOpState(state_->HandleDockActive(dock_ac_.action()));
}

void Executive::DockFeedbackCallback(
                                ff_msgs::DockFeedbackConstPtr const& feedback) {
  SetOpState(state_->HandleDockFeedback(feedback));
}

void Executive::DockResultCallback(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::DockResultConstPtr const& result) {
  // Remove action from the running action vector
  RemoveAction(dock_ac_.action());

  SetOpState(state_->HandleDockResult(state,
                                      result,
                                      dock_ac_.cmd_id(),
                                      dock_ac_.cmd_origin(),
                                      dock_ac_.action()));
  // Reset command id so we don't try to ack the same id twice
  dock_ac_.SetCmdInfo(NONE, "", "");
}

void Executive::SwitchResultCallback(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::SwitchResultConstPtr const& result) {
  // Remove action from the running action vector
  RemoveAction(SWITCH);

  SetOpState(state_->HandleSwitchResult(state,
                                        result,
                                        switch_ac_.cmd_id(),
                                        switch_ac_.cmd_origin()));
  // Reset command id so we don't try to ack the same id twice
  switch_ac_.SetCmdInfo(NONE, "", "");
}

void Executive::MotionActiveCallback() {
  SetOpState(state_->HandleMotionActive(motion_ac_.action()));
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
  // Remove action from the running action vector
  RemoveAction(motion_ac_.action());

  std::string cmd_id = motion_ac_.cmd_id();
  std::string cmd_origin = motion_ac_.cmd_origin();
  Action action = motion_ac_.action();

  // Sometimes the handle motion result starts an action so we need to clear the
  // action info before starting another action
  // Reset command id so we don't try to ack the same id twice
  motion_ac_.SetCmdInfo(NONE, "", "");

  SetOpState(state_->HandleMotionResult(state,
                                        result,
                                        cmd_id,
                                        cmd_origin,
                                        action));
}

void Executive::PublishCmdAck(std::string const& cmd_id,
                              std::string const& cmd_origin,
                              uint8_t completed_status,
                              std::string const& message,
                              uint8_t status) {
  ack_.header.stamp = ros::Time::now();
  ack_.cmd_id = cmd_id;
  ack_.cmd_origin = cmd_origin;
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

void Executive::SetProximity(float proximity) {
  agent_state_.proximity = proximity;
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

// TODO(Katie) finish me and maybe check values
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
    err_msg = "Flight mode doesn't exist!.";
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

bool Executive::ConfigureMobility(std::string const& cmd_id,
                                  std::string const& cmd_origin,
                                  std::string& err_msg,
                                  bool plan) {
  bool successful = true;

  // Initialize config clients if they haven't been initialized
  if (!choreographer_cfg_) {
    choreographer_cfg_ =
              std::make_shared<ff_util::ConfigClient>(&nh_, NODE_CHOREOGRAPHER);
  }

  /*if (!mapper_cfg_) {
    mapper_cfg_ =
                  std::make_shared<ff_util::ConfigClient>(&nh_, NODE_MAPPER);
  }*/

  // Set values for configuring, these values will persist until changed
  // Choreographer
  choreographer_cfg_->Set<bool>("enable_immediate", true);
  choreographer_cfg_->Set<bool>("enable_timesync", false);

  // This function is not used for the first segment of a plan so always disable
  // move to start
  choreographer_cfg_->Set<bool>("enable_bootstrapping", false);

  choreographer_cfg_->Set<bool>("enable_replanning", false);
  choreographer_cfg_->Set<bool>("enable_validation", true);
  choreographer_cfg_->Set<bool>("enable_faceforward",
                                              !agent_state_.holonomic_enabled);
  choreographer_cfg_->Set<bool>("enable_collision_checking",
                                                  agent_state_.check_obstacles);

  choreographer_cfg_->Set<double>("desired_vel",
                                          agent_state_.target_linear_velocity);
  choreographer_cfg_->Set<double>("desired_accel",
                                          agent_state_.target_linear_accel);
  choreographer_cfg_->Set<double>("desired_omega",
                                          agent_state_.target_angular_velocity);
  choreographer_cfg_->Set<double>("desired_alpha",
                                          agent_state_.target_angular_accel);

  // TODO(Katie) Configure the sentinel with enable collision checking and
  // collision distance

  // TODO(Katie) Change to mapper
  // mapper_cfg_->Set<bool>("enable_keepouts", agent_state_.check_zones);

  // Clear err_msg
  err_msg = "";

  // Reconfigure choreographer, planner, sentinel
  if (!choreographer_cfg_->Reconfigure()) {
    successful = false;
    err_msg = "Couldn't configure the mobilty::choreographer node! ";
  }

/*  if (!mapper_cfg_->Reconfigure()) {
    successful = false;
    err_msg += "Couldn't configure the mobility::mapper node!";
  }*/

  // Ack error
  if (!successful && !plan) {
    ROS_ERROR("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd_id,
                  cmd_origin,
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

  /*if (!sentinel_cfg_) {
    sentinel_cfg_ =
                  std::make_shared<ff_util::ConfigClient>(&nh_, NODE_SENTINEL);
  }*/

  // Set values for configuring, these values will persist until changed
  // Choreographer

  // This function is not used for the first segment of a plan so always disable
  // move to start
  choreographer_cfg_->Set<bool>("enable_bootstrapping", move_to_start);
  choreographer_cfg_->Set<bool>("enable_immediate", true);
  choreographer_cfg_->Set<bool>("enable_timesync", false);

  // TODO(Katie) What do I do with this
  // choreographer_cfg_->Set<bool>("enable_replanning", ?);

  // Planner
  // TODO(Katie) Fix once Andrew figures out what he is doing
  /*planner_cfg_->Set<double>("limits_lin_vel", linear_vel_limit_);
  planner_cfg_->Set<double>("limits_lin_acc", linear_acc_limit_);
  planner_cfg_->Set<double>("limits_ang_vel", angular_vel_limit_);
  planner_cfg_->Set<double>("limits_ang_acc", angular_acc_limit_);*/
  choreographer_cfg_->Set<bool>("enable_replanning", false);
  choreographer_cfg_->Set<bool>("enable_validation", true);
  choreographer_cfg_->Set<bool>("enable_faceforward", enable_holonomic);
  // choreographer_cfg_->Set<bool>("enable_obstacles", false);  // TODO(Katie) Change
  // choreographer_cfg_->Set<bool>("enable_keepouts", false);  // TODO(Katie) Change

  // Sentinel
  // cfg_sentinel->Set<bool>("enable_collision_checking", )

  // Clear err_msg
  err_msg = "";

  // Reconfigure choreographer, planner, sentinel
  if (!choreographer_cfg_->Reconfigure()) {
    successful = false;
    err_msg = "Couldn't configure the mobilty::choreographer node! ";
  }

  /* if (!sentinel_cfg_->Reconfigure()) {
    successful = false;
    err_msg += "Couldn't configure the mobility::sentinel node!";
  } */

  return successful;
}

bool Executive::ResetEkf(std::string const& cmd_id,
                         std::string const& cmd_origin) {
  // Check to make sure the service is valid and running
  if (!reset_ekf_client_.exists()) {
    PublishCmdAck(cmd_id, cmd_origin, ff_msgs::AckCompletedStatus::EXEC_FAILED,
                  "Reset ekf service isn't running! Ekf node may have died.");
    return false;
  }

  std_srvs::Empty::Request req;
  std_srvs::Empty::Response res;

  // Check to see if the reset succeeded
  if (!reset_ekf_client_.call(req, res)) {
    PublishCmdAck(cmd_id, cmd_origin, ff_msgs::AckCompletedStatus::EXEC_FAILED,
                  "Resetting the ekf failed.");
    return false;
  }

  return true;
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
                              std::string const& cmd_origin,
                              bool plan) {
  // We pretty much always start stop action even if stopped. See cases below
  // for situations we don't want to stop in
  bool successful = true, start_stop = true;
  std::string err_msg;

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
    ROS_ERROR("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd_id,
                  cmd_origin,
                  ff_msgs::AckCompletedStatus::EXEC_FAILED,
                  err_msg);
  } else if (successful && !start_stop) {
    // Ack successful since we cancelled an action that hadn't moved the robot
    PublishCmdAck(cmd_id, cmd_origin);
  }

  stop_started = false;
  if (start_stop) {
    if (FillMotionGoal(STOP)) {
      successful = StartAction(STOP, cmd_id, cmd_origin, err_msg);
      if (successful) {
        stop_started = true;
      }
    }
  }

  return successful;
}

bool Executive::EnableAutoReturn(ff_msgs::CommandStampedPtr const& cmd) {
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
    ROS_ERROR("Executive: Malformed arguments for enable auto return command!");
    PublishCmdAck(cmd->cmd_id,
                  cmd->cmd_origin,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  "Malformed arguments for enable auto return command!");
    return false;
  }

  agent_state_.auto_return_enabled = cmd->args[0].b;
  PublishAgentState();
  PublishCmdAck(cmd->cmd_id, cmd->cmd_origin);
  return true;
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
                                            ff_msgs::MobilityState::DRIFTING) {
    err_msg = "Astrobee is drifting. Please stop before trying to dock.";
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

    if (!StartAction(DOCK, cmd->cmd_id, cmd->cmd_origin, err_msg, plan)) {
      return false;
    }
  }

  // Fill dock goal and start action publish failed command acks so we only
  // need to fail an ack if we aren't stopped and thus cannot dock
  if (!stopped && !plan) {
    ROS_ERROR("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id, cmd->cmd_origin, completed_status, err_msg);
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

    if (!StartAction(UNDOCK, cmd->cmd_id, cmd->cmd_origin, err_msg, plan)) {
      return false;
    }
  }

  // Start action publish failed command acks so we only need to fail an ack if
  // we aren't docked and thus cannot undock
  if (!docked && !plan) {
    ROS_ERROR("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id,
                  cmd->cmd_origin,
                  ff_msgs::AckCompletedStatus::EXEC_FAILED,
                  err_msg);
  }
  return docked;
}

bool Executive::SetCheckObstacles(ff_msgs::CommandStampedPtr const& cmd) {
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
    ROS_ERROR("Executive: Malformed arguments for set check obstacles!");
    PublishCmdAck(cmd->cmd_id,
                  cmd->cmd_origin,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  "Malformed arguments for set check obstacles!");
    return false;
  }

  agent_state_.check_obstacles = cmd->args[0].b;
  PublishAgentState();
  PublishCmdAck(cmd->cmd_id, cmd->cmd_origin);
  return true;
}

bool Executive::SetCheckZones(ff_msgs::CommandStampedPtr const& cmd) {
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
    ROS_ERROR("Executive: Malformed arguments for set check zones!");
    PublishCmdAck(cmd->cmd_id,
                  cmd->cmd_origin,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  "Malformed arguments for set check zones!");
    return false;
  }

  agent_state_.check_zones = cmd->args[0].b;
  PublishAgentState();
  PublishCmdAck(cmd->cmd_id, cmd->cmd_origin);
  return true;
}

bool Executive::SetHolonomicMode(ff_msgs::CommandStampedPtr const& cmd) {
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
    ROS_ERROR("Executive: Malformed arguments for set holonomic mode command!");
    PublishCmdAck(cmd->cmd_id,
                  cmd->cmd_origin,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  "Malformed arguments for set holonomic mode command!");
    return false;
  }

  agent_state_.holonomic_enabled = cmd->args[0].b;
  PublishAgentState();
  PublishCmdAck(cmd->cmd_id, cmd->cmd_origin);
  return true;
}

void Executive::StopArm(std::string const& cmd_id,
                        std::string const& cmd_origin) {
  // TODO(Katie) stub, add actual code
  ROS_WARN("Executive: Stop arm not implemented yet! Stay tuned!");
  PublishCmdAck(cmd_id,
                cmd_origin,
                ff_msgs::AckCompletedStatus::EXEC_FAILED,
                "Stop arm not implemented yet! Stay tuned!");
}

void Executive::StowArm(std::string const& cmd_id,
                        std::string const& cmd_origin) {
  // TODO(Katie) stub, add actual code
  ROS_WARN("Executive: Stow arm not implemented yet! Stay tuned!");
  PublishCmdAck(cmd_id,
                cmd_origin,
                ff_msgs::AckCompletedStatus::EXEC_FAILED,
                "Stow arm not implemented yet!");
}

void Executive::SkipPlanStep(std::string const& cmd_id,
                             std::string const& cmd_origin) {
  // Make sure plan execution state is paused
  if (GetPlanExecState() != ff_msgs::ExecState::PAUSED) {
    ROS_ERROR("Executive: Got command to skip plan step but plan not paused.");
    PublishCmdAck(cmd_id,
                  cmd_origin,
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
  PublishCmdAck(cmd_id, cmd_origin);
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
    ROS_ERROR("Download data not implemented yet!");
    completed_status = ff_msgs::AckCompletedStatus::OK;
  }

  if (!successful && !plan) {
    ROS_ERROR("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id, cmd->cmd_origin, completed_status, err_msg);
  } else if (successful && !plan) {
    PublishCmdAck(cmd->cmd_id, cmd->cmd_origin);
  }
  return successful;
}

void Executive::StopDownload(ff_msgs::CommandStampedPtr const& cmd) {
  std::string err_msg;
  // Check to make sure command is formatted as expected
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
    err_msg = "Malformed arguments for stop download command!";
    ROS_ERROR("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id,
                  cmd->cmd_origin,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  err_msg);
  }

  if (cmd->args[0].s != CommandConstants::PARAM_NAME_DOWNLOAD_METHOD_IMMEDIATE
      && cmd->args[0].s !=
                        CommandConstants::PARAM_NAME_DOWNLOAD_METHOD_DELAYED) {
    err_msg = "Download method not recognized. Needs to be immediate or delay.";
    ROS_ERROR("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id,
                  cmd->cmd_origin,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  err_msg);
  }

  // TODO(Katie) Can only stop download if download occurring, check class
  // variables to see if a download is in progress and what kind of data
  // TODO(Katie) Stub, change to be actual code
  err_msg = "Stop download not implemented yet!";
  ROS_ERROR("Executive: %s", err_msg.c_str());
  PublishCmdAck(cmd->cmd_id,
                cmd->cmd_origin,
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
    ROS_ERROR("Clear data not implemented yet!");
    completed_status = ff_msgs::AckCompletedStatus::OK;
  }

  if (!successful && !plan) {
    ROS_ERROR("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id, cmd->cmd_origin, completed_status, err_msg);
  } else if (successful && !plan) {
    PublishCmdAck(cmd->cmd_id, cmd->cmd_origin);
  }
  return successful;
}

// TODO(Katie) Need to add all the other items we can power on
bool Executive::PowerOnItem(ff_msgs::CommandStampedPtr const& cmd,
                            std::string& err_msg,
                            uint8_t& completed_status,
                            bool plan) {
  // Check to make sure command is formatted as expected
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
    err_msg = "Malformed arguments for power on item command!";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
    ROS_ERROR("%s", err_msg.c_str());
    if (!plan) {
      PublishCmdAck(cmd->cmd_id, cmd->cmd_origin, completed_status, err_msg);
    }
    return false;
  }

  ROS_WARN("Item %s is being powered on!", cmd->args[0].s.c_str());

  if (cmd->args[0].s ==
                CommandConstants::PARAM_NAME_POWERED_COMPONENT_LASER_POINTER) {
    // Check to make sure service is valid and running
    if (!laser_enable_client_.exists()) {
      err_msg = "Laser enable service isn't running! Laser node may have died";
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      ROS_ERROR("%s", err_msg.c_str());
      if (!plan) {
        PublishCmdAck(cmd->cmd_id, cmd->cmd_origin, completed_status, err_msg);
      }
      return false;
    }

    ff_hw_msgs::SetEnabled enable_srv;
    enable_srv.request.enabled = true;
    laser_enable_client_.call(enable_srv);
    // Check to see if the laser was succesfully enabled
    if (!enable_srv.response.success) {
      err_msg = enable_srv.response.status_message;
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      ROS_ERROR("%s", err_msg.c_str());
      if (!plan) {
        PublishCmdAck(cmd->cmd_id, cmd->cmd_origin, completed_status, err_msg);
      }
      return false;
    }
  } else {
    err_msg = "Item " + cmd->args[0].s + "not recognized.";
    completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
    ROS_ERROR("%s", err_msg.c_str());
    if (!plan) {
      PublishCmdAck(cmd->cmd_id, cmd->cmd_origin, completed_status, err_msg);
    }
    return false;
  }

  if (!plan) {
    PublishCmdAck(cmd->cmd_id, cmd->cmd_origin);
  }
  return true;
}

// TODO(Katie) Need to add all the other items we can power off
bool Executive::PowerOffItem(ff_msgs::CommandStampedPtr const& cmd,
                             std::string& err_msg,
                             uint8_t& completed_status,
                             bool plan) {
  // Check to make sure command is formatted as expected
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
    err_msg = "Malformed arguments for power off item command!";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
    ROS_ERROR("%s", err_msg.c_str());
    if (!plan) {
      PublishCmdAck(cmd->cmd_id, cmd->cmd_origin, completed_status, err_msg);
    }
    return false;
  }

  ROS_WARN("Item %s is being powered off!", cmd->args[0].s.c_str());

  if (cmd->args[0].s ==
                CommandConstants::PARAM_NAME_POWERED_COMPONENT_LASER_POINTER) {
    // Check to make sure the laser enable service is valid
    if (!laser_enable_client_.exists()) {
      err_msg = "Laser enable service isn't running! Laser node may have died!";
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      ROS_ERROR("%s", err_msg.c_str());
      if (!plan) {
        PublishCmdAck(cmd->cmd_id, cmd->cmd_origin, completed_status, err_msg);
      }
      return false;
    }
    ff_hw_msgs::SetEnabled disable_srv;
    disable_srv.request.enabled = false;
    laser_enable_client_.call(disable_srv);
    // Check to see if the laser was succesfully disabled
    if (!disable_srv.response.success) {
      err_msg = disable_srv.response.status_message;
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      ROS_ERROR("%s", err_msg.c_str());
      if (!plan) {
        PublishCmdAck(cmd->cmd_id, cmd->cmd_origin, completed_status, err_msg);
      }
      return false;
    }
  } else {
    err_msg = "Item " + cmd->args[0].s + "not recognized.";
    completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
    ROS_ERROR("%s", err_msg.c_str());
    if (!plan) {
      PublishCmdAck(cmd->cmd_id, cmd->cmd_origin, completed_status, err_msg);
    }
    return false;
  }

  if (!plan) {
    PublishCmdAck(cmd->cmd_id, cmd->cmd_origin);
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
    ROS_ERROR("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id, cmd->cmd_origin, completed_status, err_msg);
  } else if (successful && !plan) {
    PublishCmdAck(cmd->cmd_id, cmd->cmd_origin);
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
    ROS_WARN("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id, cmd->cmd_origin, completed_status, err_msg);
  } else if (successful && !plan) {
    PublishCmdAck(cmd->cmd_id, cmd->cmd_origin);
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
    ROS_WARN("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id, cmd->cmd_origin, completed_status, err_msg);
  } else if (successful && !plan) {
    PublishCmdAck(cmd->cmd_id, cmd->cmd_origin);
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
    ROS_WARN("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id, cmd->cmd_origin, completed_status, err_msg);
  } else if (successful && !plan) {
    PublishCmdAck(cmd->cmd_id, cmd->cmd_origin);
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
    ROS_ERROR("Executive: %s", err_msg.c_str());
    PublishCmdAck(cmd->cmd_id, cmd->cmd_origin, completed_status, err_msg);
  }
  return successful;
}

void Executive::DetermineStartupMobilityState() {
  // TODO(Katie) Lots!!!!!! For now, set mobility state to stopped so GDS works
  agent_state_.mobility_state.state = ff_msgs::MobilityState::STOPPING;
  PublishAgentState();
}

void Executive::SetOpState(OpState* state) {
  if (state_->id() != state->id()) {
    ROS_INFO("Executive state changing from [%s(%i)] to [%s(%i)].",
             state_->name().c_str(), state_->id(),
             state->name().c_str(), state->id());
    agent_state_.operating_state.state = state->id();
    state_ = state;
    PublishAgentState();
  }
}

void Executive::Shutdown(std::string const& cmd_id,
                         std::string const& cmd_origin) {
  // TODO(Katie) Stub, change to be actual code, ack complete immediately
  // TODO(Katie) Add code to shutdown the robot
  PublishCmdAck(cmd_id, cmd_origin);
  ros::shutdown();
}

void Executive::Initialize(ros::NodeHandle *nh) {
  // Set executive in op state repo so the op_states can call this executive
  OpStateRepo::Instance()->SetExec(this);

  nh_ = *nh;

  // Read in all the action timeouts. They are in config files so that they can
  // be changed on the fly.
  config_params_.AddFile("management/executive.config");
  ReadParams();

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
  arm_ac_.Create(nh, ACTION_PROCEDURES_ARM);

  dock_ac_.SetActiveTimeout(action_active_timeout_);
  dock_ac_.SetDeadlineTimeout(dock_result_timeout_);
  dock_ac_.SetActiveCallback(std::bind(&Executive::DockActiveCallback, this));
  dock_ac_.SetFeedbackCallback(std::bind(&Executive::DockFeedbackCallback,
                               this,
                               std::placeholders::_1));
  dock_ac_.SetResultCallback(std::bind(&Executive::DockResultCallback,
                             this,
                             std::placeholders::_1,
                             std::placeholders::_2));
  dock_ac_.Create(nh, ACTION_PROCEDURES_DOCK);

  switch_ac_.SetActiveTimeout(action_active_timeout_);
  switch_ac_.SetDeadlineTimeout(switch_result_timeout_);
  switch_ac_.SetResultCallback(std::bind(&Executive::SwitchResultCallback,
                               this,
                               std::placeholders::_1,
                               std::placeholders::_2));
  switch_ac_.Create(nh, ACTION_LOCALIZATION_MANAGER_SWITCH);

  motion_ac_.SetActiveTimeout(action_active_timeout_);
  motion_ac_.SetResponseTimeout(motion_feedback_timeout_);
  motion_ac_.SetActiveCallback(std::bind(&Executive::MotionActiveCallback,
                               this));
  motion_ac_.SetFeedbackCallback(std::bind(&Executive::MotionFeedbackCallback,
                                 this,
                                 std::placeholders::_1));
  motion_ac_.SetResultCallback(std::bind(&Executive::MotionResultCallback,
                               this,
                               std::placeholders::_1,
                               std::placeholders::_2));
  motion_ac_.Create(nh, ACTION_MOBILITY_MOTION);

  // initialize subs
  cmd_sub_ = nh_.subscribe(TOPIC_COMMAND, sub_queue_size_,
                           &Executive::CmdCallback, this);

  dock_state_sub_ = nh_.subscribe(TOPIC_PROCEDURES_DOCKING_STATE,
                                  sub_queue_size_,
                                  &Executive::DockStateCallback,
                                  this);

  gs_ack_sub_ = nh_.subscribe(TOPIC_GUEST_SCIENCE_MANAGER_ACK, sub_queue_size_,
                              &Executive::GuestScienceAckCallback, this);

  plan_sub_ = nh_.subscribe(TOPIC_COMMUNICATIONS_DDS_PLAN, sub_queue_size_,
                            &Executive::PlanCallback, this);

  zones_sub_ = nh_.subscribe(TOPIC_COMMUNICATIONS_DDS_ZONES, sub_queue_size_,
                             &Executive::ZonesCallback, this);

  // initialize pubs
  agent_state_pub_ = nh_.advertise<ff_msgs::AgentStateStamped>(
                      TOPIC_MANAGEMENT_EXEC_AGENT_STATE, pub_queue_size_, true);

  cf_ack_pub_ = nh_.advertise<ff_msgs::CompressedFileAck>(
                                                  TOPIC_MANAGEMENT_EXEC_CF_ACK,
                                                  pub_queue_size_, false);

  cmd_ack_pub_ = nh_.advertise<ff_msgs::AckStamped>(TOPIC_MANAGEMENT_ACK,
                                           pub_queue_size_, false);

  gs_cmd_pub_ = nh_.advertise<ff_msgs::CommandStamped>(
                                                  TOPIC_MANAGEMENT_EXEC_COMMAND,
                                                  pub_queue_size_, false);

  plan_pub_ = nh_.advertise<ff_msgs::CompressedFile>(TOPIC_MANAGEMENT_EXEC_PLAN,
                                                     pub_queue_size_, false);

  plan_status_pub_ = nh_.advertise<ff_msgs::PlanStatusStamped>(
                      TOPIC_MANAGEMENT_EXEC_PLAN_STATUS, pub_queue_size_, true);

  // initialize services
  zones_client_ =
            nh_.serviceClient<ff_msgs::SetZones>(SERVICE_MOBILITY_SET_ZONES);

  laser_enable_client_ = nh_.serviceClient<ff_hw_msgs::SetEnabled>(
                                                SERVICE_HARDWARE_LASER_ENABLE);

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
  // of executive, choreographer, planner, or sentinel

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
    ROS_ERROR("Executive: Couldn't get flight mode nominal.");
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
  wait_timer_ = nh_.createTimer(ros::Duration(1), &Executive::WaitCallback,
                                                            this, true, false);

  // Initialize switch goal because the executive is only ever going to switch
  // to the mapped landmarks ("ml") pipeline...
  switch_goal_.pipeline = "ml";

  DetermineStartupMobilityState();
}

void Executive::ReadParams() {
  // Read config files into lua
  if (!config_params_.ReadFiles()) {
    ROS_ERROR("Executive: Error loading executive parameters.");
    return;
  }

  // get action active timeout
  if (!config_params_.GetPosReal("action_active_timeout",
                                                    &action_active_timeout_)) {
    ROS_ERROR("Executive: Action active timeout not specified.");
  }

  // get action feedback timeouts
  if (!config_params_.GetPosReal("motion_feedback_timeout",
                                                &motion_feedback_timeout_)) {
    ROS_ERROR("Executive: Motion feedback timeout not specified.");
  }

  if (!config_params_.GetPosReal("arm_feedback_timeout",
                                                      &arm_feedback_timeout_)) {
    ROS_ERROR("Executive: Arm feedback timeout not specified.");
  }

  // get action results timeouts
  if (!config_params_.GetPosReal("dock_result_timeout",
                                                      &dock_result_timeout_)) {
    ROS_ERROR("Executive: Dock result timeout not specified.");
  }

  if (!config_params_.GetPosReal("perch_result_timeout",
                                                      &perch_result_timeout_)) {
    ROS_ERROR("Executive: Perch result timeout not specified.");
  }

  if (!config_params_.GetPosReal("switch_result_timeout",
                                                    &switch_result_timeout_)) {
    ROS_ERROR("Executive: Switch result timeout not specified.");
  }
}

void Executive::PublishAgentState() {
  agent_state_.header.stamp = ros::Time::now();
  agent_state_pub_.publish(agent_state_);
}

}  // namespace executive

PLUGINLIB_EXPORT_CLASS(executive::Executive, nodelet::Nodelet)
