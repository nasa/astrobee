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

#include "executive/op_state.h"

// TODO(Katie) Get rid of error output. Output mainly for debug purposes
namespace executive {
OpState::OpState(std::string const& name, unsigned char id) :
  name_(name),
  id_(id),
  exec_(NULL) {
}

OpState* OpState::StartupState(std::string const& cmd_id) {
  return this;
}

// The executive pointer can only be set once since there will only be one
// executive running. Since the executive is created after the op state repo,
// it couldn't be a const pointer
void OpState::SetExec(Executive *const exec) {
  if (exec_ == NULL) {
    exec_ = exec;
  }
}

OpState* OpState::HandleCmd(ff_msgs::CommandStampedPtr const& cmd) {
  ROS_ERROR("Executive: Handle command not implemented yet!");
  return this;
}

// TODO(Katie) Add guest science, no op, set data to disk, set enable auto
// return, and set telemetry rate
// Handles commands that are excepted in every op state
OpState* OpState::HandleCmd(ff_msgs::CommandStampedPtr const& cmd,
                            bool& completed,
                            bool& successful,
                            std::string& err_msg,
                            uint8_t& status,
                            bool plan) {
  completed = true;
  successful = true;
  if (cmd->cmd_name == CommandConstants::CMD_NAME_NO_OP) {
    exec_->PublishCmdAck(cmd->cmd_id);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CAMERA) {
    successful = exec_->SetCamera(cmd, err_msg, status, plan);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CAMERA_STREAMING) {
    successful = exec_->SetCameraStreaming(cmd, err_msg, status, plan);
  } else if (cmd->cmd_name ==
                          CommandConstants::CMD_NAME_SET_CAMERA_RECORDING) {
    successful = exec_->SetCameraRecording(cmd, err_msg, status, plan);
  } else if (cmd->cmd_name ==
                            CommandConstants::CMD_NAME_SET_ENABLE_AUTO_RETURN) {
    // Never part of a plan so we can ack it after finishing
    successful = exec_->SetEnableAutoReturn(cmd);
  } else {
    completed = false;
    successful = false;
  }
  return this;
}

OpState* OpState::HandleResult(ff_util::FreeFlyerActionState::Enum const& state,
                               std::string const& result_response,
                               std::string const& cmd_id,
                               Action const& action) {
  ROS_ERROR("Executive: Handle action result not implemented yet!");
  return this;
}

OpState* OpState::HandleWaitCallback() {
  ROS_ERROR("Executive: Handle wait callback not implemented yet!");
  return this;
}

// TODO(Katie) Remove if you end up changing the start, custom, and stop
// commands to actions.
OpState* OpState::HandleGuestScienceAck(ff_msgs::AckStampedConstPtr const&
                                                                          ack) {
  ROS_ERROR("Executive: Handle guest science ack not implemented yet!");
  return this;
}

// Used as a helper function to send a failed ack when the command is not
// accepted in the current mobility state
void OpState::AckMobilityStateIssue(std::string cmd_id,
                                    std::string cmd_name,
                                    std::string current_mobility_state,
                                    std::string accepted_mobility_state) {
  std::string err_msg = cmd_name + " not accepted while " +
                                                  current_mobility_state + "!";

  if (accepted_mobility_state != "") {
    err_msg += " Resend command when " + accepted_mobility_state + ".";
  }

  exec_->PublishCmdAck(cmd_id,
                       ff_msgs::AckCompletedStatus::EXEC_FAILED,
                       err_msg);
  ROS_WARN("Executive: %s", err_msg.c_str());
}


// Used to check the mobility state for commands that can only be executed when
// the astrobee is in some sort of stopped state. Send a failed execution ack
// and return false if mobility state is flying, docking, perching, or stopping.
bool OpState::CheckNotMoving(std::string cmd_id,
                             std::string cmd_name) {
  ff_msgs::MobilityState state = exec_->GetMobilityState();
  if (state.state == ff_msgs::MobilityState::FLYING) {
    AckMobilityStateIssue(cmd_id, cmd_name, "flying");
  } else if (state.state == ff_msgs::MobilityState::DOCKING &&
             state.sub_state != 0) {
    // Check if astrobee is docking vs. undocking
    if (state.sub_state > 0) {
      AckMobilityStateIssue(cmd_id, cmd_name, "docking", "docked");
    } else {
      AckMobilityStateIssue(cmd_id,
                            cmd_name,
                            "undocking",
                            "stopped");
    }
  } else if (state.state == ff_msgs::MobilityState::PERCHING &&
             state.sub_state != 0) {
    // Check if astrobee is perching vs. unperching
    if (state.sub_state > 0) {
      AckMobilityStateIssue(cmd_id,
                            cmd_name,
                            "perching",
                            "perched");
    } else {
      AckMobilityStateIssue(cmd_id,
                            cmd_name,
                            "unperching",
                            "stopped");
    }
  } else if (state.state == ff_msgs::MobilityState::STOPPING &&
             state.sub_state == 1) {
    AckMobilityStateIssue(cmd_id, cmd_name, "stopping", "stopped");
  } else {
    return true;
  }
  return false;
}

std::string OpState::GenerateActionFailedMsg(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              Action const& action,
                              std::string const& action_result) {
  std::string err_msg = "";
  std::string action_name = GetActionString(action);
  if (state == ff_util::FreeFlyerActionState::Enum::PREEMPTED) {
    err_msg = action_name + " goal was preempted by another node. This really" +
      " shouldn't happen.";
  } else if (state == ff_util::FreeFlyerActionState::Enum::ABORTED) {
    if (action_result != "") {
      err_msg = action_name + " goal failed with response: " + action_result;
    } else {
      err_msg = action_name + " goal failed!";
    }
  } else if (state == ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_ACTIVE) {
    err_msg = action_name + " goal didn't go active in the time specified in " +
      "the executive config file.";
  } else if (state ==
                    ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_RESPONSE) {
    std::string temp_string = "";
    temp_string += std::tolower(action_name[0]);
    err_msg = "Didn't receive " + temp_string + action_name.substr(1) +
              " feedback within the time specified in the executive config " +
              "file.";
  } else if (state ==
                    ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_DEADLINE) {
    err_msg = action_name + " goal didn't complete in the time specified in " +
      "the executive config file.";
  } else {
    err_msg = "Executive didn't recognize or doesn't handle free flyer action";
    err_msg += " state " +  std::to_string(state);
    err_msg += ". Fix code to handle this.";
  }

  return err_msg;
}

std::string OpState::GetActionString(Action const& action) {
  std::string action_str = "?";
  switch (action) {
    case ARM:
      action_str = "Arm";
      break;
    case DOCK:
      action_str = "Dock";
      break;
    case EXECUTE:
      action_str = "Execute";
      break;
    case IDLE:
      action_str = "Idle";
      break;
    case MOVE:
      action_str = "Move";
      break;
    case PERCH:
      action_str = "Perch";
      break;
    case STOP:
      action_str = "Stop";
      break;
    case SWITCH:
      action_str = "Switch";
      break;
    case UNDOCK:
      action_str = "Undock";
      break;
    case UNPERCH:
      action_str = "Unperch";
      break;
    default:
      ROS_ERROR("Executive: Action unknown or wrong in action result.");
  }

  return action_str;
}
}  // namespace executive
