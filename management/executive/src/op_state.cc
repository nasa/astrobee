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

OpState* OpState::StartupState(std::string const& cmd_id,
                               std::string const& cmd_origin) {
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
  if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CAMERA) {
    successful = exec_->SetCamera(cmd, err_msg, status, plan);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CAMERA_STREAMING) {
    successful = exec_->SetCameraStreaming(cmd, err_msg, status, plan);
  } else if (cmd->cmd_name ==
                          CommandConstants::CMD_NAME_SET_CAMERA_RECORDING) {
    successful = exec_->SetCameraRecording(cmd, err_msg, status, plan);
  } else if (cmd->cmd_name ==
                            CommandConstants::CMD_NAME_SET_ENABLE_AUTO_RETURN) {
    // Never part of a plan so we can ack it after finishing
    successful = exec_->EnableAutoReturn(cmd);
  } else {
    completed = false;
    successful = false;
  }
  return this;
}

OpState* OpState::HandleArmResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::ArmResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin) {
  ROS_ERROR("Executive: Handle arm result not implemented yet!");
  return this;
}

OpState* OpState::HandleDockActive() {
  ROS_ERROR("Executive: Handle dock active not implemented yet!");
  return this;
}

OpState* OpState::HandleDockFeedback(
                                ff_msgs::DockFeedbackConstPtr const& feedback) {
  ROS_ERROR("Executive: Handle dock feedback not implemented yet");
  return this;
}

OpState* OpState::HandleDockResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::DockResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin) {
  ROS_ERROR("Executive: Handle dock result not implemented yet!");
  return this;
}

OpState* OpState::HandleExecuteActive() {
  ROS_ERROR("Executive: Handle execute active not implemented yet!");
  return this;
}

OpState* OpState::HandleExecuteResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::ExecuteResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin) {
  ROS_ERROR("Executive: Handle execute result not implemented yet!");
  return this;
}

OpState* OpState::HandleIdleResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::IdleResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin) {
  ROS_ERROR("Executive: Handle idle result not implemented yet!");
  return this;
}

OpState* OpState::HandlePerchActive() {
  ROS_ERROR("Executive: Handle perch active not implemented yet!");
  return this;
}

OpState* OpState::HandlePerchFeedback() {
  ROS_ERROR("Executive: Handle perch feedback not implemented yet!");
  return this;
}

OpState* OpState::HandlePerchResult(std::string const& cmd_id,
                                    std::string const& cmd_origin) {
  ROS_ERROR("Executive: Handle perch result not implemented yet!");
  return this;
}

OpState* OpState::HandleMoveActive() {
  ROS_ERROR("Executive: Handle move active not implemented yet!");
  return this;
}

OpState* OpState::HandleMoveFeedback(
                                ff_msgs::MoveFeedbackConstPtr const& feedback) {
  ROS_ERROR("Executive: Handle move feedback not implemented yet!");
  return this;
}

OpState* OpState::HandleMoveResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::MoveResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin) {
  ROS_ERROR("Executive: Handle move result not implemented yet!");
  return this;
}

OpState* OpState::HandleStopActive() {
  ROS_ERROR("Executive: Handle stop active not implemented yet!");
  return this;
}

OpState* OpState::HandleStopResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::StopResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin) {
  ROS_ERROR("Executive: Handle stop result not implemented yet!");
  return this;
}

OpState* OpState::HandleSwitchResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::SwitchResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin) {
  ROS_ERROR("Executive: Handle switch result not implemented yet!");
  return this;
}

OpState* OpState::HandleUndockFeedback(
                              ff_msgs::UndockFeedbackConstPtr const& feedback) {
  ROS_ERROR("Executive: Handle undock feedback not implemented yet!");
  return this;
}

OpState* OpState::HandleUndockResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::UndockResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin) {
  ROS_ERROR("Executive: Handle undock result not implemented yet!");
  return this;
}

OpState* OpState::HandleUnperchActive() {
  ROS_ERROR("Executive: Handle unperch active not implemented yet!");
  return this;
}

OpState* OpState::HandleUnperchFeedback() {
  ROS_ERROR("Executive: Handle unperch feedback not implemented yet!");
  return this;
}

OpState* OpState::HandleUnperchResult(std::string const& cmd_id,
                                      std::string const& cmd_origin) {
  ROS_ERROR("Executive: Handle unperch result not implemented yet!");
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
                                    std::string cmd_origin,
                                    std::string cmd_name,
                                    std::string current_mobility_state,
                                    std::string accepted_mobility_state) {
  std::string err_msg = cmd_name + " not accepted while " +
                                                  current_mobility_state + "!";

  if (accepted_mobility_state != "") {
    err_msg += " Resend command when " + accepted_mobility_state + ".";
  }

  exec_->PublishCmdAck(cmd_id,
                       cmd_origin,
                       ff_msgs::AckCompletedStatus::EXEC_FAILED,
                       err_msg);
  ROS_WARN("Executive: %s", err_msg.c_str());
}


// Used to check the mobility state for commands that can only be executed when
// the astrobee is in some sort of stopped state. Send a failed execution ack
// and return false if mobility state is flying, docking, perching, or stopping.
bool OpState::CheckNotMoving(std::string cmd_id,
                             std::string cmd_origin,
                             std::string cmd_name) {
  ff_msgs::MobilityState state = exec_->GetMobilityState();
  if (state.state == ff_msgs::MobilityState::FLYING) {
    AckMobilityStateIssue(cmd_id, cmd_origin, cmd_name, "flying");
  } else if (state.state == ff_msgs::MobilityState::DOCKING &&
             state.sub_state != 0) {
    // Check if astrobee is docking vs. undocking
    if (state.sub_state > 0) {
      AckMobilityStateIssue(cmd_id, cmd_origin, cmd_name, "docking", "docked");
    } else {
      AckMobilityStateIssue(cmd_id,
                            cmd_origin,
                            cmd_name,
                            "undocking",
                            "stopped");
    }
  } else if (state.state == ff_msgs::MobilityState::PERCHING &&
             state.sub_state != 0) {
    // Check if astrobee is perching vs. unperching
    if (state.sub_state > 0) {
      AckMobilityStateIssue(cmd_id,
                            cmd_origin,
                            cmd_name,
                            "perching",
                            "perched");
    } else {
      AckMobilityStateIssue(cmd_id,
                            cmd_origin,
                            cmd_name,
                            "unperching",
                            "stopped");
    }
  } else if (state.state == ff_msgs::MobilityState::STOPPING &&
             state.sub_state == 1) {
    AckMobilityStateIssue(cmd_id, cmd_origin, cmd_name, "stopping", "stopped");
  } else {
    return true;
  }
  return false;
}

std::string OpState::GenerateActionFailedMsg(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              std::string const& goal_name,
                              std::string const& action_result) {
  std::string err_msg = "";
  if (state == ff_util::FreeFlyerActionState::Enum::PREEMPTED) {
    err_msg = goal_name + " goal was preempted by another node. This really " +
      "shouldn't happen.";
  } else if (state == ff_util::FreeFlyerActionState::Enum::ABORTED) {
    if (action_result == "") {
      err_msg = goal_name + " goal failed with response: " + action_result;
    } else {
      err_msg = goal_name + "goal failed!";
    }
  } else if (state == ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_ACTIVE) {
    err_msg = goal_name + " goal didn't go active in the time specified in " +
      "the executive config file.";
  } else if (state ==
                    ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_RESPONSE) {
    std::string temp_string = "";
    temp_string += std::tolower(goal_name[0]);
    err_msg = "Didn't receive " + temp_string + goal_name.substr(1) +
              " feedback within the time specified in the executive config " +
              "file.";
  } else if (state ==
                    ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_DEADLINE) {
    err_msg = goal_name + " goal didn't complete in the time specified in the" +
      " executive config file.";
  } else {
    err_msg = "Executive didn't recognize or doesn't handle free flyer action";
    err_msg += " state " +  std::to_string(state);
    err_msg += ". Fix code to handle this.";
  }

  return err_msg;
}
}  // namespace executive
