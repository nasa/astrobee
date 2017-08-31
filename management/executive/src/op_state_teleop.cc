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

#include "executive/op_state_teleop.h"

namespace executive {
OpState* OpStateTeleop::HandleCmd(ff_msgs::CommandStampedPtr const& cmd) {
  bool completed = false, successful = false;
  std::string err_msg;
  uint8_t status;

  // Check if command is accepted in every op state and if so, execute it
  OpState::HandleCmd(cmd, completed, successful, err_msg, status);
  if (completed) {
    return this;
  }

  if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_ON_ITEM) {
    exec_->PowerOnItem(cmd, err_msg, status);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_OFF_ITEM) {
    exec_->PowerOffItem(cmd, err_msg, status);
  } else if (cmd->cmd_name ==
                        CommandConstants::CMD_NAME_SET_FLASHLIGHT_BRIGHTNESS) {
    exec_->SetFlashlightBrightness(cmd, err_msg, status);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ARM) {
    // Arm should not be out while docking
    if (exec_->GetMobilityState().state == ff_msgs::MobilityState::DOCKING) {
      AckMobilityStateIssue(cmd->cmd_id,
                            cmd->cmd_origin,
                            cmd->cmd_name,
                            "(un)docking");
    } else {
      exec_->StopArm(cmd->cmd_id, cmd->cmd_origin);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOW_ARM) {
    // Don't stow arm when docking, perching, or perched
    ff_msgs::MobilityState state = exec_->GetMobilityState();
    if (state.state == ff_msgs::MobilityState::DOCKING) {
      AckMobilityStateIssue(cmd->cmd_id,
                            cmd->cmd_origin,
                            cmd->cmd_name,
                            "(un)docking");
    } else if (state.state == ff_msgs::MobilityState::PERCHING) {
      AckMobilityStateIssue(cmd->cmd_id,
                            cmd->cmd_origin,
                            cmd->cmd_name,
                            "perching/perched/unperching");
    } else {
      exec_->StowArm(cmd->cmd_id, cmd->cmd_origin);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CHECK_OBSTACLES) {
    // Don't set whether to check obstacles when moving
    if (CheckNotMoving(cmd->cmd_id, cmd->cmd_origin, cmd->cmd_name)) {
      exec_->SetCheckObstacles(cmd);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CHECK_ZONES) {
    // Don't set whether to check zones when moving
    if (CheckNotMoving(cmd->cmd_id, cmd->cmd_origin, cmd->cmd_name)) {
      exec_->SetCheckZones(cmd);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_HOLONOMIC_MODE) {
    // Don't set holonomic mode when moving
    if (CheckNotMoving(cmd->cmd_id, cmd->cmd_origin, cmd->cmd_name)) {
      exec_->SetHolonomicMode(cmd);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SKIP_PLAN_STEP) {
    exec_->SkipPlanStep(cmd->cmd_id, cmd->cmd_origin);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ALL_MOTION) {
    ROS_DEBUG("Executive: Got stop command in teleop!");
    bool stop_started = true;
    if (!exec_->StopAllMotion(stop_started, cmd->cmd_id, cmd->cmd_origin)) {
      // Stop all motion will only fail if we are docked or perched, already
      // idling or stopping or we failed to start the stop action
      // If we arer idling or stopping, we want to stay in the teleop op state'
      // Also have to check if we are docking since we don't stop when
      // deactivating the pmcs and we want to stay in teleop op state until we
      // receive the docker result
      if (!exec_->IsActionRunning(STOP) && !exec_->IsActionRunning(IDLE) &&
          !exec_->IsActionRunning(DOCK)) {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else {
      // If a stop wasn't started, it means the action that was executing wasn't
      // moving the Astrobee but the action was stopped so we can go into the
      // ready state. Otherwise a stop was started and we want to stay in the
      // teleop op state until the stop is completed
      if (!stop_started) {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    }
  } else {
    err_msg = "Command " + cmd->cmd_name + " not accepted in op state" +
                                                                    " teleop.";
    exec_->PublishCmdAck(cmd->cmd_id,
                         cmd->cmd_origin,
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
                         err_msg);
    ROS_ERROR("Executive: %s", err_msg.c_str());
  }
  return this;
}

OpState* OpStateTeleop::HandleArmResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::ArmResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin) {
  if (state == ff_util::FreeFlyerActionState::Enum::SUCCESS) {
    exec_->PublishCmdAck(cmd_id, cmd_origin);
  } else {
    std::string err_msg = "";
    if (result != nullptr) {
      err_msg = GenerateActionFailedMsg(state, "Arm",
                                              std::to_string(result->response));
    } else {
      err_msg = GenerateActionFailedMsg(state, "Arm");
    }

    exec_->PublishCmdAck(cmd_id,
                         cmd_origin,
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
                         err_msg);
    ROS_ERROR("Executive: %s", err_msg.c_str());
  }

  return OpStateRepo::Instance()->ready()->StartupState();
}

OpState* OpStateTeleop::HandleDockActive() {
  // Substate for docking needs to go from N to 1. Add 1 to the dock max state
  // since dock doesn't take into account that 0 is docked
  exec_->SetMobilityState(ff_msgs::MobilityState::DOCKING,
                          (ff_msgs::DockFeedback::MAX_STATE + 1));
  return this;
}

OpState* OpStateTeleop::HandleDockFeedback(
                                ff_msgs::DockFeedbackConstPtr const& feedback) {
  // Substate for docking needs to go from N to 1 and docking node publishes
  // increasing feedback so we need to subtract the feedback from the max state.
  // Also add 1 to the max state since dock doesn't take into account that 0 is
  // docked
  exec_->SetMobilityState(ff_msgs::MobilityState::DOCKING,
                  ((ff_msgs::DockFeedback::MAX_STATE + 1) - feedback->status));
  // Progress is only set when the robot is ingressing
  if (feedback->status == ff_msgs::DockFeedback::INGRESSING) {
    exec_->SetProximity(feedback->progress);
  }
  return this;
}

OpState* OpStateTeleop::HandleDockResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::DockResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin) {
  if (state == ff_util::FreeFlyerActionState::Enum::SUCCESS &&
      (result->status == ff_msgs::DockResult::DOCKED ||
       result->status == ff_msgs::DockResult::ALREADY_DOCKED)) {
    exec_->SetMobilityState(ff_msgs::MobilityState::DOCKING);
    exec_->SetProximity(0);
    exec_->PublishCmdAck(cmd_id, cmd_origin);
  } else {
    exec_->SetMobilityState(ff_msgs::MobilityState::STOPPING);
    exec_->SetProximity(0);
    std::string err_msg = "";
    if (result != nullptr) {
      err_msg = GenerateActionFailedMsg(state, "Dock",
                                                std::to_string(result->status));
    } else {
      err_msg = GenerateActionFailedMsg(state, "Dock");
    }

    exec_->PublishCmdAck(cmd_id,
                         cmd_origin,
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
                         err_msg);
    ROS_ERROR("Executive: %s", err_msg.c_str());
  }

  return OpStateRepo::Instance()->ready()->StartupState();
}

OpState* OpStateTeleop::HandleIdleResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::IdleResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin) {
  if (state == ff_util::FreeFlyerActionState::Enum::SUCCESS &&
      result->result.response == ff_msgs::MobilityResult::SUCCESS) {
    exec_->SetMobilityState(ff_msgs::MobilityState::DRIFTING);
    exec_->PublishCmdAck(cmd_id, cmd_origin);
  } else {
    std::string err_msg = "";
    if (result != nullptr) {
      err_msg = GenerateActionFailedMsg(state, "Idle",
                                      std::to_string(result->result.response));
    } else {
      err_msg = GenerateActionFailedMsg(state, "Idle");
    }

    exec_->PublishCmdAck(cmd_id,
                         cmd_origin,
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
                         err_msg);
    ROS_ERROR("Executive: %s", err_msg.c_str());
  }

  return OpStateRepo::Instance()->ready()->StartupState();
}

OpState* OpStateTeleop::HandleMoveActive() {
  exec_->SetMobilityState(ff_msgs::MobilityState::FLYING);
  return this;
}

// TODO(Katie) Sounds like we may want to send this feedback to the ground
OpState* OpStateTeleop::HandleMoveFeedback(
                                ff_msgs::MoveFeedbackConstPtr const& feedback) {
  // For now, don't do anything with the feedback
  return this;
}

OpState* OpStateTeleop::HandleMoveResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::MoveResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin) {
  // TODO(Katie) Check for preempted if we got a stop during the move
  if (state == ff_util::FreeFlyerActionState::Enum::SUCCESS) {
    exec_->PublishCmdAck(cmd_id, cmd_origin);
  } else {
    std::string err_msg = "";
    if (result != nullptr) {
      err_msg = GenerateActionFailedMsg(state, "Move",
                                      std::to_string(result->result.response));
    } else {
      err_msg = GenerateActionFailedMsg(state, "Move");
    }

    exec_->PublishCmdAck(cmd_id,
                         cmd_origin,
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
                         err_msg);
    ROS_ERROR("Executive: %s", err_msg.c_str());
  }

  exec_->SetMobilityState(ff_msgs::MobilityState::STOPPING);
  // TODO(Katie) see if any other actions are excuting before transitioning to
  // ready
  return OpStateRepo::Instance()->ready()->StartupState();
}

// TODO(Katie) Need to figure out how to set the stop substate
OpState* OpStateTeleop::HandleStopActive() {
  exec_->SetMobilityState(ff_msgs::MobilityState::STOPPING, 1);
  return this;
}

// TODO(Katie) Add more to me! Like an else
OpState* OpStateTeleop::HandleStopResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::StopResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin) {
  if (state == ff_util::FreeFlyerActionState::Enum::SUCCESS &&
      result->result.response == ff_msgs::MobilityResult::SUCCESS) {
    exec_->SetMobilityState(ff_msgs::MobilityState::STOPPING);
    exec_->PublishCmdAck(cmd_id, cmd_origin);
  } else {
    std::string err_msg = "";
    if (result != nullptr) {
      err_msg = GenerateActionFailedMsg(state, "Stop",
                                      std::to_string(result->result.response));
    } else {
      err_msg = GenerateActionFailedMsg(state, "Stop");
    }

    // TODO(Katie) Possiblly get mobility state from the localization manager so
    // that the ground knows what state GNC is in
    exec_->PublishCmdAck(cmd_id,
                         cmd_origin,
                         ff_msgs::AckCompletedStatus:: EXEC_FAILED,
                         err_msg);
    ROS_ERROR("Executive: %s", err_msg.c_str());
  }
  return OpStateRepo::Instance()->ready()->StartupState();
}

OpState* OpStateTeleop::HandleSwitchResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::SwitchResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin) {
  // A result of -3 is already on this mode which is not an error in this case
  if ((state == ff_util::FreeFlyerActionState::Enum::SUCCESS &&
      result->response == ff_msgs::SwitchResult::SUCCESS) ||
      (state == ff_util::FreeFlyerActionState::Enum::ABORTED &&
       result->response == ff_msgs::SwitchResult::PIPELINE_ALREADY_ACTIVE)) {
    // Switch action is only used for reacquire position command. So after the
    // switch, we need to reset the ekf
    if (exec_->ResetEkf(cmd_id, cmd_origin)) {
      // Ack successful if reset worked, reset function will ack failed if it
      // has issues with the reset
      exec_->PublishCmdAck(cmd_id, cmd_origin);
    }
  } else {
    std::string err_msg = "";
    if (result != nullptr) {
      err_msg = GenerateActionFailedMsg(state, "Switch",
                                              std::to_string(result->response));
    } else {
      err_msg = GenerateActionFailedMsg(state, "Switch");
    }

    exec_->PublishCmdAck(cmd_id,
                         cmd_origin,
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
                         err_msg);
    ROS_ERROR("Executive: %s", err_msg.c_str());
  }
  return OpStateRepo::Instance()->ready()->StartupState();
}

OpState* OpStateTeleop::HandleUndockFeedback(ff_msgs::UndockFeedbackConstPtr
                                                              const& feedback) {
  // Invert status since we are undocking
  exec_->SetMobilityState(ff_msgs::MobilityState::DOCKING,
                                                      (-1 * feedback->status));
  // Progress is only set when the robot is egressing
  if (feedback->status == ff_msgs::UndockFeedback::EGRESSING) {
    exec_->SetProximity(feedback->progress);
  }
  return this;
}

OpState* OpStateTeleop::HandleUndockResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::UndockResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin) {
  // Set mobility state since the robot stops after undocking
  if (state == ff_util::FreeFlyerActionState::Enum::SUCCESS &&
      (result->status == ff_msgs::UndockResult::UNDOCKED ||
       result->status == ff_msgs::UndockResult::ALREADY_UNDOCKED)) {
    exec_->SetMobilityState(ff_msgs::MobilityState::STOPPING);
    exec_->PublishCmdAck(cmd_id, cmd_origin);
  } else {
    exec_->SetMobilityState(ff_msgs::MobilityState::DOCKING);
    std::string err_msg = "";
    if (result != nullptr) {
      err_msg = GenerateActionFailedMsg(state, "Undock",
                                                std::to_string(result->status));
    } else {
      err_msg = GenerateActionFailedMsg(state, "Undock");
    }

    exec_->PublishCmdAck(cmd_id,
                         cmd_origin,
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
                         err_msg);
    ROS_ERROR("Executive: %s", err_msg.c_str());
  }
  exec_->SetProximity(0);
  return OpStateRepo::Instance()->ready()->StartupState();
}

}  // namespace executive
