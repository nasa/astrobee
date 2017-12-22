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

OpState* OpStateTeleop::HandleDockActive(Action const& action) {
  // We only need to set the mobility state when we start docking since the
  // mobility state is already set to the first undocking state when we are
  // docked.
  if (action == DOCK) {
    exec_->SetMobilityState(ff_msgs::MobilityState::DOCKING,
                            ff_msgs::DockState::DOCKING_MAX_STATE);
  }
  return this;
}

OpState* OpStateTeleop::HandleDockFeedback(
                                ff_msgs::DockFeedbackConstPtr const& feedback) {
  // If we are recovering because un/docking failed or the docker doesn't know
  // what state it is in, don't change the mobility state since there isn't a
  // mobility state to reflect that
  if (feedback->state.state < ff_msgs::DockState::UNKNOWN) {
    exec_->SetMobilityState(ff_msgs::MobilityState::DOCKING,
      feedback->state.state);
  }

  // TODO(Katie) Do we need/care about progress?
  // Progress is only set when the robot is ingressing
  /*if (feedback->status == ff_msgs::DockState::INGRESSING) {
    exec_->SetProximity(feedback->progress);
  }*/
  return this;
}

OpState* OpStateTeleop::HandleDockResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::DockResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin,
                              Action const& action) {
  // TODO(Katie) Do we need/care about proximity ?
  // exec_->SetProximity(0);

  if (state == ff_util::FreeFlyerActionState::Enum::SUCCESS &&
      (result->response == ff_msgs::DockResult::DOCKED ||
       result->response == ff_msgs::DockResult::ALREADY_DOCKED) &&
       action == DOCK) {
    exec_->SetMobilityState(ff_msgs::MobilityState::DOCKING);
    exec_->PublishCmdAck(cmd_id, cmd_origin);
  } else if (state == ff_util::FreeFlyerActionState::Enum::SUCCESS &&
            (result->response == ff_msgs::DockResult::UNDOCKED ||
             result->response == ff_msgs::DockResult::ALREADY_UNDOCKED) &&
             action == UNDOCK) {
    exec_->SetMobilityState(ff_msgs::MobilityState::STOPPING);
    exec_->PublishCmdAck(cmd_id, cmd_origin);
  } else {
    std::string err_msg = "", action_str = "";
    if (action == DOCK) {
      exec_->SetMobilityState(ff_msgs::MobilityState::STOPPING);
      action_str = "Dock";
    } else if (action == UNDOCK) {
      exec_->SetMobilityState(ff_msgs::MobilityState::DOCKING);
      action_str = "Undock";
    } else {
      ROS_ERROR("Executive: WTF Action isn't dock or undock in dock result.");
      action_str = "?";
    }

    if (result != nullptr) {
      err_msg = GenerateActionFailedMsg(state,
                                        action_str,
                                        std::to_string(result->response));
    } else {
      err_msg = GenerateActionFailedMsg(state, action_str);
    }

    exec_->PublishCmdAck(cmd_id,
                         cmd_origin,
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
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

OpState* OpStateTeleop::HandleMotionActive(Action const& action) {
  if (action == MOVE) {
    exec_->SetMobilityState(ff_msgs::MobilityState::FLYING);
  } else if (action == STOP) {
    exec_->SetMobilityState(ff_msgs::MobilityState::STOPPING, 1);
  }

  return this;
}

OpState* OpStateTeleop::HandleMotionResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::MotionResultConstPtr const& result,
                              std::string const& cmd_id,
                              std::string const& cmd_origin,
                              Action const& action) {
  std::string action_str;
  // Don't need case for execute since it is only a plan command
  switch (action) {
    case IDLE:
      action_str = "Idle";
      break;
    case MOVE:
      action_str = "Move";
      break;
    case STOP:
      action_str = "Stop";
      break;
    default:
      ROS_ERROR("Executive: Action unknown or wrong in motion result!");
      action_str = "?";
  }

  if (state == ff_util::FreeFlyerActionState::Enum::SUCCESS &&
      (result->response == ff_msgs::MotionResult::SUCCESS ||
       result->response == ff_msgs::MotionResult::ALREADY_THERE)) {
    exec_->PublishCmdAck(cmd_id, cmd_origin);
    if (action == IDLE) {
      exec_->SetMobilityState(ff_msgs::MobilityState::DRIFTING);
    } else {
      // Move and stop actions end in a stopped state
      exec_->SetMobilityState(ff_msgs::MobilityState::STOPPING);
    }
  } else {
    std::string err_msg = "";
    if (result != nullptr) {
      err_msg = GenerateActionFailedMsg(state,
                                        action_str,
                                        std::to_string(result->response));
    } else {
      err_msg = GenerateActionFailedMsg(state, action_str);
    }

    exec_->PublishCmdAck(cmd_id,
                        cmd_origin,
                        ff_msgs::AckCompletedStatus::EXEC_FAILED,
                        err_msg);
    ROS_ERROR("Executive: %s", err_msg.c_str());
    // If the action is stop and the action failed, don't set the mobility state
    // to anything since we have no idea what state we are in if we can't stop.
    // Probably idle but we will set that based on the GN&C control state.
    if (action == MOVE || action == IDLE) {
      exec_->SetMobilityState(ff_msgs::MobilityState::STOPPING);
    }
  }

  // TODO(Katie) see if any other actions are excuting before transitioning to
  // ready
  return OpStateRepo::Instance()->ready()->StartupState();
}

}  // namespace executive
