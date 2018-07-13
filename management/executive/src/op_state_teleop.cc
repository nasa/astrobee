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
                            cmd->cmd_name,
                            "(un)docking");
    } else {
      exec_->StopArm(cmd->cmd_id);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOW_ARM) {
    // Don't stow arm when docking, perching, or perched
    ff_msgs::MobilityState state = exec_->GetMobilityState();
    if (state.state == ff_msgs::MobilityState::DOCKING) {
      AckMobilityStateIssue(cmd->cmd_id,
                            cmd->cmd_name,
                            "(un)docking");
    } else if (state.state == ff_msgs::MobilityState::PERCHING) {
      AckMobilityStateIssue(cmd->cmd_id,
                            cmd->cmd_name,
                            "perching/perched/unperching");
    } else {
      exec_->StowArm(cmd->cmd_id);
    }
  // Below are all the commands used to configure mobility
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CHECK_OBSTACLES) {
    // Don't set whether to check obstacles when moving
    if (CheckNotMoving(cmd->cmd_id, cmd->cmd_name)) {
      exec_->SetCheckObstacles(cmd);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CHECK_ZONES) {
    // Don't set whether to check zones when moving
    if (CheckNotMoving(cmd->cmd_id, cmd->cmd_name)) {
      exec_->SetCheckZones(cmd);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_ENABLE_IMMEDIATE) {
    // Don't set enable immediate when moving
    if (CheckNotMoving(cmd->cmd_id, cmd->cmd_name)) {
      exec_->SetEnableImmediate(cmd);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_HOLONOMIC_MODE) {
    // Don't set holonomic mode when moving
    if (CheckNotMoving(cmd->cmd_id, cmd->cmd_name)) {
      exec_->SetHolonomicMode(cmd);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_OPERATING_LIMITS) {
    // Don't set operating limits when moving
    if (CheckNotMoving(cmd->cmd_id, cmd->cmd_name)) {
      if (!exec_->SetOperatingLimits(cmd->args, err_msg)) {
        ROS_ERROR("Executive: %s", err_msg.c_str());
        exec_->PublishCmdAck(cmd->cmd_id,
                             ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                             err_msg);
      } else {
        exec_->PublishCmdAck(cmd->cmd_id);
      }
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_PLANNER) {
    // Don't set planner when moving
    if (CheckNotMoving(cmd->cmd_id, cmd->cmd_name)) {
      exec_->SetPlanner(cmd);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_TIME_SYNC) {
    // Don't set time sync when moving
    if (CheckNotMoving(cmd->cmd_id, cmd->cmd_name)) {
      exec_->SetTimeSync(cmd);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SKIP_PLAN_STEP) {
    exec_->SkipPlanStep(cmd->cmd_id);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_FAULT) {
    exec_->PublishCmdAck(cmd->cmd_id);
    return OpStateRepo::Instance()->fault()->StartupState();
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ALL_MOTION) {
    ROS_DEBUG("Executive: Got stop command in teleop!");
    bool stop_started = true;
    if (!exec_->StopAllMotion(stop_started, cmd->cmd_id, cmd->cmd_src)) {
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
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
                         err_msg);
    ROS_ERROR("Executive: %s", err_msg.c_str());
  }
  return this;
}

OpState* OpStateTeleop::HandleResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              std::string const& result_response,
                              std::string const& cmd_id,
                              Action const& action) {
  if (state == ff_util::FreeFlyerActionState::Enum::SUCCESS) {
    // If the action was switch, we have more work to do
    if (action == SWITCH) {
      // The switch action is only used for reacquire position command. So after
      // the switch, we need to reset the ekf. The reset ekf functions acks the
      // command either way so we don't need to worry about acking it.
      exec_->ResetEkf(cmd_id);
    } else {
      exec_->PublishCmdAck(cmd_id);
    }
  } else {
    std::string err_msg = "";
    if (result_response != "") {
      err_msg = GenerateActionFailedMsg(state, action, result_response);
    } else {
      err_msg = GenerateActionFailedMsg(state, action);
    }

    exec_->PublishCmdAck(cmd_id,
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
                         err_msg);

    ROS_ERROR("Executive: %s", err_msg.c_str());
  }

  // TODO(Katie) see if any other actions are excuting before transitioning to
  // ready
  return OpStateRepo::Instance()->ready()->StartupState();
}

}  // namespace executive
