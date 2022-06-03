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

  // Check if command is accepted in every op state and if so, execute it
  OpState::HandleCmd(cmd, completed, successful);
  if (completed) {
    return this;
  }

  if (cmd->cmd_name == CommandConstants::CMD_NAME_ARM_PAN_AND_TILT) {
    exec_->ArmPanAndTilt(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_CUSTOM_GUEST_SCIENCE) {
    exec_->CustomGuestScience(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_DEPLOY_ARM) {
    exec_->DeployArm(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_FAULT) {
    if (exec_->Fault(cmd)) {
      return OpStateRepo::Instance()->fault()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_GRIPPER_CONTROL) {
    exec_->GripperControl(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_IDLE_PROPULSION) {
    exec_->IdlePropulsion(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_OFF_ITEM) {
    exec_->PowerItemOff(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_ON_ITEM) {
    exec_->PowerItemOn(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_PREPARE) {
    exec_->Prepare(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_REACQUIRE_POSITION) {
    exec_->ReacquirePosition(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_RESET_EKF) {
    exec_->ResetEkf(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CHECK_OBSTACLES) {
    exec_->SetCheckObstacles(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CHECK_ZONES) {
    exec_->SetCheckZones(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_ENABLE_IMMEDIATE) {
    exec_->SetEnableImmediate(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_ENABLE_REPLAN) {
    exec_->SetEnableReplan(cmd);
  } else if (cmd->cmd_name ==
                        CommandConstants::CMD_NAME_SET_FLASHLIGHT_BRIGHTNESS) {
    exec_->SetFlashlightBrightness(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_HOLONOMIC_MODE) {
    exec_->SetHolonomicMode(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_INERTIA) {
    exec_->SetInertia(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_OPERATING_LIMITS) {
    exec_->SetOperatingLimits(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_PLAN) {
    exec_->SetPlan(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_PLANNER) {
    exec_->SetPlanner(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_ZONES) {
    exec_->SetZones(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SIMPLE_MOVE6DOF) {
    // Make sure we aren't docked, docking, perched, or perching
    if (exec_->GetMobilityState().state != ff_msgs::MobilityState::DOCKING &&
        exec_->GetMobilityState().state != ff_msgs::MobilityState::PERCHING) {
      // If flying, need to stop
      if (exec_->GetMobilityState().state == ff_msgs::MobilityState::FLYING) {
        exec_->CancelAction(MOVE, "move");
        // Need to stop before we can issue the next move
        exec_->FillMotionGoal(STOP);
        if (!exec_->StartAction(STOP, cmd->cmd_id)) {
          return OpStateRepo::Instance()->ready()->StartupState();
        }
        // Store move command for when stop is done
        move_cmd_ = cmd;
      } else {
        if (!exec_->FillMotionGoal(MOVE, cmd)) {
          return this;
        }

        if (!exec_->ConfigureMobility(false, err_msg)) {
          AckCmd(cmd->cmd_id,
                 ff_msgs::AckCompletedStatus::EXEC_FAILED,
                 err_msg);
          return this;
        }

        exec_->StartAction(MOVE, cmd->cmd_id);
      }
    } else {
      AckCmd(cmd->cmd_id,
             ff_msgs::AckCompletedStatus::EXEC_FAILED,
             "Cannot execute a move when docked, docking, perched or perching");
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SKIP_PLAN_STEP) {
    exec_->SkipPlanStep(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_START_GUEST_SCIENCE) {
    exec_->StartGuestScience(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ALL_MOTION) {
    exec_->StopAllMotion(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ARM) {
    exec_->StopArm(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_GUEST_SCIENCE) {
    exec_->StopGuestScience(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOW_ARM) {
    exec_->StowArm(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SWITCH_LOCALIZATION) {
    exec_->SwitchLocalization(cmd);
  } else {
    err_msg = "Command " + cmd->cmd_name + " not accepted in op state" +
                                                                    " teleop.";
    AckCmd(cmd->cmd_id, ff_msgs::AckCompletedStatus::EXEC_FAILED, err_msg);
  }

  // Check if actions are running. If they aren't, go back to ready mode
  if (!exec_->AreActionsRunning()) {
    return OpStateRepo::Instance()->ready()->StartupState();
  }

  return this;
}

OpState* OpStateTeleop::HandleResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              std::string const& result_response,
                              std::string const& cmd_id,
                              Action const& action) {
  std::string err_msg;

  if (state == ff_util::FreeFlyerActionState::Enum::SUCCESS) {
    // If the action was reacquire, we have more work to do
    if (action == REACQUIRE) {
      // For the reacquire position command, we switch localization to mapped
      // landmarks and then reset the EKF. At this point we have only switched
      // localization so we need to reset the ekf. The reset ekf functions acks
      // the command either way so we don't need to worry about acking it.
      if (exec_->ResetEkf(cmd_id)) {
        return this;
      }
    } else if (action == STOP) {
      // If we stopped so we could move again, we need to issue the move
      if (move_cmd_ != NULL) {
        if (!exec_->FillMotionGoal(MOVE, move_cmd_)) {
          move_cmd_ = NULL;
          return OpStateRepo::Instance()->ready()->StartupState();
        }

        if (!exec_->ConfigureMobility(false, err_msg)) {
          AckCmd(move_cmd_->cmd_id,
                 ff_msgs::AckCompletedStatus::EXEC_FAILED,
                 err_msg);
          move_cmd_ = NULL;
          return OpStateRepo::Instance()->ready()->StartupState();
        }

        if (exec_->StartAction(MOVE, move_cmd_->cmd_id)) {
          move_cmd_ = NULL;
          return this;
        }
        // Clear out move command
        move_cmd_ = NULL;
      } else {
        AckCmd(cmd_id);
      }
    } else {
      // Need to check if command was part of a plan in the case that we got a
      // fault and it cleared before the action completed
      if (cmd_id == "plan") {
        SetPlanStatus(true);
      } else {
        AckCmd(cmd_id);
      }
    }
  } else {
    std::string err_msg = "";
    err_msg = GenerateActionFailedMsg(state, action, result_response);
    // Need to check if command was part of a plan in the case that we got a
    // fault and it cleared before the action completed
    if (cmd_id == "plan") {
      SetPlanStatus(false, err_msg);
    } else {
      AckCmd(cmd_id, ff_msgs::AckCompletedStatus::EXEC_FAILED, err_msg);
    }
  }

  // TODO(Katie) see if any other actions are excuting before transitioning to
  // ready
  return OpStateRepo::Instance()->ready()->StartupState();
}

}  // namespace executive
