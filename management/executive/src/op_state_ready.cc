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

#include "executive/op_state_ready.h"

#include <ff_util/config_client.h>

namespace executive {
OpState* OpStateReady::HandleCmd(ff_msgs::CommandStampedPtr const& cmd) {
  std::string err_msg;
  bool completed = false, successful = false;

  // Check if command is accepted in every op state and if so, execute it
  OpState::HandleCmd(cmd, completed, successful);
  if (completed) {
    return this;
  }

  // TODO(Katie) Check source before setting this! Executive will also receive
  // commands from the fault manager and sequencer
  if (cmd->cmd_name == CommandConstants::CMD_NAME_ARM_PAN_AND_TILT) {
    if (exec_->ArmPanAndTilt(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_AUTO_RETURN) {
    if (exec_->AutoReturn(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_CLEAR_DATA) {
    exec_->ClearData(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_CUSTOM_GUEST_SCIENCE) {
    exec_->CustomGuestScience(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_DOCK) {
    if (exec_->Dock(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_DOWNLOAD_DATA) {
    exec_->DownloadData(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_FAULT) {
    if (exec_->Fault(cmd)) {
      return OpStateRepo::Instance()->fault()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_GRIPPER_CONTROL) {
    if (exec_->GripperControl(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_IDLE_PROPULSION) {
    if (exec_->IdlePropulsion(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_INITIALIZE_BIAS) {
    if (exec_->InitializeBias(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_PERCH) {
    if (exec_->Perch(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_OFF_ITEM) {
    exec_->PowerItemOff(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_ON_ITEM) {
    exec_->PowerItemOn(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_PREPARE) {
    if (exec_->Prepare(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_REACQUIRE_POSITION) {
    if (exec_->ReacquirePosition(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_RESET_EKF) {
    if (exec_->ResetEkf(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_RUN_PLAN) {
    // Store run plan command idea just in case we need to ack it in this state
    if (exec_->RunPlan(cmd)) {
      return OpStateRepo::Instance()->plan_exec()->StartupState(cmd->cmd_id);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CHECK_OBSTACLES) {
    exec_->SetCheckObstacles(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CHECK_ZONES) {
    exec_->SetCheckZones(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_ENABLE_IMMEDIATE) {
    exec_->SetEnableImmediate(cmd);
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
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_TIME_SYNC) {
    exec_->SetTimeSync(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_ZONES) {
    exec_->SetZones(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SHUTDOWN) {
    exec_->Shutdown(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SIMPLE_MOVE6DOF) {
    // Make sure we are stopped, not docked or perched, before moving
    if ((exec_->GetMobilityState().state == ff_msgs::MobilityState::STOPPING &&
        exec_->GetMobilityState().sub_state == 0) ||
        exec_->GetMobilityState().state == ff_msgs::MobilityState::DRIFTING) {
      if (!exec_->FillMotionGoal(MOVE, cmd)) {
        return this;
      }

      if (!exec_->ConfigureMobility(cmd->cmd_id)) {
        return this;
      }

      if (exec_->StartAction(MOVE, cmd->cmd_id)) {
        return OpStateRepo::Instance()->teleop()->StartupState();
      }
    } else {
      AckCmd(cmd->cmd_id,
             ff_msgs::AckCompletedStatus::EXEC_FAILED,
             "Cannot move when in ready and not stopped.");
      return this;
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SKIP_PLAN_STEP) {
    exec_->SkipPlanStep(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_START_GUEST_SCIENCE) {
    exec_->StartGuestScience(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ALL_MOTION) {
    // We shouldn't be executing any actions so we don't need to cancel any
    // actions. Stop is used in op state ready to transition from the drifting
    // mobility state to the stopped mobility state.
    exec_->StopAllMotion(cmd);
    if (exec_->IsActionRunning(STOP)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ARM) {
    if (exec_->StopArm(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_DOWNLOAD) {
    exec_->StopDownload(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_GUEST_SCIENCE) {
    exec_->StopGuestScience(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOW_ARM) {
    if (exec_->StowArm(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SWITCH_LOCALIZATION) {
    if (exec_->SwitchLocalization(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_UNDOCK) {
    if (exec_->Undock(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_UNPERCH) {
    if (exec_->Unperch(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_UNTERMINATE) {
    exec_->Unterminate(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_WIPE_HLP) {
    exec_->WipeHlp(cmd);
  } else {
    err_msg = "Command " + cmd->cmd_name + " not accepted in operating state " +
        "ready.";
    AckCmd(cmd->cmd_id, ff_msgs::AckCompletedStatus::EXEC_FAILED, err_msg);
    ROS_WARN("Executive: %s", err_msg.c_str());
  }
  return this;
}

OpState* OpStateReady::HandleGuestScienceAck(
                                      ff_msgs::AckStampedConstPtr const& ack) {
  // There is a small possibility that the ready state will have to handle a
  // guest science ack for a plan. This is possible if the plan state starts a
  // guest science command, a fault occurs so we transition to the fault state,
  // the fault gets cleared so we transition to ready, and then the guest
  // science command completes.
  // If the command is not part of a plan, it gets acked in the executive
  // If the command is not done, don't do anything.
  if (ack->completed_status.status == ff_msgs::AckCompletedStatus::NOT) {
    return this;
  } else if (ack->completed_status.status != ff_msgs::AckCompletedStatus::OK) {
    SetPlanStatus(false, ack->message);
  } else {
    SetPlanStatus(true);
  }
  return this;
}
}  // namespace executive
