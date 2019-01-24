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

#include "executive/op_state_fault.h"

namespace executive {

OpState* OpStateFault::StartupState(std::string const& cmd_id) {
  run_plan_cmd_id_ = cmd_id;
  return this;
}

OpState* OpStateFault::HandleCmd(ff_msgs::CommandStampedPtr const& cmd) {
  bool completed = false, successful = false;
  std::string err_msg;
  // Check if command is accepted in every op state and execute it if it is
  OpState::HandleCmd(cmd, completed, successful);
  if (completed) {
    return this;
  }

  if (cmd->cmd_name == CommandConstants::CMD_NAME_ARM_PAN_AND_TILT) {
    exec_->ArmPanAndTilt(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_CLEAR_DATA) {
    exec_->ClearData(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_DOWNLOAD_DATA) {
    exec_->DownloadData(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_FAULT) {
    exec_->Fault(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_GRIPPER_CONTROL) {
    exec_->GripperControl(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_IDLE_PROPULSION) {
    exec_->IdlePropulsion(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_INITIALIZE_BIAS) {
    exec_->InitializeBias(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_OFF_ITEM) {
    exec_->PowerItemOff(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_ON_ITEM) {
    exec_->PowerItemOn(cmd);
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
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ALL_MOTION) {
    exec_->StopAllMotion(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ARM) {
    exec_->StopArm(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_DOWNLOAD) {
    exec_->StopDownload(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOW_ARM) {
    exec_->StowArm(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SWITCH_LOCALIZATION) {
    exec_->SwitchLocalization(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_WIPE_HLP) {
    exec_->WipeHlp(cmd);
  } else {
    err_msg = "Command " + cmd->cmd_name + " not accepted in op state"
        + " fault.";
    AckCmd(cmd->cmd_id, ff_msgs::AckCompletedStatus::EXEC_FAILED, err_msg);
    ROS_WARN("Executive: %s", err_msg.c_str());
  }
  return this;
}

OpState* OpStateFault::HandleResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              std::string const& result_response,
                              std::string const& cmd_id,
                              Action const& action) {
  if (state == ff_util::FreeFlyerActionState::Enum::SUCCESS) {
    // If the action was reacquire, we have more work to do
    if (action == REACQUIRE) {
      // For the reacquire position command, we switch localization to mapped
      // landmarks and then reset the EKF. At this point we have only switched
      // localization so we need to reset the ekf. The reset ekf functions acks
      // the command either way so we don't need to worry about acking it.
      exec_->ResetEkf(cmd_id);
    } else {
      // Need to check if the command was part of a plan or a teleop command
      if (cmd_id == "plan") {
        SetPlanStatus(true);
      } else {
        AckCmd(cmd_id);
      }
    }
  } else {
    std::string err_msg = "";
    err_msg = GenerateActionFailedMsg(state, action, result_response);
    if (cmd_id == "plan") {
      ROS_ERROR("Executive: %s", err_msg.c_str());
      SetPlanStatus(false, err_msg);
    } else {
      AckCmd(cmd_id, ff_msgs::AckCompletedStatus::EXEC_FAILED, err_msg);
    }
  }

  return this;
}

// TODO(Katie) Remove if you end up changing the start, custom, and stop
// commands to actions.
OpState* OpStateFault::HandleGuestScienceAck(
                                      ff_msgs::AckStampedConstPtr const& ack) {
  // If the command is not part of a plan, just pass the ack through to the
  // ground
  if (ack->cmd_id != "plan") {
    AckCmd(ack->cmd_id,
           ack->completed_status.status,
           ack->message,
           ack->status.status);
    return this;
  }

  // If the command isn't done, don't do anything.
  if (ack->completed_status.status == ff_msgs::AckCompletedStatus::NOT) {
    return this;
  } else if (ack->completed_status.status != ff_msgs::AckCompletedStatus::OK) {
    ROS_ERROR("Executive: %s", ack->message.c_str());
    SetPlanStatus(false, ack->message);
  } else {
    SetPlanStatus(true);
  }
  return this;
}

void OpStateFault::SetPlanStatus(bool successful, std::string err_msg) {
  exec_->SetPlanExecState(ff_msgs::ExecState::PAUSED);
  if (successful) {
    // Ack run plan command as cancelled since we are pausing the plan until the
    // fault is cleared
    AckCmd(run_plan_cmd_id_,
           ff_msgs::AckCompletedStatus::CANCELED,
           "Executive had to execute the fault command.");

    exec_->AckCurrentPlanItem();
    exec_->PublishPlanStatus(ff_msgs::AckStatus::QUEUED);
  } else {
    err_msg.append(" Executive also received a fault!");
    ROS_ERROR("Executive: %s", err_msg.c_str());
    AckCmd(run_plan_cmd_id_,
           ff_msgs::AckCompletedStatus::EXEC_FAILED,
           err_msg);
    exec_->PublishPlanStatus(ff_msgs::AckStatus::REQUEUED);
  }

  // Clear out the run plan command id and origin so we don't ack the command
  // again
  run_plan_cmd_id_ = "";
}

}  // namespace executive
