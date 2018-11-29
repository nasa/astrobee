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
  uint8_t status;
  // Check if command is accepted in every op state and execute it if it is
  OpState::HandleCmd(cmd, completed, successful, err_msg, status);
  if (completed) {
    return this;
  }

  if (cmd->cmd_name == CommandConstants::CMD_NAME_DOWNLOAD_DATA) {
    exec_->DownloadData(cmd, err_msg, status);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_DOWNLOAD) {
    exec_->StopDownload(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_CLEAR_DATA) {
    // Don't clear data when flying, docking, perching, or trying to stop
    if (CheckNotMoving(cmd->cmd_id, cmd->cmd_name)) {
      exec_->ClearData(cmd, err_msg, status);
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
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_ON_ITEM) {
    exec_->PowerItem(cmd, err_msg, status, true);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_OFF_ITEM) {
    exec_->PowerItem(cmd, err_msg, status, false);
  } else if (cmd->cmd_name ==
                        CommandConstants::CMD_NAME_SET_FLASHLIGHT_BRIGHTNESS) {
    exec_->SetFlashlightBrightness(cmd, err_msg, status);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ARM) {
    // Arm should not be out while docking
    if (exec_->GetMobilityState().state == ff_msgs::MobilityState::DOCKING) {
      AckMobilityStateIssue(cmd->cmd_id,
                            cmd->cmd_name,
                            "docking/docked/undocking");
    } else {
      exec_->StopArm(cmd->cmd_id);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOW_ARM) {
    // Don't stow arm when docking, docked, perching, perched
    ff_msgs::MobilityState state = exec_->GetMobilityState();
    if (state.state == ff_msgs::MobilityState::DOCKING) {
      AckMobilityStateIssue(cmd->cmd_id,
                            cmd->cmd_name,
                            "docking/docked/undocking");
    } else if (state.state == ff_msgs::MobilityState::PERCHING) {
      AckMobilityStateIssue(cmd->cmd_id,
                            cmd->cmd_name,
                            "perching/perched/unperching");
    } else {
      exec_->StowArm(cmd->cmd_id);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ALL_MOTION) {
    bool stop_started;
    exec_->StopAllMotion(stop_started, cmd->cmd_id, cmd->cmd_src);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_FAULT) {
    exec_->PublishCmdAck(cmd->cmd_id);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SHUTDOWN) {
    // Don't want to shutdown when flying, docking, perching, or trying to stop
    if (CheckNotMoving(cmd->cmd_id, cmd->cmd_name)) {
      exec_->Shutdown(cmd->cmd_id);
    }
  } else {
    err_msg = "Command " + cmd->cmd_name + " not accepted in op state"
        + " fault.";
    exec_->PublishCmdAck(cmd->cmd_id,
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
                         err_msg);
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
    // If the action was switch, we have more work to do
    if (action == SWITCH) {
      // The switch action is only used for reacquire position command which is
      // not a plan command. So after the switch, we need to reset the ekf. The
      // reset ekf functions acks the command either way so we don't need to
      // worry about acking it.
      exec_->ResetEkf(cmd_id);
    } else {
      // Need to check if the command was part of a plan or a teleop command
      if (cmd_id == "plan") {
        SetPlanStatus(true);
      } else {
        exec_->PublishCmdAck(cmd_id);
      }
    }
  } else {
    std::string err_msg = "";
    if (result_response != "") {
      err_msg = GenerateActionFailedMsg(state, action, result_response);
    } else {
      err_msg = GenerateActionFailedMsg(state, action);
    }
    ROS_ERROR("Executive: %s", err_msg.c_str());
    if (cmd_id == "plan") {
      SetPlanStatus(false, err_msg);
    } else {
      exec_->PublishCmdAck(cmd_id,
                           ff_msgs::AckCompletedStatus::EXEC_FAILED,
                           err_msg);
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
    exec_->PublishCmdAck(ack->cmd_id,
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
    exec_->PublishCmdAck(run_plan_cmd_id_,
                         ff_msgs::AckCompletedStatus::CANCELED,
                         "Executive had to execute the fault command.");

    exec_->AckCurrentPlanItem();
    exec_->PublishPlanStatus(ff_msgs::AckStatus::QUEUED);
  } else {
    err_msg.append(" Executive also received a fault!");
    ROS_ERROR("Executive: %s", err_msg.c_str());
    exec_->PublishCmdAck(run_plan_cmd_id_,
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
                         err_msg);
    exec_->PublishPlanStatus(ff_msgs::AckStatus::REQUEUED);
  }

  // Clear out the run plan command id and origin so we don't ack the command
  // again
  run_plan_cmd_id_ = "";
}

}  // namespace executive
