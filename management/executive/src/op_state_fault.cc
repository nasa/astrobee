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
    if (CheckNotMoving(cmd->cmd_id, cmd->cmd_origin, cmd->cmd_name)) {
      exec_->ClearData(cmd, err_msg, status);
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
                            "docking/docked/undocking");
    } else {
      exec_->StopArm(cmd->cmd_id, cmd->cmd_origin);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOW_ARM) {
    // Don't stow arm when docking, docked, perching, perched
    ff_msgs::MobilityState state = exec_->GetMobilityState();
    if (state.state == ff_msgs::MobilityState::DOCKING) {
      AckMobilityStateIssue(cmd->cmd_id,
                            cmd->cmd_origin,
                            cmd->cmd_name,
                            "docking/docked/undocking");
    } else if (state.state == ff_msgs::MobilityState::PERCHING) {
      AckMobilityStateIssue(cmd->cmd_id,
                            cmd->cmd_origin,
                            cmd->cmd_name,
                            "perching/perched/unperching");
    } else {
      exec_->StowArm(cmd->cmd_id, cmd->cmd_origin);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ALL_MOTION) {
    bool stop_started;
    exec_->StopAllMotion(stop_started, cmd->cmd_id, cmd->cmd_origin);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SHUTDOWN) {
    // Don't want to shutdown when flying, docking, perching, or trying to stop
    if (CheckNotMoving(cmd->cmd_id, cmd->cmd_origin, cmd->cmd_name)) {
      exec_->Shutdown(cmd->cmd_id, cmd->cmd_origin);
    }
  } else {
    err_msg = "Command " + cmd->cmd_name + " not accepted in op state"
        + " fault.";
    exec_->PublishCmdAck(cmd->cmd_id,
                         cmd->cmd_origin,
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
                         err_msg);
    ROS_WARN("Executive: %s", err_msg.c_str());
  }
  return this;
}

}  // namespace executive
