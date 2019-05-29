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

#include "executive/op_state_auto_return.h"

namespace executive {
OpState* OpStateAutoReturn::HandleCmd(ff_msgs::CommandStampedPtr const& cmd) {
  bool completed = false, successful = false;
  // Check if command is accepted in every op state and if so, execute it
  OpState::HandleCmd(cmd, completed, successful);
  if (completed) {
    return this;
  }

  if (cmd->cmd_name == CommandConstants::CMD_NAME_FAULT) {
    if (exec_->Fault(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_IDLE_PROPULSION) {
    if (exec_->IdlePropulsion(cmd)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_OFF_ITEM) {
    exec_->PowerItemOff(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_ON_ITEM) {
    exec_->PowerItemOn(cmd);
  } else if (cmd->cmd_name ==
                        CommandConstants::CMD_NAME_SET_FLASHLIGHT_BRIGHTNESS) {
    exec_->SetFlashlightBrightness(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_PLAN) {
    exec_->SetPlan(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ALL_MOTION) {
    if (exec_->StopAllMotion(cmd)) {
      // Check if we are stopping and if so, transition to teleop. Otherwise
      // transition to ready
      if (exec_->IsActionRunning(STOP)) {
        // If stop started, need to transition to teleop
        return OpStateRepo::Instance()->teleop()->StartupState();
      } else {  // Stop was successful but stop not started so go to ready
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ARM) {
    exec_->StopArm(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_WIPE_HLP) {
    exec_->WipeHlp(cmd);
  } else {
    std::string err_msg = "Command " + cmd->cmd_name +
                                      " not accepted in op state auto return.";
    AckCmd(cmd->cmd_id, ff_msgs::AckCompletedStatus::EXEC_FAILED, err_msg);
  }
  return this;
}
}  // namespace executive
