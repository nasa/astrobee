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
  std::string err_msg;
  uint8_t status;
  // Check if command is accepted in every op state and if so, execute it
  OpState::HandleCmd(cmd, completed, successful, err_msg, status);
  if (completed) {
    return this;
  }

  if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_ON_ITEM) {
    exec_->PowerItem(cmd, err_msg, status, true);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_OFF_ITEM) {
    exec_->PowerItem(cmd, err_msg, status, false);
  } else if (cmd->cmd_name ==
                        CommandConstants::CMD_NAME_SET_FLASHLIGHT_BRIGHTNESS) {
    exec_->SetFlashlightBrightness(cmd, err_msg, status);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ARM) {
    // TODO(Katie) Stub, add actual code later
    exec_->StopArm(cmd->cmd_id);
  } else {
    err_msg = "Command " + cmd->cmd_name + " not accepted in op state" +
                                                                " auto return.";
    exec_->PublishCmdAck(cmd->cmd_id,
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
                         err_msg);
    ROS_ERROR("Executive: %s", err_msg.c_str());
  }
  return this;
}
}  // namespace executive
