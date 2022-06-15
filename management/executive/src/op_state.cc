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

#include "executive/op_state.h"
#include "executive/op_state_repo.h"

// TODO(Katie) Get rid of error output. Output mainly for debug purposes
namespace executive {
OpState::OpState(std::string const& name, unsigned char id) :
  name_(name),
  id_(id),
  exec_(NULL) {
}

OpState* OpState::StartupState(std::string const& cmd_id) {
  return this;
}

// The executive pointer can only be set once since there will only be one
// executive running. Since the executive is created after the op state repo,
// it couldn't be a const pointer
void OpState::SetExec(Executive *const exec) {
  if (exec_ == NULL) {
    exec_ = exec;
  }
}

OpState* OpState::HandleCmd(ff_msgs::CommandStampedPtr const& cmd) {
  ROS_ERROR("Executive: Handle command not implemented yet!");
  return this;
}

// Handles commands that are excepted in every op state
OpState* OpState::HandleCmd(ff_msgs::CommandStampedPtr const& cmd,
                            bool& completed,
                            bool& successful) {
  completed = true;
  successful = true;
  if (cmd->cmd_name == CommandConstants::CMD_NAME_ENABLE_ASTROBEE_INTERCOMMS) {
    successful = exec_->EnableAstrobeeIntercomms(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_NO_OP) {
    successful = exec_->NoOp(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CAMERA) {
    successful = exec_->SetCamera(cmd);
  } else if (cmd->cmd_name ==
                          CommandConstants::CMD_NAME_SET_CAMERA_RECORDING) {
    successful = exec_->SetCameraRecording(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CAMERA_STREAMING) {
    successful = exec_->SetCameraStreaming(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_DATA_TO_DISK) {
    successful = exec_->SetDataToDisk(cmd);
  } else if (cmd->cmd_name ==
                            CommandConstants::CMD_NAME_SET_ENABLE_AUTO_RETURN) {
    successful = exec_->SetEnableAutoReturn(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_TELEMETRY_RATE) {
    successful = exec_->SetTelemetryRate(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_START_RECORDING) {
    successful = exec_->StartRecording(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_GUEST_SCIENCE) {
    successful = exec_->StopGuestScience(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_RECORDING) {
    successful = exec_->StopRecording(cmd);
  } else {
    completed = false;
    successful = false;
  }
  return this;
}

OpState* OpState::HandleResult(ff_util::FreeFlyerActionState::Enum const& state,
                               std::string const& result_response,
                               std::string const& cmd_id,
                               Action const& action) {
  ROS_ERROR("Executive: Handle action result not implemented yet!");
  return this;
}

OpState* OpState::HandleWaitCallback() {
  ROS_ERROR("Executive: Handle wait callback not implemented yet!");
  return this;
}

OpState* OpState::HandleGuestScienceAck(ff_msgs::AckStampedConstPtr const&
                                                                          ack) {
  ROS_ERROR("Executive: Handle guest science ack not implemented yet!");
  return this;
}

void OpState::AckCmd(std::string const& cmd_id,
                     uint8_t completed_status,
                     std::string const& message,
                     uint8_t status) {
  // Don't ack internal stop commands
  if (cmd_id != "internal") {
    exec_->PublishCmdAck(cmd_id, completed_status, message, status);
  }
}

std::string OpState::GenerateActionFailedMsg(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              Action const& action,
                              std::string const& action_result) {
  std::string err_msg = "";
  std::string action_name = GetActionString(action);
  if (state == ff_util::FreeFlyerActionState::Enum::PREEMPTED) {
    err_msg = action_name + " goal was preempted by another node. This really" +
      " shouldn't happen.";
  } else if (state == ff_util::FreeFlyerActionState::Enum::ABORTED) {
    if (action_result != "") {
      err_msg = action_name + " goal failed with response: " + action_result;
    } else {
      err_msg = action_name + " goal failed!";
    }
  } else if (state == ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_ACTIVE) {
    err_msg = action_name + " goal didn't go active in the time specified in " +
      "the executive config file.";
  } else if (state ==
                    ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_RESPONSE) {
    std::string temp_string = "";
    temp_string += std::tolower(action_name[0]);
    err_msg = "Didn't receive " + temp_string + action_name.substr(1) +
              " feedback within the time specified in the executive config " +
              "file.";
  } else if (state ==
                    ff_util::FreeFlyerActionState::Enum::TIMEOUT_ON_DEADLINE) {
    err_msg = action_name + " goal didn't complete in the time specified in " +
      "the executive config file.";
  } else {
    err_msg = "Executive didn't recognize or doesn't handle free flyer action";
    err_msg += " state " +  std::to_string(state);
    err_msg += ". Fix code to handle this.";
  }

  return err_msg;
}

std::string OpState::GetActionString(Action const& action) {
  std::string action_str = "?";
  switch (action) {
    case ARM:
      action_str = "Arm";
      break;
    case DOCK:
      action_str = "Dock";
      break;
    case EXECUTE:
      action_str = "Execute";
      break;
    case IDLE:
      action_str = "Idle";
      break;
    case LOCALIZATION:
    case REACQUIRE:
      action_str = "Localization";
      break;
    case MOVE:
      action_str = "Move";
      break;
    case PERCH:
      action_str = "Perch";
      break;
    case STOP:
      action_str = "Stop";
      break;
    case UNDOCK:
      action_str = "Undock";
      break;
    case UNPERCH:
      action_str = "Unperch";
      break;
    default:
      ROS_ERROR("Executive: Action unknown or wrong in action result.");
  }

  return action_str;
}

bool OpState::PausePlan(ff_msgs::CommandStampedPtr const& cmd) {
  AckCmd(cmd->cmd_id,
         ff_msgs::AckCompletedStatus::EXEC_FAILED,
         ("Pause plan not accepted in opstate " + name() + "!"));
  return false;
}

OpState* OpState::TransitionToState(unsigned char id) {
  if (id == ff_msgs::OpState::READY) {
    return OpStateRepo::Instance()->ready()->StartupState();
  } else if (id == ff_msgs::OpState::PLAN_EXECUTION) {
    return OpStateRepo::Instance()->plan_exec()->StartupState();
  } else if (id == ff_msgs::OpState::TELEOPERATION) {
    return OpStateRepo::Instance()->teleop()->StartupState();
  } else if (id == ff_msgs::OpState::AUTO_RETURN) {
    return OpStateRepo::Instance()->auto_return()->StartupState();
  } else if (id == ff_msgs::OpState::FAULT) {
    return OpStateRepo::Instance()->fault()->StartupState();
  }

  ROS_WARN("Executive: unknown state id in transition to state.");
  return this;
}

void OpState::SetPlanStatus(bool successful, std::string err_msg) {
  exec_->SetPlanExecState(ff_msgs::ExecState::PAUSED);
  if (successful) {
    // Ack run plan command as cancelled since we are pausing the plan until the
    // fault is cleared
    AckCmd(exec_->GetRunPlanCmdId(),
           ff_msgs::AckCompletedStatus::CANCELED,
           "Executive had to execute the fault command.");
    exec_->AckCurrentPlanItem();
    exec_->PublishPlanStatus(ff_msgs::AckStatus::QUEUED);
  } else {
    err_msg.append(" Executive also received a fault!");
    AckCmd(exec_->GetRunPlanCmdId(),
           ff_msgs::AckCompletedStatus::EXEC_FAILED,
           err_msg);
    exec_->PublishPlanStatus(ff_msgs::AckStatus::REQUEUED);
  }
}

}  // namespace executive
