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
  uint8_t status;

  // Check if command is accepted in every op state and if so, execute it
  OpState::HandleCmd(cmd, completed, successful, err_msg, status);
  if (completed) {
    return this;
  }

  // TODO(Katie) Check source before setting this! Executive will also receive
  // commands from the fault manager and sequencer
  // TODO(Katie) Add more commands
  if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_PLAN) {
    ROS_INFO("Set plan command received!!");

    if (exec_->SetPlan()) {
      exec_->PublishCmdAck(cmd->cmd_id, cmd->cmd_origin);
    } else {
      // If it isn't valid, send failed command ack
      exec_->PublishCmdAck(cmd->cmd_id,
                           cmd->cmd_origin,
                           ff_msgs::AckCompletedStatus::EXEC_FAILED,
                           "No plan found or invalid syntax in plan uploaded");
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_OPERATING_LIMITS) {
    if (!exec_->SetOperatingLimits(cmd->args, err_msg)) {
      ROS_ERROR("Executive: %s", err_msg.c_str());
      exec_->PublishCmdAck(cmd->cmd_id,
                           cmd->cmd_origin,
                           ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                           err_msg);
    } else {
      exec_->PublishCmdAck(cmd->cmd_id, cmd->cmd_origin);
    }
    return this;
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_ZONES) {
    ROS_INFO("Set zones received!");
    err_msg = exec_->SetZones();

    // Set zones returns an empty string if the zones loaded correctly
    if (err_msg.size() == 0) {
      exec_->PublishCmdAck(cmd->cmd_id,
                           cmd->cmd_origin);
    } else {
      // If the zones file wasn't loaded correctly, send a failed ack
      exec_->PublishCmdAck(cmd->cmd_id,
                           cmd->cmd_origin,
                           ff_msgs::AckCompletedStatus::EXEC_FAILED,
                           err_msg);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_RUN_PLAN) {
    if (exec_->GetPlanExecState() != ff_msgs::ExecState::PAUSED) {
      ROS_ERROR("Got command to run plan but plan status is not paused!");
      exec_->PublishCmdAck(cmd->cmd_id,
                           cmd->cmd_origin,
                           ff_msgs::AckCompletedStatus::EXEC_FAILED,
                           "Got run plan cmd but plan status is not paused!");
      return this;
    }

    // Check if plan is empty before switch operating states
    // Also send successful ack because technically we didn't fail, the plan
    // was just empty
    if (exec_->GetCurrentPlanItemType() == sequencer::ItemType::NONE) {
      exec_->SetPlanExecState(ff_msgs::ExecState::IDLE);
      exec_->PublishCmdAck(cmd->cmd_id, cmd->cmd_origin);
      return this;
    }

    // We can start plan!
    exec_->PublishCmdAck(cmd->cmd_id,
                         cmd->cmd_origin,
                         ff_msgs::AckCompletedStatus::NOT,
                         "",
                         ff_msgs::AckStatus::EXECUTING);
    return OpStateRepo::Instance()->plan_exec()->StartupState(cmd->cmd_id,
                                                              cmd->cmd_origin);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SKIP_PLAN_STEP) {
    exec_->SkipPlanStep(cmd->cmd_id, cmd->cmd_origin);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_REACQUIRE_POSITION) {
    if (exec_->StartAction(SWITCH, cmd->cmd_id, cmd->cmd_origin, err_msg)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CHECK_OBSTACLES) {
    exec_->SetCheckObstacles(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CHECK_ZONES) {
    exec_->SetCheckZones(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_HOLONOMIC_MODE) {
    exec_->SetHolonomicMode(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SIMPLE_MOVE6DOF) {
    // Make sure we are stopped, not docked or perched, before moving
    if (exec_->GetMobilityState().state == ff_msgs::MobilityState::STOPPING &&
        exec_->GetMobilityState().sub_state == 0) {
      if (!exec_->FillMoveGoal(cmd)) {
        return this;
      }

      if (!exec_->ConfigureMobility(cmd->cmd_id, cmd->cmd_origin, err_msg)) {
        return this;
      }

      if (exec_->StartAction(MOVE, cmd->cmd_id, cmd->cmd_origin, err_msg)) {
        return OpStateRepo::Instance()->teleop()->StartupState();
      }
    } else {
      exec_->PublishCmdAck(cmd->cmd_id,
                           cmd->cmd_origin,
                           ff_msgs::AckCompletedStatus::EXEC_FAILED,
                           "Cannot moved when in ready and not stopped.");
      return this;
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_IDLE_PROPULSION) {
    // TODO(Katie) Change to not idle if perched or docked
    if (exec_->StartAction(IDLE, cmd->cmd_id, cmd->cmd_origin, err_msg)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ALL_MOTION) {
    // We shouldn't be executing any actions so we don't need to cancel any
    // actions. Stop is used in op state ready to transition from the drifting
    // mobility state to the stopped mobility state.
    bool stop_started;
    exec_->StopAllMotion(stop_started, cmd->cmd_id, cmd->cmd_origin);
    if (stop_started) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_AUTO_RETURN) {
    if (exec_->GetMobilityState().state == ff_msgs::MobilityState::STOPPING &&
        exec_->GetMobilityState().sub_state == 0) {
      // TODO(Katie) Stub, change to be actual code
      exec_->PublishCmdAck(cmd->cmd_id,
                           cmd->cmd_origin,
                           ff_msgs::AckCompletedStatus::EXEC_FAILED,
                           "Auto return not implemented yet! Stay tuned!");
    } else {
      exec_->PublishCmdAck(cmd->cmd_id,
                           cmd->cmd_origin,
                           ff_msgs::AckCompletedStatus::EXEC_FAILED,
                           "Must be stopped before attempting to auto return!");
      ROS_ERROR("Executive: Must be stopped before attempting to auto return!");
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_DOCK) {
    if (exec_->Dock(cmd, err_msg, status)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_UNDOCK) {
    if (exec_->Undock(cmd->cmd_id, cmd->cmd_origin, err_msg)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_PERCH) {
    if (exec_->GetMobilityState().state == ff_msgs::MobilityState::STOPPING &&
        exec_->GetMobilityState().sub_state == 0) {
      // TODO(Katie) Stub, change to be actual code
      exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, 4);
      ros::Duration(1).sleep();
      exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, 3);
      ros::Duration(1).sleep();
      exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, 2);
      ros::Duration(1).sleep();
      exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, 1);
      ros::Duration(1).sleep();
      exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, 0);
      exec_->PublishCmdAck(cmd->cmd_id,
                           cmd->cmd_origin,
                           ff_msgs::AckCompletedStatus::EXEC_FAILED,
                           "Perch not implemented yet! Stay tuned!");
    } else {
      exec_->PublishCmdAck(cmd->cmd_id,
                           cmd->cmd_origin,
                           ff_msgs::AckCompletedStatus::EXEC_FAILED,
                           "Must be stopped before attempting to perch!");
      ROS_ERROR("Executive: Must be stopped before attempting to perch!");
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_UNPERCH) {
    if (exec_->GetMobilityState().state == ff_msgs::MobilityState::PERCHING &&
        exec_->GetMobilityState().sub_state == 0) {
      // TODO(Katie) Stub, change to be actual code
      exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, -1);
      ros::Duration(1).sleep();
      exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, -2);
      ros::Duration(1).sleep();
      exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, -3);
      ros::Duration(1).sleep();
      exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, -4);
      ros::Duration(1).sleep();
      exec_->SetMobilityState(ff_msgs::MobilityState::STOPPING, 0);
      exec_->PublishCmdAck(cmd->cmd_id,
                           cmd->cmd_origin,
                           ff_msgs::AckCompletedStatus::EXEC_FAILED,
                           "Unperch not implemented yet! Stay tuned!");
    } else {
      exec_->PublishCmdAck(cmd->cmd_id,
                           cmd->cmd_origin,
                           ff_msgs::AckCompletedStatus::EXEC_FAILED,
                           "Must be perched to unperch!");
      ROS_ERROR("Executive: Must be perched to unperch!");
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_DOWNLOAD_DATA) {
    exec_->DownloadData(cmd, err_msg, status);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_DOWNLOAD) {
    exec_->StopDownload(cmd);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_CLEAR_DATA) {
    exec_->ClearData(cmd, err_msg, status);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_ON_ITEM) {
    exec_->PowerOnItem(cmd, err_msg, status);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_OFF_ITEM) {
    exec_->PowerOffItem(cmd, err_msg, status);
  } else if (cmd->cmd_name ==
                        CommandConstants::CMD_NAME_SET_FLASHLIGHT_BRIGHTNESS) {
    exec_->SetFlashlightBrightness(cmd, err_msg, status);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ARM) {
    // Don't stop arm in a state where the arm isn't used
    if (exec_->GetMobilityState().state == ff_msgs::MobilityState::DOCKING) {
      AckMobilityStateIssue(cmd->cmd_id,
                            cmd->cmd_origin,
                            cmd->cmd_name,
                            "docked");
    } else {
      exec_->StopArm(cmd->cmd_id, cmd->cmd_origin);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_START_GUEST_SCIENCE) {
    // TODO(Katie) need to do some sort of check for primary vs. secondary since
    // we will have to go into guest science op mode if primary
    exec_->SendGuestScienceCommand(cmd, err_msg, status);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_CUSTOM_GUEST_SCIENCE) {
    // TODO(Katie) May need to add code to track command to make sure it gets
    // acked
    exec_->SendGuestScienceCommand(cmd, err_msg, status);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_GUEST_SCIENCE) {
    // TODO(Katie) May need to add code to track command to make sure it gets
    // acked
    exec_->SendGuestScienceCommand(cmd, err_msg, status);
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_ARM_PAN_AND_TILT) {
    if (!exec_->FillArmGoal(cmd, err_msg)) {
      return this;
    }

    if (exec_->StartAction(ARM, cmd->cmd_id, cmd->cmd_origin, err_msg)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_GRIPPER_CONTROL) {
    if (!exec_->FillArmGoal(cmd, err_msg)) {
        return this;
    }

    if (exec_->StartAction(ARM, cmd->cmd_id, cmd->cmd_origin, err_msg)) {
      return OpStateRepo::Instance()->teleop()->StartupState();
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOW_ARM) {
    // Don't stow arm when perched or docked
    ff_msgs::MobilityState state = exec_->GetMobilityState();
    if (state.state == ff_msgs::MobilityState::DOCKING) {
      AckMobilityStateIssue(cmd->cmd_id,
                            cmd->cmd_origin,
                            cmd->cmd_name,
                            "docked");
    } else if (state.state == ff_msgs::MobilityState::PERCHING) {
      AckMobilityStateIssue(cmd->cmd_id,
                            cmd->cmd_origin,
                            cmd->cmd_name,
                            "perched");
    } else {
      exec_->StowArm(cmd->cmd_id, cmd->cmd_origin);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SHUTDOWN) {
    exec_->Shutdown(cmd->cmd_id, cmd->cmd_origin);
  } else {
    err_msg = "Command " + cmd->cmd_name + " not accepted in operating state " +
        "ready.";
    exec_->PublishCmdAck(cmd->cmd_id,
                         cmd->cmd_origin,
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
                         err_msg);
    ROS_WARN("Executive: %s", err_msg.c_str());
  }
  return this;
}

}  // namespace executive
