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

#include "executive/op_state_plan_exec.h"
#include "executive/executive.h"
#include "executive/op_state_repo.h"

namespace executive {
OpState* OpStatePlanExec::StartupState(std::string const& cmd_id) {
  std::string err_msg;

  first_segment_ = true;
  run_plan_cmd_id_ = cmd_id;

  // Change operating state and plan state stuff since the first thing in the
  // plan may not be an action meaning the op state would be ready while
  // executing a plan which is no good
  exec_->SetOpState(this);
  exec_->SetPlanExecState(ff_msgs::ExecState::EXECUTING);
  exec_->PublishPlanStatus(ff_msgs::AckStatus::EXECUTING);

  // Don't need to check for empty plan since this was checked before the
  // transition to plan execution state but do need to check for invalid
  // command. If the first item in the plan is a command and invalid, the ready
  // operating state will be returned. Also, if the plan was only instantaneous
  // commands, the ready operating state is returned. We need to check for this.
  OpState* temp_op_state = StartNextPlanItem();
  if (temp_op_state != this) {
    // If the plan execution status is completed then we know that the plan
    // contained instantaneous commands only and we don't do anything. If the
    // plan execution state is executing, we know the first item wasn't
    // successful, so we need to pause the plan.
    if (exec_->GetPlanExecState() == ff_msgs::AckStatus::EXECUTING) {
      exec_->SetPlanExecState(ff_msgs::ExecState::PAUSED);
      exec_->PublishPlanStatus(ff_msgs::AckStatus::QUEUED);
    }
  }
  return temp_op_state;
}

OpState* OpStatePlanExec::HandleCmd(ff_msgs::CommandStampedPtr const& cmd) {
  std::string err_msg;
  bool completed = false, successful = false;

  // TODO(Katie) Add more commands
  if (cmd->cmd_id == "plan" && cmd->cmd_src == "plan") {
    OpState::HandleCmd(cmd, completed, successful);
    if (completed) {
      if (successful) {
        return AckStartPlanItem();
      } else {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    }

    // Don't have to worry about preempting an action since an action has to
    // complete before getting the next item in the plan
    if (cmd->cmd_name == CommandConstants::CMD_NAME_ARM_PAN_AND_TILT) {
      if (!exec_->ArmPanAndTilt(cmd)) {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_CLEAR_DATA) {
      if (exec_->ClearData(cmd)) {
        return AckStartPlanItem();
      } else {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_DOCK) {
      if (!exec_->Dock(cmd)) {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_DOWNLOAD_DATA) {
      if (exec_->DownloadData(cmd)) {
        return AckStartPlanItem();
      } else {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_GRIPPER_CONTROL) {
      if (!exec_->GripperControl(cmd)) {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_IDLE_PROPULSION) {
      if (!exec_->IdlePropulsion(cmd)) {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_INITIALIZE_BIAS) {
      if (!exec_->InitializeBias(cmd)) {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_PAUSE_PLAN) {
      exec_->PausePlan(cmd);
      return OpStateRepo::Instance()->ready()->StartupState();
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_PERCH) {
      if (!exec_->Perch(cmd)) {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_OFF_ITEM) {
      if (exec_->PowerItemOff(cmd)) {
        return AckStartPlanItem();
      } else {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_ON_ITEM) {
      if (exec_->PowerItemOn(cmd)) {
        return AckStartPlanItem();
      } else {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name ==
                              CommandConstants::CMD_NAME_SET_CHECK_OBSTACLES) {
      if (exec_->SetCheckObstacles(cmd)) {
        return AckStartPlanItem();
      } else {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_CHECK_ZONES) {
      if (exec_->SetCheckZones(cmd)) {
        return AckStartPlanItem();
      } else {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name ==
                        CommandConstants::CMD_NAME_SET_FLASHLIGHT_BRIGHTNESS) {
      if (exec_->SetFlashlightBrightness(cmd)) {
        return AckStartPlanItem();
      } else {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_HOLONOMIC_MODE) {
      if (exec_->SetHolonomicMode(cmd)) {
        return AckStartPlanItem();
      } else {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_SET_PLANNER) {
      if (exec_->SetPlanner(cmd)) {
        return AckStartPlanItem();
      } else {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name ==
                              CommandConstants::CMD_NAME_START_GUEST_SCIENCE) {
      // TODO(Katie) Need to do some sort of check for primary vs. secondary
      // since we will have to check for which commands to accept if primary
      if (!exec_->SendGuestScienceCommand(cmd)) {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_GUEST_SCIENCE) {
      // TODO(Katie) Need to do some sort of check for primary vs. secondary
      // since we will have to check for which commands to accept if primary
      if (!exec_->SendGuestScienceCommand(cmd)) {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name ==
                              CommandConstants::CMD_NAME_SWITCH_LOCALIZATION) {
      if (!exec_->SwitchLocalization(cmd)) {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_UNDOCK) {
      if (!exec_->Undock(cmd)) {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_UNPERCH) {
      if (!exec_->Unperch(cmd)) {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_WAIT) {
      if (!exec_->Wait(cmd)) {
        return OpStateRepo::Instance()->ready()->StartupState();
      }
      waiting_ = true;
    } else {
      err_msg = "Plan contains unknown command: " + cmd->cmd_name;
      ROS_ERROR("%s", err_msg.c_str());
      AckPlanCmdFailed(ff_msgs::AckCompletedStatus::BAD_SYNTAX, err_msg);
      return OpStateRepo::Instance()->ready()->StartupState();
    }
  } else {
    // TODO(Katie) Possibly check to see if the robot is executing a waiting
    // command
    // TODO(Katie) For pause, actions will be pausable but other commands will
    // not. It is TBD what happens for a wait command
    // TODO(Katie) Fill this out
    OpState::HandleCmd(cmd, completed, successful);
    if (completed) {
      return this;
    }

    if (cmd->cmd_name == CommandConstants::CMD_NAME_FAULT) {
      if (exec_->Fault(cmd)) {
        // Need to pass the run plan command id and origin to fault state so
        // that when the current step completes, the fault state can ack the run
        // plan command as canceled
        std::string cmd_id = run_plan_cmd_id_;

        // Check if we are executing a wait command. If we are, the wait is
        // stopped and marked as complete.
        if (waiting_) {
          exec_->StopWaitTimer();
          waiting_ = false;
          exec_->AckCurrentPlanItem();
          exec_->PublishPlanStatus(ff_msgs::AckStatus::QUEUED);
          // Ack run plan command here since the current step has completed
          AckCmd(run_plan_cmd_id_,
                 ff_msgs::AckCompletedStatus::EXEC_FAILED,
                 "Executive had to execute the fault command.");

          // Clear out the cmd_id since we just acked the run plan
          // command and will not need to ack it in the fault state
          cmd_id = "";
        }

        // Clear out the run plan command id and origin
        run_plan_cmd_id_ = "";

        return OpStateRepo::Instance()->fault()->StartupState(cmd_id);
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_IDLE_PROPULSION) {
      AckCmd(run_plan_cmd_id_,
             ff_msgs::AckCompletedStatus::CANCELED,
             "Run plan command failed due to an idle propulsion command.");
      // Clear command id since we acked it and don't want to ack it again
      run_plan_cmd_id_ = "";

      exec_->SetPlanExecState(ff_msgs::ExecState::PAUSED);

      if (waiting_) {
        exec_->StopWaitTimer();
        waiting_ = false;
        exec_->AckCurrentPlanItem();
        exec_->PublishPlanStatus(ff_msgs::AckStatus::QUEUED);
      } else {
        exec_->PublishPlanStatus(ff_msgs::AckStatus::REQUEUED);
      }

      if (exec_->IdlePropulsion(cmd)) {
        return OpStateRepo::Instance()->teleop()->StartupState();
      }
      return OpStateRepo::Instance()->ready()->StartupState();
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_PAUSE_PLAN ||
        cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ALL_MOTION) {
      exec_->PausePlan(cmd);
      // Check if we are stopping and if so, transition to teleop. Otherwise
      // transition to ready
      if (exec_->IsActionRunning(STOP)) {
        return OpStateRepo::Instance()->teleop()->StartupState();
      }
      return OpStateRepo::Instance()->ready()->StartupState();
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ARM) {
      // If the plan is executing an arm action, pause plan. If not, just issue
      // a stop action to the arm
      if (exec_->IsActionRunning(ARM)) {
        // Stop Arm will cancel the arm action so just need to pause plan
        exec_->PublishCmdAck(run_plan_cmd_id_,
                             ff_msgs::AckCompletedStatus::CANCELED);
        // Clear command id since we acked it and don't want to ack it again
        run_plan_cmd_id_ = "";
        exec_->SetPlanExecState(ff_msgs::ExecState::PAUSED);
        exec_->PublishPlanStatus(ff_msgs::AckStatus::REQUEUED);
        if (exec_->StopArm(cmd)) {
          return OpStateRepo::Instance()->teleop()->StartupState();
        } else {
          return OpStateRepo::Instance()->ready()->StartupState();
        }
      } else {
        exec_->StopArm(cmd);
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_WIPE_HLP) {
      exec_->WipeHlp(cmd);
    } else {
      err_msg = "Command " + cmd->cmd_name + "not accepted in op state"
          + " plan execution.";
      // Don't stop plan, just send a failed ack
      AckCmd(cmd->cmd_id, ff_msgs::AckCompletedStatus::EXEC_FAILED, err_msg);
      ROS_WARN("Executive: %s", err_msg.c_str());
    }
  }
  return this;
}

OpState* OpStatePlanExec::HandleResult(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              std::string const& result_response,
                              std::string const& cmd_id,
                              Action const& action) {
  return HandleActionComplete(state, action, result_response);
}

OpState* OpStatePlanExec::HandleWaitCallback() {
  waiting_ = false;
  // Ack plan item and start next plan item
  return AckStartPlanItem();
}

// TODO(Katie) Remove if you end up changing the start, custom, and stop
// commands to actions.
OpState* OpStatePlanExec::HandleGuestScienceAck(
                                      ff_msgs::AckStampedConstPtr const& ack) {
  // Check if command is still executing the command and return if so
  if (ack->completed_status.status == ff_msgs::AckCompletedStatus::NOT) {
    return this;
  } else if (ack->completed_status.status != ff_msgs::AckCompletedStatus::OK) {
    AckCmd(run_plan_cmd_id_,
           ack->completed_status.status,
           ack->message,
           ack->status.status);
    run_plan_cmd_id_ = "";
    exec_->SetPlanExecState(ff_msgs::ExecState::PAUSED);
    exec_->PublishPlanStatus(ff_msgs::AckStatus::REQUEUED);
    return OpStateRepo::Instance()->ready()->StartupState();
  } else {
    return AckStartPlanItem();
  }
  return this;
}

void OpStatePlanExec::AckCmd(std::string const& cmd_id,
                             uint8_t completed_status,
                             std::string const& message,
                             uint8_t status) {
  // Check if command is a plan command
  if (cmd_id == "plan") {
    // Only need to check for commands are completed and that fail
    if (completed_status != ff_msgs::AckCompletedStatus::OK &&
        completed_status != ff_msgs::AckCompletedStatus::NOT &&
        completed_status != ff_msgs::AckCompletedStatus::CANCELED) {
      AckPlanCmdFailed(completed_status, message);
    }
  } else {
    // If command isn't a plan command, ack it
    // Don't ack internal stop commands
    if (cmd_id != "internal") {
      exec_->PublishCmdAck(cmd_id, completed_status, message, status);
    }
  }
}

void OpStatePlanExec::AckPlanCmdFailed(uint8_t completed_status,
                                       std::string const& message) {
  // Call ack command with the run plan command id so the run plan command id
  // gets acked
  AckCmd(run_plan_cmd_id_, completed_status, message);
  run_plan_cmd_id_ = "";
  exec_->SetPlanExecState(ff_msgs::ExecState::PAUSED);
  exec_->PublishPlanStatus(ff_msgs::AckStatus::REQUEUED);
}

bool OpStatePlanExec::PausePlan(ff_msgs::CommandStampedPtr const& cmd) {
  // Check to see if the command came from the plan. If it did, we don't need
  // to cancel actions since plans are sequential
  if (cmd->cmd_id == "plan") {
    exec_->PublishCmdAck(run_plan_cmd_id_);
    // Clear command id since we acked it and we don't want to ack it again
    run_plan_cmd_id_ = "";

    // Ack the pause as completed in the plan
    exec_->AckCurrentPlanItem();
    // Publish plan status with the next item queued
    exec_->PublishPlanStatus(ff_msgs::AckStatus::QUEUED);

    exec_->SetPlanExecState(ff_msgs::ExecState::PAUSED);
  } else {
    exec_->PublishCmdAck(run_plan_cmd_id_,
                         ff_msgs::AckCompletedStatus::CANCELED);
    // Clear command id since we acked it and we don't want to ack it again
    run_plan_cmd_id_ = "";

    exec_->SetPlanExecState(ff_msgs::ExecState::PAUSED);
    if (exec_->AreActionsRunning()) {
      // TODO(Katie) Cancel instead of requeueing if the pause came in while
      // executing a segment
      if (exec_->IsActionRunning(EXECUTE)) {
        exec_->PublishPlanStatus(ff_msgs::AckStatus::REQUEUED);
      } else {
        exec_->PublishPlanStatus(ff_msgs::AckStatus::REQUEUED);
      }

      exec_->StopAllMotion(cmd);
    } else if (waiting_) {
      // If paused or stopped during a wait, the wait is stopped and marked as
      // complete. The next item in the plan will be executed when the run plan
      // command is received.
      exec_->StopWaitTimer();
      waiting_ = false;
      exec_->AckCurrentPlanItem();
      exec_->PublishPlanStatus(ff_msgs::AckStatus::QUEUED);
      exec_->PublishCmdAck(cmd->cmd_id);
    } else {
      AckCmd(cmd->cmd_id,
             ff_msgs::AckCompletedStatus::EXEC_FAILED,
             "Executive: Don't know how to pause command being executed!");
      return false;
    }
  }

  return true;
}

OpState* OpStatePlanExec::HandleActionComplete(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              Action const& action,
                              std::string const& result) {
  if (state == ff_util::FreeFlyerActionState::Enum::SUCCESS) {
    return AckStartPlanItem();
  }

  std::string err_msg = GenerateActionFailedMsg(state, action, result);

  AckCmd(run_plan_cmd_id_, ff_msgs::AckCompletedStatus::EXEC_FAILED, err_msg);

  // Start a stop action since we don't know what the mobility state
  // TODO(Katie) Possibly store gnc state and only start a stop when flying
  exec_->FillMotionGoal(STOP);
  exec_->StartAction(STOP, "internal");

  run_plan_cmd_id_ = "";
  exec_->SetPlanExecState(ff_msgs::ExecState::PAUSED);
  exec_->PublishPlanStatus(ff_msgs::AckStatus::REQUEUED);
  return OpStateRepo::Instance()->teleop()->StartupState();
}

OpState* OpStatePlanExec::AckStartPlanItem() {
  // Returns true if there are more commands/segments in the plan
  if (exec_->AckCurrentPlanItem()) {
    ROS_DEBUG("Starting next item in plan!");
    exec_->PublishPlanStatus(ff_msgs::AckStatus::EXECUTING);
    return StartNextPlanItem();
  } else {
    ROS_INFO("Plan complete!");
    AckCmd(run_plan_cmd_id_);
    run_plan_cmd_id_ = "";
    exec_->PublishPlanStatus(ff_msgs::AckStatus::COMPLETED);
    exec_->SetPlanExecState(ff_msgs::ExecState::IDLE);
    return OpStateRepo::Instance()->ready()->StartupState();
  }
  return this;
}

OpState* OpStatePlanExec::StartNextPlanItem() {
  sequencer::ItemType it = exec_->GetCurrentPlanItemType();
  std::string err_msg;

  if (it == sequencer::ItemType::SEGMENT) {
    ROS_DEBUG("Got and sending segment.");
    exec_->FillMotionGoal(EXECUTE);

    // TODO(Katie) Temporarily force holonomic mode on the initial segment, as
    // the granite table doesn't permit a faceforward move to start because
    // there are no visual features near spheresgoat
    // TODO(Katie) Change this to extract facefoward out of plan
    if (!exec_->ConfigureMobility(first_segment_, true, err_msg)) {
      AckPlanCmdFailed(ff_msgs::AckCompletedStatus::EXEC_FAILED, err_msg);
      return OpStateRepo::Instance()->ready()->StartupState();
    }

    if (!exec_->StartAction(EXECUTE, "plan")) {
      AckPlanCmdFailed(ff_msgs::AckCompletedStatus::EXEC_FAILED, err_msg);
      return OpStateRepo::Instance()->ready()->StartupState();
    }

    if (first_segment_) {
      first_segment_ = false;
    }
  } else if (it == sequencer::ItemType::COMMAND) {
    ROS_DEBUG("Executing next command.");
    return HandleCmd(exec_->GetPlanCommand());
  } else {
    // Plan is empty so it must have completed successfully
    // This covers the crazy case of a paused plan where the wait command was
    // the part of the plan that got paused and it was the last item in the
    // plan.
    ROS_INFO("Plan complete!!!");
    AckCmd(run_plan_cmd_id_);
    run_plan_cmd_id_ = "";
    exec_->SetPlanExecState(ff_msgs::ExecState::IDLE);
    return OpStateRepo::Instance()->ready()->StartupState();
  }
  return this;
}
}  // namespace executive
