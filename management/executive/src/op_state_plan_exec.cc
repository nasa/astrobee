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
  // operating state will be returned.
  OpState* temp_op_state = StartNextPlanItem();
  if (temp_op_state != this) {
    // If the first item wasn't successful, pause plan
    exec_->SetPlanExecState(ff_msgs::ExecState::PAUSED);
    exec_->PublishPlanStatus(ff_msgs::AckStatus::QUEUED);
  }
  return temp_op_state;
}

OpState* OpStatePlanExec::HandleCmd(ff_msgs::CommandStampedPtr const& cmd) {
  std::string err_msg;
  bool completed = false, successful = false;
  uint8_t status;

  // TODO(Katie) Add more commands
  if (cmd->cmd_id == "plan" && cmd->cmd_src == "plan") {
    OpState::HandleCmd(cmd, completed, successful, err_msg, status, true);
    if (completed) {
      return HandleCommandComplete(successful, err_msg, status);
    }

    // Don't have to worry about preempting an action since an action has to
    // complete before getting the next item in the plan
    if (cmd->cmd_name == CommandConstants::CMD_NAME_ARM_PAN_AND_TILT ||
        cmd->cmd_name == CommandConstants::CMD_NAME_GRIPPER_CONTROL) {
      ROS_INFO("Executing pan and tilt or gripper control command.");
      if (!exec_->FillArmGoal(cmd, err_msg)) {
        return HandleCommandComplete(false,
                                     err_msg,
                                     ff_msgs::AckCompletedStatus::BAD_SYNTAX);
      }

      if (!exec_->StartAction(ARM, cmd->cmd_id, err_msg)) {
        return HandleCommandComplete(false,
                                     err_msg,
                                     ff_msgs::AckCompletedStatus::EXEC_FAILED);
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_CLEAR_DATA) {
      ROS_INFO("Executing clear data command.");
      successful = exec_->ClearData(cmd, err_msg, status, true);
      return HandleCommandComplete(successful, err_msg, status);
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_DOCK) {
      ROS_INFO("Executing dock command.");
      if (!exec_->Dock(cmd, err_msg, status, true)) {
        return HandleCommandComplete(successful, err_msg, status);
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_DOWNLOAD_DATA) {
      ROS_INFO("Executing download data command.");
      successful = exec_->DownloadData(cmd, err_msg, status, true);
      return HandleCommandComplete(successful, err_msg, status);
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_IDLE_PROPULSION) {
      ROS_INFO("Executing idle propulsion command.");
      if (!exec_->FillMotionGoal(IDLE)) {
        return HandleCommandComplete(false,
                                    "Fill motion goal function failed!",
                                    ff_msgs::AckCompletedStatus::EXEC_FAILED);
      }

      if (!exec_->StartAction(IDLE, cmd->cmd_id, err_msg)) {
        return HandleCommandComplete(false,
                                     err_msg,
                                     ff_msgs::AckCompletedStatus::EXEC_FAILED);
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_PAUSE_PLAN) {
      exec_->PublishCmdAck(run_plan_cmd_id_);
      // Clear command id since we acked it and we don't want to ack it again
      run_plan_cmd_id_ = "";

      // Ack the pause as completed in the plan
      exec_->AckCurrentPlanItem();
      // Publish plan status with the next item queued
      exec_->PublishPlanStatus(ff_msgs::AckStatus::QUEUED);

      exec_->SetPlanExecState(ff_msgs::ExecState::PAUSED);

      return OpStateRepo::Instance()->ready()->StartupState();
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_PERCH) {
      ROS_INFO("Executing perch command.");
      // TODO(Katie) Stub, change to be actual code.
      if (exec_->GetMobilityState().state == ff_msgs::MobilityState::STOPPING &&
          exec_->GetMobilityState().sub_state == 0) {
        exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, 4);
        ros::Duration(1).sleep();
        exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, 3);
        ros::Duration(1).sleep();
        exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, 2);
        ros::Duration(1).sleep();
        exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, 1);
        ros::Duration(1).sleep();
        exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, 0);
        return HandleCommandComplete(true, "", ff_msgs::AckCompletedStatus::OK);
      } else {
        err_msg = "Must be stopped before attempting to perch!";
        ROS_ERROR("Executive: %s", err_msg.c_str());
        return HandleCommandComplete(false,
                                     err_msg,
                                     ff_msgs::AckCompletedStatus::EXEC_FAILED);
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_OFF_ITEM) {
      ROS_INFO("Executing power off command.");
      successful = exec_->PowerOffItem(cmd, err_msg, status, true);
      return HandleCommandComplete(successful, err_msg, status);
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_POWER_ON_ITEM) {
      ROS_INFO("Execute power on command.");
      successful = exec_->PowerOnItem(cmd, err_msg, status, true);
      return HandleCommandComplete(successful, err_msg, status);
    } else if (cmd->cmd_name ==
                        CommandConstants::CMD_NAME_SET_FLASHLIGHT_BRIGHTNESS) {
      ROS_INFO("Executing set flashlight brightness command.");
      successful = exec_->SetFlashlightBrightness(cmd, err_msg, status, true);
      return HandleCommandComplete(successful, err_msg, status);
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_UNDOCK) {
      ROS_INFO("Executing undock command!");
      if (!exec_->Undock(cmd, err_msg, true)) {
        return HandleCommandComplete(false, err_msg,
                                     ff_msgs::AckCompletedStatus::EXEC_FAILED);
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_UNPERCH) {
      ROS_INFO("Executing unperch command.");
      // TODO(Katie) Stub, change to be actual code.
      if (exec_->GetMobilityState().state == ff_msgs::MobilityState::PERCHING &&
          exec_->GetMobilityState().sub_state == 0) {
        exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, -1);
        ros::Duration(1).sleep();
        exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, -2);
        ros::Duration(1).sleep();
        exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, -3);
        ros::Duration(1).sleep();
        exec_->SetMobilityState(ff_msgs::MobilityState::PERCHING, -4);
        ros::Duration(1).sleep();
        exec_->SetMobilityState(ff_msgs::MobilityState::STOPPING, 0);
        return HandleCommandComplete(true, "", ff_msgs::AckCompletedStatus::OK);
      } else {
        err_msg = "Must be perched to unperch!";
        ROS_ERROR("Executive: %s", err_msg.c_str());
        return HandleCommandComplete(false,
                                     err_msg,
                                     ff_msgs::AckCompletedStatus::EXEC_FAILED);
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_WAIT) {
      ROS_INFO("Executive got wait command! duration: %f", cmd->args[0].f);
      if (cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT ||
          cmd->args[0].f < 0) {
        return HandleCommandComplete(false,
                                     "Malformed arguments for wait command!",
                                      ff_msgs::AckCompletedStatus::BAD_SYNTAX);
      }
      exec_->StartWaitTimer(cmd->args[0].f);
      waiting_ = true;
    } else if (cmd->cmd_name ==
                              CommandConstants::CMD_NAME_START_GUEST_SCIENCE) {
      // TODO(Katie) Need to do some sort of check for primary vs. secondary
      // since we will have to check for which commands to accept if primary
      if (!exec_->SendGuestScienceCommand(cmd, err_msg, status, true)) {
        return HandleCommandComplete(false, err_msg, status);
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_GUEST_SCIENCE) {
      // TODO(Katie) Need to do some sort of check for primary vs. secondary
      // since we will have to check for which commands to accept if primary
      if (!exec_->SendGuestScienceCommand(cmd, err_msg, status, true)) {
        return HandleCommandComplete(false, err_msg, status);
      }
    } else if (cmd->cmd_name ==
                              CommandConstants::CMD_NAME_CUSTOM_GUEST_SCIENCE) {
      // TODO(Katie) Need to do some sort of check for primary vs. secondary
      // since we will have to check for which commands to accept if primary
      if (!exec_->SendGuestScienceCommand(cmd, err_msg, status, true)) {
        return HandleCommandComplete(false, err_msg, status);
      }
    } else {
      err_msg = "Plan contains unknown command: " + cmd->cmd_name;
      ROS_ERROR("%s", err_msg.c_str());
      return HandleCommandComplete(false, err_msg,
                                   ff_msgs::AckCompletedStatus::BAD_SYNTAX);
    }
  } else {
    // TODO(Katie) Possibly check to see if the robot is executing a waiting
    // command
    // TODO(Katie) For pause, actions will be pausable but other commands will
    // not. It is TBD what happens for a wait command
    // TODO(Katie) Fill this out
    OpState::HandleCmd(cmd, completed, successful, err_msg, status);
    if (completed) {
      return this;
    }

    // Pause and stop commands do the same thing in plan execution mode
    if (cmd->cmd_name == CommandConstants::CMD_NAME_PAUSE_PLAN ||
        cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ALL_MOTION) {
      // Assumes run plan command isn't a fault response
      exec_->PublishCmdAck(run_plan_cmd_id_,
                           ff_msgs::AckCompletedStatus::CANCELED);
      // Clear command id since we acked it and don't want to ack it again
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
        bool stop_started = false;
        if (exec_->StopAllMotion(stop_started,
                                 cmd->cmd_id,
                                 cmd->cmd_src,
                                 true)) {
          // If stop was successful and a stop was started, we need to go into
          // teleop op state
          if (stop_started) {
            return OpStateRepo::Instance()->teleop()->StartupState();
          } else {
            // An action that wasn't moving Astrobee was stopped so Astrobee is
            // already stopped and we can go into ready op state
            return OpStateRepo::Instance()->ready()->StartupState();
          }
        } else {
          // Stop all motion will only fail if we are docked or perched or we
          // failed to start the stop action. In any case, we need to transition
          // to the ready op state
          return OpStateRepo::Instance()->ready()->StartupState();
        }
      } else if (waiting_) {
        // If paused during a wait, the wait is stopped and marked as complete.
        // The next item in the plan will be executed when the run plan command
        // is received
        exec_->StopWaitTimer();
        waiting_ = false;
        exec_->AckCurrentPlanItem();
        exec_->PublishPlanStatus(ff_msgs::AckStatus::QUEUED);
        exec_->PublishCmdAck(cmd->cmd_id);
        return OpStateRepo::Instance()->ready()->StartupState();
      } else {
        ROS_ERROR("Executive: Don't know how to pause command being executed!");
        return OpStateRepo::Instance()->ready()->StartupState();
      }
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_FAULT) {
      // Need to pass the run plan command id and origin to fault state so that
      // when the current step completes, the fault state can ack the run plan
      // command as canceled
      std::string cmd_id = run_plan_cmd_id_;

      // Check if we are executing a wait command. If we are, the wait is
      // stopped and marked as complete.
      if (waiting_) {
        exec_->StopWaitTimer();
        waiting_ = false;
        exec_->AckCurrentPlanItem();
        exec_->PublishPlanStatus(ff_msgs::AckStatus::QUEUED);
        // Ack run plan command here since the current step has completed
        exec_->PublishCmdAck(run_plan_cmd_id_,
                             ff_msgs::AckCompletedStatus::CANCELED,
                             "Executive had to execute the fault command.");

        // Clear out the cmd_id since we just acked the run plan
        // command and will not need to ack it in the fault state
        cmd_id = "";
      }

      // Clear out the run plan command id and origin
      run_plan_cmd_id_ = "";

      // Ack fault command
      exec_->PublishCmdAck(cmd->cmd_id);
      return OpStateRepo::Instance()->fault()->StartupState(cmd_id);
    } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ARM) {
      // TODO(Katie) Stub, add actual code later
      exec_->StopArm(cmd->cmd_id);
    } else if (cmd->cmd_name ==
                              CommandConstants::CMD_NAME_CUSTOM_GUEST_SCIENCE) {
      // TODO(Katie) May need to add code to track command to make sure it gets
      // acked
      exec_->SendGuestScienceCommand(cmd, err_msg, status);
    } else {
      err_msg = "Command " + cmd->cmd_name + "not accepted in op state"
          + " plan execution.";
      // Don't stop plan, just send a failed ack
      exec_->PublishCmdAck(cmd->cmd_id,
                           ff_msgs::AckCompletedStatus::EXEC_FAILED,
                           err_msg);
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
    ROS_ERROR("Executive: %s", ack->message.c_str());
    exec_->PublishCmdAck(run_plan_cmd_id_,
                         ack->completed_status.status,
                         ack->message, ack->status.status);
    run_plan_cmd_id_ = "";
    exec_->SetPlanExecState(ff_msgs::ExecState::PAUSED);
    exec_->PublishPlanStatus(ff_msgs::AckStatus::REQUEUED);
    return OpStateRepo::Instance()->ready()->StartupState();
  } else {
    return AckStartPlanItem();
  }
  return this;
}

OpState* OpStatePlanExec::HandleCommandComplete(bool successful,
                                          std::string const& err_msg,
                                          uint8_t status) {
  if (successful) {
    ROS_DEBUG("Executive: command completed successfully.");
    return AckStartPlanItem();
  } else {
    ROS_ERROR("Executive: %s", err_msg.c_str());
    exec_->PublishCmdAck(run_plan_cmd_id_,
                         status,
                         err_msg);
    run_plan_cmd_id_ = "";
    exec_->SetPlanExecState(ff_msgs::ExecState::PAUSED);
    exec_->PublishPlanStatus(ff_msgs::AckStatus::REQUEUED);
    return OpStateRepo::Instance()->ready()->StartupState();
  }
  return this;
}

OpState* OpStatePlanExec::HandleActionComplete(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              Action const& action,
                              std::string const& result) {
  if (state == ff_util::FreeFlyerActionState::Enum::SUCCESS) {
    return AckStartPlanItem();
  }

  std::string err_msg = GenerateActionFailedMsg(state, action, result);
  ROS_ERROR("Executive: %s", err_msg.c_str());

  exec_->PublishCmdAck(run_plan_cmd_id_,
                       ff_msgs::AckCompletedStatus::EXEC_FAILED,
                       err_msg);

  // Start a stop action since we don't know what the mobility state
  // TODO(Katie) Possibly store gnc state and only start a stop when flying
  exec_->FillMotionGoal(STOP);
  exec_->StartAction(STOP, "plan", err_msg, true);

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
    ROS_DEBUG("Plan complete!");
    exec_->PublishCmdAck(run_plan_cmd_id_);
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
      return HandleCommandComplete(false, err_msg,
                                   ff_msgs::AckCompletedStatus::EXEC_FAILED);
    }

    if (!exec_->StartAction(EXECUTE, "plan", err_msg, true)) {
      return HandleCommandComplete(false, err_msg,
                                   ff_msgs::AckCompletedStatus::EXEC_FAILED);
    }

    if (first_segment_) {
      first_segment_ = false;
    }
  } else if (it == sequencer::ItemType::COMMAND) {
    return HandleCmd(exec_->GetPlanCommand());
  } else {
    // Plan is empty so it must have completed successfully
    // This covers the crazy case of a paused plan where the wait command was
    // the part of the plan that got paused and it was the last item in the
    // plan.
    ROS_INFO("Plan complete!!!");
    exec_->PublishCmdAck(run_plan_cmd_id_);
    run_plan_cmd_id_ = "";
    exec_->SetPlanExecState(ff_msgs::ExecState::IDLE);
    return OpStateRepo::Instance()->ready()->StartupState();
  }
  return this;
}
}  // namespace executive
