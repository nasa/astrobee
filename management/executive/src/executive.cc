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

#include <jsonloader/planio.h>

#include "executive/executive.h"
#include "executive/op_state.h"
#include "executive/op_state_repo.h"

namespace executive {

Executive::Executive() :
  ff_util::FreeFlyerNodelet(NODE_EXECUTIVE, true),
  state_(OpStateRepo::Instance()->ready()),
  sys_monitor_init_fault_response_(new ff_msgs::CommandStamped()),
  sys_monitor_heartbeat_fault_response_(new ff_msgs::CommandStamped()),
  dock_state_(NULL),
  fault_state_(NULL),
  guest_science_config_(NULL),
  motion_state_(NULL),
  perch_state_(NULL),
  current_inertia_(NULL),
  primary_apk_running_("None"),
  run_plan_cmd_id_(""),
  gs_start_stop_cmd_id_(""),
  gs_custom_cmd_id_(""),
  action_active_timeout_(1),
  gs_command_timeout_(4),
  arm_feedback_timeout_(4),
  motion_feedback_timeout_(1),
  dock_result_timeout_(360),
  perch_result_timeout_(360),
  localization_result_timeout_(30),
  led_connected_timeout_(10),
  pub_queue_size_(10),
  sub_queue_size_(10),
  live_led_on_(false),
  sys_monitor_heartbeat_fault_occurring_(false),
  sys_monitor_init_fault_occurring_(false) {
}

Executive::~Executive() {
}

/************************ Message and timeout callbacks ***********************/
void Executive::CameraStatesCallback(ff_msgs::CameraStatesStampedConstPtr const&
                                                                        state) {
  unsigned int i, j;
  bool streaming = false;
  ff_hw_msgs::ConfigureSystemLeds led_srv;

  // State array is only one array and the camera states array is at most 5
  // elements so this doesn't waste too much time
  // Don't care about cameras not in the camera states array
  for (i = 0; i < state->states.size(); i++) {
    for (j = 0; j < camera_states_.states.size(); j++) {
      if (state->states[i].camera_name ==
                                        camera_states_.states[j].camera_name) {
        camera_states_.states[j].streaming = state->states[i].streaming;
      }
    }
  }

  // The state message usually only contains one camera so we have to go through
  // all the camera states to see if any are streaming
  for (i = 0; i < camera_states_.states.size(); i++) {
    streaming |= camera_states_.states[i].streaming;
  }

  if (streaming && !live_led_on_) {
    led_srv.request.live = ff_hw_msgs::ConfigureSystemLeds::Request::ON;
    if (ConfigureLed(led_srv)) {
      live_led_on_ = true;
    }
  } else if (!streaming && live_led_on_) {
    led_srv.request.live = ff_hw_msgs::ConfigureSystemLeds::Request::OFF;
    if (ConfigureLed(led_srv)) {
      live_led_on_ = false;
    }
  }
}

void Executive::CmdCallback(ff_msgs::CommandStampedPtr const& cmd) {
  // Check to see if the command came from a guest science apk. If it did,
  // make sure a primary apk is running
  if (cmd->cmd_origin == "guest_science") {
    // TODO(Katie) Change this when the astrobee api is changed to set cmd
    // origin to the apk name. Need to make sure it matches the primary apk
    // running
    if (primary_apk_running_ == "None") {
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::EXEC_FAILED,
                     "Can't run a gs command when a primary apk not running.");
      return;
    }
  }

  // TODO(Katie) add more checks

  SetOpState(state_->HandleCmd(cmd));
}

void Executive::DataToDiskCallback(ff_msgs::CompressedFileConstPtr const&
                                                                        data) {
  data_to_disk_ = data;

  cf_ack_.header.stamp = ros::Time::now();
  cf_ack_.id = data_to_disk_->id;
  cf_ack_pub_.publish(cf_ack_);
}

void Executive::DockStateCallback(ff_msgs::DockStateConstPtr const& state) {
  dock_state_ = state;

  // Check to see if the dock state is docking/docked/undocking. If it is, we
  // can change the mobility state to docking/docked.
  // Docking max state signifies that we are docking
  if (dock_state_->state <= ff_msgs::DockState::DOCKING_MAX_STATE &&
      dock_state_->state > ff_msgs::DockState::UNDOCKED) {
    SetMobilityState(ff_msgs::MobilityState::DOCKING, dock_state_->state);
  }

  // If the dock state is undocked, the mobility state needs to be set to
  // whatever the motion state is.
  if (dock_state_->state == ff_msgs::DockState::UNDOCKED) {
    SetMobilityState();
  }
}

void Executive::FaultStateCallback(ff_msgs::FaultStateConstPtr const& state) {
  ff_hw_msgs::ConfigureSystemLeds led_srv;
  fault_state_ = state;

  // Check if we are in the fault state
  if (state_->id() == ff_msgs::OpState::FAULT) {
    // Check if the blocked fault is cleared
    if (state->state != ff_msgs::FaultState::BLOCKED) {
      // Turn a2 led off so the astronauts can see there is no longer a fault
      led_srv.request.status_a2 = ff_hw_msgs::ConfigureSystemLeds::Request::OFF;
      ConfigureLed(led_srv);

      // Check if an action is in progress, if so transition to teleop
      // Otherwise transition to ready
      if (AreActionsRunning()) {
        SetOpState(state_->TransitionToState(ff_msgs::OpState::TELEOPERATION));
      } else {
        SetOpState(state_->TransitionToState(ff_msgs::OpState::READY));
      }
    }
  } else {
    // Check if a blocking fault is occurring
    if (state->state == ff_msgs::FaultState::BLOCKED) {
      // Turn a2 led on and blinking so the astronauts can see there is a fault
      led_srv.request.status_a2 =
                                ff_hw_msgs::ConfigureSystemLeds::Request::FAST;
      ConfigureLed(led_srv);

      // If so, transiton to fault state
      SetOpState(state_->TransitionToState(ff_msgs::OpState::FAULT));
    }
  }
}

void Executive::GuestScienceAckCallback(ff_msgs::AckStampedConstPtr const&
                                                                          ack) {
  if (ack->cmd_id == "plan") {
    SetOpState(state_->HandleGuestScienceAck(ack));
  } else {
    cmd_ack_pub_.publish(ack);
  }

  // Clear guest science command timers
  if (ack->cmd_id == gs_start_stop_cmd_id_) {
    gs_start_stop_command_timer_.stop();
    gs_start_stop_cmd_id_ = "";
  } else if (ack->cmd_id == gs_custom_cmd_id_) {
    gs_custom_command_timer_.stop();
    gs_custom_cmd_id_ = "";
  }
}

void Executive::GuestScienceConfigCallback(ff_msgs::GuestScienceConfigConstPtr
                                                                const& config) {
  guest_science_config_ = config;
}

void Executive::GuestScienceStateCallback(ff_msgs::GuestScienceStateConstPtr
                                                                const& state) {
  unsigned int i = 0;
  std::string primary_apk = "None";

  // This should only happen on start up if we receive the state before the
  // config. In this case, no gs apks should be running so this should be okay
  if (guest_science_config_ == NULL) {
    return;
  }

  // Check that the state and config serial values are the same. If not, we
  // cannot compare the config and state and we must wait until they match
  // Currently, this shouldn't happen since we only poll for the current apks on
  // startup.
  if (guest_science_config_->serial != state->serial) {
    NODELET_WARN("Guest science state and config serial doesn't match.");
    return;
  }

  // Also check that the state and config are the same size because it would be
  // really bad if they weren't.
  if (state->runningApks.size() != guest_science_config_->apks.size()) {
    NODELET_ERROR("Guest science apk array size doesn't match but serial does");
    return;
  }

  // Check to see if any apks are running
  for (i = 0; i < state->runningApks.size(); i++) {
    if (state->runningApks[i]) {
      // Check if primary
      if (guest_science_config_->apks[i].primary) {
        if (primary_apk == "None") {
          primary_apk = guest_science_config_->apks[i].apk_name;
        } else {
          NODELET_ERROR("More than 1 primary apk running in gs state.");
        }
      }
    }
  }

  primary_apk_running_ = primary_apk;

  if (primary_apk_running_ != "None") {
    agent_state_.guest_science_state.state = ff_msgs::ExecState::EXECUTING;
  } else {
    agent_state_.guest_science_state.state = ff_msgs::ExecState::IDLE;
  }
}

void Executive::GuestScienceCustomCmdTimeoutCallback(
                                                    ros::TimerEvent const& te) {
  std::string err_msg = "GS manager didn't return an ack for the custom guest ";
  err_msg += "science command in the timeout specified. The GS manager may not";
  err_msg += " have started or it may have died.";
  PublishCmdAck(gs_custom_cmd_id_,
                ff_msgs::AckCompletedStatus::EXEC_FAILED,
                err_msg);
  // Don't need to stop timer because it is a one shot timer
  gs_custom_cmd_id_ = "";
}

void Executive::GuestScienceStartStopCmdTimeoutCallback(
                                                    ros::TimerEvent const& te) {
  std::string err_msg = "GS manager didn't return an ack for the start/stop ";
  err_msg += "guest science command in the timeout specified. The GS manager ";
  err_msg += "may not have started or it may have died.";
  PublishCmdAck(gs_start_stop_cmd_id_,
                ff_msgs::AckCompletedStatus::EXEC_FAILED,
                err_msg);
  // Don't need to stop timer because it is a one shot timer
  gs_start_stop_cmd_id_ = "";
}


void Executive::InertiaCallback(
                        geometry_msgs::InertiaStampedConstPtr const& inertia) {
  current_inertia_ = inertia;
}

void Executive::LedConnectedCallback() {
  ff_hw_msgs::ConfigureSystemLeds led_srv;

  // Set video light since this means the fsw started
  led_srv.request.video = ff_hw_msgs::ConfigureSystemLeds::Request::ON;

  // Check if we are in the fault state
  if (state_->id() == ff_msgs::OpState::FAULT) {
    // If so, turn a2 led on and blinking so astronauts can see there is a fault
    led_srv.request.status_a2 = ff_hw_msgs::ConfigureSystemLeds::Request::FAST;
    ConfigureLed(led_srv);
  } else {
    // Turn a2 led off to signify that the executive is ready.
    led_srv.request.status_a2 = ff_hw_msgs::ConfigureSystemLeds::Request::OFF;
    ConfigureLed(led_srv);
  }
}

void Executive::MotionStateCallback(ff_msgs::MotionStatePtr const& state) {
  if (motion_state_ == NULL) {
    motion_state_ = state;
  }

  // Check the current motion state to see if it maps to our mobility state. If
  // it does, set the mobility state and our stored motion state.
  if (state->state == ff_msgs::MotionState::INITIALIZING) {
    // Set motion state to idle a.k.a mobility state to drifting when the
    // mobility subsystem is initializing
    motion_state_->state = ff_msgs::MotionState::IDLE;
  } else if (state->state == ff_msgs::MotionState::IDLE ||
             state->state == ff_msgs::MotionState::IDLING ||
             state->state == ff_msgs::MotionState::STOPPED ||
             state->state == ff_msgs::MotionState::STOPPING ||
             state->state == ff_msgs::MotionState::CONTROLLING ||
             state->state == ff_msgs::MotionState::BOOTSTRAPPING) {
    motion_state_->state = state->state;
  }

  // Check to see if we are docking or undocking. If we are, don't use the
  // motion state as our mobility state.
  if (dock_state_ != NULL) {
    if (dock_state_->state > ff_msgs::DockState::UNDOCKED) {
      // If dock state is unknown or initializing, use motion state to set
      // mobility state
      if (dock_state_->state != ff_msgs::DockState::UNKNOWN &&
          dock_state_->state != ff_msgs::DockState::INITIALIZING) {
        return;
      }
    }
  }

  // Check to see if we are perching or unperching. If we are, don't use the
  // motion state as our mobility state.
  if (perch_state_ != NULL) {
    if (perch_state_->state > ff_msgs::PerchState::UNPERCHED) {
      // If perching state is unknown or initializing, use motion state to set
      // mobility state
      if (perch_state_->state != ff_msgs::PerchState::UNKNOWN &&
          perch_state_->state != ff_msgs::PerchState::INITIALIZING) {
        return;
      }
    }
  }

  // Set mobility state
  SetMobilityState();
}

void Executive::PerchStateCallback(ff_msgs::PerchStateConstPtr const& state) {
  perch_state_ = state;

  // Check to see if the perch state is perching/perched/unperching. If it is,
  // we can change the mobility state to perching/perched.
  // Perching max state signifies that we are perching
  if (perch_state_->state <= ff_msgs::PerchState::PERCHING_MAX_STATE &&
      perch_state_->state > ff_msgs::PerchState::UNPERCHED) {
    SetMobilityState(ff_msgs::MobilityState::PERCHING, perch_state_->state);
  }

  // Check to see if we are docked. If we are, don't use the motion state
  // as our mobility state.
  // This prevents the robot from sometimes going into drifting state if we are
  // already docked when FSW starts.
  // More docking and undocking states might be added to the check if needed
  if (dock_state_ != NULL) {
    if (dock_state_->state == ff_msgs::DockState::DOCKED) {
      return;
    }
  }

  // If the perch state is unperched, the mobility state needs to be set to
  // whatever the motion state is.
  if (perch_state_->state == ff_msgs::PerchState::UNPERCHED) {
    SetMobilityState();
  }
}

void Executive::PlanCallback(ff_msgs::CompressedFileConstPtr const& plan) {
  plan_ = plan;

  cf_ack_.header.stamp = ros::Time::now();
  cf_ack_.id = plan_->id;
  cf_ack_pub_.publish(cf_ack_);
}

void Executive::SysMonitorHeartbeatCallback(
                                  ff_msgs::HeartbeatConstPtr const& heartbeat) {
  sys_monitor_heartbeat_timer_.stop();

  // Stop the startup timer everytime since it isn't an expensive operation
  sys_monitor_startup_timer_.stop();

  // Check to see if the system monitor came back on line
  if (sys_monitor_heartbeat_fault_occurring_) {
    sys_monitor_heartbeat_fault_occurring_ = false;
    if (sys_monitor_heartbeat_fault_blocking_) {
      // Check fault state before transitioning to ready because we need to make
      // sure there aren't any other blocking faults occurring
      if (fault_state_ != NULL) {
        if (fault_state_->state != ff_msgs::FaultState::BLOCKED) {
          if (AreActionsRunning()) {
            SetOpState(state_->TransitionToState(
                                              ff_msgs::OpState::TELEOPERATION));
          } else {
            SetOpState(state_->TransitionToState(ff_msgs::OpState::READY));
          }
        }
      }
    }
  }

  // System monitor has only one fault and it is an initialization fault. Thus
  // if there is a fault in the fault array, the executive needs to trigger the
  // system monitor initialization fault.
  if (heartbeat->faults.size() > 0 && !sys_monitor_init_fault_occurring_) {
    NODELET_ERROR("System monitor initalization fault detected in executive.");
    sys_monitor_init_fault_occurring_ = true;
    sys_monitor_init_fault_response_->cmd_id = "executive" +
                                          std::to_string(ros::Time::now().sec);
    CmdCallback(sys_monitor_init_fault_response_);
    // If fault is blocking, transition to fault state
    if (sys_monitor_init_fault_blocking_) {
      SetOpState(state_->TransitionToState(ff_msgs::OpState::FAULT));
    }
    return;
  // Check initialiization fault went away
  } else if (heartbeat->faults.size() == 0 &&
                                            sys_monitor_init_fault_occurring_) {
    sys_monitor_init_fault_occurring_ = false;
    if (sys_monitor_init_fault_blocking_) {
      // Check fault state before transitioning to ready because we need to
      // make sure there aren't any other blocking faults occurring
      if (fault_state_ != NULL) {
        if (fault_state_->state != ff_msgs::FaultState::BLOCKED) {
          if (AreActionsRunning()) {
            SetOpState(state_->TransitionToState(
                                              ff_msgs::OpState::TELEOPERATION));
          } else {
            SetOpState(state_->TransitionToState(ff_msgs::OpState::READY));
          }
        }
      }
    }
  }

  sys_monitor_heartbeat_timer_.start();
}

void Executive::SysMonitorTimeoutCallback(ros::TimerEvent const& te) {
  NODELET_ERROR("System monitor heartbeat fault detected in executive.");
  // If the executive doesn't receive a heartbeat from the system monitor, it
  // needs to trigger the system monitor heartbeat missed fault.
  sys_monitor_heartbeat_fault_occurring_ = true;
  sys_monitor_heartbeat_fault_response_->cmd_id = "executive" +
                                          std::to_string(ros::Time::now().sec);
  CmdCallback(sys_monitor_heartbeat_fault_response_);
  // If fault is blocking, transition to fault state
  if (sys_monitor_heartbeat_fault_blocking_) {
    SetOpState(state_->TransitionToState(ff_msgs::OpState::FAULT));
  }
  sys_monitor_heartbeat_timer_.stop();
}

void Executive::WaitCallback(ros::TimerEvent const& te) {
  wait_timer_.stop();
  SetOpState(state_->HandleWaitCallback());
}

void Executive::ZonesCallback(ff_msgs::CompressedFileConstPtr const& zones) {
  zones_ = zones;

  cf_ack_.header.stamp = ros::Time::now();
  cf_ack_.id = zones_->id;
  cf_ack_pub_.publish(cf_ack_);
}

/************************ Action based commands *******************************/
bool Executive::AreActionsRunning() {
  if (running_actions_.size() > 0) {
    return true;
  }
  return false;
}

void Executive::CancelAction(Action action, std::string cmd) {
  std::string err_msg;
  // We don't receive a result if we cancel an action so we need to ack the
  // command id that started the action as canceled and remove the action
  // from the running actions vector.
  RemoveAction(action);
  switch (action) {
    case ARM:
      arm_ac_.CancelGoal();
      err_msg = "Arm action was canceled due to a " + cmd + " command.";
      state_->AckCmd(arm_ac_.cmd_id(),
                     ff_msgs::AckCompletedStatus::CANCELED,
                     err_msg);
      arm_ac_.SetCmdInfo(NONE, "");
      break;
    case DOCK:
    case UNDOCK:
      dock_ac_.CancelGoal();
      err_msg = "Dock action was canceled due to a " + cmd + " command.";
      state_->AckCmd(dock_ac_.cmd_id(),
                     ff_msgs::AckCompletedStatus::CANCELED,
                     err_msg);
      dock_ac_.SetCmdInfo(NONE, "");
      break;
    case EXECUTE:
    case MOVE:
    case STOP:
      motion_ac_.CancelGoal();
      err_msg = "Motion action was canceled due to a " + cmd + " command.";
      state_->AckCmd(motion_ac_.cmd_id(),
                     ff_msgs::AckCompletedStatus::CANCELED,
                     err_msg);
      motion_ac_.SetCmdInfo(NONE, "");
      break;
    case PERCH:
    case UNPERCH:
      perch_ac_.CancelGoal();
      err_msg = "Perch action was canceled due to a " + cmd + " command.";
      state_->AckCmd(perch_ac_.cmd_id(),
                     ff_msgs::AckCompletedStatus::CANCELED,
                     err_msg);
      perch_ac_.SetCmdInfo(NONE, "");
      break;
    default:
      NODELET_ERROR("Action to cancel not recognized!");
      return;
  }
}

// TODO(Katie) Add stow check
bool Executive::FillArmGoal(ff_msgs::CommandStampedPtr const& cmd) {
  bool successful = true;
  std::string err_msg;
  if (cmd->cmd_name == CommandConstants::CMD_NAME_ARM_PAN_AND_TILT) {
    // Pan and tilt command has 3 arguments: one is the pan value, one is the
    // tilt value, and the last is a string specifying whether to pan, tilt or
    // both
    if (cmd->args.size() != 3 ||
        cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT ||
        cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT ||
        cmd->args[2].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
      successful = false;
      err_msg = "Malformed arguments for pan and tilt command!";
    } else {
      // Don't check pan and tilt values since limits may change. The arm node
      // is responsible for checking these values.
      // Convert to radians
      arm_goal_.pan = cmd->args[0].f;
      arm_goal_.tilt = cmd->args[1].f;
      if (cmd->args[2].s == "Pan" || cmd->args[2].s == "pan") {
        arm_goal_.command = ff_msgs::ArmGoal::ARM_PAN;
      } else if (cmd->args[2].s == "Tilt" || cmd->args[2].s == "tilt") {
        arm_goal_.command = ff_msgs::ArmGoal::ARM_TILT;
      } else if (cmd->args[2].s == "Both" || cmd->args[2].s == "both") {
        arm_goal_.command = ff_msgs::ArmGoal::ARM_MOVE;
      } else {
        successful = false;
        err_msg = "Unrecognized which parameter in pan and tilt command. Got: "
          + cmd->args[2].s;
      }
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_GRIPPER_CONTROL) {
    // Gripper control has one argument which is a booleanused to specify
    // whether to open or close the arm
    if (cmd->args.size() != 1 ||
       cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
      successful = false;
      err_msg = "Malformed arguments for gripper control command!";
    } else {
      // True means open
      if (cmd->args[0].b) {
        arm_goal_.command = ff_msgs::ArmGoal::GRIPPER_OPEN;
      } else {
        arm_goal_.command = ff_msgs::ArmGoal::GRIPPER_CLOSE;
      }
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOW_ARM) {
    arm_goal_.command = ff_msgs::ArmGoal::ARM_STOW;
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_STOP_ARM) {
    arm_goal_.command = ff_msgs::ArmGoal::ARM_STOP;
  } else {
    successful = false;
    err_msg = "Arm command not recognized in fill arm goal.";
  }

  if (!successful) {
    NODELET_ERROR("%s", err_msg.c_str());
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                   err_msg);
  }
  return successful;
}

bool Executive::FillDockGoal(ff_msgs::CommandStampedPtr const& cmd,
                             bool return_to_dock) {
  bool successful = true;
  std::string err_msg;

  // Set return dock first as it is the same for dock and undock
  dock_goal_.return_dock = return_to_dock;

  if (cmd->cmd_name == CommandConstants::CMD_NAME_DOCK) {
    if (cmd->args.size() != 1 ||
        cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_INT) {
      successful = false;
      err_msg = "Malformed argument for dock command in plan.";
    }

    dock_goal_.command = ff_msgs::DockGoal::DOCK;
    if (cmd->args[0].i == 1) {
      dock_goal_.berth = ff_msgs::DockGoal::BERTH_1;
    } else if (cmd->args[0].i == 2) {
      dock_goal_.berth = ff_msgs::DockGoal::BERTH_2;
    } else {
      successful = false;
      err_msg = "Berth must be 1 or 2 not " +  std::to_string(cmd->args[0].i);
    }
  } else if (cmd->cmd_name == CommandConstants::CMD_NAME_UNDOCK) {
    dock_goal_.command = ff_msgs::DockGoal::UNDOCK;
    // We don't need a berth to undock
    dock_goal_.berth = ff_msgs::DockGoal::BERTH_UNKNOWN;
  } else {
    successful = false;
    err_msg = "Dock command not recognized in fill dock goal.";
  }

  if (!successful) {
    NODELET_ERROR("%s", err_msg.c_str());
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                   err_msg);
  }

  return successful;
}

bool Executive::FillMotionGoal(Action action,
                               ff_msgs::CommandStampedPtr const& cmd) {
  jsonloader::Segment segment;
  // Flight mode needs to be set for all motion actions
  motion_goal_.flight_mode = agent_state_.flight_mode;

  switch (action) {
    case EXECUTE:
      segment = sequencer_.CurrentSegment();

      motion_goal_.command = ff_msgs::MotionGoal::EXEC;
      // Convert JSON to a segment type
      motion_goal_.segment =
                    sequencer::Segment2Trajectory(sequencer_.CurrentSegment());

      motion_goal_.states.clear();
      break;
    case IDLE:
      // Need to set flight mode to off so the PMCs shutdown
      motion_goal_.flight_mode = ff_msgs::MotionGoal::OFF;
      motion_goal_.command = ff_msgs::MotionGoal::IDLE;
      break;
    case STOP:
      motion_goal_.command = ff_msgs::MotionGoal::STOP;
      break;
    case MOVE:
      if (cmd == nullptr) {
        NODELET_ERROR("Executive: move cmd is null in fill motion goal.");
        return false;
      }

      if (cmd->args.size() != 4 ||
          cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
          cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_VEC3d ||
          cmd->args[2].data_type != ff_msgs::CommandArg::DATA_TYPE_VEC3d ||
          cmd->args[3].data_type != ff_msgs::CommandArg::DATA_TYPE_MAT33f) {
        state_->AckCmd(cmd->cmd_id,
                       ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                       "Malformed arguments for simple move 6dof command!");
        return false;
      }

      motion_goal_.command = ff_msgs::MotionGoal::MOVE;
      if (motion_goal_.states.size() != 1) {
        motion_goal_.states.resize(1);
      }

      motion_goal_.states[0].header.stamp = ros::Time::now();

      // Copying the reference frame to the goal
      if (cmd->args[0].s == "ISS") {
        motion_goal_.reference_frame = FRAME_NAME_WORLD;
      } else {
        motion_goal_.reference_frame = cmd->args[0].s;
      }

      // Copying the target location to the goal
      motion_goal_.states[0].pose.position.x = cmd->args[1].vec3d[0];
      motion_goal_.states[0].pose.position.y = cmd->args[1].vec3d[1];
      motion_goal_.states[0].pose.position.z = cmd->args[1].vec3d[2];

      // Copying the target attitude to the goal
      motion_goal_.states[0].pose.orientation.x = cmd->args[3].mat33f[0];
      motion_goal_.states[0].pose.orientation.y = cmd->args[3].mat33f[1];
      motion_goal_.states[0].pose.orientation.z = cmd->args[3].mat33f[2];
      motion_goal_.states[0].pose.orientation.w = cmd->args[3].mat33f[3];

      motion_goal_.segment.clear();
      break;
    default:
      if (cmd != nullptr) {
        state_->AckCmd(cmd->cmd_id,
                       ff_msgs::AckCompletedStatus::EXEC_FAILED,
                       "Command isn't a mobility action in fill motion goal!");
      }
      return false;
  }
  return true;
}

bool Executive::IsActionRunning(Action action) {
  for (unsigned int i = 0; i < running_actions_.size(); i++) {
    if (action == running_actions_[i]) {
      return true;
    }
  }
  return false;
}

bool Executive::StartAction(Action action, std::string const& cmd_id) {
  std::string err_msg;
  bool successful = true;

  switch (action) {
    case ARM:
      if (arm_ac_.IsConnected()) {
        arm_ac_.SetCmdInfo(action, cmd_id);
        arm_ac_.SendGoal(arm_goal_);
        NODELET_DEBUG("Arm action goal sent/started.");
      } else {
        successful = false;
        err_msg = "Arm action server not connected! Arm node may have died!";
      }
      break;
    case DOCK:
    case UNDOCK:
      if (dock_ac_.IsConnected()) {
        dock_ac_.SetCmdInfo(action, cmd_id);
        dock_ac_.SendGoal(dock_goal_);
        NODELET_DEBUG("Dock action goal sent/started.");
      } else {
        successful = false;
        err_msg = "Dock action server not connected! Dock node may have died!";
      }
      break;
    case EXECUTE:
    case IDLE:
    case MOVE:
    case STOP:
      if (motion_ac_.IsConnected()) {
        motion_ac_.SetCmdInfo(action, cmd_id);
        motion_ac_.SendGoal(motion_goal_);
        NODELET_DEBUG("Motion action %i goal sent/started.",
                                                          motion_goal_.command);
      } else {
        successful = false;
        err_msg = "Motion action server not connected! Node may have died!";
      }
      break;
    case LOCALIZATION:
    case REACQUIRE:
      if (localization_ac_.IsConnected()) {
        localization_ac_.SetCmdInfo(action, cmd_id);
        localization_ac_.SendGoal(localization_goal_);
        NODELET_DEBUG("Localization action goal sent/started.");
      } else {
         successful = false;
         err_msg = "Localization action server not connected. Node may be dead";
      }
      break;
    case PERCH:
    case UNPERCH:
      if (perch_ac_.IsConnected()) {
        perch_ac_.SetCmdInfo(action, cmd_id);
        perch_ac_.SendGoal(perch_goal_);
        NODELET_DEBUG("Perch action goal sent/started.");
      } else {
        successful = false;
        err_msg = "Perch action server not connected. Node may have died!";
      }
      break;
    default:
      successful = false;
      err_msg = "Action to start not recognized!";
  }

  if (!successful) {
    state_->AckCmd(cmd_id, ff_msgs::AckCompletedStatus::EXEC_FAILED, err_msg);
  } else {
    state_->AckCmd(cmd_id,
                   ff_msgs::AckCompletedStatus::NOT,
                   "",
                   ff_msgs::AckStatus::EXECUTING);
  }

  if (successful) {
    // Add action to actions running so we know what actions are running
    running_actions_.push_back(action);
  }

  return successful;
}

bool Executive::RemoveAction(Action action) {
  unsigned int i = 0;

  bool found = false;
  for (i = 0; i < running_actions_.size(); i++) {
    if (action == running_actions_[i]) {
      found = true;
      break;
    }
  }

  if (found) {
    running_actions_.erase(running_actions_.begin() + i);
  } else {
    NODELET_ERROR_STREAM("Action " << action <<
                                            " not in running actions vector!");
  }
  return found;
}

/************************ Action callbacks ************************************/
void Executive::ArmResultCallback(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::ArmResultConstPtr const& result) {
  std::string response = "";
  Action current_action = arm_ac_.action();
  std::string cmd_id = arm_ac_.cmd_id();

  // Remove action from the running action vector
  RemoveAction(current_action);

  // Get response for op state functions
  if (result != nullptr) {
    response = result->fsm_result;
  }

  // In the case of a plan, handle result may start another arm action so we
  // need to clear the action info before starting another action.
  // Reset command id so we don't try to ack the same id twice
  arm_ac_.SetCmdInfo(NONE, "");

  SetOpState(state_->HandleResult(state,
                                  response,
                                  cmd_id,
                                  current_action));
}

void Executive::DockResultCallback(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::DockResultConstPtr const& result) {
  std::string response = "";
  Action current_action = dock_ac_.action();
  std::string cmd_id = dock_ac_.cmd_id();

  // Remove action from the running action vector
  RemoveAction(current_action);

  // Get response for op state functions
  if (result != nullptr) {
    response = result->fsm_result;
  }

  // In the case of a plan, handle result may start another dock action so we
  // need to clear the action info before starting another action.
  // Reset command id so we don't try to ack the same id twice
  dock_ac_.SetCmdInfo(NONE, "");

  SetOpState(state_->HandleResult(state,
                                  response,
                                  cmd_id,
                                  current_action));
}

void Executive::LocalizationResultCallback(
                            ff_util::FreeFlyerActionState::Enum const& state,
                            ff_msgs::LocalizationResultConstPtr const& result) {
  std::string response = "";
  Action current_action = localization_ac_.action();
  std::string cmd_id = localization_ac_.cmd_id();

  // Remove action from the running action vector
  RemoveAction(current_action);

  // Get response for op state functions
  if (result != nullptr) {
    response = result->fsm_result;
  }

  // In the case of a plan, handle result may start another localization action
  // so we need to clear the action info before starting another action
  // Reset command id so we don't try to ack the same id twice
  // Only reset command id if we didn't just start reset ekf which only happens
  // if we are executing a reacquire position command
  if (current_action != REACQUIRE) {
    localization_ac_.SetCmdInfo(NONE, "");
  }

  SetOpState(state_->HandleResult(state,
                                  response,
                                  cmd_id,
                                  current_action));
}

void Executive::MotionFeedbackCallback(
                              ff_msgs::MotionFeedbackConstPtr const& feedback) {
  // The only feedback used from the motion action is the execute feedback and
  // it goes to the sequencer. Otherwise there isn't much to with the feedback
  if (motion_ac_.action() == EXECUTE) {
    sequencer_.Feedback(feedback->progress);
  }
}

void Executive::MotionResultCallback(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::MotionResultConstPtr const& result) {
  std::string response = "";
  Action current_action = motion_ac_.action();
  std::string cmd_id = motion_ac_.cmd_id();

  // Remove action from the running action vector
  RemoveAction(current_action);

  // Get response for op state functions
  if (result != nullptr) {
    response = result->fsm_result;
  }

  // Sometimes the handle motion result starts an action so we need to clear the
  // action info before starting another action
  // Reset command id so we don't try to ack the same id twice
  motion_ac_.SetCmdInfo(NONE, "");

  SetOpState(state_->HandleResult(state,
                                  response,
                                  cmd_id,
                                  current_action));
}

void Executive::PerchResultCallback(
                              ff_util::FreeFlyerActionState::Enum const& state,
                              ff_msgs::PerchResultConstPtr const& result) {
  std::string response = "";
  Action current_action = perch_ac_.action();
  std::string cmd_id = perch_ac_.cmd_id();

  // Remove action from the running action vector
  RemoveAction(current_action);

  // Get response for op state functions
  if (result != nullptr) {
      response = result->fsm_result;
  }

  // In the case of a plan, handle result may start another perch action so we
  // need to clear the action info before starting another perch action.
  // Reset command id so we don't try to ack the same id twice
  perch_ac_.SetCmdInfo(NONE, "");

  SetOpState(state_->HandleResult(state,
                                  response,
                                  cmd_id,
                                  current_action));
}

/************************ Publishers ******************************************/
void Executive::PublishCmdAck(std::string const& cmd_id,
                              uint8_t completed_status,
                              std::string const& message,
                              uint8_t status) {
  // Output if the command failed
  if (completed_status != ff_msgs::AckCompletedStatus::OK &&
      completed_status != ff_msgs::AckCompletedStatus::NOT &&
      completed_status != ff_msgs::AckCompletedStatus::CANCELED) {
    NODELET_ERROR("Executive: Command failed with message: %s",
                                                              message.c_str());
  }
  ack_.header.stamp = ros::Time::now();
  ack_.cmd_id = cmd_id;
  ack_.status.status = status;
  ack_.completed_status.status = completed_status;
  ack_.message = message;
  cmd_ack_pub_.publish(ack_);
}

void Executive::PublishPlan() {
  plan_pub_.publish(plan_);
}

void Executive::PublishPlanStatus(uint8_t status) {
  // The sequencer sets the plan status to executing for every plan status but
  // since the executive has knowledge of if the plan has just started,
  // is paused, or is executing, the executive sets the status.
  ff_msgs::PlanStatusStamped plan_status = sequencer_.plan_status();
  plan_status.header.stamp = ros::Time::now();
  plan_status.status.status = status;
  plan_status_pub_.publish(plan_status);
}

/************************ Getters *********************************************/
ff_msgs::MobilityState Executive::GetMobilityState() {
  return agent_state_.mobility_state;
}

uint8_t Executive::GetPlanExecState() {
  return agent_state_.plan_execution_state.state;
}

std::string Executive::GetRunPlanCmdId() {
  return run_plan_cmd_id_;
}

/************************ Setters *********************************************/
// Set the mobility state based on the stored motion state
void Executive::SetMobilityState() {
  if (motion_state_ == NULL) {
    return;
  }

  if (motion_state_->state == ff_msgs::MotionState::IDLE) {
    agent_state_.mobility_state.state = ff_msgs::MobilityState::DRIFTING;
    agent_state_.mobility_state.sub_state = 0;
  } else if (motion_state_->state == ff_msgs::MotionState::IDLING) {
    agent_state_.mobility_state.state = ff_msgs::MobilityState::DRIFTING;
    agent_state_.mobility_state.sub_state = 1;
  } else if (motion_state_->state == ff_msgs::MotionState::STOPPED) {
    agent_state_.mobility_state.state = ff_msgs::MobilityState::STOPPING;
    agent_state_.mobility_state.sub_state = 0;
  } else if (motion_state_->state == ff_msgs::MotionState::STOPPING) {
    agent_state_.mobility_state.state = ff_msgs::MobilityState::STOPPING;
    agent_state_.mobility_state.sub_state = 1;
  } else if (motion_state_->state == ff_msgs::MotionState::CONTROLLING ||
             motion_state_->state == ff_msgs::MotionState::BOOTSTRAPPING) {
    agent_state_.mobility_state.state = ff_msgs::MobilityState::FLYING;
    agent_state_.mobility_state.sub_state = 0;
  }

  PublishAgentState();
}

void Executive::SetMobilityState(uint8_t state, uint32_t sub_state) {
  agent_state_.mobility_state.state = state;
  agent_state_.mobility_state.sub_state = sub_state;
  PublishAgentState();
}

void Executive::SetOpState(OpState* state) {
  if (state_->id() != state->id()) {
    NODELET_INFO("Executive state changing from [%s(%i)] to [%s(%i)].",
             state_->name().c_str(), state_->id(),
             state->name().c_str(), state->id());
    agent_state_.operating_state.state = state->id();
    state_ = state;
    PublishAgentState();
  }
}

void Executive::SetPlanExecState(uint8_t state) {
  agent_state_.plan_execution_state.state = state;
  PublishAgentState();
}

void Executive::SetRunPlanCmdId(std::string cmd_id) {
  run_plan_cmd_id_ = cmd_id;
}

/************************ Helper functions ************************************/
// Used as a helper function to send a failed ack when the command is not
// accepted in the current mobility state
void Executive::AckMobilityStateIssue(ff_msgs::CommandStampedPtr const& cmd,
                                  std::string const& current_mobility_state,
                                  std::string const& accepted_mobility_state) {
  std::string err_msg = cmd->cmd_name + " not accepted while " +
                                                  current_mobility_state + "!";
  if (accepted_mobility_state != "") {
    err_msg += " Resend command when " + accepted_mobility_state + ".";
  }

  state_->AckCmd(cmd->cmd_id,
                 ff_msgs::AckCompletedStatus::EXEC_FAILED,
                 err_msg);
}

bool Executive::ArmControl(ff_msgs::CommandStampedPtr const& cmd) {
  // Check to make sure we aren't trying to dock or perch
  if (agent_state_.mobility_state.state == ff_msgs::MobilityState::DOCKING) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   "Cannot move arm while (un)docking or docked!");
    return false;
  } else if (agent_state_.mobility_state.state ==
                                            ff_msgs::MobilityState::PERCHING &&
             agent_state_.mobility_state.sub_state != 0) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   "Cannot move arm while (un)perching!");
    return false;
  }

  // Check to make sure another arm command isn't being executed
  if (IsActionRunning(ARM)) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   "Arm command already being executed!");
    return false;
  }

  if (!FillArmGoal(cmd)) {
    return false;
  }

  NODELET_INFO("Executing arm command.");
  if (!StartAction(ARM, cmd->cmd_id)) {
    return false;
  }
  return true;
}

// Used to check the mobility state for commands that can only be executed when
// the astrobee is in some sort of stopped state. Send a failed execution ack
// and return false if mobility state is flying, docking, perching, or stopping.
bool Executive::CheckNotMoving(ff_msgs::CommandStampedPtr const& cmd) {
  if (agent_state_.mobility_state.state == ff_msgs::MobilityState::FLYING) {
    AckMobilityStateIssue(cmd, "flying");
  } else if (agent_state_.mobility_state.state ==
                                              ff_msgs::MobilityState::DOCKING &&
             agent_state_.mobility_state.sub_state != 0) {
    // Check if astrobee is docking vs. undocking
    if (agent_state_.mobility_state.sub_state > 0) {
      AckMobilityStateIssue(cmd, "docking", "docked");
    } else {
      AckMobilityStateIssue(cmd, "undocking", "stopped");
    }
  } else if (agent_state_.mobility_state.state ==
                                            ff_msgs::MobilityState::PERCHING &&
             agent_state_.mobility_state.sub_state != 0) {
    // Check if astrobee is perching vs. unperching
    if (agent_state_.mobility_state.sub_state > 0) {
      AckMobilityStateIssue(cmd, "perching", "perched");
    } else {
      AckMobilityStateIssue(cmd, "unperching", "stopped");
    }
  } else if (agent_state_.mobility_state.state ==
                                            ff_msgs::MobilityState::STOPPING &&
             agent_state_.mobility_state.sub_state == 1) {
    AckMobilityStateIssue(cmd, "stopping", "stopped");
  } else {
    return true;
  }
  return false;
}

bool Executive::CheckServiceExists(ros::ServiceClient& serviceIn,
                                   std::string const& serviceName,
                                   std::string const& cmd_id) {
  std::string err_msg = "";
  if (!serviceIn.exists()) {
    err_msg = serviceName + " service isn't running! Node may have died!";
    state_->AckCmd(cmd_id, ff_msgs::AckCompletedStatus::EXEC_FAILED, err_msg);
    return false;
  }
  return true;
}

bool Executive::CheckStoppedOrDrifting(std::string const& cmd_id,
                                       std::string const& cmd_name) {
  if ((agent_state_.mobility_state.state == ff_msgs::MobilityState::STOPPING &&
       agent_state_.mobility_state.sub_state == 0) ||
       agent_state_.mobility_state.state == ff_msgs::MobilityState::DRIFTING) {
    return true;
  }

  state_->AckCmd(cmd_id,
                 ff_msgs::AckCompletedStatus::EXEC_FAILED,
                 ("Must be stopped or drifting before " + cmd_name + "."));
  return false;
}

bool Executive::ConfigureLed(ff_hw_msgs::ConfigureSystemLeds& led_srv) {
  if (!led_client_.Call(led_srv)) {
    NODELET_ERROR("Configure system leds service not running!");
    return false;
  }

  if (!led_srv.response.success) {
    NODELET_ERROR("Configure system leds failed with message %s.",
                  led_srv.response.status.c_str());
    return false;
  }

  return true;
}

// Functions used to set variables that are used to configure mobility before a
// move or execute
bool Executive::ConfigureMobility(std::string const& cmd_id) {
  bool successful = true;

  // Initialize config clients if they haven't been initialized
  if (!choreographer_cfg_) {
    choreographer_cfg_ =
              std::make_shared<ff_util::ConfigClient>(&nh_, NODE_CHOREOGRAPHER);
  }

  if (!mapper_cfg_) {
    mapper_cfg_ = std::make_shared<ff_util::ConfigClient>(&nh_, NODE_MAPPER);
  }

  // Set values for configuring, these values will persist until changed
  // Choreographer
  choreographer_cfg_->Set<double>("desired_vel",
                                          agent_state_.target_linear_velocity);
  choreographer_cfg_->Set<double>("desired_accel",
                                          agent_state_.target_linear_accel);
  choreographer_cfg_->Set<double>("desired_omega",
                                          agent_state_.target_angular_velocity);
  choreographer_cfg_->Set<double>("desired_alpha",
                                          agent_state_.target_angular_accel);
  choreographer_cfg_->Set<bool>("enable_faceforward",
                                              !agent_state_.holonomic_enabled);
  choreographer_cfg_->Set<bool>("enable_collision_checking",
                                                  agent_state_.check_obstacles);
  choreographer_cfg_->Set<bool>("enable_validation", agent_state_.check_zones);
  choreographer_cfg_->Set<bool>("enable_timesync", false);
  choreographer_cfg_->Set<bool>("enable_immediate",
                                                agent_state_.immediate_enabled);
  choreographer_cfg_->Set<std::string>("planner", agent_state_.planner);
  // This function is not used for the first segment of a plan so always disable
  // move to start
  choreographer_cfg_->Set<bool>("enable_bootstrapping", false);
  choreographer_cfg_->Set<bool>("enable_replanning",
                                              agent_state_.replanning_enabled);

  // Mapper
  mapper_cfg_->Set<double>("inflate_radius", agent_state_.collision_distance);

  std::string err_msg = "";

  // Reconfigure choreographer, mapper
  if (!choreographer_cfg_->Reconfigure()) {
    successful = false;
    err_msg = "Couldn't configure the mobilty::choreographer node! ";
  }

  if (!mapper_cfg_->Reconfigure()) {
    successful = false;
    err_msg += "Couldn't configure the mobility::mapper node!";
  }

  // Ack error
  if (!successful) {
    NODELET_ERROR("%s", err_msg.c_str());
    state_->AckCmd(cmd_id, ff_msgs::AckCompletedStatus::EXEC_FAILED, err_msg);
  }

  return successful;
}

bool Executive::ConfigureMobility(bool move_to_start,
                                  std::string& err_msg) {
  bool successful = true;

  // TODO(Katie) Change when Ted changes the sequencer

  // Initialize config clients if they haven't been initialized
  if (!choreographer_cfg_) {
    choreographer_cfg_ =
              std::make_shared<ff_util::ConfigClient>(&nh_, NODE_CHOREOGRAPHER);
  }

  if (!mapper_cfg_) {
    mapper_cfg_ =
                  std::make_shared<ff_util::ConfigClient>(&nh_, NODE_MAPPER);
  }

  // Set values for configuring, these values will persist until changed
  // Choreographer
  choreographer_cfg_->Set<double>("desired_vel",
                                          agent_state_.target_linear_velocity);
  choreographer_cfg_->Set<double>("desired_accel",
                                          agent_state_.target_linear_accel);
  choreographer_cfg_->Set<double>("desired_omega",
                                          agent_state_.target_angular_velocity);
  choreographer_cfg_->Set<double>("desired_alpha",
                                          agent_state_.target_angular_accel);
  choreographer_cfg_->Set<bool>("enable_faceforward",
                                              !agent_state_.holonomic_enabled);
  choreographer_cfg_->Set<bool>("enable_collision_checking",
                                                  agent_state_.check_obstacles);
  choreographer_cfg_->Set<bool>("enable_validation", agent_state_.check_zones);
  choreographer_cfg_->Set<bool>("enable_timesync", false);
  choreographer_cfg_->Set<bool>("enable_immediate",
                                                agent_state_.immediate_enabled);
  choreographer_cfg_->Set<std::string>("planner", agent_state_.planner);
  choreographer_cfg_->Set<bool>("enable_bootstrapping", move_to_start);
  choreographer_cfg_->Set<bool>("enable_replanning",
                                              agent_state_.replanning_enabled);

  // Mapper
  mapper_cfg_->Set<double>("inflate_radius", agent_state_.collision_distance);

  // Clear err_msg
  err_msg = "";

  // Reconfigure choreographer, planner, mapper
  if (!choreographer_cfg_->Reconfigure()) {
    successful = false;
    err_msg = "Couldn't configure the mobilty::choreographer node! ";
  }

  if (!mapper_cfg_->Reconfigure()) {
    successful = false;
    err_msg += "Couldn't configure the mobility::mapper node!";
  }

  return successful;
}

bool Executive::LoadUnloadNodelet(ff_msgs::CommandStampedPtr const& cmd) {
  bool load = true;
  std::string which = "Load";
  int num_args = cmd->args.size();

  if (cmd->cmd_name == CommandConstants::CMD_NAME_UNLOAD_NODELET) {
    load = false;
    which = "Unload";
  }

  ff_msgs::UnloadLoadNodelet unload_load_nodelet_srv;
  unload_load_nodelet_srv.request.load = load;

  // Don't load/unload a nodelet while moving
  if (CheckNotMoving(cmd)) {
    // Only one argument is required for load/unload nodelet, the nodelet name
    if (num_args < 1) {
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                     (which + " nodelet must have one argument."));
      return false;
    }

    // The unload nodelet command takes only 1 or 2 arguments
    if (num_args > 2 && !load) {
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                     (which + " nodelet takes no more than two arguments."));
      return false;
    }

    // The load nodelet command takes 1 or up to 4 arguments
    if (num_args > 4 && load) {
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                     (which + " nodelet takes no more than four arguments."));
      return false;
    }

    // Extract arguments
    for (int i = 0; i < num_args; i++) {
      if (cmd->args[i].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
        state_->AckCmd(cmd->cmd_id,
                       ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                       (which + " nodelet parameters must be strings."));
        return false;
      }

      if (i == 0) {
        unload_load_nodelet_srv.request.name = cmd->args[0].s;
      } else if (i == 1) {
        unload_load_nodelet_srv.request.manager_name = cmd->args[1].s;
      } else if (i == 2) {
        unload_load_nodelet_srv.request.type = cmd->args[2].s;
      } else {
        unload_load_nodelet_srv.request.bond_id = cmd->args[3].s;
      }
    }

    // Check if the load/unload nodelet service is running
    if (!CheckServiceExists(unload_load_nodelet_client_,
                            "Load/unload nodelet",
                            cmd->cmd_id)) {
      return false;
    }

    // Call the unload load nodelet service
    if (!unload_load_nodelet_client_.call(unload_load_nodelet_srv)) {
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::EXEC_FAILED,
                     "Unload load nodelet service returned false.");
      return false;
    }

    if (unload_load_nodelet_srv.response.result !=
                            ff_msgs::UnloadLoadNodelet::Response::SUCCESSFUL) {
      //err_msg = which + " nodelet failed with result ";
      //err_msg += std::to_string(unload_load_nodelet_srv.response.result);
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::EXEC_FAILED,
                     (which + " nodelet failed with result " +
                      std::to_string(unload_load_nodelet_srv.response.result)));
      return false;
    }

    state_->AckCmd(cmd->cmd_id);
    return true;
  }

  return false;
}

ros::Time Executive::MsToSec(std::string timestamp) {
  uint64_t time, secs, nsecs;
  time = std::stoull(timestamp);
  secs = time/1000;
  nsecs = (time % 1000) * 1000000;

  return ros::Time(secs, nsecs);
}

bool Executive::PowerItem(ff_msgs::CommandStampedPtr const& cmd, bool on) {
  uint8_t completed_status = ff_msgs::AckCompletedStatus::OK;
  std::string err_msg = "";
  bool success;

  // Check to make sure command is formatted as expected
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                   "Malformed arguments for power item command!");
    return false;
  }

  NODELET_INFO("Item %s is being powered on/off!", cmd->args[0].s.c_str());

  // Handle pmcs and laser the same since they use the same service message
  if (cmd->args[0].s ==
        CommandConstants::PARAM_NAME_POWERED_COMPONENT_LASER_POINTER ||
      cmd->args[0].s ==
        CommandConstants::PARAM_NAME_POWERED_COMPONENT_PMCS_AND_SIGNAL_LIGHTS) {
    ff_hw_msgs::SetEnabled enable_srv;
    enable_srv.request.enabled = on;

    if (cmd->args[0].s ==
                CommandConstants::PARAM_NAME_POWERED_COMPONENT_LASER_POINTER) {
      // Check to make sure the laser service is valid and running
      if (!CheckServiceExists(laser_enable_client_,
                              "Enable laser",
                              cmd->cmd_id)) {
        return false;
      }
      success = laser_enable_client_.call(enable_srv);
    } else {  // PMCS
      // Check to make sure the pmc service is valid and running
      if (!CheckServiceExists(pmc_enable_client_, "Enable PMC", cmd->cmd_id)) {
        return false;
      }
      success = pmc_enable_client_.call(enable_srv);
    }

    // Check to see if the service was successfully enabled/disabled
    if (!success) {
      err_msg = "Service returned false.";
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
    } else if (!enable_srv.response.success) {
      err_msg = enable_srv.response.status_message;
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
    }
  } else {  // Item is probably a payload
    ff_hw_msgs::ConfigurePayloadPower config_srv;
    config_srv.request.top_front = config_srv.request.PERSIST;
    config_srv.request.bottom_front = config_srv.request.PERSIST;
    config_srv.request.top_aft = config_srv.request.PERSIST;
    config_srv.request.bottom_aft = config_srv.request.PERSIST;

    uint8_t power;
    if (on) {
      power = ff_hw_msgs::ConfigurePayloadPower::Request::ON;
    } else {
      power = ff_hw_msgs::ConfigurePayloadPower::Request::OFF;
    }

    if (cmd->args[0].s ==
              CommandConstants::PARAM_NAME_POWERED_COMPONENT_PAYLOAD_TOP_AFT) {
      config_srv.request.top_aft = power;
    } else if (cmd->args[0].s ==
            CommandConstants::PARAM_NAME_POWERED_COMPONENT_PAYLOAD_BOTTOM_AFT) {
      config_srv.request.bottom_aft = power;
    } else if (cmd->args[0].s ==
          CommandConstants::PARAM_NAME_POWERED_COMPONENT_PAYLOAD_BOTTOM_FRONT) {
      config_srv.request.bottom_front = power;
    } else {  // Item wasn't recognized
      err_msg = "Item " + cmd->args[0].s + " not recognized in power item.";
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::EXEC_FAILED,
                     err_msg);
      return false;
    }

    if (!CheckServiceExists(payload_power_client_,
                            "Power payload",
                            cmd->cmd_id)) {
      return false;
    }

    success = payload_power_client_.call(config_srv);

    // Check to see if the payload was successfully enabled/disabled
    if (!success) {
      err_msg = "Power payload service returned false.";
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
    } else if (!config_srv.response.success) {
      err_msg = config_srv.response.status;
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
    }
  }

  state_->AckCmd(cmd->cmd_id, completed_status, err_msg);
  return true;
}

bool Executive::ResetEkf(std::string const& cmd_id) {
  localization_goal_.command = ff_msgs::LocalizationGoal::COMMAND_RESET_FILTER;
  // Don't need to specify a pipeline for reset but clear it just in case
  localization_goal_.pipeline = "";

  return StartAction(LOCALIZATION, cmd_id);
}

void Executive::StartWaitTimer(float duration) {
  wait_timer_.setPeriod(ros::Duration(duration));
  wait_timer_.start();
}

void Executive::StopWaitTimer() {
  wait_timer_.stop();
}

/************************ Plan related functions ******************************/
// Returns false if there are no more command/segments in the plan
bool Executive::AckCurrentPlanItem() {
  ff_msgs::AckCompletedStatus ack;
  ack.status = ff_msgs::AckCompletedStatus::OK;
  return sequencer_.Feedback(ack);
}

sequencer::ItemType Executive::GetCurrentPlanItemType() {
  return sequencer_.CurrentType();
}

ff_msgs::CommandStampedPtr Executive::GetPlanCommand() {
  return sequencer_.CurrentCommand();
}

bool Executive::GetSetPlanInertia(std::string const& cmd_id) {
  // If the plan has inertia, set it. Otherwise, leave the inertia the way it is
  if (sequencer_.HaveInertia()) {
    ff_msgs::SetInertia inertia_srv;
    inertia_srv.request.inertia = sequencer_.GetInertia();
    // Plan header doesn't contain the center of mass so we need to get the
    // current center of mass
    inertia_srv.request.inertia.inertia.com = current_inertia_->inertia.com;

    if (!CheckServiceExists(set_inertia_client_, "Set inertia", cmd_id)) {
      return false;
    }

    if (!set_inertia_client_.call(inertia_srv)) {
      state_->AckCmd(cmd_id,
                     ff_msgs::AckCompletedStatus::EXEC_FAILED,
                     "Set inertia service returned false for plan inertia.");
      return false;
    }

    if (!inertia_srv.response.success) {
      state_->AckCmd(cmd_id,
                     ff_msgs::AckCompletedStatus::EXEC_FAILED,
                     "Set inertia srv returned unsuccessful for plan inertia");
      return false;
    }
  }
  return true;
}

void Executive::GetSetPlanOperatingLimits() {
  // If the plan has operating limits, set them. Otherwise, use the current ones
  if (sequencer_.HaveOperatingLimits()) {
    sequencer_.GetOperatingLimits(agent_state_);
    // Don't configure mobility. That is done in the plan op state before we
    // execute a segment.
    PublishAgentState();
  }
}

/************************ Commands ********************************************/
bool Executive::ArmPanAndTilt(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing arm pan and tilt command!");
  return ArmControl(cmd);
}

bool Executive::AutoReturn(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing auto return command!");
  bool successful = false;
  std::string err_msg;

  // Astrobee needs to be either stopped or drifting
  if (agent_state_.mobility_state.state == ff_msgs::MobilityState::DOCKING) {
    err_msg = "Already docked!";
  } else if (agent_state_.mobility_state.state ==
                                            ff_msgs::MobilityState::PERCHING) {
    err_msg = "Astrobee cannot attempt to dock while it is perched(ing).";
  } else {
    // TODO(Katie) Currently this is just the dock 1 command with return to dock
    // set to true! Change to be actual code
    successful = true;
    cmd->cmd_name = "dock";
    cmd->args.resize(1);
    cmd->args[0].data_type = ff_msgs::CommandArg::DATA_TYPE_INT;
    cmd->args[0].i = 1;
    if (!FillDockGoal(cmd, true)) {
      return false;
    }

    if (!StartAction(DOCK, cmd->cmd_id)) {
      return false;
    }
  }

  // Ack command as failed if we are already docked or perched
  if (!successful) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   err_msg);
  }
  return successful;
}

bool Executive::CustomGuestScience(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing custom guest science command!");
  // Check command arguments are correcy before sending to the guest science
  // manager
  if (cmd->args.size() != 2 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
      cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                   "Malformed arguments for custom guest science command.");
    return false;
  }

  if (gs_custom_cmd_id_ != "") {
    std::string msg = "Already executing a custom gs command. Please wait for ";
    msg += "it to finish before issuing another custom gs command.";
    state_->AckCmd(cmd->cmd_id, ff_msgs::AckCompletedStatus::EXEC_FAILED, msg);
    return false;
  }

  gs_cmd_pub_.publish(cmd);
  gs_custom_command_timer_.setPeriod(ros::Duration(gs_command_timeout_));
  gs_custom_command_timer_.start();
  gs_custom_cmd_id_ = cmd->cmd_id;
  return true;
}

bool Executive::Dock(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing dock command!");
  bool successful = false;
  std::string err_msg;

  // Make sure robot is stopped before attempting to dock. Only accept dock in
  // ready op state so perched and docked are the only mobility states we need
  // to check for
  if (agent_state_.mobility_state.state == ff_msgs::MobilityState::DOCKING) {
    err_msg = "Already docked.";
  } else if (agent_state_.mobility_state.state ==
                                            ff_msgs::MobilityState::PERCHING) {
    err_msg = "Astrobee cannot attempt to dock while it is perched.";
  } else {
    successful = true;
    if (!FillDockGoal(cmd, false)) {
      return false;
    }

    if (!StartAction(DOCK, cmd->cmd_id)) {
      return false;
    }
  }

  // Fill dock goal and start action publish failed command acks so we only
  // need to fail an ack if we aren't stopped and thus cannot dock
  if (!successful) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   err_msg);
  }
  return successful;
}

bool Executive::Fault(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing fault command!");
  // Only transition to the fault state if the fault command came from the
  // system monitor or executive. The only way to transition out of the fault
  // state is if the system monitor state changes to functional so we don't wan
  // the ground to issue a fault command because there will be no way to
  // transition out of the fault state
  if (cmd->cmd_src == "sys_monitor" || cmd->cmd_src == "executive") {
    // Configure leds to be blinking since there is a fault
    ff_hw_msgs::ConfigureSystemLeds led_srv;
    led_srv.request.status_a2 = ff_hw_msgs::ConfigureSystemLeds::Request::FAST;
    ConfigureLed(led_srv);
    state_->AckCmd(cmd->cmd_id);
    return true;
  }
  state_->AckCmd(cmd->cmd_id,
                 ff_msgs::AckCompletedStatus::EXEC_FAILED,
                 "The fault command can only come from the system monitor.");
  return false;
}

bool Executive::GripperControl(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing gripper control command!");
  return ArmControl(cmd);
}

bool Executive::IdlePropulsion(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing idle propulsion command!");
  // Cancel any motion actions being executed include the arm
  unsigned int i = 0;
  for (i = 0; i < running_actions_.size(); i++) {
    if (running_actions_[i] == ARM) {
      CancelAction(ARM, "idle");
      i--;
    } else if (running_actions_[i] == DOCK) {
      CancelAction(DOCK, "idle");
      i--;
    } else if (running_actions_[i] == EXECUTE) {
      CancelAction(EXECUTE, "idle");
      i--;
    } else if (running_actions_[i] == IDLE) {
      // Don't try to idle if we are already trying to idle
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::EXEC_FAILED,
                     "Can't idle, already trying to idle");
      return false;
    } else if (running_actions_[i] == MOVE) {
      CancelAction(MOVE, "idle");
      i--;
    } else if (running_actions_[i] == PERCH) {
      CancelAction(PERCH, "idle");
      i--;
    } else if (running_actions_[i] == STOP) {
      CancelAction(STOP, "idle");
      i--;
    } else if (running_actions_[i] == UNDOCK) {
      CancelAction(UNDOCK, "idle");
      i--;
    } else if (running_actions_[i] == UNPERCH) {
      CancelAction(UNPERCH, "idle");
      i--;
    }
  }

  if (!FillMotionGoal(IDLE)) {
    return false;
  }

  return StartAction(IDLE, cmd->cmd_id);
}

bool Executive::InitializeBias(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing initialize bias command!");
  // We don't want to initialize the bias when stopped because we will not be
  // completely still
  if (agent_state_.mobility_state.state == ff_msgs::MobilityState::STOPPING) {
    AckMobilityStateIssue(cmd, "stopped",  "docked, idle, or perched");
    return false;
  }

  // We also cannot be moving when we initialize the bias
  if (CheckNotMoving(cmd)) {
    localization_goal_.command =
                              ff_msgs::LocalizationGoal::COMMAND_ESTIMATE_BIAS;
    // Don't need to specify a pipeline for init bias but clear it just in case
    localization_goal_.pipeline = "";
    return StartAction(LOCALIZATION, cmd->cmd_id);
  }
  return false;
}

bool Executive::LoadNodelet(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing load nodelet command!");
  return LoadUnloadNodelet(cmd);
}

bool Executive::NoOp(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing noop command!");
  state_->AckCmd(cmd->cmd_id);
  return true;
}

bool Executive::PausePlan(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing pause plan command!");
  return state_->PausePlan(cmd);
}

bool Executive::Perch(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing perch command!");
  bool successful = false;
  std::string err_msg;

  // Make sure robot is stopped before attempting to perch. Only accept perch in
  // ready op state so perched and docked are the only mobility states we need
  // to check for
  if (agent_state_.mobility_state.state == ff_msgs::MobilityState::DOCKING) {
    err_msg = "Astrobee cannot attempt to perch while it is perched.";
  } else if (agent_state_.mobility_state.state ==
                                            ff_msgs::MobilityState::PERCHING) {
    err_msg = "Already perched.";
  } else {
    successful = true;
    perch_goal_.command = ff_msgs::PerchGoal::PERCH;

    if (!StartAction(PERCH, cmd->cmd_id)) {
      return false;
    }
  }

  // Start action publishes failed command acks so we only need to fail an ack
  // if we aren't stopped and thus cannot perch
  if (!successful) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   err_msg);
  }
  return successful;
}

bool Executive::PowerItemOff(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing power item off command!");
  return PowerItem(cmd, false);
}

bool Executive::PowerItemOn(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing power item on command!");
  return PowerItem(cmd, true);
}

bool Executive::Prepare(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing prepare command!");
  // TODO(Katie) Stub, change to be actual code
  // Astrobee needs to be either stopped or drifting
  if (CheckStoppedOrDrifting(cmd->cmd_id, "preparing")) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   "Prepare not implemented yet! Stay tune!");
    return false;
  }
  return false;
}

bool Executive::ReacquirePosition(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing reacquire position command!");
  if (CheckNotMoving(cmd)) {
    // Reacquire position tries to get astrobee localizing again with mapped
    // landmarks
    localization_goal_.command =
                          ff_msgs::LocalizationGoal::COMMAND_SWITCH_PIPELINE;
    localization_goal_.pipeline =
                          ff_msgs::LocalizationGoal::PIPELINE_MAP_LANDMARKS;
    return StartAction(REACQUIRE, cmd->cmd_id);
  }
  return false;
}

bool Executive::ResetEkf(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing reset ekf command!");
  if (CheckNotMoving(cmd)) {
    return ResetEkf(cmd->cmd_id);
  }
  return false;
}

bool Executive::RunPlan(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing run plan command!");
  if (agent_state_.plan_execution_state.state != ff_msgs::ExecState::PAUSED) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   "Got run plan cmd but plan status is not paused!");
    return false;
  }

  // Check if plan is empty before switch operating states
  // Also send successful ack because technically we didn't fail, the plan
  // was just empty
  if (GetCurrentPlanItemType() == sequencer::ItemType::NONE) {
    SetPlanExecState(ff_msgs::ExecState::IDLE);
    state_->AckCmd(cmd->cmd_id);
    return false;
  }

  // We can start the plan!
  state_->AckCmd(cmd->cmd_id,
                 ff_msgs::AckCompletedStatus::NOT,
                 "",
                 ff_msgs::AckStatus::EXECUTING);
  return true;
}

bool Executive::SetCamera(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set camera command!");
  std::string err_msg = "";
  uint8_t completed_status = ff_msgs::AckCompletedStatus::OK;
  bool successful = true;
  // Check to make sure command is formatted as expected
  if (cmd->args.size() != 5 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
      cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
      cmd->args[2].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
      cmd->args[3].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT ||
      cmd->args[4].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT) {
    successful = false;
    err_msg = "Malformed arguments for set camera command!";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else {
    // Third argument is a string specifing the width and height, need to
    // parse it
    std::string width, height;
    std::size_t pos;
    if (cmd->args[2].s.find("_") != std::string::npos) {
      pos = cmd->args[2].s.find("_");
    } else if (cmd->args[2].s.find("X") != std::string::npos) {
      pos = cmd->args[2].s.find("X");
    } else if (cmd->args[2].s.find("x") != std::string::npos) {
      pos = cmd->args[2].s.find("x");
    } else {
      successful = false;
      err_msg = "Camera resolution needs format w_h, wXh, or wxh!";
      completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
    }

    // Need to ensure mode is valid
    int mode;
    if (cmd->args[1].s == CommandConstants::PARAM_NAME_CAMERA_MODE_BOTH) {
      mode = ff_msgs::ConfigureCamera::Request::BOTH;
    } else if (cmd->args[1].s ==
                          CommandConstants::PARAM_NAME_CAMERA_MODE_RECORDING) {
      mode = ff_msgs::ConfigureCamera::Request::RECORDING;
    } else if (cmd->args[1].s ==
                          CommandConstants::PARAM_NAME_CAMERA_MODE_STREAMING) {
      mode = ff_msgs::ConfigureCamera::Request::STREAMING;
    } else {
      successful = false;
      err_msg = "Camera mode invalid. Options are Both, Streaming, Recording";
      completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
    }

    if (successful) {
      width = cmd->args[2].s.substr(0, pos);
      height = cmd->args[2].s.substr((pos + 1));

      ff_msgs::ConfigureCamera config_img_srv;
      config_img_srv.request.mode = mode;
      config_img_srv.request.rate = cmd->args[3].f;
      config_img_srv.request.width = std::stoi(width);
      config_img_srv.request.height = std::stoi(height);
      config_img_srv.request.bitrate = cmd->args[4].f;

      if (cmd->args[0].s == CommandConstants::PARAM_NAME_CAMERA_NAME_DOCK) {
        // Check to make sure the dock cam service is valid
        if (!dock_cam_config_client_.exists()) {
          successful = false;
          err_msg = "Dock cam config service not running! Node may have died";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        } else {
          // Check to see if the dock cam was successfully configured
          if (!dock_cam_config_client_.call(config_img_srv)) {
            // Service only fails if height, width or rate are less than or
            // equal to 0
            successful = false;
            err_msg = "Height, width, and/or rate was invalid for dock camera.";
            completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
          }
        }
      } else if (cmd->args[0].s ==
                                CommandConstants::PARAM_NAME_CAMERA_NAME_HAZ) {
        // Check to make sure the haz cam service is valid
        if (!haz_cam_config_client_.exists()) {
          successful = false;
          err_msg = "Haz cam config service not running! Node may have died";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        } else {
          // Check to see if the haz cam was successfully configured
          if (!haz_cam_config_client_.call(config_img_srv)) {
            // Service only fails if height, width or rate are less than or
            // equal to 0
            successful = false;
            err_msg = "Height, width, and/or rate was invalid for haz camera.";
            completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
          }
        }
      } else if (cmd->args[0].s ==
                                CommandConstants::PARAM_NAME_CAMERA_NAME_NAV) {
        // Check to make sure the nav cam service is valid
        if (!nav_cam_config_client_.exists()) {
          successful = false;
          err_msg = "Nav cam configure service not running! Node may have died";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        } else {
          // Check to see if the nav cam was successfully configured
          if (!nav_cam_config_client_.call(config_img_srv)) {
            // Service only fails if height, width or rate are less than or
            // equal to 0
            successful = false;
            err_msg = "Height, width, and/or rate was invalid for nav camera.";
            completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
          }
        }
      } else if (cmd->args[0].s ==
                              CommandConstants::PARAM_NAME_CAMERA_NAME_PERCH) {
        // Check to make sure the perch cam service is valid
        if (!perch_cam_config_client_.exists()) {
          successful = false;
          err_msg = "Perch cam config service not running! Node may have died";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        } else {
          // Check to see if the perch cam was successfully configured
          if (!perch_cam_config_client_.call(config_img_srv)) {
            // Service only fails if height, width or rate are less than or
            // equal to 0
            successful = false;
            err_msg = "Height, width, and/or rate was invalid for perch camera";
            completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
          }
        }
      } else if (cmd->args[0].s ==
                                CommandConstants::PARAM_NAME_CAMERA_NAME_SCI) {
        // Check to make sure the sci cam service is valid
        if (!sci_cam_config_client_.exists()) {
          successful = false;
          err_msg = "Sci cam configure service not running! Node may have died";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        } else {
          // Check to see if the sci cam was successfully configured
          if (!sci_cam_config_client_.call(config_img_srv)) {
            // Service only fails if height, width, rate or bitrate are less
            // than or equal to 0
            successful = false;
            err_msg = "Height, width, rate, and/or bitrate was invalid for sci";
            err_msg += " camera.";
            completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
          }
        }
      } else {
        successful = false;
        err_msg = "Camera " + cmd->args[0].s + " not recognized.";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      }
    }
  }

  state_->AckCmd(cmd->cmd_id, completed_status, err_msg);
  return successful;
}

bool Executive::SetCameraRecording(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set camera recording command!");
  bool successful = true;
  std::string err_msg;
  uint8_t completed_status = ff_msgs::AckCompletedStatus::OK;
  // Check to make sure command is formatted as expected
  if (cmd->args.size() != 2 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
      cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
    successful = false;
    err_msg = "Malformed arguments for set camera recording command!";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else {
    ff_msgs::EnableCamera enable_img_srv;
    enable_img_srv.request.mode = ff_msgs::EnableCamera::Request::RECORDING;
    enable_img_srv.request.enable = cmd->args[1].b;

    if (cmd->args[0].s == CommandConstants::PARAM_NAME_CAMERA_NAME_DOCK) {
      // Check to make sure the dock cam enable service exists
      if (!dock_cam_enable_client_.exists()) {
        successful = false;
        err_msg = "Dock cam enable service not running! Node may have died!";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        // Check to see if recording was set successfully
        if (!dock_cam_enable_client_.call(enable_img_srv)) {
          successful = false;
          err_msg = "Enable recording failed for dock cam.";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    } else if (cmd->args[0].s == CommandConstants::PARAM_NAME_CAMERA_NAME_HAZ) {
      // Check to make sure the haz cam enable service exists
      if (!haz_cam_enable_client_.exists()) {
        successful = false;
        err_msg = "Haz cam enable service not running! Node may have died!";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        // Check to see if recording was set successfully
        if (!haz_cam_enable_client_.call(enable_img_srv)) {
          successful = false;
          err_msg = "Enable recording failed for haz cam.";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    } else if (cmd->args[0].s == CommandConstants::PARAM_NAME_CAMERA_NAME_NAV) {
      // Check to make sure the nav cam enable service exists
      if (!nav_cam_enable_client_.exists()) {
        successful = false;
        err_msg = "Nav cam enable service not running! Node may have died!";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        // Check to see if recording was set successfully
        if (!nav_cam_enable_client_.call(enable_img_srv)) {
          successful = false;
          err_msg = "Enable recording failed for nav cam.";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    } else if (cmd->args[0].s ==
                              CommandConstants::PARAM_NAME_CAMERA_NAME_PERCH) {
      // Check to make sure the perch cam enable service exists
      if (!perch_cam_enable_client_.exists()) {
        successful = false;
        err_msg = "Perch cam enable service not running! Node may have died!";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        // Check to see if recording was set successfully
        if (!perch_cam_enable_client_.call(enable_img_srv)) {
          successful = false;
          err_msg = "Enable recording failed for perch cam.";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    } else if (cmd->args[0].s == CommandConstants::PARAM_NAME_CAMERA_NAME_SCI) {
      // Check to make sure the sci enable service exists
      if (!sci_cam_enable_client_.exists()) {
        successful = false;
        err_msg = "Sci cam enable service not running! Node may have died!";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        // Check to see if recording was set successfully
        if (!sci_cam_enable_client_.call(enable_img_srv)) {
          successful = false;
          err_msg = "Enable recording failed for sci cam.";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    } else {
      successful = false;
      err_msg = "Camera " + cmd->args[0].s + " not recognized.";
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
    }
  }

  state_->AckCmd(cmd->cmd_id, completed_status, err_msg);
  return successful;
}

bool Executive::SetCameraStreaming(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set camera streaming command!");
  bool successful = true;
  std::string err_msg = "";
  uint8_t completed_status = ff_msgs::AckCompletedStatus::OK;
  // Check to make sure command is formatted as expected
  if (cmd->args.size() != 2 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
      cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
    successful = false;
    err_msg = "Malformed arguments for set camera streaming!";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else {
    ff_msgs::EnableCamera enable_img_srv;
    enable_img_srv.request.mode = ff_msgs::EnableCamera::Request::STREAMING;
    enable_img_srv.request.enable = cmd->args[1].b;

    if (cmd->args[0].s == CommandConstants::PARAM_NAME_CAMERA_NAME_DOCK) {
      // Check to make sure the dock cam service is valid
      if (!dock_cam_enable_client_.exists()) {
          successful = false;
          err_msg = "Dock cam enable service not running! Node may have died!";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        // Check to see if streaming was successfully set
        if (!dock_cam_enable_client_.call(enable_img_srv)) {
          successful = false;
          err_msg = "Enable streaming failed for dock cam.";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    } else if (cmd->args[0].s == CommandConstants::PARAM_NAME_CAMERA_NAME_HAZ) {
      // Check to make sure the haz cam service is valid
      if (!haz_cam_enable_client_.exists()) {
        successful = false;
        err_msg = "Haz cam enable service not running! Node may have died!";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        // Check to see if streaming was successfully set
        if (!haz_cam_enable_client_.call(enable_img_srv)) {
          successful = false;
          err_msg = "Enable streaming failed for haz cam.";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    } else if (cmd->args[0].s == CommandConstants::PARAM_NAME_CAMERA_NAME_NAV) {
      // Check to make sure the nav cam service is valid
      if (!nav_cam_enable_client_.exists()) {
        successful = false;
        err_msg = "Nav cam enable service not running! Node may have died!";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        // Check to see if streaming was successfully set
        if (!nav_cam_enable_client_.call(enable_img_srv)) {
          successful = false;
          err_msg = "Enable streaming failed for nav cam.";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    } else if (cmd->args[0].s ==
                              CommandConstants::PARAM_NAME_CAMERA_NAME_PERCH) {
      // Check to make sure the perch cam service is valid
      if (!perch_cam_enable_client_.exists()) {
        successful = false;
        err_msg = "Perch cam enable service not running! Node may have died!";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        // Check to see if streaming was successfully set
        if (!perch_cam_enable_client_.call(enable_img_srv)) {
          successful = false;
          err_msg = "Enable streaming failed for perch cam.";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    } else if (cmd->args[0].s == CommandConstants::PARAM_NAME_CAMERA_NAME_SCI) {
      // Check to make sure the sci cam service is valid
      if (!sci_cam_enable_client_.exists()) {
        successful = false;
        err_msg = "Sci cam enable service not running! Node may have died!";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        // Check to see if streaming was successfully set
        if (!sci_cam_enable_client_.call(enable_img_srv)) {
          successful = false;
          err_msg = "Enable streaming failed for sci cam.";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    } else {
      successful = false;
      err_msg = "Camera " + cmd->args[0].s + " not recognized.";
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
    }
  }

  state_->AckCmd(cmd->cmd_id, completed_status, err_msg);
  return successful;
}

bool Executive::SetCheckObstacles(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set check obstacles command!");
  // Don't set whether to check obstacles when moving
  if (CheckNotMoving(cmd)) {
    if (cmd->args.size() != 1 ||
        cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
      NODELET_ERROR("Malformed arguments for set check obstacles!");
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                     "Malformed arguments for set check obstacles!");
      return false;
    }

    agent_state_.check_obstacles = cmd->args[0].b;
    PublishAgentState();
    state_->AckCmd(cmd->cmd_id);
    return true;
  }
  return false;
}

bool Executive::SetCheckZones(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set check zones command!");
  // Don't set whether to check zones when moving
  if (CheckNotMoving(cmd)) {
    if (cmd->args.size() != 1 ||
        cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
      NODELET_ERROR("Malformed arguments for set check zones!");
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                     "Malformed arguments for set check zones!");
      return false;
    }

    agent_state_.check_zones = cmd->args[0].b;
    PublishAgentState();
    state_->AckCmd(cmd->cmd_id);
    return true;
  }
  return false;
}

bool Executive::SetDataToDisk(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set data to disk command!");
  if (data_to_disk_) {
    ff_msgs::SetDataToDisk data_srv;
    std::string file_contents;

    // Decompress file into a string
    if (!sequencer::DecompressData(
                      reinterpret_cast<const char*>(data_to_disk_->file.data()),
                      data_to_disk_->file.size(),
                      data_to_disk_->type,
                      &file_contents)) {
      // Reset data to disk so that the same file isn't reloaded
      data_to_disk_.reset();
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::EXEC_FAILED,
                     "Unable to decompress data to disk file.");
      return false;
    }

    // Reset data to disk so that the same file isn't reloaded
    data_to_disk_.reset();

    // Convert string into a json object
    Json::Value file_obj;
    if (!jsonloader::LoadData(file_contents, &file_obj)) {
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                     "Error parsing json.");
      return false;
    }

    // Check to make sure the name exists in the file
    if (!file_obj.isMember("name") || !file_obj["name"].isString()) {
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                     "Name of data profile doesn't exist or isn't a string.");
      return false;
    }

    // Get name
    data_srv.request.state.name = file_obj["name"].asString();

    // Check to make sure topic settings exists
    if (!file_obj.isMember("topicSettings") ||
        !file_obj["topicSettings"].isArray()) {
      state_->AckCmd(cmd->cmd_id,
                    ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                    "Topic settings doesn't exist in file or is not an array!");
      return false;
    }

    ff_msgs::SaveSettings save_settings;
    for (Json::Value const& data_obj : file_obj["topicSettings"]) {
      // Check to make sure topic name exists
      if (!data_obj.isMember("topicName") ||
          !data_obj["topicName"].isString()) {
        state_->AckCmd(cmd->cmd_id,
                       ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                       "Topic name for entry doesn't exist or isn't a string.");
        return false;
      }
      save_settings.topic_name = data_obj["topicName"].asString();

      // Check to make sure downlink option exists
      if (!data_obj.isMember("downlinkOption") ||
          !data_obj["downlinkOption"].isString()) {
        state_->AckCmd(cmd->cmd_id,
                       ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                       "Downlink option doesn't exist or isn't a string.");
        return false;
      }

      if (data_obj["downlinkOption"].asString() == "immediate") {
        save_settings.downlinkOption = ff_msgs::SaveSettings::IMMEDIATE;
      } else if (data_obj["downlinkOption"].asString() == "delayed") {
        save_settings.downlinkOption = ff_msgs::SaveSettings::DELAYED;
      } else {
        state_->AckCmd(cmd->cmd_id,
                       ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                       "Downlink option must be either delayed or immediate.");
        return false;
      }

      // Check to make sure the frequency exists
      if (!data_obj.isMember("frequency") ||
          !data_obj["frequency"].isNumeric()) {
        state_->AckCmd(cmd->cmd_id,
                       ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                       "Frequency for entry doesn't exist or isn't a float.");
        return false;
      }
      save_settings.frequency = data_obj["frequency"].asFloat();

      data_srv.request.state.topic_save_settings.push_back(save_settings);
    }

    // Check to make sure the service is valid and running
    if (!CheckServiceExists(set_data_client_,
                            "Set data to disk",
                            cmd->cmd_id)) {
      return false;
    }

    if (!set_data_client_.call(data_srv)) {
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::EXEC_FAILED,
                     "Set data to disk service returned false.");
      return false;
    }

    if (!data_srv.response.success) {
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::EXEC_FAILED,
                     data_srv.response.status);
      return false;
    }

    // Everything was successful
    state_->AckCmd(cmd->cmd_id);
    return true;
  }

  // Data to disk pointer is null
  state_->AckCmd(cmd->cmd_id,
                 ff_msgs::AckCompletedStatus::EXEC_FAILED,
                 "Data to disk file not found.");
  return false;
}

bool Executive::SetEnableAutoReturn(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set enable auto return command!");
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
    NODELET_ERROR("Malformed arguments for enable auto return command!");
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                   "Malformed arguments for enable auto return command!");
    return false;
  }

  agent_state_.auto_return_enabled = cmd->args[0].b;
  PublishAgentState();
  state_->AckCmd(cmd->cmd_id);
  return true;
}

bool Executive::SetEnableImmediate(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set enable immediate command!");
  if (CheckNotMoving(cmd)) {
    if (cmd->args.size() != 1 ||
        cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
      NODELET_ERROR("Malformed arguments for enable immediate command!");
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                     "Malformed arguments for enable immediate command!");
      return false;
    }

    agent_state_.immediate_enabled = cmd->args[0].b;
    PublishAgentState();
    state_->AckCmd(cmd->cmd_id);
    return true;
  }
  return false;
}

bool Executive::SetEnableReplan(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set enable replan command!");
  if (CheckNotMoving(cmd)) {
    if (cmd->args.size() != 1 ||
        cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
      NODELET_ERROR("Malformed arguments for enable replan command!");
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                     "Malformed arguments for enable replan command!");
      return false;
    }

    agent_state_.replanning_enabled = cmd->args[0].b;
    PublishAgentState();
    state_->AckCmd(cmd->cmd_id);
    return true;
  }
  return false;
}

bool Executive::SetFlashlightBrightness(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set flashlight brightness command!");
  bool successful = true;
  uint8_t completed_status = ff_msgs::AckCompletedStatus::OK;
  std::string err_msg = "";

  // Check to make sure command is formatted as expected
  if (cmd->args.size() != 2 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
      cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT) {
    successful = false;
    err_msg = "Malformed arguments for set flashlight brightness command!";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else if (cmd->args[0].s !=
             CommandConstants::PARAM_NAME_FLASHLIGHT_LOCATION_BACK &&
             cmd->args[0].s !=
             CommandConstants::PARAM_NAME_FLASHLIGHT_LOCATION_FRONT) {
    successful = false;
    err_msg = "Flashlight location not recognized. Must be Front or Back.";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else if (cmd->args[1].f < 0 || cmd->args[1].f > 1) {
    successful = false;
    err_msg = "Flashlight brightness must be a value between 0 and 1.";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else {
    ff_hw_msgs::SetFlashlight flashlight_srv;
    // Flashlight brightness needs to be a value between 0 and 200
    flashlight_srv.request.brightness = 200 * cmd->args[1].f;
    if (cmd->args[0].s ==
                        CommandConstants::PARAM_NAME_FLASHLIGHT_LOCATION_BACK) {
      if (!back_flashlight_client_.exists()) {
        successful = false;
        err_msg = "Back flashlight control service isn't running.";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        if (!back_flashlight_client_.call(flashlight_srv)) {
          successful = false;
          err_msg = "Back flashlight service returned false.";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }

        // Check to see if flashlight brightness was successfully set
        if (successful && !flashlight_srv.response.success) {
          successful = false;
          err_msg = flashlight_srv.response.status_message;
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    } else {
      if (!front_flashlight_client_.exists()) {
        successful = false;
        err_msg = "Front flashlight control service isn't running.";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else {
        if (!front_flashlight_client_.call(flashlight_srv)) {
          successful = false;
          err_msg = "Front flashlight service returned false.";
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }

        // Check to see if flashlight brightness was successfully set
        if (successful && !flashlight_srv.response.success) {
          successful = false;
          err_msg = flashlight_srv.response.status_message;
          completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
        }
      }
    }
  }

  state_->AckCmd(cmd->cmd_id, completed_status, err_msg);
  return successful;
}

bool Executive::SetHolonomicMode(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set holonomic mode command!");
  if (CheckNotMoving(cmd)) {
    if (cmd->args.size() != 1 ||
        cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_BOOL) {
      NODELET_ERROR("Malformed arguments for set holonomic mode command!");
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                     "Malformed arguments for set holonomic mode command!");
      return false;
    }

    agent_state_.holonomic_enabled = cmd->args[0].b;
    PublishAgentState();
    state_->AckCmd(cmd->cmd_id);
    return true;
  }
  return false;
}

bool Executive::SetInertia(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set inertia command!");
  if (CheckNotMoving(cmd)) {
    if (cmd->args.size() != 4 ||
        cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
        cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT ||
        cmd->args[2].data_type != ff_msgs::CommandArg::DATA_TYPE_VEC3d ||
        cmd->args[3].data_type != ff_msgs::CommandArg::DATA_TYPE_MAT33f) {
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                     "Malformed arguments for set inertia command!");
      return false;
    }

    ff_msgs::SetInertia inertia_srv;
    // Inertia profile name
    inertia_srv.request.inertia.header.frame_id = cmd->args[0].s;
    // Mass
    inertia_srv.request.inertia.inertia.m = cmd->args[1].f;
    // Center of mass
    inertia_srv.request.inertia.inertia.com.x = cmd->args[2].vec3d[0];
    inertia_srv.request.inertia.inertia.com.y = cmd->args[2].vec3d[1];
    inertia_srv.request.inertia.inertia.com.z = cmd->args[2].vec3d[2];
    // Inertia Tensor
    //      | ixx ixy ixz |
    // I =  | ixy iyy iyz |
    //      | ixz iyz izz |
    inertia_srv.request.inertia.inertia.ixx = cmd->args[3].mat33f[0];
    inertia_srv.request.inertia.inertia.ixy = cmd->args[3].mat33f[1];
    inertia_srv.request.inertia.inertia.ixz = cmd->args[3].mat33f[2];
    inertia_srv.request.inertia.inertia.iyy = cmd->args[3].mat33f[4];
    inertia_srv.request.inertia.inertia.iyz = cmd->args[3].mat33f[5];
    inertia_srv.request.inertia.inertia.izz = cmd->args[3].mat33f[8];

    if (!CheckServiceExists(set_inertia_client_, "Set inertia", cmd->cmd_id)) {
      return false;
    }

    if (!set_inertia_client_.call(inertia_srv)) {
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::EXEC_FAILED,
                     "Set inertia service returned false!");
      return false;
    }

    if (!inertia_srv.response.success) {
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::EXEC_FAILED,
                    "Set inertia service returned unsuccessful.");
      return false;
    }

    // Inertia call was successful
    state_->AckCmd(cmd->cmd_id);
    return true;
  }
  return false;
}

bool Executive::SetOperatingLimits(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set operating limits command!");
  if (CheckNotMoving(cmd)) {
    if (cmd->args.size() != 7 ||
        cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
        cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
        cmd->args[2].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT  ||
        cmd->args[3].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT  ||
        cmd->args[4].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT  ||
        cmd->args[5].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT  ||
        cmd->args[6].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT) {
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                     "Malformed arguments for set operating limits command!");
      return false;
    }

    // Check to make sure the flight mode exists before setting everything
    ff_msgs::FlightMode mode;
    if (!ff_util::FlightUtil::GetFlightMode(mode, cmd->args[1].s)) {
      std::string err_msg = "Flight mode " + cmd->args[1].s +" doesn't exist!.";
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                     err_msg);
      return false;
    }

    // string profile name
    agent_state_.profile_name = cmd->args[0].s;
    // string flight mode
    agent_state_.flight_mode = cmd->args[1].s;
    // target linear velocity
    agent_state_.target_linear_velocity = cmd->args[2].f;
    // target linear acceleration
    agent_state_.target_linear_accel = cmd->args[3].f;
    // target angular velocity
    agent_state_.target_angular_velocity = cmd->args[4].f;
    // target angular acceleration
    agent_state_.target_angular_accel = cmd->args[5].f;
    // collision distance
    agent_state_.collision_distance = cmd->args[6].f;

    PublishAgentState();

    state_->AckCmd(cmd->cmd_id);

    return true;
  }
  return false;
}

bool Executive::SetPlan(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set plan command!");
  std::string err_msg;
  if (plan_) {
    if (sequencer::LoadPlan(plan_, &sequencer_)) {
      // Set plan execution state to paused, apparently this was the way
      // spheres worked
      SetPlanExecState(ff_msgs::ExecState::PAUSED);
      // Publish plan stuff for ground
      PublishPlan();
      PublishPlanStatus(ff_msgs::AckStatus::QUEUED);
      // Clear plan so that the operator has to upload a new plan after this
      // plan is done running
      plan_.reset();
      state_->AckCmd(cmd->cmd_id);
      return true;
    }
    plan_.reset();
    err_msg = "Invalid syntax in uploaded plan!";
  } else {
    err_msg = "No plan found! Plan must have failed to upload.";
  }
  SetPlanExecState(ff_msgs::ExecState::IDLE);
  state_->AckCmd(cmd->cmd_id,
                 ff_msgs::AckCompletedStatus::EXEC_FAILED,
                 err_msg);
  return false;
}

bool Executive::SetPlanner(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set planner command!");
  // Don't set planner when moving
  if (CheckNotMoving(cmd)) {
    if (cmd->args.size() != 1 ||
        cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
      NODELET_ERROR("Malformed arguments for set planner command!");
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                     "Malformed arguments for set planner command!");
      return false;
    }

    // Check that the planner string is valid
    if (cmd->args[0].s != CommandConstants::PARAM_NAME_PLANNER_TYPE_TRAPEZOIDAL
        && cmd->args[0].s !=
                CommandConstants::PARAM_NAME_PLANNER_TYPE_QUADRATIC_PROGRAM) {
      NODELET_ERROR("Planner must be either Trapezoidal or QuadraticProgram");
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                     "Planner must be either Trapezoidal or QuadraticProgram");
      return false;
    }

    agent_state_.planner = cmd->args[0].s;
    PublishAgentState();
    state_->AckCmd(cmd->cmd_id);
    return true;
  }
  return false;
}

bool Executive::SetTelemetryRate(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set telemetry rate command!");
  if (cmd->args.size() != 2 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING ||
      cmd->args[1].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT) {
    state_->AckCmd(cmd->cmd_id,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  "Malformed arguments for set telemetry rate!");
    return false;
  }

  ff_msgs::SetRate set_rate_srv;
  set_rate_srv.request.rate = cmd->args[1].f;
  if (cmd->args[0].s ==
                      CommandConstants::PARAM_NAME_TELEMETRY_TYPE_COMM_STATUS) {
    set_rate_srv.request.which = ff_msgs::SetRate::Request::COMM_STATUS;
  } else if (cmd->args[0].s ==
                        CommandConstants::PARAM_NAME_TELEMETRY_TYPE_CPU_STATE) {
    set_rate_srv.request.which = ff_msgs::SetRate::Request::CPU_STATE;
  } else if (cmd->args[0].s ==
                      CommandConstants::PARAM_NAME_TELEMETRY_TYPE_DISK_STATE) {
    set_rate_srv.request.which = ff_msgs::SetRate::Request::DISK_STATE;
  } else if (cmd->args[0].s ==
                        CommandConstants::PARAM_NAME_TELEMETRY_TYPE_EKF_STATE) {
    set_rate_srv.request.which = ff_msgs::SetRate::Request::EKF_STATE;
  } else if (cmd->args[0].s ==
                        CommandConstants::PARAM_NAME_TELEMETRY_TYPE_GNC_STATE) {
    set_rate_srv.request.which = ff_msgs::SetRate::Request::GNC_STATE;
  } else if (cmd->args[0].s ==
                    CommandConstants::PARAM_NAME_TELEMETRY_TYPE_PMC_CMD_STATE) {
    set_rate_srv.request.which = ff_msgs::SetRate::Request::PMC_CMD_STATE;
  } else if (cmd->args[0].s ==
                        CommandConstants::PARAM_NAME_TELEMETRY_TYPE_POSITION) {
    set_rate_srv.request.which = ff_msgs::SetRate::Request::POSITION;
  } else if (cmd->args[0].s ==
              CommandConstants::PARAM_NAME_TELEMETRY_TYPE_SPARSE_MAPPING_POSE) {
    set_rate_srv.request.which = ff_msgs::SetRate::Request::SPARSE_MAPPING_POSE;
  } else {
    state_->AckCmd(cmd->cmd_id,
                  ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                  "Unknown name in set telemetry rate command!");
    return false;
  }

  if (!CheckServiceExists(set_rate_client_, "Set telem rate", cmd->cmd_id)) {
    return false;
  }

  if (!set_rate_client_.call(set_rate_srv)) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   "Set rate service returned false.");
    return false;
  }

  // Check to see if the rate was set successfully
  if (!set_rate_srv.response.success) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   set_rate_srv.response.status);
    return false;
  }

  state_->AckCmd(cmd->cmd_id);
  return true;
}

bool Executive::SetZones(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing set zones command!");
  if (CheckNotMoving(cmd)) {
    if (zones_) {
      ff_msgs::SetZones zones_srv;
      std::string file_contents;

      // Decompress file into a string
      if (!sequencer::DecompressData(
                          reinterpret_cast<const char*>(zones_->file.data()),
                          zones_->file.size(), zones_->type, &file_contents)) {
        // Reset zones so that the same file isn't reloaded
        zones_.reset();
        state_->AckCmd(cmd->cmd_id,
                       ff_msgs::AckCompletedStatus::EXEC_FAILED,
                       "Unable to decompress zones file.");
        return false;
      }

      // Reset zones so that the same file isn't reloaded
      zones_.reset();

      // Convert string into a json object
      Json::Value file_obj;
      if (!jsonloader::LoadData(file_contents, &file_obj)) {
        state_->AckCmd(cmd->cmd_id,
                       ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                       "Error parsing json.");
        return false;
      }

      // Check to make sure timestamp exists in the file
      if (!file_obj.isMember("timestamp") ||
          !file_obj["timestamp"].isString()) {
        state_->AckCmd(cmd->cmd_id,
                       ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                       "File timestamp doesn't exist or is not a string.");
        return false;
      }

      // Get timestamp in milliseconds and convert it to a number
      std::string timestamp = file_obj["timestamp"].asString();
      zones_srv.request.timestamp = MsToSec(timestamp);

      // Check to make sure zones array exists
      if (!file_obj.isMember("zones") || !file_obj["zones"].isArray()) {
        state_->AckCmd(cmd->cmd_id,
                       ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                       "Parser error: zones don't exist or are not an array!");
        return false;
      }

      std::string err_msg;
      ff_msgs::Zone zone;
      int i = 0;
      for (Json::Value const& zone_obj : file_obj["zones"]) {
        // Check to make sure zone name exists
        if (!zone_obj.isMember("name") || !zone_obj["name"].isString()) {
          state_->AckCmd(cmd->cmd_id,
                         ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                         "Name in zone doesn't exist or is not a string.");
          return false;
        }
        zone.name = zone_obj["name"].asString();

        // Check to make sure safe exists for zone
        if (!zone_obj.isMember("safe") || !zone_obj["safe"].isBool()) {
          state_->AckCmd(cmd->cmd_id,
                         ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                         "Safe flag in zone doesn't exist or is not a boolean");
          return false;
        }

        if (zone_obj["safe"].asBool()) {
          zone.type = ff_msgs::Zone::KEEPIN;
        } else {
          zone.type = ff_msgs::Zone::KEEPOUT;
        }

        // Check to make sure the sequence exists for the zone
        if (!zone_obj.isMember("sequence") || !zone_obj["sequence"].isArray()) {
          state_->AckCmd(cmd->cmd_id,
                         ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                         "Sequence in zone doesn't exist or isn't an array.");
          return false;
        }

        i = 0;
        for (Json::Value const& box_array : zone_obj["sequence"]) {
          zone.index = i;
          if (!box_array.isArray() || box_array.size() != 6) {
            state_->AckCmd(cmd->cmd_id,
                           ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                           "Box isn't an array or doesn't have 6 points.");
            return false;
          }

          if (!box_array[0].isNumeric() || !box_array[1].isNumeric() ||
              !box_array[2].isNumeric() || !box_array[3].isNumeric() ||
              !box_array[4].isNumeric() || !box_array[5].isNumeric()) {
            state_->AckCmd(cmd->cmd_id,
                           ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                           "One of the box points in zone is not numeric!");
            return false;
          }

          // First 3 elements are the x, y, z of one corner and the last three
          // are the x, y, z of the other corner
          zone.min.x = box_array[0].asFloat();
          zone.min.y = box_array[1].asFloat();
          zone.min.z = box_array[2].asFloat();
          zone.max.x = box_array[3].asFloat();
          zone.max.y = box_array[4].asFloat();
          zone.max.z = box_array[5].asFloat();

          zones_srv.request.zones.push_back(zone);
          i++;
        }
      }

      // Check to make sure the service is valid and running
      if (!zones_client_.exists()) {
        state_->AckCmd(cmd->cmd_id,
                ff_msgs::AckCompletedStatus::EXEC_FAILED,
                "Set zones service isn't running! Choreographer may have died");
        return false;
      }

      if (!zones_client_.call(zones_srv)) {
        state_->AckCmd(cmd->cmd_id,
                       ff_msgs::AckCompletedStatus::EXEC_FAILED,
                       "Set zones service returned false.");
        return false;
      }

      if (!zones_srv.response.success) {
        state_->AckCmd(cmd->cmd_id,
                       ff_msgs::AckCompletedStatus::EXEC_FAILED,
                       "Set zones was not successful.");
        return false;
      }

      state_->AckCmd(cmd->cmd_id);
      return true;
    }
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   "No zones file found.");
    return false;
  }
  return false;
}

bool Executive::SkipPlanStep(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing skip plan step command!");
  // Make sure plan execution state is paused
  if (agent_state_.plan_execution_state.state != ff_msgs::ExecState::PAUSED) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   "Got command to skip a plan step but plan isn't paused.");
    return false;
  }

  ff_msgs::AckCompletedStatus ack;
  ack.status = ff_msgs::AckCompletedStatus::CANCELED;
  // Check to see if we are skipping the last step in the plan
  if (sequencer_.Feedback(ack)) {
    PublishPlanStatus(ff_msgs::AckStatus::QUEUED);
  } else {
    PublishPlanStatus(ff_msgs::AckStatus::COMPLETED);
    SetPlanExecState(ff_msgs::ExecState::IDLE);
  }
  state_->AckCmd(cmd->cmd_id);

  return true;
}

bool Executive::StartGuestScience(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing start guest science!");
  // Check command arguments are correct before sending to the guest science
  // manager
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                   "Malformed arguments for start guest science command.");
    return false;
  }

  if (guest_science_config_ == NULL) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   "Executive never got GS config. GS manager may have died.");
    return false;
  }

  if (gs_start_stop_cmd_id_ != "") {
    std::string msg = "Already executing a start or stop guest science command";
    msg += ". Please wait for it to finish before issuing another start guest ";
    msg += "science command.";
    state_->AckCmd(cmd->cmd_id, ff_msgs::AckCompletedStatus::EXEC_FAILED, msg);
    return false;
  }

  // Check to see if the operator is trying to start a primary apk
  for (unsigned int i = 0; i < guest_science_config_->apks.size(); i++) {
    if (cmd->args[0].s == guest_science_config_->apks[i].apk_name) {
      if (guest_science_config_->apks[i].primary) {
        // We cannot start a primary apk if another primary apk is running or
        // if we are executing a plan or teleop command. However we can start
        // a primary apk if it is a plan command
        if (primary_apk_running_ != "None") {
          state_->AckCmd(cmd->cmd_id,
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
                         "Can't start primary apk when one is already running");
          return false;
        } else if ((state_->id() == ff_msgs::OpState::PLAN_EXECUTION &&
                    cmd->cmd_id != "plan" && cmd->cmd_src != "plan") ||
                    state_->id() == ff_msgs::OpState::TELEOPERATION) {
          state_->AckCmd(cmd->cmd_id,
                         ff_msgs::AckCompletedStatus::EXEC_FAILED,
                         "Must be in the ready state to start a primary apk");
          return false;
        }
        break;
      }
    }
  }

  // Don't worry if an apk is not in the config message, the guest science
  // manager will take care of this
  gs_cmd_pub_.publish(cmd);
  gs_start_stop_command_timer_.setPeriod(ros::Duration(gs_command_timeout_));
  gs_start_stop_command_timer_.start();
  gs_start_stop_cmd_id_ = cmd->cmd_id;
  return true;
}

bool Executive::StartRecording(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing start recordiing command.");
  bool successful = true;
  std::string err_msg;
  uint8_t completed_status = ff_msgs::AckCompletedStatus::OK;
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
    successful = false;
    err_msg = "Malformed arguments for start recordinng command.";
    completed_status = ff_msgs::AckCompletedStatus::BAD_SYNTAX;
  } else {
    ff_msgs::EnableRecording enable_rec_srv;
    enable_rec_srv.request.enable = true;
    enable_rec_srv.request.bag_description = cmd->args[0].s;

    // Check to make sure the enable recording service exists
    if (!enable_recording_client_.exists()) {
      successful = false;
      err_msg = "Enable recording service not running! Node may have died!";
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
    } else {
      // Call enable service and make sure it worked
      if (!enable_recording_client_.call(enable_rec_srv)) {
        successful = false;
        err_msg = "Enable recording service failed!";
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      } else if (!enable_rec_srv.response.success) {
        successful = false;
        err_msg = enable_rec_srv.response.status;
        completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
      }
    }
  }
  state_->AckCmd(cmd->cmd_id, completed_status, err_msg);
  return successful;
}

// Stop all motion is a tricky command since we may have multiple actions
// running at one time. We also use stop to transition from idle to stopped
// so we will wanted to start a stop pretty much all the time. The only time
// that we don't send a stop is when we are already trying to stop, trying to
// idle, docked, perched, docking and deactiviting the pmcs, or undocking and
// haven't activated the pmcs yet.
// This function is also as pause for a plan so if the plan flag is set, we
// need to check if we are downloading data and if so, stop it.
bool Executive::StopAllMotion(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing stop all motion command!");
  // We pretty much always start stop action even if stopped. See cases below
  // for situations we don't want to stop in
  bool successful = true, start_stop = true;
  std::string err_msg;

  // The first thing we need to check is if the stop was a fault response and if
  // it is, we need to check if mobility is idle. If mobility is idle, we don't
  // want to spin up the pmcs by excuting a stop
  if (cmd->cmd_src == "sys_monitor") {
    if (agent_state_.mobility_state.state == ff_msgs::MobilityState::DRIFTING) {
      // Ack command as failed and don't stop
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::EXEC_FAILED,
                     "PMCs are idle so don't want spin them up due to a fault");
      return false;
    }
  }

  // If an action is executing, we want to cancel it. We will not wait for a
  // result as not waiting simplifies the code so start a stop after canceling
  // the running actions.

  // If we are perched or docked, we will be idle and don't want to
  // spin up the motors to try to stop so we need to fail the stop command in
  // this case
  if (agent_state_.mobility_state.state == ff_msgs::MobilityState::DOCKING &&
      agent_state_.mobility_state.sub_state == 0) {
    err_msg = "Astrobee cannot stop while docked.";
    start_stop = false;
    successful = false;
    // Will check if we started the undock action but haven't received any
    // feedback in main for loop
  } else if (agent_state_.mobility_state.state ==
                                            ff_msgs::MobilityState::PERCHING &&
             agent_state_.mobility_state.sub_state == 0) {
    err_msg = "Astrobee cannot stop while perched.";
    start_stop = false;
    successful = false;
    // Will check if we started the unperch action but haven't received any
    // feedback in main for loop. Will also check if we are executing arm action
    // in main loop
  }

  // Stop all motion stops both mobility and the arm so we need to check if
  // both are in use
  unsigned int i = 0;
  for (i = 0; i < running_actions_.size(); i++) {
    if (running_actions_[i] == ARM) {
      // Arm node doesn't have a stop, cancelling the current goal will stop the
      // arm
      CancelAction(ARM, "stop");
      i--;
      // Set successful to true since it may have been set to false in perch
      // check
      successful = true;
    } else if (running_actions_[i] == DOCK) {
      // Don't stop if we are deactivating the PMCs or already docked but
      // switching localiization
      if (agent_state_.mobility_state.sub_state <=
                        ff_msgs::DockState::DOCKING_WAITING_FOR_SPIN_DOWN) {
        err_msg = "Already deactivating pmcs or docked. Cannot stop.";
        start_stop = false;
        successful = false;
      } else {
        CancelAction(DOCK, "stop");
        i--;
      }
    } else if (running_actions_[i] == EXECUTE) {
      CancelAction(EXECUTE, "stop");
      i--;
    } else if (running_actions_[i] == IDLE) {
      err_msg = "Cannot stop while trying to idle.";
      successful = false;
      start_stop = false;
    } else if (running_actions_[i] == MOVE) {
      CancelAction(MOVE, "stop");
      i--;
    } else if (running_actions_[i] == PERCH) {
      // Don't stop if we are deactivating the PMCs or already perched but
      // switching localization
      if (agent_state_.mobility_state.sub_state <=
                        ff_msgs::PerchState::PERCHING_WAITING_FOR_SPIN_DOWN) {
        err_msg = "Already deactivating the pmcs or perched. Cannot stop.";
        start_stop = false;
        successful = false;
      } else {
        CancelAction(PERCH, "stop");
        i--;
      }
    } else if (running_actions_[i] == STOP) {
      err_msg = "Already stopping.";
      successful = false;
      start_stop = false;
    } else if (running_actions_[i] == UNDOCK) {
      CancelAction(UNDOCK, "stop");
      i--;
      // Figure out where we are in the undock process and only send a stop if
      // we are in or have completed the egressing state
      if (agent_state_.mobility_state.sub_state >
                        ff_msgs::DockState::UNDOCKING_MOVING_TO_APPROACH_POSE) {
        start_stop = false;
        // Set successful to true since it may have been set to false in the
        // if docked statement
        successful = true;
        // Set mobility state to docked
        agent_state_.mobility_state.sub_state = 0;
      } else {  // Off dock so we need to send a stop
        // Set successful and start stop to true since it may have been set to
        // false in the if docked checked
        successful = true;
        start_stop = true;
      }
    } else if (running_actions_[i] == UNPERCH) {
      CancelAction(UNPERCH, "stop");
      i--;
      // Figure out where we are in the unperching process and only send a stop
      // if we might not be perched any more.
      if (agent_state_.mobility_state.sub_state >
                              ff_msgs::PerchState::UNPERCHING_OPENING_GRIPPER) {
        start_stop = false;
        // Set successful to true since it may have been set to false in the if
        // perched statement
        successful = true;
        // Set mobility state to perched
        agent_state_.mobility_state.sub_state = 0;
      } else {  // Off handle rail so we need to send a stop
        // Set successful and start stop to true since it may have been set to
        // false in the if perched checked
        successful = true;
        start_stop = true;
      }
    }
  }

  if (cmd->cmd_src == "plan") {
    // TODO(Katie) Check to see if we are downloading data and cancel the
    // action. Only check this for the plan because a pause and stop
    // command are the same and if a operator has started the download in
    // the teleop tab then they can find the stop download button in the
    // teleop tab
  }

  // Ack before start action since start action will ack for us if it fails
  if (!successful) {
    NODELET_ERROR("%s", err_msg.c_str());
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   err_msg);
  } else if (successful && !start_stop) {
    // Ack successful since we cancelled an action that hadn't moved the robot
     state_->AckCmd(cmd->cmd_id);
  }

  if (start_stop) {
    if (FillMotionGoal(STOP)) {
      successful = StartAction(STOP, cmd->cmd_id);
    }
  }

  return successful;
}

bool Executive::StopArm(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing stop arm command!");
  // Check for an arm action already being executed. If so, cancel it.
  if (IsActionRunning(ARM)) {
    CancelAction(ARM, "stop arm");
  }

  if (!FillArmGoal(cmd)) {
    return false;
  }

  if (!StartAction(ARM, cmd->cmd_id)) {
    return false;
  }

  return true;
}

bool Executive::StopGuestScience(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing stop guest science command!");
  // Check command arguments are correct before sending to the guest science
  // manager
  if (cmd->args.size() != 1 ||
      cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                   "Malformed arguments for stop guest science command.");
    return false;
  }

  if (gs_start_stop_cmd_id_ != "") {
    std::string msg = "Already executing a start or stop guest science command";
    msg += ". Please wait for it to finish before issuing another start guest ";
    msg += "science command.";
    state_->AckCmd(cmd->cmd_id, ff_msgs::AckCompletedStatus::EXEC_FAILED, msg);
    return false;
  }

  gs_cmd_pub_.publish(cmd);
  gs_start_stop_command_timer_.setPeriod(ros::Duration(gs_command_timeout_));
  gs_start_stop_command_timer_.start();
  gs_start_stop_cmd_id_ = cmd->cmd_id;
  return true;
}

bool Executive::StopRecording(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing stop recording command!");
  bool successful = true;
  std::string err_msg;
  uint8_t completed_status = ff_msgs::AckCompletedStatus::OK;

  ff_msgs::EnableRecording enable_rec_srv;
  enable_rec_srv.request.enable = false;

  // Check to make sure the enable recording service exists
  if (!enable_recording_client_.exists()) {
    successful = false;
    err_msg = "Enable recording service not running! Node may have died!";
    completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
  } else {
    // Call enable service and make sure it worked
    if (!enable_recording_client_.call(enable_rec_srv)) {
      successful = false;
      err_msg = "Enable recording service failed!";
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
    } else if (!enable_rec_srv.response.success) {
      successful = false;
      err_msg = enable_rec_srv.response.status;
      completed_status = ff_msgs::AckCompletedStatus::EXEC_FAILED;
    }
  }

  state_->AckCmd(cmd->cmd_id, completed_status, err_msg);
  return successful;
}

bool Executive::StowArm(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing stow arm command!");
  // Check if Astrobee is perched. Arm control will check the rest.
  if (agent_state_.mobility_state.state == ff_msgs::MobilityState::PERCHING) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   "Can't stow arm while perched or (un)perching!");
    return false;
  }

  return ArmControl(cmd);
}

bool Executive::SwitchLocalization(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_DEBUG("Executive executing switch localization command!");
  if (CheckNotMoving(cmd)) {
    if (cmd->args.size() != 1 ||
        cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_STRING) {
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                     "Malformed arguments for switch localization!");
      return false;
    }

    localization_goal_.command =
                            ff_msgs::LocalizationGoal::COMMAND_SWITCH_PIPELINE;
    if (cmd->args[0].s == CommandConstants::PARAM_NAME_LOCALIZATION_MODE_NONE) {
      localization_goal_.pipeline = ff_msgs::LocalizationGoal::PIPELINE_NONE;
    } else if (cmd->args[0].s ==
              CommandConstants::PARAM_NAME_LOCALIZATION_MODE_MAPPED_LANDMARKS) {
      localization_goal_.pipeline =
                              ff_msgs::LocalizationGoal::PIPELINE_MAP_LANDMARKS;
    } else if (cmd->args[0].s ==
                        CommandConstants::PARAM_NAME_LOCALIZATION_MODE_ARTAGS) {
      localization_goal_.pipeline = ff_msgs::LocalizationGoal::PIPELINE_AR_TAGS;
    } else if (cmd->args[0].s ==
                      CommandConstants::PARAM_NAME_LOCALIZATION_MODE_HANDRAIL) {
      localization_goal_.pipeline =
                                  ff_msgs::LocalizationGoal::PIPELINE_HANDRAIL;
    } else if (cmd->args[0].s ==
                        CommandConstants::PARAM_NAME_LOCALIZATION_MODE_PERCH) {
      localization_goal_.pipeline = ff_msgs::LocalizationGoal::PIPELINE_PERCH;
    } else if (cmd->args[0].s ==
                        CommandConstants::PARAM_NAME_LOCALIZATION_MODE_TRUTH) {
      localization_goal_.pipeline = ff_msgs::LocalizationGoal::PIPELINE_TRUTH;
    }
    return StartAction(LOCALIZATION, cmd->cmd_id);
  }
  return false;
}

bool Executive::Undock(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing undock command!");
  bool docked = false;
  std::string err_msg = "";

  // Make sure robot is docked before attempting to undock. Only accept undock
  // in the ready op state so only need to check perched, stopped, or drifting
  if (agent_state_.mobility_state.state == ff_msgs::MobilityState::DRIFTING) {
    err_msg = "Can't undock when not docked. Astrobee is currently drifting.";
  } else if (agent_state_.mobility_state.state ==
                                            ff_msgs::MobilityState::PERCHING) {
    err_msg = "Can't undock when not docked. Astrobee is currently perched.";
  } else if (agent_state_.mobility_state.state ==
                                            ff_msgs::MobilityState::STOPPING) {
    err_msg = "Can't undock when not docked. Astrobee is currently stopped.";
  } else {
    docked = true;
    if (!FillDockGoal(cmd, false)) {
      return false;
    }

    if (!StartAction(UNDOCK, cmd->cmd_id)) {
      return false;
    }
  }

  // Fill dock goal and start action publishes failed command acks so we only
  // need to fail an ack if we aren't docked and thus cannot undock
  if (!docked) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   err_msg);
  }
  return docked;
}

bool Executive::UnloadNodelet(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing unload nodelet command!");
  return LoadUnloadNodelet(cmd);
}

bool Executive::Unperch(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing unperch command!");
  bool perched = false;
  std::string err_msg = "";

  // Make sure robot is perched before attempting to unperch. Only accept
  // unperch in the ready op state so only need to check docked, stopped, or
  // drifting
  if (agent_state_.mobility_state.state == ff_msgs::MobilityState::DOCKING) {
    err_msg = "Can't unperch when not perched. Astrobee is currently docked.";
  } else if (agent_state_.mobility_state.state ==
                                            ff_msgs::MobilityState::DRIFTING) {
    err_msg = "Can't unperch when not perched. Astrobeep is currently drifting";
  } else if (agent_state_.mobility_state.state ==
                                            ff_msgs::MobilityState::STOPPING) {
    err_msg = "Can't unperch when not perched. Astrobee is currently stopped.";
  } else {
    perched = true;
    perch_goal_.command = ff_msgs::PerchGoal::UNPERCH;

    if (!StartAction(UNPERCH, cmd->cmd_id)) {
      return false;
    }
  }

  // Start action publishes failed command acks so we only need to fail an ack
  // if we aren't perched and thus cannot unperch
  if (!perched) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   err_msg);
  }
  return perched;
}

bool Executive::Unterminate(ff_msgs::CommandStampedPtr const& cmd) {
  ff_hw_msgs::ClearTerminate clear_srv;

  // Clear eps terminate flag
  if (!CheckServiceExists(eps_terminate_client_,
                          "EPS terminate",
                          cmd->cmd_id)) {
    return false;
  }

  if (eps_terminate_client_.call(clear_srv)) {
    if (!clear_srv.response.success) {
      state_->AckCmd(cmd->cmd_id,
                     ff_msgs::AckCompletedStatus::EXEC_FAILED,
                     clear_srv.response.status_message);
      return false;
    }
  } else {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::EXEC_FAILED,
                   "Eps clear terminate service returned false.");
    return false;
  }

  state_->AckCmd(cmd->cmd_id);
  return true;
}

bool Executive::Wait(ff_msgs::CommandStampedPtr const& cmd) {
  NODELET_INFO("Executive executing wait command! Duration %f", cmd->args[0].f);
  if (cmd->args[0].data_type != ff_msgs::CommandArg::DATA_TYPE_FLOAT ||
      cmd->args[0].f < 0) {
    state_->AckCmd(cmd->cmd_id,
                   ff_msgs::AckCompletedStatus::BAD_SYNTAX,
                   "Malformed arguments for wait command!");
    return false;
  }

  StartWaitTimer(cmd->args[0].f);
  return true;
}

/************************ Protected *******************************************/
void Executive::Initialize(ros::NodeHandle *nh) {
  std::string err_msg;
  // Set executive in op state repo so the op_states can call this executive
  OpStateRepo::Instance()->SetExec(this);

  nh_ = *nh;

  // Read in all the action timeouts. They are in config files so that they can
  // be changed on the fly.
  config_params_.AddFile("management/executive.config");
  config_params_.AddFile("management/sys_monitor_fault_info.config");
  if (!ReadParams()) {
    return;
  }

  // Set up a timer to check and reload timeouts if they are changed.
  reload_params_timer_ = nh_.createTimer(ros::Duration(1),
      [this](ros::TimerEvent e) {
          config_params_.CheckFilesUpdated(std::bind(&Executive::ReadParams,
                                                                      this));},
      false, true);

  // initialize actions
  arm_ac_.SetActiveTimeout(action_active_timeout_);
  arm_ac_.SetResponseTimeout(arm_feedback_timeout_);
  arm_ac_.SetResultCallback(std::bind(&Executive::ArmResultCallback,
                            this,
                            std::placeholders::_1,
                            std::placeholders::_2));
  arm_ac_.Create(nh, ACTION_BEHAVIORS_ARM);

  dock_ac_.SetActiveTimeout(action_active_timeout_);
  dock_ac_.SetDeadlineTimeout(dock_result_timeout_);
  dock_ac_.SetResultCallback(std::bind(&Executive::DockResultCallback,
                             this,
                             std::placeholders::_1,
                             std::placeholders::_2));
  dock_ac_.Create(nh, ACTION_BEHAVIORS_DOCK);

  localization_ac_.SetActiveTimeout(action_active_timeout_);
  localization_ac_.SetDeadlineTimeout(localization_result_timeout_);
  localization_ac_.SetResultCallback(
                              std::bind(&Executive::LocalizationResultCallback,
                              this,
                              std::placeholders::_1,
                              std::placeholders::_2));
  localization_ac_.Create(nh, ACTION_LOCALIZATION_MANAGER_LOCALIZATION);

  motion_ac_.SetActiveTimeout(action_active_timeout_);
  motion_ac_.SetResponseTimeout(motion_feedback_timeout_);
  motion_ac_.SetFeedbackCallback(std::bind(&Executive::MotionFeedbackCallback,
                                 this,
                                 std::placeholders::_1));
  motion_ac_.SetResultCallback(std::bind(&Executive::MotionResultCallback,
                               this,
                               std::placeholders::_1,
                               std::placeholders::_2));
  motion_ac_.Create(nh, ACTION_MOBILITY_MOTION);

  perch_ac_.SetActiveTimeout(action_active_timeout_);
  perch_ac_.SetDeadlineTimeout(perch_result_timeout_);
  perch_ac_.SetResultCallback(std::bind(&Executive::PerchResultCallback,
                              this,
                              std::placeholders::_1,
                              std::placeholders::_2));
  perch_ac_.Create(nh, ACTION_BEHAVIORS_PERCH);

  // initialize subs
  camera_state_sub_ = nh_.subscribe(TOPIC_MANAGEMENT_CAMERA_STATE,
                                    sub_queue_size_,
                                    &Executive::CameraStatesCallback,
                                    this);

  cmd_sub_ = nh_.subscribe(TOPIC_COMMAND,
                           sub_queue_size_,
                           &Executive::CmdCallback,
                           this);

  data_sub_ = nh_.subscribe(TOPIC_COMMUNICATIONS_DDS_DATA,
                            sub_queue_size_,
                            &Executive::DataToDiskCallback,
                            this);

  dock_state_sub_ = nh_.subscribe(TOPIC_BEHAVIORS_DOCKING_STATE,
                                  sub_queue_size_,
                                  &Executive::DockStateCallback,
                                  this);

  fault_state_sub_ = nh_.subscribe(TOPIC_MANAGEMENT_SYS_MONITOR_STATE,
                                   sub_queue_size_,
                                   &Executive::FaultStateCallback,
                                   this);

  gs_ack_sub_ = nh_.subscribe(TOPIC_GUEST_SCIENCE_MANAGER_ACK,
                              sub_queue_size_,
                              &Executive::GuestScienceAckCallback,
                              this);

  gs_config_sub_ = nh_.subscribe(TOPIC_GUEST_SCIENCE_MANAGER_CONFIG,
                                 sub_queue_size_,
                                 &Executive::GuestScienceConfigCallback,
                                 this);

  gs_state_sub_ = nh_.subscribe(TOPIC_GUEST_SCIENCE_MANAGER_STATE,
                                sub_queue_size_,
                                &Executive::GuestScienceStateCallback,
                                this);

  heartbeat_sub_ = nh_.subscribe(TOPIC_MANAGEMENT_SYS_MONITOR_HEARTBEAT,
                                 sub_queue_size_,
                                 &Executive::SysMonitorHeartbeatCallback,
                                 this);


  inertia_sub_ = nh_.subscribe(TOPIC_MOBILITY_INERTIA,
                               sub_queue_size_,
                               &Executive::InertiaCallback,
                               this);

  motion_sub_ = nh_.subscribe(TOPIC_MOBILITY_MOTION_STATE,
                              sub_queue_size_,
                              &Executive::MotionStateCallback,
                              this);

  perch_state_sub_ = nh_.subscribe(TOPIC_BEHAVIORS_PERCHING_STATE,
                                   sub_queue_size_,
                                   &Executive::PerchStateCallback,
                                   this);

  plan_sub_ = nh_.subscribe(TOPIC_COMMUNICATIONS_DDS_PLAN,
                            sub_queue_size_,
                            &Executive::PlanCallback,
                            this);

  zones_sub_ = nh_.subscribe(TOPIC_COMMUNICATIONS_DDS_ZONES,
                             sub_queue_size_,
                             &Executive::ZonesCallback,
                             this);

  // initialize pubs
  agent_state_pub_ = nh_.advertise<ff_msgs::AgentStateStamped>(
                                              TOPIC_MANAGEMENT_EXEC_AGENT_STATE,
                                              pub_queue_size_,
                                              true);

  cf_ack_pub_ = nh_.advertise<ff_msgs::CompressedFileAck>(
                                                  TOPIC_MANAGEMENT_EXEC_CF_ACK,
                                                  pub_queue_size_,
                                                  false);

  cmd_ack_pub_ = nh_.advertise<ff_msgs::AckStamped>(TOPIC_MANAGEMENT_ACK,
                                                    pub_queue_size_,
                                                    false);

  gs_cmd_pub_ = nh_.advertise<ff_msgs::CommandStamped>(
                                                  TOPIC_MANAGEMENT_EXEC_COMMAND,
                                                  pub_queue_size_,
                                                  false);

  plan_pub_ = nh_.advertise<ff_msgs::CompressedFile>(TOPIC_MANAGEMENT_EXEC_PLAN,
                                                     pub_queue_size_,
                                                     false);

  plan_status_pub_ = nh_.advertise<ff_msgs::PlanStatusStamped>(
                                              TOPIC_MANAGEMENT_EXEC_PLAN_STATUS,
                                              pub_queue_size_,
                                              true);

  // initialize services
  zones_client_ = nh_.serviceClient<ff_msgs::SetZones>(
                                                    SERVICE_MOBILITY_SET_ZONES);

  laser_enable_client_ = nh_.serviceClient<ff_hw_msgs::SetEnabled>(
                                                SERVICE_HARDWARE_LASER_ENABLE);

  payload_power_client_ = nh_.serviceClient<ff_hw_msgs::ConfigurePayloadPower>(
                                      SERVICE_HARDWARE_EPS_CONF_PAYLOAD_POWER);

  pmc_enable_client_ = nh_.serviceClient<ff_hw_msgs::SetEnabled>(
                                              SERVICE_HARDWARE_EPS_ENABLE_PMCS);

  front_flashlight_client_ = nh_.serviceClient<ff_hw_msgs::SetFlashlight>(
                                          SERVICE_HARDWARE_LIGHT_FRONT_CONTROL);

  back_flashlight_client_ = nh_.serviceClient<ff_hw_msgs::SetFlashlight>(
                                          SERVICE_HARDWARE_LIGHT_AFT_CONTROL);

  dock_cam_config_client_ = nh_.serviceClient<ff_msgs::ConfigureCamera>(
                                    SERVICE_MANAGEMENT_IMG_SAMPLER_CONFIG_DOCK);

  dock_cam_enable_client_ = nh_.serviceClient<ff_msgs::EnableCamera>(
                                    SERVICE_MANAGEMENT_IMG_SAMPLER_ENABLE_DOCK);

  haz_cam_config_client_ = nh_.serviceClient<ff_msgs::ConfigureCamera>(
                                    SERVICE_MANAGEMENT_IMG_SAMPLER_CONFIG_HAZ);

  haz_cam_enable_client_ = nh_.serviceClient<ff_msgs::EnableCamera>(
                                    SERVICE_MANAGEMENT_IMG_SAMPLER_ENABLE_HAZ);

  nav_cam_config_client_ = nh_.serviceClient<ff_msgs::ConfigureCamera>(
                                    SERVICE_MANAGEMENT_IMG_SAMPLER_CONFIG_NAV);

  nav_cam_enable_client_ = nh_.serviceClient<ff_msgs::EnableCamera>(
                                    SERVICE_MANAGEMENT_IMG_SAMPLER_ENABLE_NAV);

  perch_cam_config_client_ = nh_.serviceClient<ff_msgs::ConfigureCamera>(
                                  SERVICE_MANAGEMENT_IMG_SAMPLER_CONFIG_PERCH);

  perch_cam_enable_client_ = nh_.serviceClient<ff_msgs::EnableCamera>(
                                  SERVICE_MANAGEMENT_IMG_SAMPLER_ENABLE_PERCH);

  sci_cam_config_client_ = nh_.serviceClient<ff_msgs::ConfigureCamera>(
                                            SERVICE_MANAGEMENT_SCI_CAM_CONFIG);

  sci_cam_enable_client_ = nh_.serviceClient<ff_msgs::EnableCamera>(
                                            SERVICE_MANAGEMENT_SCI_CAM_ENABLE);

  set_inertia_client_ = nh_.serviceClient<ff_msgs::SetInertia>(
                                                  SERVICE_MOBILITY_SET_INERTIA);

  set_rate_client_ = nh_.serviceClient<ff_msgs::SetRate>(
                                    SERVICE_COMMUNICATIONS_DDS_SET_TELEM_RATES);

  set_data_client_ = nh_.serviceClient<ff_msgs::SetDataToDisk>(
                              SERVICE_MANAGEMENT_DATA_BAGGER_SET_DATA_TO_DISK);

  enable_recording_client_ = nh_.serviceClient<ff_msgs::EnableRecording>(
                              SERVICE_MANAGEMENT_DATA_BAGGER_ENABLE_RECORDING);

  eps_terminate_client_ = nh_.serviceClient<ff_hw_msgs::ClearTerminate>(
                                          SERVICE_HARDWARE_EPS_CLEAR_TERMINATE);

  unload_load_nodelet_client_ = nh_.serviceClient<ff_msgs::UnloadLoadNodelet>(
                            SERVICE_MANAGEMENT_SYS_MONITOR_UNLOAD_LOAD_NODELET);

  // initialize configure clients later, when initialized here, the service is
  // invalid when we try to use it. Must have something to do with startup order
  // of executive, choreographer, planner, or mapper

  // initialize agent state
  agent_state_.operating_state.state = ff_msgs::OpState::READY;
  SetPlanExecState(ff_msgs::ExecState::IDLE);
  agent_state_.mobility_state.state = ff_msgs::MobilityState::DRIFTING;
  agent_state_.mobility_state.sub_state = 0;
  agent_state_.guest_science_state.state = ff_msgs::ExecState::IDLE;
  agent_state_.proximity = 0;
  agent_state_.profile_name = "";
  agent_state_.flight_mode = "nominal";

  // Get nominal limits
  ff_msgs::FlightMode flight_mode;
  if (!ff_util::FlightUtil::GetFlightMode(flight_mode, "nominal")) {
    err_msg = "Couldn't get flight mode nominal.";
    NODELET_ERROR("%s", err_msg.c_str());
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return;
  } else {
    agent_state_.target_linear_velocity = flight_mode.hard_limit_vel;
    agent_state_.target_linear_accel = flight_mode.hard_limit_accel;
    agent_state_.target_angular_velocity = flight_mode.hard_limit_omega;
    agent_state_.target_angular_accel = flight_mode.hard_limit_alpha;
    agent_state_.collision_distance = flight_mode.collision_radius;
  }

  agent_state_.holonomic_enabled = false;
  agent_state_.check_obstacles = true;
  agent_state_.check_zones = true;
  agent_state_.auto_return_enabled = true;
  agent_state_.immediate_enabled = true;
  agent_state_.replanning_enabled = false;
  agent_state_.boot_time = ros::Time::now().sec;

  PublishAgentState();

  // Publish blank plan status so that the GDS displays the correct plan info
  ff_msgs::PlanStatusStamped plan_status;
  plan_status.header.stamp = ros::Time::now();
  plan_status.name = "";
  plan_status.command = -1;
  plan_status_pub_.publish(plan_status);

  // Initialize camera states vector. All we care about is if we are streaming
  camera_states_.states.resize(3);
  camera_states_.states[0].camera_name = "nav_cam";
  camera_states_.states[0].streaming = false;
  camera_states_.states[1].camera_name = "dock_cam";
  camera_states_.states[1].streaming = false;
  camera_states_.states[2].camera_name = "sci_cam";
  camera_states_.states[2].streaming = false;

  // Create timer for wait command with a dummy duration since it will be
  // changed everytime it is started. Make it one shot and don't start until
  // wait command received
  wait_timer_ = nh_.createTimer(ros::Duration(1),
                                &Executive::WaitCallback,
                                this,
                                true,
                                false);


  // Create timer for monitoring the system monitor heartbeat. Don't start it
  // until we receive the first heartbeat from the system monitor
  sys_monitor_heartbeat_timer_ = nh_.createTimer(
                                ros::Duration(sys_monitor_heartbeat_timeout_),
                                &Executive::SysMonitorTimeoutCallback,
                                this,
                                false,
                                false);

  // Create timer to make sure the system monitor was started
  sys_monitor_startup_timer_ = nh_.createTimer(
                                  ros::Duration(sys_monitor_startup_time_secs_),
                                  &Executive::SysMonitorTimeoutCallback,
                                  this,
                                  true,
                                  true);

  // Create timer for guest science start and stop command timeout. If the guest
  // science manager doesn't respond to a start or stop guest science command
  // in the time specified, we need to ack command as failed. Make it one shot
  // and don't start until we send a guest science start or stop command
  gs_start_stop_command_timer_ = nh_.createTimer(
                            ros::Duration(gs_command_timeout_),
                            &Executive::GuestScienceStartStopCmdTimeoutCallback,
                            this,
                            true,
                            false);

  // Create timer for guest science custom command timeout. If the guest science
  // manager doesn't respond to a custom guest science command in the time
  // specified, we need to ack command as failed. Make it one shot and don't
  // start until we send a guest science custom command
  gs_custom_command_timer_ = nh_.createTimer(
                              ros::Duration(gs_command_timeout_),
                              &Executive::GuestScienceCustomCmdTimeoutCallback,
                              this,
                              true,
                              false);

  // Initialize the led service at the end of the initialize function as this
  // will turn off the booting up light and we only want to do this when the
  // executive has started up and is done initializing
  led_client_.SetConnectedTimeout(led_connected_timeout_);
  led_client_.SetConnectedCallback(
                            std::bind(&Executive::LedConnectedCallback, this));
  led_client_.Create(nh, SERVICE_HARDWARE_EPS_CONF_LED_STATE);
}

bool Executive::ReadParams() {
  std::string err_msg;
  // Read config files into lua
  if (!config_params_.ReadFiles()) {
    err_msg = "Error loading executive parameters. Couldn't read config files.";
    NODELET_ERROR("%s", err_msg.c_str());
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }

  // get world name
  if (!config_params_.GetStr("world_name", &agent_state_.world)) {
    NODELET_WARN("Unable to get world name.");
    agent_state_.world = "none";
  }

  // get action active timeout
  if (!config_params_.GetPosReal("action_active_timeout",
                                 &action_active_timeout_)) {
    NODELET_WARN("Action active timeout not specified.");
    action_active_timeout_ = 1;
  }

  // get led service available timeout
  if (!config_params_.GetPosReal("led_service_available_timeout",
                                 &led_connected_timeout_)) {
    NODELET_WARN("Led service available timeout not specified.");
    led_connected_timeout_ = 10;
  }

  // get gs manager timeout
  if (!config_params_.GetPosReal("gs_command_timeout",
                                 &gs_command_timeout_)) {
    NODELET_WARN("Guest science command timeout not specified.");
    gs_command_timeout_ = 4;
  }

  // get action feedback timeouts
  if (!config_params_.GetPosReal("motion_feedback_timeout",
                                 &motion_feedback_timeout_)) {
    NODELET_WARN("Motion feedback timeout not specified.");
    motion_feedback_timeout_ = 1;
  }

  if (!config_params_.GetPosReal("arm_feedback_timeout",
                                 &arm_feedback_timeout_)) {
    NODELET_WARN("Arm feedback timeout not specified.");
    arm_feedback_timeout_ = 4;
  }

  // get action results timeouts
  if (!config_params_.GetPosReal("dock_result_timeout",
                                 &dock_result_timeout_)) {
    NODELET_WARN("Dock result timeout not specified.");
    dock_result_timeout_ = 360;
  }

  if (!config_params_.GetPosReal("perch_result_timeout",
                                 &perch_result_timeout_)) {
    NODELET_WARN("Perch result timeout not specified.");
    perch_result_timeout_ = 360;
  }

  if (!config_params_.GetPosReal("localization_result_timeout",
                                              &localization_result_timeout_)) {
    NODELET_WARN("Localization result timeout not specified.");
    localization_result_timeout_ = 10;
  }

  if (!config_params_.GetPosReal("sys_monitor_startup_time_secs",
                                 &sys_monitor_startup_time_secs_)) {
    NODELET_WARN("System monitor startup time not specified.");
    sys_monitor_startup_time_secs_ = 30;
  }

  // get planner
  if (!config_params_.GetStr("planner", &agent_state_.planner)) {
    NODELET_WARN("System monitor planner not specified.");
    agent_state_.planner = "QuadraticProgram";
  }

  if (!config_params_.GetPosReal("sys_monitor_heartbeat_timeout",
                                 &sys_monitor_heartbeat_timeout_)) {
    err_msg = "System monitor heartbeat timeout not specified.";
    NODELET_ERROR("%s", err_msg.c_str());
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }

  if (!config_params_.CheckValExists("sys_monitor_heartbeat_fault_response")) {
    err_msg = "Sys monitor heartbeat fault response not specified.";
    NODELET_ERROR("%s", err_msg.c_str());
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }

  config_reader::ConfigReader::Table hb_response(&config_params_,
                                        "sys_monitor_heartbeat_fault_response");

  if (!ReadCommand(&hb_response, sys_monitor_heartbeat_fault_response_)) {
    err_msg = "Unable to read sys monitor heartbeat fault response.";
    NODELET_ERROR("%s", err_msg.c_str());
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }

  if (!config_params_.GetBool("sys_monitor_heartbeat_fault_blocking",
                              &sys_monitor_heartbeat_fault_blocking_)) {
    err_msg == "Sys monitor heartbeat fault blocking not specified.";
    NODELET_ERROR("%s", err_msg.c_str());
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }

  if (!config_params_.CheckValExists("sys_monitor_init_fault_response")) {
    err_msg = "System monitor init fault response not specified.";
    NODELET_ERROR("%s", err_msg.c_str());
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }

  config_reader::ConfigReader::Table init_response(&config_params_,
                                            "sys_monitor_init_fault_response");

  if (!ReadCommand(&init_response, sys_monitor_init_fault_response_)) {
    err_msg = "Unable to read sys monitor init fault response.";
    NODELET_ERROR("%s", err_msg.c_str());
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }

  if (!config_params_.GetBool("sys_monitor_init_fault_blocking",
                              &sys_monitor_init_fault_blocking_)) {
    err_msg = "Sys monitor init fault blocking not specified.";
    NODELET_ERROR("%s", err_msg.c_str());
    this->AssertFault(ff_util::INITIALIZATION_FAILED, err_msg);
    return false;
  }

  return true;
}

bool Executive::ReadCommand(config_reader::ConfigReader::Table *response,
                            ff_msgs::CommandStampedPtr cmd) {
  std::string cmd_name;
  if (!response->GetStr("name", &cmd_name)) {
    NODELET_ERROR("Fault response command name not specified.");
    return false;
  }

  cmd->cmd_name = cmd_name;
  cmd->cmd_src = "executive";
  cmd->cmd_origin = "executive";
  cmd->subsys_name = "Astrobee";

  if (response->CheckValExists("args")) {
    config_reader::ConfigReader::Table args(response, "args");
    int num_args = args.GetSize(), i;
    unsigned int type;
    cmd->args.resize(num_args);
    for (i = 0; i < num_args; ++i) {
      // Lua indices start at 1
      config_reader::ConfigReader::Table arg(&args, (i + 1));
      // First element in table is the type
      if (!arg.GetUInt(1, &type)) {
        NODELET_ERROR("First command argument value is not a uint");
        return false;
      }

      // Remaining elements are the parameter values
      switch (type) {
        case ff_msgs::CommandArg::DATA_TYPE_BOOL:
          {
            bool val;
            if (!arg.GetBool(2, &val)) {
              NODELET_ERROR("Expected command argument to be a bool!");
              return false;
            }
            cmd->args[i].data_type = ff_msgs::CommandArg::DATA_TYPE_BOOL;
            cmd->args[i].b = val;
          }
          break;
        case ff_msgs::CommandArg::DATA_TYPE_DOUBLE:
          {
            double val;
            if (!arg.GetReal(2, &val)) {
              NODELET_ERROR("Expected command argument to be a double");
              return false;
            }
            cmd->args[i].data_type = ff_msgs::CommandArg::DATA_TYPE_DOUBLE;
            cmd->args[i].d = val;
          }
          break;
        case ff_msgs::CommandArg::DATA_TYPE_FLOAT:
          {
            float val;
            if (!arg.GetReal(2, &val)) {
              NODELET_ERROR("Expected command argument to be a float.");
              return false;
            }
            cmd->args[i].data_type = ff_msgs::CommandArg::DATA_TYPE_FLOAT;
            cmd->args[i].f = val;
          }
          break;
        case ff_msgs::CommandArg::DATA_TYPE_INT:
          {
            int val;
            if (!arg.GetInt(2, &val)) {
              NODELET_ERROR("Expected command argument to be an int.");
              return false;
            }
            cmd->args[i].data_type = ff_msgs::CommandArg::DATA_TYPE_INT;
            cmd->args[i].i = val;
          }
          break;
        case ff_msgs::CommandArg::DATA_TYPE_LONGLONG:
          {
            int64_t val;
            if (!arg.GetLongLong(2, &val)) {
              NODELET_ERROR("Expected command argument to be an int.");
              return false;
            }
            cmd->args[i].data_type = ff_msgs::CommandArg::DATA_TYPE_LONGLONG;
            cmd->args[i].ll = val;
          }
          break;
        case ff_msgs::CommandArg::DATA_TYPE_STRING:
          {
            std::string val;
            if (!arg.GetStr(2, &val)) {
              NODELET_ERROR("Expected command argument to be a string");
              return false;
            }
            cmd->args[i].data_type = ff_msgs::CommandArg::DATA_TYPE_STRING;
            cmd->args[i].s = val;
          }
          break;
        case ff_msgs::CommandArg::DATA_TYPE_VEC3d:
          {
            int j;
            double val;
            cmd->args[i].data_type = ff_msgs::CommandArg::DATA_TYPE_VEC3d;
            for (j = 0; j < 3; ++j) {
              // Index to get vector values in table starts at 2
              if (!arg.GetReal((j + 2), &val)) {
                NODELET_ERROR("Expected command argument to be double.");
                return false;
              }
              cmd->args[i].vec3d[j] = val;
            }
          }
          break;
        case ff_msgs::CommandArg::DATA_TYPE_MAT33f:
          {
            int j;
            float val;
            cmd->args[i].data_type = ff_msgs::CommandArg::DATA_TYPE_MAT33f;
            for (j = 0; j < 9; ++j) {
              // Index in get matrix values in table starts at 2
              if (!arg.GetReal((j + 2), &val)) {
              NODELET_ERROR("Expected command argument to be a float.");
                return false;
              }
              cmd->args[i].mat33f[j] = val;
            }
          }
          break;
        default:
          NODELET_ERROR("Type for command argument unrecognized!");
          return false;
      }
    }
  }

  return true;
}

void Executive::PublishAgentState() {
  agent_state_.header.stamp = ros::Time::now();
  agent_state_pub_.publish(agent_state_);
}

}  // namespace executive

PLUGINLIB_EXPORT_CLASS(executive::Executive, nodelet::Nodelet)
