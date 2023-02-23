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

#include <ctl/ctl_ros.h>

#include <ff_msgs/msg/control_command.hpp>
#include <ff_msgs/msg/control_state.hpp>
#include <msg_conversions/msg_conversions.h>

#include <ff_common/ff_names.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

using std::placeholders::_1;
using std::placeholders::_2;

namespace ff_msgs {
  typedef msg::ControlCommand ControlCommand;
}  // namespace ff_msgs

using RESPONSE = ff_msgs::action::Control::Result;

namespace mc = msg_conversions;

FF_DEFINE_LOGGER("ctl");

namespace ctl {  // Nodehandle for <robot>/gnc/wrapper

Ctl::Ctl(NodeHandle& nh, std::string const& name) :
  fsm_(WAITING, std::bind(&Ctl::UpdateCallback,
    this, std::placeholders::_1, std::placeholders::_2)),
      name_(name), inertia_received_(false), control_enabled_(true), flight_enabled_(false) {
  // Add the state transition lambda functions - refer to the FSM diagram
  // [0]
  fsm_.Add(WAITING,
    GOAL_NOMINAL,
    [this](FSM::Event const& event) -> FSM::State {
      // If the timestamp is in the past, then we need to check its not too
      // stale. If it is stale, it might be indicative of timesync issues
      // between the MLP and LLP so we should reject the command
      mutex_segment_.lock();
      rclcpp::Time event_time(segment_.front().when);
      rclcpp::Duration delta = event_time - clock_->now();
      if (delta.toSec() < -MAX_LATENCY) {
        mutex_segment_.unlock();
        return Result(RESPONSE::TIMESYNC_ISSUE);
      }
      // For deferred executions
      timer_.stop();
      timer_.setPeriod(delta);
      timer_.start();
      mutex_segment_.unlock();
      // Set to stopping mode until we start the segment
      Command(ff_msgs::ControlCommand::MODE_STOP);
      // Update state
      return NOMINAL;
    });
  // [1]
  fsm_.Add(NOMINAL,
    GOAL_COMPLETE,
    [this](FSM::Event const& event) -> FSM::State {
      if (!Command(ff_msgs::ControlCommand::MODE_STOP))
        return Result(RESPONSE::CONTROL_FAILED);
      return STOPPING;
    });
  // [2]
  fsm_.Add(NOMINAL,
    GOAL_CANCEL,
    [this](FSM::Event const& event) -> FSM::State {
      if (!Command(ff_msgs::ControlCommand::MODE_STOP))
        return Result(RESPONSE::CONTROL_FAILED);
      return Result(RESPONSE::CANCELLED);
    });
  // [3]
  fsm_.Add(WAITING,
    GOAL_STOP,
    [this](FSM::Event const& event) -> FSM::State {
      if (!Command(ff_msgs::ControlCommand::MODE_STOP))
        return Result(RESPONSE::CONTROL_FAILED);
      return STOPPING;
    });
  // [4]
  fsm_.Add(STOPPING,
    GOAL_COMPLETE,
    [this](FSM::Event const& event) -> FSM::State {
      return Result(RESPONSE::SUCCESS);
    });

  clock_ = nh->get_clock();

  // Set the operating mode to STOP by default, so that when the speed ramps
  // up we don't drift off position because of small forces and torques
  mutex_cmd_msg_.lock();
  mode_ = ff_msgs::ControlCommand::MODE_STOP;
  mutex_cmd_msg_.unlock();

  controller_.Initialize();

  config_.AddFile("gnc.config");
  config_.AddFile("geometry.config");
  ReadParams();
  config_timer_.createTimer(1.0, [this]() {
      config_.CheckFilesUpdated(std::bind(&Ctl::ReadParams, this));}, nh, false, true);

  // Subscribers
  ekf_sub_ = FF_CREATE_SUBSCRIBER(nh, ff_msgs::msg::EkfState,
    TOPIC_GNC_EKF, 1,
    std::bind(&Ctl::EkfCallback, this, std::placeholders::_1));
  pose_sub_ = FF_CREATE_SUBSCRIBER(nh, geometry_msgs::msg::PoseStamped,
    TOPIC_LOCALIZATION_POSE, 1,
    std::bind(&Ctl::PoseCallback, this, std::placeholders::_1));
  twist_sub_ = FF_CREATE_SUBSCRIBER(nh, geometry_msgs::msg::TwistStamped,
    TOPIC_LOCALIZATION_TWIST, 1,
    std::bind(&Ctl::TwistCallback, this, std::placeholders::_1));
  inertia_sub_ = FF_CREATE_SUBSCRIBER(nh, geometry_msgs::msg::InertiaStamped,
    TOPIC_MOBILITY_INERTIA, 1,
    std::bind(&Ctl::InertiaCallback, this, std::placeholders::_1));
  flight_mode_sub_ = FF_CREATE_SUBSCRIBER(nh, ff_msgs::msg::FlightMode,
    TOPIC_MOBILITY_FLIGHT_MODE, 1,
    std::bind(&Ctl::FlightModeCallback, this, std::placeholders::_1));
  command_sub_ = FF_CREATE_SUBSCRIBER(nh, ff_msgs::msg::ControlState,
    TOPIC_GNC_CTL_SETPOINT, 1,
    std::bind(&Ctl::SetpointCallback, this, std::placeholders::_1));

  // Advertised messages
  ctl_pub_ = FF_CREATE_PUBLISHER(nh, ff_msgs::msg::FamCommand,
    TOPIC_GNC_CTL_COMMAND, 5);
  traj_pub_ = FF_CREATE_PUBLISHER(nh, ff_msgs::msg::ControlState,
    TOPIC_GNC_CTL_TRAJ, 5);
  // TODO(kbrowne): make segment_pub_ latching
  segment_pub_ = FF_CREATE_PUBLISHER(nh, ff_msgs::msg::Segment,
    TOPIC_GNC_CTL_SEGMENT, 5);

  // Enable / Disable control module
  enable_srv_ = nh->create_service<std_srvs::srv::SetBool>(SERVICE_GNC_CTL_ENABLE,
    std::bind(&Ctl::EnableCtl, this, _1, _2));

  // Action client to accept control
  action_.SetGoalCallback(
      std::bind(&Ctl::GoalCallback, this, std::placeholders::_1));
  action_.SetPreemptCallback(std::bind(&Ctl::PreemptCallback, this));
  action_.SetCancelCallback(std::bind(&Ctl::CancelCallback, this));
  action_.Create(nh, ACTION_GNC_CTL_CONTROL);

  // This timer will be used to throttle control to GNC
  timer_.createTimer(0.0, [this]() {TimerCallback();}, nh, true, false);
}


// Destructor
Ctl::~Ctl() {}

FSM::State Ctl::Result(int32_t response) {
  FF_DEBUG_STREAM("Control action completed with code " << response);
  // Stop the platform and
  if (!Command(ff_msgs::ControlCommand::MODE_STOP))
    FF_ERROR("Could not stop the platform at end of control action");
  // Send a response
  std::shared_ptr<ff_msgs::action::Control::Result> result = std::make_shared<ff_msgs::action::Control::Result>();
  result->response = response;
  mutex_segment_.lock();
  if (!segment_.empty()) {
    result->segment = segment_;
    result->index = std::distance(setpoint_, segment_.begin());
  }

  if (response == RESPONSE::SUCCESS) {
    FF_INFO_STREAM("Result: Control succeeded, result sent to client");
    action_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
  } else if (response == RESPONSE::PREEMPTED) {
      FF_INFO_STREAM("Result: Control Preempted, result sent to client");
      action_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, result);
  } else {
    FF_INFO_STREAM("Result: Control action aborted, result sent to client");
    action_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
  }

  // Publish an empty segment to notify that nominal mode is over
  static ff_msgs::msg::Segment msg;
  segment_pub_->publish(msg);
  // Clear local variables
  timer_.stop();
  segment_.clear();
  setpoint_ = segment_.end();
  mutex_segment_.unlock();
  // Always return to the waiting state
  return WAITING;
}

bool Ctl::EnableCtl(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  bool request = req->data;

  // Check if we are asking for the current state
  if (request == control_enabled_) {
    response->success = false;
    response->message = "Requested current state.";
    return true;
  }

  // Check required action
  response->success = true;
  control_enabled_ = request;
  if (control_enabled_ == false) {
    response->message = "Onboard controller disabled.";
    FF_DEBUG_STREAM("Onboard controller disabled.");
  } else {
    response->message = "Onboard controller enabled.";
    FF_DEBUG_STREAM("Onboard controller enabled.");
  }
  return true;
}

void Ctl::UpdateCallback(FSM::State const& state, FSM::Event const& event) {
  // Debug events
  std::string str = "UNKNOWN";
  switch (event) {
  case GOAL_NOMINAL:            str = "GOAL_NOMINAL";       break;
  case GOAL_COMPLETE:           str = "GOAL_COMPLETE";      break;
  case GOAL_CANCEL:             str = "GOAL_CANCEL";        break;
  case GOAL_STOP:               str = "GOAL_STOP";          break;
  }
  FF_DEBUG_STREAM("Received event " << str);
  // Debug state changes
  switch (state) {
  case WAITING:                 str = "WAITING";            break;
  case STOPPING:                str = "STOPPING";           break;
  case NOMINAL:                 str = "NOMINAL";            break;
  }
  FF_DEBUG_STREAM("State changed to " << str);
}

// Callback in regular mode
void Ctl::EkfCallback(const std::shared_ptr<ff_msgs::msg::EkfState> state) {
  if (!use_truth_) {
    mutex_cmd_msg_.lock();
    state_.est_omega_B_ISS_B = msg_conversions::ros_to_eigen_vector(state->omega).cast<float>();
    state_.est_V_B_ISS_ISS = msg_conversions::ros_to_eigen_vector(state->velocity).cast<float>();
    state_.est_P_B_ISS_ISS = msg_conversions::ros_point_to_eigen_vector(state->pose.position).cast<float>();
    state_.est_quat_ISS2B = msg_conversions::ros_to_eigen_quat(state->pose.orientation).cast<float>();
    state_.est_confidence = state->confidence;
    mutex_cmd_msg_.unlock();

    // Check if Astrobee is dynamically stopped
    if (fsm_.GetState() == STOPPING) {
      double v_x = state->velocity.x, v_y = state->velocity.y, v_z = state->velocity.z;
      double omega_x = state->omega.x, omega_y = state->omega.y, omega_z = state->omega.z;
      double v_magnitude = sqrt(v_x*v_x + v_y*v_y + v_z*v_z);
      double omega_magnitude = sqrt(omega_x*omega_x + omega_y*omega_y + omega_z*omega_z);
      FF_DEBUG_STREAM("Validating velocity when stopping:  |v|=  " << (float) v_magnitude
        << "  |omega|= " << (float) omega_magnitude);

      if ((v_magnitude <= sqrt(stopping_vel_thresh_squared_)) &&
        (omega_magnitude <= sqrt(stopping_omega_thresh_squared_))) {
        FF_INFO_STREAM("Stopping Complete: |v| <=  " << (float) sqrt(stopping_vel_thresh_squared_)
          << "  and |omega| <=  " << (float) sqrt(stopping_omega_thresh_squared_));
        fsm_.Update(GOAL_COMPLETE);
      }
    }

    // advance control forward whenever the pose is updated
    Step(state->header.stamp);
  }
}

// Callback in ground truth mode
void Ctl::PoseCallback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> truth) {
  if (use_truth_) {
    mutex_cmd_msg_.lock();
    state_.est_P_B_ISS_ISS = msg_conversions::ros_point_to_eigen_vector(truth->pose.position).cast<float>();
    state_.est_quat_ISS2B = msg_conversions::ros_to_eigen_quat(truth->pose.orientation).cast<float>();
    state_.est_confidence = 0;  // Localization manager now deals with this
    mutex_cmd_msg_.unlock();
    // advance control forward whenever the pose is updated
    Step(truth->header.stamp);
  }
}

// Called when localization has new twist data
void Ctl::TwistCallback(const std::shared_ptr<geometry_msgs::msg::TwistStamped> truth) {
  if (use_truth_) {
    mutex_cmd_msg_.lock();
    state_.est_V_B_ISS_ISS = mc::ros_to_eigen_vector(truth->twist.linear).cast<float>();
    state_.est_omega_B_ISS_B = mc::ros_to_eigen_vector(truth->twist.angular).cast<float>();
    mutex_cmd_msg_.unlock();
    // Don't advance control, until the pose arrives
  }
}

// Called when management updates inertial info
void Ctl::InertiaCallback(const std::shared_ptr<geometry_msgs::msg::InertiaStamped> inertia) {
  std::lock_guard<std::mutex> lock(mutex_cmd_msg_);
  state_.mass = inertia->inertia.m;
  auto& i = inertia->inertia;
  state_.inertia << i.ixx, i.ixy, i.ixz, i.ixy, i.iyy, i.iyz, i.ixz, i.iyz, i.izz;
  inertia_received_ = true;
}

// Called when choreographer updates the flight mode
void Ctl::FlightModeCallback(const std::shared_ptr<ff_msgs::msg::FlightMode> mode) {
  std::lock_guard<std::mutex> lock(mutex_cmd_msg_);
  state_.att_kp = mc::ros_to_eigen_vector(mode->att_kp).cast<float>();
  state_.att_ki = mc::ros_to_eigen_vector(mode->att_ki).cast<float>();
  state_.omega_kd = mc::ros_to_eigen_vector(mode->omega_kd).cast<float>();
  state_.pos_kp = mc::ros_to_eigen_vector(mode->pos_kp).cast<float>();
  state_.pos_ki = mc::ros_to_eigen_vector(mode->pos_ki).cast<float>();
  state_.vel_kd = mc::ros_to_eigen_vector(mode->vel_kd).cast<float>();
  flight_enabled_ =
    (mode->speed > 0 ? mode->control_enabled : false);
}

// Command GNC directly, bypassing the action-base dinterface
void Ctl::SetpointCallback(const std::shared_ptr<ff_msgs::msg::ControlState> command) {
  // Package up a command using the curren timestamp
  ff_msgs::ControlCommand msg;
  msg.header.stamp = clock_->now();
  msg.header.frame_id = "world";
  msg.mode = ff_msgs::ControlCommand::MODE_NOMINAL;
  msg.current = *command;
  msg.current.when = clock_->now();
  msg.next = msg.current;
  // Send the command
  if (!Command(msg.mode, msg))
    FF_WARN("Could not send direct control command");
}

// When the previous setpoint is sent a timer is created, which fires on the
// edge of the next setpoint. This essentially provides us with a non-blocking
// method for feeding control.
void Ctl::TimerCallback() {
  static ff_msgs::ControlCommand msg;
  msg.mode = ff_msgs::ControlCommand::MODE_NOMINAL;
  do {
    mutex_segment_.lock();
    // Get the index of the current setpoint
    size_t idx = std::distance(segment_.begin(), setpoint_);
    // If the setpoint is pointing to the last segment, that's it!
    if (setpoint_ == segment_.end()) {
      mutex_segment_.unlock();
      FF_DEBUG_STREAM("Final setpoint " << idx);
      return fsm_.Update(GOAL_COMPLETE);
    }
    // Otherwise, we have at least one more setpoint
    msg.current = *setpoint_;
    // If we only have one setpoint, just copy the first one
    if (++setpoint_ == segment_.end()) {
      mutex_segment_.unlock();
      FF_DEBUG_STREAM("Penultimate setpoint " << idx);
      msg.next = msg.current;
      break;
    // We have two valid set points - if the times are equal, then we will
    // treat the last setpoint as the valid one and move on...
    } else {
      msg.next = *setpoint_;
      if (msg.current.when != msg.next.when) {
        mutex_segment_.unlock();
        FF_DEBUG_STREAM("Progressing to setpoint " << idx);
        break;
      }
    }
    mutex_segment_.unlock();
    FF_DEBUG_STREAM("Skipping setpoint " << idx);
  } while (msg.current.when == msg.next.when);
  // Send the control
  if (!Command(ff_msgs::ControlCommand::MODE_NOMINAL, msg)) {
    Result(RESPONSE::CONTROL_FAILED);
    return;
  }
  // Set off a timer to wait until the next setpoint
  mutex_segment_.lock();
  rclcpp::Time msg_time(msg.next.when);
  rclcpp::Duration delta = msg_time - clock_->now();
  timer_.stop();
  timer_.setPeriod(delta);
  timer_.start();
  mutex_segment_.unlock();
  FF_DEBUG_STREAM("Sleep: " << delta.nanoseconds() / 1.0e9);
}

// Goal callback
void Ctl::GoalCallback(std::shared_ptr<const ff_msgs::action::Control::Goal> goal) {
  FF_INFO_STREAM("GoalCallback: new control goal received");
  // What have we been instructed to do?
  switch (goal->command) {
    // IDLE - returns instantly
    case ff_msgs::action::Control::Goal::IDLE:
      FF_DEBUG_STREAM("Received IDLE command");
      if (!Command(ff_msgs::ControlCommand::MODE_IDLE))
        Result(RESPONSE::CONTROL_FAILED);
      Result(RESPONSE::SUCCESS);
      return;
    // STOP - returns instantly
    case ff_msgs::action::Control::Goal::STOP:
      FF_DEBUG_STREAM("Received STOP command");
      if (!Command(ff_msgs::ControlCommand::MODE_STOP))
        Result(RESPONSE::CONTROL_FAILED);
      fsm_.Update(GOAL_STOP);
      return;
    // NOMINAL - waits until completion
    case ff_msgs::action::Control::Goal::NOMINAL: {
      FF_DEBUG_STREAM("Received NOMINAL command with segment...");
      // FF_DEBUG_STREAM(goal->segment);
      if (goal->segment.empty()) {
        Result(RESPONSE::EMPTY_SEGMENT);
        return;
      }
      // Save the segment internally
      mutex_segment_.lock();
      segment_ = goal->segment;       // Copy over the segment to be processed
      setpoint_ = segment_.begin();   // We are processing the first setpoint
      mutex_segment_.unlock();
      // Publish the segment to used by the sentinel (and perhaps others)
      static ff_msgs::msg::Segment msg;
      msg.segment = goal->segment;
      segment_pub_->publish(msg);

      // Start executing and return
      fsm_.Update(GOAL_NOMINAL);
      return;
    }
    // No other control mode makes sense
    default:
      break;
  }
  Result(RESPONSE::INVALID_COMMAND);
}

// Cancellation
void Ctl::CancelCallback() {
  FF_DEBUG_STREAM("CancelCallback called");
  fsm_.Update(GOAL_CANCEL);
}

// Preemption - this might be a replan
void Ctl::PreemptCallback() {
  FF_DEBUG_STREAM("PreemptCallback called");
  fsm_.Update(GOAL_CANCEL);
}

// Update a control with a new command
bool Ctl::Command(uint8_t const mode, ff_msgs::ControlCommand const& poseVel) {
  // Set the operating mode
  std::lock_guard<std::mutex> lock(mutex_cmd_msg_);
  mode_ = mode;
  if (mode != ff_msgs::ControlCommand::MODE_NOMINAL)
    return true;

  command_ = poseVel;

  return true;
}

float Ctl::GetCommand(ControlCommand* cmd, rclcpp::Time tim) {
  auto* curr = &command_.next;
  rclcpp::Time msg_time(curr->when);
  if (msg_time > tim)
    curr = &command_.current;
  cmd->P_B_ISS_ISS = mc::ros_point_to_eigen_vector(curr->pose.position).cast<float>();
  cmd->quat_ISS2B = mc::ros_to_eigen_quat(curr->pose.orientation).cast<float>();
  cmd->V_B_ISS_ISS = mc::ros_to_eigen_vector(curr->twist.linear).cast<float>();
  cmd->A_B_ISS_ISS = mc::ros_to_eigen_vector(curr->accel.linear).cast<float>();
  cmd->omega_B_ISS_ISS = mc::ros_to_eigen_vector(curr->twist.angular).cast<float>();
  cmd->alpha_B_ISS_ISS = mc::ros_to_eigen_vector(curr->accel.angular).cast<float>();
  cmd->mode = mode_;
  return (tim - msg_time).toSec();
}

bool Ctl::Step(rclcpp::Time curr_time) {
  if (!inertia_received_) {
    // FF_DEBUG_STREAM_THROTTLE(10, "GNC step waiting for inertia");
    return false;
  }
  if (!flight_enabled_ || !control_enabled_) {
    // FF_DEBUG_STREAM_THROTTLE(10, "GNC control disabled in flight mode");
    return false;
  }

  ControlCommand command;
  ControlOutput output;
  float time_delta;
  {
    std::lock_guard<std::mutex> cmd_lock(mutex_cmd_msg_);

    time_delta = GetCommand(&command, curr_time);
  }

  // Step GNC forward
  controller_.Step(time_delta, state_, command, &output);

  // Publish the FAM command
  static ff_msgs::msg::FamCommand cmd_msg_;
  cmd_msg_.header.stamp = curr_time;
  cmd_msg_.header.frame_id = "body";
  cmd_msg_.wrench.force = mc::eigen_to_ros_vector(output.body_force_cmd.cast<double>());
  cmd_msg_.wrench.torque = mc::eigen_to_ros_vector(output.body_torque_cmd.cast<double>());
  cmd_msg_.accel = mc::eigen_to_ros_vector(output.body_accel_cmd.cast<double>());
  cmd_msg_.alpha = mc::eigen_to_ros_vector(output.body_alpha_cmd.cast<double>());
  cmd_msg_.position_error = mc::eigen_to_ros_vector(output.pos_err.cast<double>());
  cmd_msg_.position_error_integrated = mc::eigen_to_ros_vector(output.pos_err_int.cast<double>());
  cmd_msg_.attitude_error = mc::eigen_to_ros_vector(output.att_err.cast<double>());
  cmd_msg_.attitude_error_integrated = mc::eigen_to_ros_vector(output.att_err_int.cast<double>());
  cmd_msg_.attitude_error_mag = output.att_err_mag;
  cmd_msg_.status = output.ctl_status;
  cmd_msg_.control_mode = mode_;

  ctl_pub_->publish(cmd_msg_);

  // Publish the traj message
  static ff_msgs::msg::ControlState current;
  current.when = curr_time;
  current.pose.position = msg_conversions::eigen_to_ros_point(output.traj_pos.cast<double>());
  current.pose.orientation = msg_conversions::eigen_to_ros_quat(output.traj_quat.cast<double>());
  current.twist.linear = msg_conversions::eigen_to_ros_vector(output.traj_vel.cast<double>());
  current.twist.angular = msg_conversions::eigen_to_ros_vector(output.traj_omega.cast<double>());
  current.accel.linear = msg_conversions::eigen_to_ros_vector(output.traj_accel.cast<double>());
  current.accel.angular = msg_conversions::eigen_to_ros_vector(output.traj_alpha.cast<double>());
  traj_pub_->publish(current);

  // Publish the current setpoint,
  if ((fsm_.GetState() == NOMINAL) || (fsm_.GetState() == STOPPING)) {
    std::lock_guard<std::mutex> lock(mutex_segment_);
    std::shared_ptr<ff_msgs::action::Control::Feedback> feedback =
          std::make_shared<ff_msgs::action::Control::Feedback>();
    feedback->setpoint.when = curr_time;
    feedback->setpoint.pose.position = mc::eigen_to_ros_point(output.traj_pos.cast<double>());
    feedback->setpoint.pose.orientation = mc::eigen_to_ros_quat(output.traj_quat.cast<double>());
    feedback->setpoint.twist.linear = mc::eigen_to_ros_vector(output.traj_vel.cast<double>());
    feedback->setpoint.twist.angular = mc::eigen_to_ros_vector(output.traj_omega.cast<double>());
    feedback->setpoint.accel.linear = mc::eigen_to_ros_vector(output.traj_accel.cast<double>());
    feedback->setpoint.accel.angular = mc::eigen_to_ros_vector(output.traj_alpha.cast<double>());
    feedback->index = std::distance(setpoint_, segment_.begin());
    // Sometimese segments arrive with a first setpoint that has a time stamp
    // sometime in the future. In this case we will be in STOPPED mode until
    // the callback timer sets the CMC mode to NOMINAL. In the interim the
    // errors being sent back from the controller make no sense.
    switch (mode_) {
    case ff_msgs::ControlCommand::MODE_NOMINAL:
      feedback->error_position = output.traj_error_pos;
      feedback->error_attitude = output.traj_error_att;
      feedback->error_velocity = output.traj_error_vel;
      feedback->error_omega    = output.traj_error_omega;
      break;
    default:
      feedback->error_position = 0;
      feedback->error_attitude = 0;
      feedback->error_velocity = 0;
      feedback->error_omega    = 0;
      break;
    }
    action_.SendFeedback(feedback);
  }

  // GNC stepped successfully
  return true;
}

std::string Ctl::getName() { return name_; }

// Chainload the readparam call
void Ctl::ReadParams(void) {
  if (!config_.ReadFiles()) {
    FF_ERROR("Failed to read config files.");
    return;
  }
  controller_.ReadParams(&config_);
  if (!config_.GetBool("tun_debug_ctl_use_truth", &use_truth_))
    FF_FATAL("tun_debug_ctl_use_truth not specified.");
  // Set linear- and angular velocity threshold for ekf
  // to decide that Astrobee is dynamically stopped.
  if (!config_.GetReal("tun_ctl_stopping_vel_thresh", &stopping_vel_thresh_squared_))
    FF_FATAL("tun_ctl_stopping_vel_thresh not specified.");
  if (!config_.GetReal("tun_ctl_stopping_omega_thresh", &stopping_omega_thresh_squared_))
    FF_FATAL("tun_ctl_stopping_omega_thresh not specified.");
}

}  // end namespace ctl
