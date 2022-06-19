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

#include <ctl/ctl.h>

#include <nodelet/nodelet.h>

#include <ff_msgs/ControlState.h>
#include <ff_msgs/FamCommand.h>
#include <ff_msgs/Segment.h>
#include <msg_conversions/msg_conversions.h>

#include <ff_util/ff_names.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// parameters ctl_controller0_P are set in
//   matlab/code_generation/ctl_controller0_ert_rtw/ctl_controller0_data.c

using RESPONSE = ff_msgs::ControlResult;

namespace mc = msg_conversions;

namespace ctl {  // Nodehandle for <robot>/gnc/wrapper

Ctl::Ctl(ros::NodeHandle* nh, std::string const& name) :
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
      ros::Duration delta = segment_.front().when - ros::Time::now();
      if (delta.toSec() < -MAX_LATENCY) {
        mutex_segment_.unlock();
        return Result(RESPONSE::TIMESYNC_ISSUE);
      }
      // For deferred executions
      timer_.stop();
      timer_.setPeriod(delta, true);
      timer_.start();
      mutex_segment_.unlock();
      // Set to stopping mode until we start the segment
      Control(ff_msgs::ControlCommand::MODE_STOP);
      // Update state
      return NOMINAL;
    });
  // [1]
  fsm_.Add(NOMINAL,
    GOAL_COMPLETE,
    [this](FSM::Event const& event) -> FSM::State {
      if (!Control(ff_msgs::ControlCommand::MODE_STOP))
        return Result(RESPONSE::CONTROL_FAILED);
      return STOPPING;
    });
  // [2]
  fsm_.Add(NOMINAL,
    GOAL_CANCEL,
    [this](FSM::Event const& event) -> FSM::State {
      if (!Control(ff_msgs::ControlCommand::MODE_STOP))
        return Result(RESPONSE::CONTROL_FAILED);
      return Result(RESPONSE::CANCELLED);
    });
  // [3]
  fsm_.Add(WAITING,
    GOAL_STOP,
    [this](FSM::Event const& event) -> FSM::State {
      if (!Control(ff_msgs::ControlCommand::MODE_STOP))
        return Result(RESPONSE::CONTROL_FAILED);
      return STOPPING;
    });
  // [4]
  fsm_.Add(STOPPING,
    GOAL_COMPLETE,
    [this](FSM::Event const& event) -> FSM::State {
      return Result(RESPONSE::SUCCESS);
    });

  // Set the operating mode to STOP by default, so that when the speed ramps
  // up we don't drift off position because of small forces and torques
  mutex_cmd_msg_.lock();
  gnc_.ctl_input_.ctl_mode_cmd = ff_msgs::ControlCommand::MODE_STOP;
  mutex_cmd_msg_.unlock();

  // Initialize GNC
  gnc_.Initialize();

  config_.AddFile("gnc.config");
  config_.AddFile("geometry.config");
  ReadParams();
  config_timer_ = nh->createTimer(ros::Duration(1), [this](ros::TimerEvent e) {
      config_.CheckFilesUpdated(std::bind(&Ctl::ReadParams, this));}, false, true);
  pt_ctl_.Initialize("ctl");

  // Subscribers
  ekf_sub_ = nh->subscribe(
    TOPIC_GNC_EKF, 1, &Ctl::EkfCallback, this);
  pose_sub_ = nh->subscribe(
    TOPIC_LOCALIZATION_POSE, 1, &Ctl::PoseCallback, this);
  twist_sub_ = nh->subscribe(
    TOPIC_LOCALIZATION_TWIST, 1, &Ctl::TwistCallback, this);
  inertia_sub_ = nh->subscribe(
    TOPIC_MOBILITY_INERTIA, 1, &Ctl::InertiaCallback, this);
  flight_mode_sub_ = nh->subscribe(
    TOPIC_MOBILITY_FLIGHT_MODE, 1, &Ctl::FlightModeCallback, this);
  command_sub_ = nh->subscribe(
    TOPIC_GNC_CTL_SETPOINT, 1, &Ctl::SetpointCallback, this);

  // Advertised messages
  ctl_pub_ = nh->advertise<ff_msgs::FamCommand>(
    TOPIC_GNC_CTL_COMMAND, 5);
  traj_pub_ = nh->advertise<ff_msgs::ControlState>(
    TOPIC_GNC_CTL_TRAJ, 5);
  segment_pub_ = nh->advertise<ff_msgs::Segment>(
    TOPIC_GNC_CTL_SEGMENT, 5, true);

  // Enable / Disable control module
  enable_srv_ = nh->advertiseService(SERVICE_GNC_CTL_ENABLE,
    &Ctl::EnableCtl, this);

  // Action client to accept control
  action_.SetGoalCallback(
      std::bind(&Ctl::GoalCallback, this, std::placeholders::_1));
  action_.SetPreemptCallback(std::bind(&Ctl::PreemptCallback, this));
  action_.SetCancelCallback(std::bind(&Ctl::CancelCallback, this));
  action_.Create(nh, ACTION_GNC_CTL_CONTROL);

  // This timer will be used to throttle control to GNC
  timer_ = nh->createTimer(ros::Duration(0),
    &Ctl::TimerCallback, this, true, false);
}


// Destructor
Ctl::~Ctl() {}

FSM::State Ctl::Result(int32_t response) {
  NODELET_DEBUG_STREAM("Control action completed with code " << response);
  // Stop the platform and
  if (!Control(ff_msgs::ControlCommand::MODE_STOP))
    NODELET_ERROR("Could not stop the platform at end of control action");
  // Send a response
  static ff_msgs::ControlResult result;
  result.response = response;
  mutex_segment_.lock();
  if (!segment_.empty()) {
    result.segment = segment_;
    result.index = std::distance(setpoint_, segment_.begin());
  }

  if (response == RESPONSE::SUCCESS) {
    NODELET_INFO_STREAM("Result: Control succeeded, result sent to client");
    action_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, result);
  } else if (response == RESPONSE::PREEMPTED) {
      NODELET_INFO_STREAM("Result: Control Preempted, result sent to client");
      action_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED, result);
  } else {
    NODELET_INFO_STREAM("Result: Control action aborted, result sent to client");
    action_.SendResult(ff_util::FreeFlyerActionState::ABORTED, result);
  }

  // Publish an empty segment to notify that nominal mode is over
  static ff_msgs::Segment msg;
  segment_pub_.publish(msg);
  // Clear local variables
  timer_.stop();
  segment_.clear();
  setpoint_ = segment_.end();
  mutex_segment_.unlock();
  // Always return to the waiting state
  return WAITING;
}

bool Ctl::EnableCtl(std_srvs::SetBoolRequest&req, std_srvs::SetBoolResponse &response) {
  bool request = req.data;

  // Check if we are asking for the current state
  if (request == control_enabled_) {
    response.success = false;
    response.message = "Requested current state.";
    return true;
  }

  // Check required action
  response.success = true;
  control_enabled_ = request;
  if (control_enabled_ == false) {
    NODELET_DEBUG_STREAM((response.message = "Onboard controller disabled."));
  } else {
    NODELET_DEBUG_STREAM((response.message = "Onboard controller enabled."));
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
  NODELET_DEBUG_STREAM("Received event " << str);
  // Debug state changes
  switch (state) {
  case WAITING:                 str = "WAITING";            break;
  case STOPPING:                str = "STOPPING";           break;
  case NOMINAL:                 str = "NOMINAL";            break;
  }
  NODELET_DEBUG_STREAM("State changed to " << str);
}

// Callback in regular mode
void Ctl::EkfCallback(const ff_msgs::EkfState::ConstPtr& state) {
  if (!use_truth_) {
    mutex_cmd_msg_.lock();
    auto & ctl = gnc_.ctl_input_;
    msg_conversions::ros_to_array_vector(state->omega, ctl.est_omega_B_ISS_B);
    msg_conversions::ros_to_array_vector(state->velocity, ctl.est_V_B_ISS_ISS);
    msg_conversions::ros_to_array_point(state->pose.position, ctl.est_P_B_ISS_ISS);
    msg_conversions::ros_to_array_quat(state->pose.orientation, ctl.est_quat_ISS2B);
    ctl.est_confidence = state->confidence;
    ctl.current_time_sec = state->header.stamp.sec;
    ctl.current_time_nsec = state->header.stamp.nsec;
    mutex_cmd_msg_.unlock();

    // Check if Astrobee is dynamically stopped
    if (fsm_.GetState() == STOPPING) {
      double v_x = state->velocity.x, v_y = state->velocity.y, v_z = state->velocity.z;
      double omega_x = state->omega.x, omega_y = state->omega.y, omega_z = state->omega.z;
      double v_magnitude = sqrt(v_x*v_x + v_y*v_y + v_z*v_z);
      double omega_magnitude = sqrt(omega_x*omega_x + omega_y*omega_y + omega_z*omega_z);
      NODELET_DEBUG_STREAM("Validating velocity when stopping:  |v|=  " << (float) v_magnitude
        << "  |omega|= " << (float) omega_magnitude);

      if ((v_magnitude <= sqrt(stopping_vel_thresh_squared_)) &&
        (omega_magnitude <= sqrt(stopping_omega_thresh_squared_))) {
        NODELET_INFO_STREAM("Stopping Complete: |v| <=  " << (float) sqrt(stopping_vel_thresh_squared_)
          << "  and |omega| <=  " << (float) sqrt(stopping_omega_thresh_squared_));
        fsm_.Update(GOAL_COMPLETE);
      }
    }

    // advance control forward whenever the pose is updated
    pt_ctl_.Tick();
    Step();
    pt_ctl_.Tock();
  }
}

// Callback in ground truth mode
void Ctl::PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& truth) {
  if (use_truth_) {
    mutex_cmd_msg_.lock();
    auto& ctl = gnc_.ctl_input_;
    msg_conversions::ros_to_array_point(truth->pose.position, ctl.est_P_B_ISS_ISS);
    msg_conversions::ros_to_array_quat(truth->pose.orientation, ctl.est_quat_ISS2B);
    ctl.est_confidence = 0;  // Localization manager now deals with this
    ctl.current_time_sec = truth->header.stamp.sec;
    ctl.current_time_nsec = truth->header.stamp.nsec;
    mutex_cmd_msg_.unlock();
    // advance control forward whenever the pose is updated
    pt_ctl_.Tick();
    Step();
    pt_ctl_.Tock();
  }
}

// Called when localization has new twist data
void Ctl::TwistCallback(const geometry_msgs::TwistStamped::ConstPtr& truth) {
  if (use_truth_) {
    mutex_cmd_msg_.lock();
    auto& ctl = gnc_.ctl_input_;
    mc::ros_to_array_vector(truth->twist.linear, ctl.est_V_B_ISS_ISS);
    mc::ros_to_array_vector(truth->twist.angular, ctl.est_omega_B_ISS_B);
    mutex_cmd_msg_.unlock();
    // Don't advance control, until the pose arrives
  }
}

// Called when management updates inertial info
void Ctl::InertiaCallback(const geometry_msgs::InertiaStamped::ConstPtr& inertia) {
  std::lock_guard<std::mutex> lock(mutex_cmd_msg_);
  auto& input = gnc_.ctl_input_;
  input.mass = inertia->inertia.m;
  // mc::ros_to_array_vector(inertia->com, input.center_of_mass);  // no longer exists
  input.inertia_matrix[0] = inertia->inertia.ixx;
  input.inertia_matrix[1] = inertia->inertia.ixy;
  input.inertia_matrix[2] = inertia->inertia.ixz;
  input.inertia_matrix[3] = inertia->inertia.ixy;
  input.inertia_matrix[4] = inertia->inertia.iyy;
  input.inertia_matrix[5] = inertia->inertia.iyz;
  input.inertia_matrix[6] = inertia->inertia.ixz;
  input.inertia_matrix[7] = inertia->inertia.iyz;
  input.inertia_matrix[8] = inertia->inertia.izz;
  inertia_received_ = true;
}

// Called when choreographer updates the flight mode
void Ctl::FlightModeCallback(const ff_msgs::FlightMode::ConstPtr& mode) {
  std::lock_guard<std::mutex> lock(mutex_cmd_msg_);
  auto& input = gnc_.ctl_input_;
  mc::ros_to_array_vector(mode->att_kp, input.att_kp);
  mc::ros_to_array_vector(mode->att_ki, input.att_ki);
  mc::ros_to_array_vector(mode->omega_kd, input.omega_kd);
  mc::ros_to_array_vector(mode->pos_kp, input.pos_kp);
  mc::ros_to_array_vector(mode->pos_ki, input.pos_ki);
  mc::ros_to_array_vector(mode->vel_kd, input.vel_kd);
  input.speed_gain_cmd = mode->speed;
  flight_enabled_ =
    (input.speed_gain_cmd > 0 ? mode->control_enabled : false);
}

// Command GNC directly, bypassing the action-base dinterface
void Ctl::SetpointCallback(const ff_msgs::ControlState::ConstPtr& command) {
  // Package up a command using the curren timestamp
  ff_msgs::ControlCommand msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.mode = ff_msgs::ControlCommand::MODE_NOMINAL;
  msg.current = *command;
  msg.current.when = ros::Time::now();
  msg.next = msg.current;
  // Send the command
  if (!Control(msg.mode, msg))
    ROS_WARN("Could not send direct control command");
}

// When the previous setpoint is sent a timer is created, which fires on the
// edge of the next setpoint. This essentially provides us with a non-blocking
// method for feeding control.
void Ctl::TimerCallback(const ros::TimerEvent& event) {
  static ff_msgs::ControlCommand msg;
  msg.mode = ff_msgs::ControlCommand::MODE_NOMINAL;
  do {
    mutex_segment_.lock();
    // Get the index of the current setpoint
    size_t idx = std::distance(segment_.begin(), setpoint_);
    // If the setpoint is pointing to the last segment, that's it!
    if (setpoint_ == segment_.end()) {
      mutex_segment_.unlock();
      NODELET_DEBUG_STREAM("Final setpoint " << idx);
      return fsm_.Update(GOAL_COMPLETE);
    }
    // Otherwise, we have at least one more setpoint
    msg.current = *setpoint_;
    // If we only have one setpoint, just copy the first one
    if (++setpoint_ == segment_.end()) {
      mutex_segment_.unlock();
      NODELET_DEBUG_STREAM("Penultimate setpoint " << idx);
      msg.next = msg.current;
      break;
    // We have two valid set points - if the times are equal, then we will
    // treat the last setpoint as the valid one and move on...
    } else {
      msg.next = *setpoint_;
      if (msg.current.when != msg.next.when) {
        mutex_segment_.unlock();
        NODELET_DEBUG_STREAM("Progressing to setpoint " << idx);
        break;
      }
    }
    mutex_segment_.unlock();
    NODELET_DEBUG_STREAM("Skipping setpoint " << idx);
  } while (msg.current.when == msg.next.when);
  // Send the control
  NODELET_DEBUG_STREAM(msg);
  if (!Control(ff_msgs::ControlCommand::MODE_NOMINAL, msg)) {
    Result(RESPONSE::CONTROL_FAILED);
    return;
  }
  // Set off a timer to wait until the next setpoint
  mutex_segment_.lock();
  ros::Duration delta = msg.next.when - ros::Time::now();
  timer_.stop();
  timer_.setPeriod(delta, true);
  timer_.start();
  mutex_segment_.unlock();
  NODELET_DEBUG_STREAM("Sleep: " << delta);
}

// Goal callback
void Ctl::GoalCallback(ff_msgs::ControlGoalConstPtr const& goal) {
  NODELET_INFO_STREAM("GoalCallback: new control goal received");
  // What have we been instructed to do?
  switch (goal->command) {
    // IDLE - returns instantly
    case ff_msgs::ControlGoal::IDLE:
      NODELET_DEBUG_STREAM("Received IDLE command");
      if (!Control(ff_msgs::ControlCommand::MODE_IDLE))
        Result(RESPONSE::CONTROL_FAILED);
      Result(RESPONSE::SUCCESS);
      return;
    // STOP - returns instantly
    case ff_msgs::ControlGoal::STOP:
      NODELET_DEBUG_STREAM("Received STOP command");
      if (!Control(ff_msgs::ControlCommand::MODE_STOP))
        Result(RESPONSE::CONTROL_FAILED);
      fsm_.Update(GOAL_STOP);
      return;
    // NOMINAL - waits until completion
    case ff_msgs::ControlGoal::NOMINAL: {
      NODELET_DEBUG_STREAM("Received NOMINAL command with segment...");
      // NODELET_DEBUG_STREAM(goal->segment);
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
      static ff_msgs::Segment msg;
      msg.segment = goal->segment;
      segment_pub_.publish(msg);

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
  NODELET_DEBUG_STREAM("CancelCallback called");
  fsm_.Update(GOAL_CANCEL);
}

// Preemption - this might be a replan
void Ctl::PreemptCallback() {
  NODELET_DEBUG_STREAM("PreemptCallback called");
  fsm_.Update(GOAL_CANCEL);
}

// Update a control with a new command
bool Ctl::Control(uint8_t const mode, ff_msgs::ControlCommand const& poseVel) {
  // Set the operating mode
  std::lock_guard<std::mutex> lock(mutex_cmd_msg_);
  auto& input= gnc_.ctl_input_;
  input.ctl_mode_cmd = mode;
  if (mode != ff_msgs::ControlCommand::MODE_NOMINAL)
    return true;

  // NOMINAL: Current setpoint
  auto& curr = poseVel.current;
  input.cmd_state_a.timestamp_sec = curr.when.sec;
  input.cmd_state_a.timestamp_nsec = curr.when.nsec;
  mc::ros_to_array_point(curr.pose.position, input.cmd_state_a.P_B_ISS_ISS);
  mc::ros_to_array_quat(curr.pose.orientation, input.cmd_state_a.quat_ISS2B);
  mc::ros_to_array_vector(curr.twist.linear, input.cmd_state_a.V_B_ISS_ISS);
  mc::ros_to_array_vector(curr.accel.linear, input.cmd_state_a.A_B_ISS_ISS);
  mc::ros_to_array_vector(curr.twist.angular,
    input.cmd_state_a.omega_B_ISS_B);
  mc::ros_to_array_vector(curr.accel.angular,
    input.cmd_state_a.alpha_B_ISS_B);

  // NOMINAL: Next setpoint
  auto& next = poseVel.next;
  input.cmd_state_b.timestamp_sec = next.when.sec;
  input.cmd_state_b.timestamp_nsec = next.when.nsec;
  mc::ros_to_array_point(next.pose.position, input.cmd_state_b.P_B_ISS_ISS);
  mc::ros_to_array_quat(next.pose.orientation, input.cmd_state_b.quat_ISS2B);
  mc::ros_to_array_vector(next.twist.linear, input.cmd_state_b.V_B_ISS_ISS);
  mc::ros_to_array_vector(next.accel.linear, input.cmd_state_b.A_B_ISS_ISS);
  mc::ros_to_array_vector(next.twist.angular,
    input.cmd_state_b.omega_B_ISS_B);
  mc::ros_to_array_vector(next.accel.angular,
    input.cmd_state_b.alpha_B_ISS_B);

  // SUCCESS
  return true;
}

bool Ctl::Step(void) {
  // Step GNC forward
  {
    if (!inertia_received_) {
      NODELET_DEBUG_STREAM_THROTTLE(10, "GNC step waiting for inertia");
      return false;
    }
    if (!flight_enabled_ || !control_enabled_) {
      NODELET_DEBUG_STREAM_THROTTLE(10, "GNC control disabled in flight mode");
      return false;
    }
    std::lock_guard<std::mutex> cmd_lock(mutex_cmd_msg_);
    gnc_.Step();
  }

  // Output of the Step() function is FAM control (ctl)
  auto& cmd = gnc_.cmd_;
  auto& ctl = gnc_.ctl_;
  auto& input= gnc_.ctl_input_;

  // Publish the FAM command
  static ff_msgs::FamCommand cmd_msg_;
  cmd_msg_.header.stamp = ros::Time::now();
  cmd_msg_.header.frame_id = "body";
  cmd_msg_.wrench.force = mc::array_to_ros_vector(ctl.body_force_cmd);
  cmd_msg_.wrench.torque = mc::array_to_ros_vector(ctl.body_torque_cmd);
  cmd_msg_.accel = mc::array_to_ros_vector(ctl.body_accel_cmd);
  cmd_msg_.alpha = mc::array_to_ros_vector(ctl.body_alpha_cmd);
  cmd_msg_.position_error = mc::array_to_ros_vector(ctl.pos_err);
  cmd_msg_.position_error_integrated = mc::array_to_ros_vector(ctl.pos_err_int);
  cmd_msg_.attitude_error = mc::array_to_ros_vector(ctl.att_err);
  cmd_msg_.attitude_error_integrated = mc::array_to_ros_vector(ctl.att_err_int);
  cmd_msg_.attitude_error_mag = ctl.att_err_mag;
  cmd_msg_.status = ctl.ctl_status;
  cmd_msg_.control_mode = cmd.cmd_mode;
  ctl_pub_.publish(cmd_msg_);

  // Publish the traj message
  static ff_msgs::ControlState current;
  current.when = cmd_msg_.header.stamp;
  current.pose.position = msg_conversions::array_to_ros_point(cmd.traj_pos);
  current.pose.orientation = msg_conversions::array_to_ros_quat(cmd.traj_quat);
  current.twist.linear = msg_conversions::array_to_ros_vector(cmd.traj_vel);
  current.twist.angular = msg_conversions::array_to_ros_vector(cmd.traj_omega);
  current.accel.linear = msg_conversions::array_to_ros_vector(cmd.traj_accel);
  current.accel.angular = msg_conversions::array_to_ros_vector(cmd.traj_alpha);
  traj_pub_.publish(current);

  // Publish the current setpoint,
  if ((fsm_.GetState() == NOMINAL) || (fsm_.GetState() == STOPPING)) {
    std::lock_guard<std::mutex> lock(mutex_segment_);
    static ff_msgs::ControlFeedback feedback;
    feedback.setpoint.when = cmd_msg_.header.stamp;
    feedback.setpoint.pose.position = mc::array_to_ros_point(cmd.traj_pos);
    feedback.setpoint.pose.orientation = mc::array_to_ros_quat(cmd.traj_quat);
    feedback.setpoint.twist.linear = mc::array_to_ros_vector(cmd.traj_vel);
    feedback.setpoint.twist.angular = mc::array_to_ros_vector(cmd.traj_omega);
    feedback.setpoint.accel.linear = mc::array_to_ros_vector(cmd.traj_accel);
    feedback.setpoint.accel.angular = mc::array_to_ros_vector(cmd.traj_alpha);
    feedback.index = std::distance(setpoint_, segment_.begin());
    // Sometimese segments arrive with a first setpoint that has a time stamp
    // sometime in the future. In this case we will be in STOPPED mode until
    // the callback timer sets the CMC mode to NOMINAL. In the interim the
    // errors being sent back from the controller make no sense.
    switch (input.ctl_mode_cmd) {
    case ff_msgs::ControlCommand::MODE_NOMINAL:
      feedback.error_position = ctl.traj_error_pos;
      feedback.error_attitude = ctl.traj_error_att;
      feedback.error_velocity = ctl.traj_error_vel;
      feedback.error_omega    = ctl.traj_error_omega;
      break;
    default:
      feedback.error_position = 0;
      feedback.error_attitude = 0;
      feedback.error_velocity = 0;
      feedback.error_omega    = 0;
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
    ROS_ERROR("Failed to read config files.");
    return;
  }
  gnc_.ReadParams(&config_);
  if (!config_.GetBool("tun_debug_ctl_use_truth", &use_truth_))
    ROS_FATAL("tun_debug_ctl_use_truth not specified.");
  // Set linear- and angular velocity threshold for ekf
  // to decide that Astrobee is dynamically stopped.
  if (!config_.GetReal("tun_ctl_stopping_vel_thresh", &stopping_vel_thresh_squared_))
    ROS_FATAL("tun_ctl_stopping_vel_thresh not specified.");
  if (!config_.GetReal("tun_ctl_stopping_omega_thresh", &stopping_omega_thresh_squared_))
    ROS_FATAL("tun_ctl_stopping_omega_thresh not specified.");
}

}  // end namespace ctl
