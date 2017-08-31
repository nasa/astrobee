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

#include <gnc_ros_wrapper/ctl.h>

#include <nodelet/nodelet.h>

#include <ff_msgs/ControlState.h>
#include <ff_msgs/FamCommand.h>
#include <msg_conversions/msg_conversions.h>

#include <ff_util/ff_names.h>

#include <Eigen/Dense>
#include <Eigen/Geometry>

// parameters ctl_controller0_P are set in
//   matlab/code_generation/ctl_controller0_ert_rtw/ctl_controller0_data.c

namespace gnc_ros_wrapper {  // Nodehandle for <robot>/gnc/wrapper
Ctl::Ctl(cmc_msg * cmc, ros::NodeHandle* nh, std::string const& name) :
  gnc_(cmc),
  ctl_enabled_(false) {
  gnc_.Initialize();
  // Save the node name
  name_ = name;

  // Set up subscribers for the pose, twist and acceleration updates
  truth_pose_sub_ =
      nh->subscribe(TOPIC_LOCALIZATION_TRUTH, 1, &Ctl::TruthPoseCallback, this);

  // Set up publishers
  ctl_pub_ = nh->advertise<ff_msgs::FamCommand>(TOPIC_GNC_CTL_COMMAND, 5);
  traj_pub_ = nh->advertise<ff_msgs::ControlState>(TOPIC_GNC_CTL_TRAJ, 5);
  segment_pub_ = nh->advertise<ff_msgs::ControlGoal>(TOPIC_GNC_CTL_SEGMENT, 5,
                                                     true);  // Latched
  progress_pub_ =
      nh->advertise<ff_msgs::ControlProgress>(TOPIC_GNC_CTL_PROGRESS, 5);

  // Enable or disable control completely
  enable_srv_ = nh->advertiseService(SERVICE_GNC_CTL_ENABLE,
                                     &Ctl::ControlEnableService, this);
  inertia_srv_ = nh->advertiseService(SERVICE_GNC_SET_INERTIA,
                                      &Ctl::SetInertiaService, this);

  // Start a simple action client to accept control
  action_.SetGoalCallback(
      std::bind(&Ctl::GoalCallback, this, std::placeholders::_1));
  action_.SetPreemptCallback(std::bind(&Ctl::PreemptCallback, this));
  action_.SetCancelCallback(std::bind(&Ctl::CancelCallback, this));
  action_.Create(nh, ACTION_GNC_CTL_CONTROL);

  // This timer will be used to throttle (deadline-based) control to GNC
  timer_c_ = nh->createTimer(ros::Duration(0), &Ctl::ControlTimerCallback, this,
                             true, false);
}

Ctl::~Ctl() {}

// Terminate execution in either IDLE or STOP mode
void Ctl::Terminate(uint8_t mode) {
  // Stop feeding commands to GNC
  timer_c_.stop();
  // Thread-safe reset of the segment and setpoint
  mutex_segment_setpoint_.lock();
  segment_.clear();
  setpoint_ = segment_.end();
  mutex_segment_setpoint_.unlock();
  // Send a stop command to avoid drift
  ff_msgs::ControlCommand msg;
  msg.mode = mode;
  UpdateControl(msg);
}

void Ctl::Complete(int32_t response) {
  NODELET_DEBUG_STREAM("Complete with response code " << response);
  // If something goes wrong, stop.
  if (response != ff_util::FreeFlyerActionState::SUCCESS)
    Terminate(ff_msgs::ControlCommand::MODE_STOP);
  // Make sure that we have a valid simple action server
  mutex_segment_setpoint_.lock();
  ff_msgs::ControlResult control_result;
  control_result.response = response;
  control_result.progress = feedback_.progress;
  control_result.segment = segment_;
  if (response > 0)
    action_.SendResult(ff_util::FreeFlyerActionState::SUCCESS, control_result);
  else if (response < 0)
    action_.SendResult(ff_util::FreeFlyerActionState::ABORTED, control_result);
  else
    action_.SendResult(ff_util::FreeFlyerActionState::PREEMPTED,
                       control_result);
  mutex_segment_setpoint_.unlock();
}

void Ctl::ControlTimerCallback(const ros::TimerEvent& event) {
  // If we get here then we must send two setpoints to GNC -- one for the
  // current point in time and the next for a future point in time
  ff_msgs::ControlCommand msg;
  msg.mode = ff_msgs::ControlCommand::MODE_NOMINAL;
  mutex_segment_setpoint_.lock();
  do {
    msg.current = *setpoint_;
    if (++setpoint_ == segment_.end()) {
      msg.next = msg.current;
      mutex_segment_setpoint_.unlock();
      UpdateControl(msg);
      return Complete(ff_msgs::ControlResult::SUCCESS);
    }
    msg.next = *setpoint_;
    if (msg.current.when == msg.next.when) {
      NODELET_DEBUG_STREAM("Skipping setpoint "
                           << std::distance(segment_.begin(), setpoint_ - 1)
                           << " because it shares a timestamp with setpoint "
                           << std::distance(segment_.begin(), setpoint_));
    } else {
      NODELET_DEBUG_STREAM("Progressing to setpoint "
                           << std::distance(segment_.begin(), setpoint_ - 1));
    }
    // Two different setpoints with the same time are not valid (we need to skip
    // the first one)
  } while (msg.current.when == msg.next.when);
  mutex_segment_setpoint_.unlock();
  // Special case: we receive two setpoints with the same time epoch.
  UpdateControl(msg);
  // Set off a timer to wait until the next setpoint
  ros::Duration deferral = msg.next.when - ros::Time::now();
  timer_c_.stop();
  timer_c_.setPeriod(deferral, true);
  timer_c_.start();
}

// If we have a ground truth
void Ctl::TruthPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& truth) {
  std::lock_guard<std::mutex> lock(mutex_env_msg_);
  // Remap ENV for convenience
  auto& env = gnc_.env_;
  msg_conversions::ros_to_array_point(truth->pose.position, env.P_B_ISS_ISS);
  msg_conversions::ros_to_array_quat(truth->pose.orientation, env.Q_ISS2B);
}

// Goal callback
void Ctl::GoalCallback(ff_msgs::ControlGoalConstPtr const& control_goal) {
  NODELET_DEBUG_STREAM("New CONTROL goal received");
  // What have we been instructed to do?
  switch (control_goal->mode) {
    // An IDLE or STOP command must be actioned immediately
    case ff_msgs::ControlCommand::MODE_IDLE: {
      Terminate(ff_msgs::ControlCommand::MODE_STOP);
      return Complete(ff_msgs::ControlResult::SUCCESS);
    } break;
    case ff_msgs::ControlCommand::MODE_STOP: {
      Terminate(ff_msgs::ControlCommand::MODE_IDLE);
      return Complete(ff_msgs::ControlResult::SUCCESS);
    } break;
    // If we receive a NOMINAL command
    case ff_msgs::ControlCommand::MODE_NOMINAL: {
      // First, check the segment for validity. We don't want to be processing
      // bad data!
      ff_util::SegmentResult ret = ff_util::SegmentUtil::Check(
          ff_util::CHECK_ALL, control_goal->segment, control_goal->flight_mode);
      if (ret != ff_util::SUCCESS) {
        NODELET_ERROR_STREAM(*control_goal);
        NODELET_WARN_STREAM("Control failed: " << ff_util::SegmentUtil::GetDescription(ret));
        return Complete(ff_msgs::ControlResult::SEGMENT_INVALID);
      }
      // How long between now and the start time
      ros::Duration deferral =
          control_goal->segment.front().when - ros::Time::now();
      if (deferral.toSec() < -general_.time_sync_threshold)
        return Complete(ff_msgs::ControlResult::POTENTIAL_TIME_SYNC_ISSUE);
      // If we are in the middle of processing a segment, then we need to
      // iterate over the current
      // segment looking for the handover timestamp. In order to handover in a
      // safe way, we also
      // need to ensure that the position, orientaiton, velocity  and angular
      // velocity match.
      mutex_segment_setpoint_.lock();
      mutex_flight_mode_.lock();
      if (!segment_.empty()) {
        NODELET_DEBUG_STREAM(
            "We are currently processing an existing segment.");
        if (!flight_mode_.name.compare(control_goal->flight_mode)) {
          std::vector<ff_msgs::ControlState>::iterator it;
          for (it = segment_.begin(); it != segment_.end(); it++) {
            if (it->when == control_goal->segment.front().when) {
              NODELET_DEBUG_STREAM(
                  "We found a matching setpoint. Assuming this is a replan.");
              segment_.erase(it, segment_.end());
              segment_.insert(segment_.end(), control_goal->segment.begin(),
                              control_goal->segment.end());
              segment_pub_.publish(*control_goal);
              mutex_flight_mode_.unlock();
              mutex_segment_setpoint_.unlock();
              return;
            }
          }
          NODELET_DEBUG_STREAM(
              "We couldn't match any set point in flight, so assuming this is "
              "a new segment.");
        } else {
          NODELET_DEBUG_STREAM(
              "Flight modes mismatch, so replanning is not possible.");
        }
      } else {
        NODELET_DEBUG_STREAM(
            "We are not currently processing an existing segment, so assuming "
            "this is a new segment.");
      }
      mutex_segment_setpoint_.unlock();
      mutex_flight_mode_.unlock();
      // If we get to this point it means that we must treat the incoming
      // segment as a new
      // segment, either because it is new or because continuation failed. For
      // safety, we'll
      // issue a STOP command, just in case the start of the new segment is in
      // the future.
      Terminate(ff_msgs::ControlCommand::MODE_STOP);
      // Switch to the requested flight mode
      mutex_flight_mode_.lock();
      if (!ff_util::ConfigUtil::GetFlightMode(control_goal->flight_mode,
                                              flight_mode_)) {
        mutex_flight_mode_.unlock();
        return Complete(ff_msgs::ControlResult::SEGMENT_INVALID);
      }
      mutex_flight_mode_.unlock();
      // Switch to the new parameters
      UpdateParams();
      // Set the new segment and reset the set point
      mutex_segment_setpoint_.lock();
      segment_ = control_goal->segment;
      setpoint_ = segment_.begin();
      timer_c_.stop();
      timer_c_.setPeriod(deferral, true);
      timer_c_.start();
      mutex_segment_setpoint_.unlock();
      // Defer the start of execution
      NODELET_DEBUG_STREAM("Waiting " << deferral.toSec()
                                      << " seconds to start");
      // Publish the new segment
      segment_pub_.publish(*control_goal);
    } break;
    // No other control mode makes sense
    default:
      return Complete(ff_msgs::ControlResult::INVALID_CONTROL_MODE);
      break;
  }
}

// Cancellation
void Ctl::CancelCallback() {
  ROS_INFO_STREAM("Existing CONTROL interrupted");
  Complete(ff_msgs::ControlResult::CANCELLED);
}

// Preemption
void Ctl::PreemptCallback() { Complete(ff_msgs::ControlResult::PREEMPTED); }

void Ctl::UpdateControl(const ff_msgs::ControlCommand& poseVel) {
  // concurrency protection
  std::lock_guard<std::mutex> lock(mutex_cmd_msg_);

  // Current setpoint
  auto& current = poseVel.current;
  gnc_.cmc_->cmc_state_cmd_a.timestamp_sec = current.when.sec;
  gnc_.cmc_->cmc_state_cmd_a.timestamp_nsec = current.when.nsec;
  msg_conversions::ros_to_array_point(current.pose.position,
                                      gnc_.cmc_->cmc_state_cmd_a.P_B_ISS_ISS);
  msg_conversions::ros_to_array_quat(current.pose.orientation,
                                     gnc_.cmc_->cmc_state_cmd_a.quat_ISS2B);
  msg_conversions::ros_to_array_vector(current.twist.linear,
                                       gnc_.cmc_->cmc_state_cmd_a.V_B_ISS_ISS);
  msg_conversions::ros_to_array_vector(current.accel.linear,
                                       gnc_.cmc_->cmc_state_cmd_a.A_B_ISS_ISS);
  msg_conversions::ros_to_array_vector(current.twist.angular,
                                       gnc_.cmc_->cmc_state_cmd_a.omega_B_ISS_B);
  msg_conversions::ros_to_array_vector(current.accel.angular,
                                       gnc_.cmc_->cmc_state_cmd_a.alpha_B_ISS_B);

  // Next setpoint
  auto& next = poseVel.next;
  gnc_.cmc_->cmc_state_cmd_b.timestamp_sec = next.when.sec;
  gnc_.cmc_->cmc_state_cmd_b.timestamp_nsec = next.when.nsec;
  msg_conversions::ros_to_array_point(next.pose.position,
                                      gnc_.cmc_->cmc_state_cmd_b.P_B_ISS_ISS);
  msg_conversions::ros_to_array_quat(next.pose.orientation,
                                     gnc_.cmc_->cmc_state_cmd_b.quat_ISS2B);
  msg_conversions::ros_to_array_vector(next.twist.linear,
                                       gnc_.cmc_->cmc_state_cmd_b.V_B_ISS_ISS);
  msg_conversions::ros_to_array_vector(next.accel.linear,
                                       gnc_.cmc_->cmc_state_cmd_b.A_B_ISS_ISS);
  msg_conversions::ros_to_array_vector(next.twist.angular,
                                       gnc_.cmc_->cmc_state_cmd_b.omega_B_ISS_B);
  msg_conversions::ros_to_array_vector(next.accel.angular,
                                       gnc_.cmc_->cmc_state_cmd_b.alpha_B_ISS_B);

  // set mode based on whether control is enabled or idle
  gnc_.cmc_->speed_gain_cmd = 3;
  if (!ctl_enabled_)
    gnc_.cmc_->cmc_mode_cmd = 0;
  else
    gnc_.cmc_->cmc_mode_cmd = poseVel.mode;
}

void Ctl::Step(kfl_msg* kfl) {
  ros::Time now = ros::Time::now();
  gnc_.time_.timestamp_sec = now.sec;
  gnc_.time_.timestamp_nsec = now.nsec;

  // Step GNC forward
  {
    std::lock_guard<std::mutex> env_lock(mutex_env_msg_);
    std::lock_guard<std::mutex> cmd_lock(mutex_cmd_msg_);
    gnc_.Step(kfl);
  }

  // publish the force and torque
  ff_msgs::FamCommand cmd_msg_;
  cmd_msg_.header.stamp = ros::Time::now();
  cmd_msg_.header.frame_id = "body";
  cmd_msg_.wrench.force =
      msg_conversions::array_to_ros_vector(gnc_.ctl_.body_force_cmd);
  cmd_msg_.wrench.torque =
      msg_conversions::array_to_ros_vector(gnc_.ctl_.body_torque_cmd);
  cmd_msg_.accel =
      msg_conversions::array_to_ros_vector(gnc_.ctl_.body_accel_cmd);
  cmd_msg_.alpha =
      msg_conversions::array_to_ros_vector(gnc_.ctl_.body_alpha_cmd);
  cmd_msg_.position_error =
      msg_conversions::array_to_ros_vector(gnc_.ctl_.pos_err);
  cmd_msg_.position_error_integrated =
      msg_conversions::array_to_ros_vector(gnc_.ctl_.pos_err_int);
  cmd_msg_.attitude_error =
      msg_conversions::array_to_ros_vector(gnc_.ctl_.att_err);
  cmd_msg_.attitude_error_integrated =
      msg_conversions::array_to_ros_vector(gnc_.ctl_.att_err_int);
  cmd_msg_.attitude_error_mag = gnc_.ctl_.att_err_mag;
  cmd_msg_.status = gnc_.ctl_.ctl_status;
  cmd_msg_.control_mode = gnc_.cmd_.cmd_mode;
  ctl_pub_.publish<ff_msgs::FamCommand>(cmd_msg_);

  auto& cmd = gnc_.cmd_;

  ff_msgs::ControlState current;
  // set timestamp
  current.when.sec = now.sec;
  current.when.nsec = now.nsec;

  // Publish the time advanced version of the input trajectory
  current.pose.position = msg_conversions::array_to_ros_point(cmd.traj_pos);
  current.pose.orientation = msg_conversions::array_to_ros_quat(cmd.traj_quat);
  current.twist.linear = msg_conversions::array_to_ros_vector(cmd.traj_vel);
  current.twist.angular = msg_conversions::array_to_ros_vector(cmd.traj_omega);
  current.accel.linear = msg_conversions::array_to_ros_vector(cmd.traj_accel);
  current.accel.angular = msg_conversions::array_to_ros_vector(cmd.traj_alpha);
  traj_pub_.publish<ff_msgs::ControlState>(current);

  // In the case where we have no segment or we are waiting to reach the first
  // set point, then we
  // might be moving to the start of an EXECUTE command. Signal with feedback
  // that we are waiting
  // to start the segment, so that the chroeographer can handle this correctly.
  mutex_segment_setpoint_.lock();
  ff_msgs::ControlFeedback feedback;
  feedback.progress.header.stamp = ros::Time::now();
  feedback.progress.waiting_to_start = true;
  if (segment_.empty() ||
      (!segment_.empty() && setpoint_->when == segment_.begin()->when)) {
    if (!segment_.empty()) {
      action_.SendFeedback(feedback);
      progress_pub_.publish(feedback.progress);
    }
    mutex_segment_setpoint_.unlock();
    return;
  }
  mutex_segment_setpoint_.unlock();

  // Tolerance checking only really makes sense if we are in nominal mode
  if (cmd_msg_.control_mode != ff_msgs::ControlCommand::MODE_NOMINAL) return;

  // If we get here, then we are active and need to examine the GNC errors
  feedback.progress.error_position = gnc_.ctl_.traj_error_pos;
  feedback.progress.error_attitude = gnc_.ctl_.traj_error_att;
  feedback.progress.error_velocity = gnc_.ctl_.traj_error_vel;
  feedback.progress.error_omega = gnc_.ctl_.traj_error_omega;

  // Thread-safe tolerance checking. This must be called before progress is
  // sent, in order for
  // move-to-start to perform as expected.
  mutex_flight_mode_.lock();
  if (flight_mode_.tolerance_pos > 0.0 &&
      feedback.progress.error_position > flight_mode_.tolerance_pos) {
    NODELET_WARN_STREAM("Position tolerance violated");
    NODELET_WARN_STREAM("- Value: " << feedback.progress.error_position
                                    << ", Thresh: "
                                    << flight_mode_.tolerance_pos);
    mutex_flight_mode_.unlock();
    Complete(ff_msgs::ControlResult::TOLERANCE_VIOLATION_POSITION);
    return;
  }
  if (flight_mode_.tolerance_att > 0.0 &&
      feedback.progress.error_attitude > flight_mode_.tolerance_att) {
    NODELET_WARN_STREAM("Attitude tolerance violated");
    NODELET_WARN_STREAM("- Value: " << feedback.progress.error_attitude
                                    << ", Thresh: "
                                    << flight_mode_.tolerance_att);
    mutex_flight_mode_.unlock();
    Complete(ff_msgs::ControlResult::TOLERANCE_VIOLATION_ATTITUDE);
    return;
  }
  if (flight_mode_.tolerance_vel > 0.0 &&
      feedback.progress.error_velocity > flight_mode_.tolerance_vel) {
    NODELET_WARN_STREAM("Velocity tolerance violated");
    NODELET_WARN_STREAM("- Value: " << feedback.progress.error_velocity
                                    << ", Thresh: "
                                    << flight_mode_.tolerance_vel);
    mutex_flight_mode_.unlock();
    Complete(ff_msgs::ControlResult::TOLERANCE_VIOLATION_VELOCITY);
    return;
  }
  if (flight_mode_.tolerance_omega > 0.0 &&
      feedback.progress.error_omega > flight_mode_.tolerance_omega) {
    NODELET_WARN_STREAM("Angular velocity tolerance violated");
    NODELET_WARN_STREAM("- Value: " << feedback.progress.error_omega
                                    << ", Thresh: "
                                    << flight_mode_.tolerance_omega);
    mutex_flight_mode_.unlock();
    Complete(ff_msgs::ControlResult::TOLERANCE_VIOLATION_OMEGA);
    return;
  }
  mutex_flight_mode_.unlock();

  // We only send progress AFTER tolerance checking. The reason is that
  // move-to-start can only
  // be invoked if we have not had one tolerance check succeed since the segment
  // activated
  mutex_segment_setpoint_.lock();
  feedback.progress.waiting_to_start = false;
  feedback.progress.index = std::distance(segment_.begin(), setpoint_);
  feedback.progress.goal_setpoint = current;
  if (setpoint_ < segment_.end()) {
    feedback.progress.prev_setpoint = *setpoint_;
    if (setpoint_ + 1 < segment_.end())
      feedback.progress.next_setpoint = *(setpoint_ + 1);
  }
  action_.SendFeedback(feedback);
  progress_pub_.publish(feedback.progress);
  feedback_ = feedback;
  mutex_segment_setpoint_.unlock();
}

void Ctl::EnableControl(bool enable) { ctl_enabled_ = enable; }

bool Ctl::ControlEnableService(ff_msgs::SetBool::Request& req,
                               ff_msgs::SetBool::Response& res) {
  EnableControl(req.enable);
  res.success = true;
  return true;
}

bool Ctl::SetInertiaService(ff_msgs::SetInertia::Request& req,
                            ff_msgs::SetInertia::Response& res) {
  mutex_inertia_.lock();
  inertia_.mass = req.inertia.m;
  inertia_.center_of_mass =
      msg_conversions::ros_to_eigen_vector(req.inertia.com);
  inertia_.inertia_matrix(0, 0) = req.inertia.ixx;
  inertia_.inertia_matrix(0, 1) = req.inertia.ixy;
  inertia_.inertia_matrix(0, 2) = req.inertia.ixz;
  inertia_.inertia_matrix(1, 0) = req.inertia.ixy;
  inertia_.inertia_matrix(1, 1) = req.inertia.iyy;
  inertia_.inertia_matrix(1, 2) = req.inertia.iyz;
  inertia_.inertia_matrix(2, 0) = req.inertia.ixz;
  inertia_.inertia_matrix(2, 1) = req.inertia.iyz;
  inertia_.inertia_matrix(2, 2) = req.inertia.izz;
  mutex_inertia_.unlock();
  UpdateParams();
  return true;
}

void Ctl::ReadParams(config_reader::ConfigReader* config) {
  gnc_.ReadParams(config);
  if (!ff_util::ConfigUtil::GetGeneralConfig(general_))
    ROS_FATAL_STREAM("General config was not found.");
  if (!ff_util::ConfigUtil::GetInertiaConfig(inertia_))
    ROS_FATAL_STREAM("inertia config was not found.");
  if (!ff_util::ConfigUtil::GetFlightMode(general_.default_flight_mode, flight_mode_))
    ROS_FATAL_STREAM("Default flight mode was not found.");
  UpdateParams();
}

void Ctl::UpdateParams() {
  std::lock_guard<std::mutex> lock(mutex_cmd_msg_);
  mutex_flight_mode_.lock();
  msg_conversions::eigen_to_array_vector(flight_mode_.att_kp, gnc_.cmc_->att_kp);
  msg_conversions::eigen_to_array_vector(flight_mode_.att_ki, gnc_.cmc_->att_ki);
  msg_conversions::eigen_to_array_vector(flight_mode_.omega_kd, gnc_.cmc_->omega_kd);
  msg_conversions::eigen_to_array_vector(flight_mode_.pos_kp, gnc_.cmc_->pos_kp);
  msg_conversions::eigen_to_array_vector(flight_mode_.pos_ki, gnc_.cmc_->pos_ki);
  msg_conversions::eigen_to_array_vector(flight_mode_.vel_kd, gnc_.cmc_->vel_kd);
  mutex_flight_mode_.unlock();
  mutex_inertia_.lock();
  gnc_.cmc_->mass = inertia_.mass;
  msg_conversions::eigen_to_array_vector(inertia_.center_of_mass,
    gnc_.cmc_->center_of_mass);
  for (size_t i = 0; i < 9; i++)
    gnc_.cmc_->inertia_matrix[i] = inertia_.inertia_matrix(i / 3, i % 3);
  mutex_inertia_.unlock();
}

std::string Ctl::getName() { return name_; }

}  // end namespace gnc_ros_wrapper
