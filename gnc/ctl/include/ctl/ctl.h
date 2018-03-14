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

#ifndef CTL_CTL_H_
#define CTL_CTL_H_

// Autocode includes
#include <gnc_autocode/ctl.h>

// For includes
#include <ros/ros.h>

// Standard messages
#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

// FSW actions
#include <ff_msgs/ControlAction.h>

// FSW messages
#include <ff_msgs/ControlCommand.h>
#include <ff_msgs/FlightMode.h>
#include <ff_msgs/EkfState.h>

// Libraries to handle flight config and segment processing
#include <ff_util/ff_flight.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_fsm.h>
#include <ff_util/perf_timer.h>

// Libraries to read LIA config file
#include <config_reader/config_reader.h>

// STL includes
#include <string>
#include <vector>
#include <mutex>
#include <memory>

namespace ctl {

using FSM = ff_util::FSM;

/**
 * @brief Controller implementation using GNC module
 * @details Controller implementation using GNC module
 */
class Ctl {
 public:
  // Declaration of all possible states
  enum : ff_util::FSM::State {
    WAITING        = 1,
    NOMINAL        = 2
  };

  // Declaration of all possible events
  enum : ff_util::FSM::Event {
    GOAL_COMPLETE  = (1<<0),
    GOAL_NOMINAL   = (1<<1),
    GOAL_CANCEL    = (1<<2)
  };

  // Maximum acceptable latency
  static constexpr double MAX_LATENCY = 0.5;

  /**
   * Ctl allocate, register, initialize model
   */
  explicit Ctl(ros::NodeHandle* nh, std::string const& name);
  /**
   * destruct model
   */
  ~Ctl();

  // Terminate execution in either IDLE or STOP mode
  FSM::State Result(int32_t response);

  // GENERAL MESSAGE CALLBACKS

  // Called when the internal state changes
  void UpdateCallback(FSM::State const& state, FSM::Event const& event);

  // Called when a pose estimate is available
  void EkfCallback(const ff_msgs::EkfState::ConstPtr& state);

  // Called when localization has a new pose data
  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& truth);

  // Called when localization has new twist data
  void TwistCallback(const geometry_msgs::TwistStamped::ConstPtr& truth);

  // Called when management updates inertial info
  void InertiaCallback(const geometry_msgs::Inertia::ConstPtr& inertia);

  // Called when a timer has called back to progress control to next setpoint
  void ControlTimerCallback(const ros::TimerEvent & event);

  // Called when the choreographer updates flight modes
  void FlightModeCallback(const ff_msgs::FlightMode::ConstPtr& mode);

  // Used to feed segments
  void TimerCallback(const ros::TimerEvent& event);

  // ACTION CLIENT

  // Called when a new goal arrives
  void GoalCallback(ff_msgs::ControlGoalConstPtr const& control_goal);

  // Called when a goal is preempted
  void PreemptCallback();

  // Called when a goal is cancelled
  void CancelCallback();

  // ORIGINAL GNC FUNCTIONS

  // Update control to take a new setpoint
  bool Control(uint8_t mode,
    ff_msgs::ControlCommand const& poseVel = ff_msgs::ControlCommand());

  // Step control forward
  bool Step(void);

  ctl_msg* GetCtlMsg(void) {return &gnc_.ctl_;}
  cmd_msg* GetCmdMsg(void) {return &gnc_.cmd_;}

  // Read the control parameters from the LUA config file
  void ReadParams(void);

  // Simple extension to allow NODELET_* logging calls
  std::string getName();

 private:
  // Proxy to gnc
  gnc_autocode::GncCtlAutocode gnc_;

  std::mutex mutex_cmd_msg_, mutex_segment_;

  ros::Subscriber truth_pose_sub_, inertia_sub_, flight_mode_sub_;
  ros::Subscriber twist_sub_, pose_sub_, ekf_sub_;
  ros::Publisher ctl_pub_, traj_pub_, segment_pub_, progress_pub_;
  ros::Timer timer_;

  ff_util::FreeFlyerActionServer<ff_msgs::ControlAction> action_;
  ff_util::FSM fsm_;
  ff_util::Segment segment_;
  ff_util::Segment::iterator setpoint_;
  ff_msgs::ControlFeedback feedback_;

  config_reader::ConfigReader config_;
  ff_util::PerfTimer pt_ctl_;
  ros::Timer config_timer_;

  std::string name_;
  bool inertia_received_;
  bool control_enabled_;
  bool use_truth_;
};

}  // end namespace ctl

#endif  // CTL_CTL_H_
