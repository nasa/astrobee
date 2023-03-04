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

#ifndef CTL_CTL_ROS_H_
#define CTL_CTL_ROS_H_

#include <ctl/ctl.h>

// Standard messages
#include <geometry_msgs/msg/inertia_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

// Service message
#include <std_srvs/srv/set_bool.hpp>

#include <ff_common/ff_ros.h>

// FSW actions
#include <ff_msgs/action/control.hpp>

// FSW messages
#include <ff_msgs/msg/control_command.hpp>
#include <ff_msgs/msg/fam_command.hpp>
#include <ff_msgs/msg/flight_mode.hpp>
#include <ff_msgs/msg/ekf_state.hpp>
#include <ff_msgs/msg/segment.hpp>

// Libraries to handle flight config and segment processing
#include <ff_util/ff_flight.h>
#include <ff_util/ff_action.h>
#include <ff_util/ff_fsm.h>

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
    NOMINAL        = 2,
    STOPPING       = 3
  };

  // Declaration of all possible events
  enum : ff_util::FSM::Event {
    GOAL_COMPLETE  = (1<<0),
    GOAL_NOMINAL   = (1<<1),
    GOAL_CANCEL    = (1<<2),
    GOAL_STOP      = (1<<3)
  };

  // Maximum acceptable latency
  static constexpr double MAX_LATENCY = 0.5;

  /**
   * Ctl allocate, register, initialize model
   */
  explicit Ctl(NodeHandle& nh, std::string const& name);
  /**
   * destruct model
   */
  ~Ctl();

  // Terminate execution in either IDLE or STOP mode
  FSM::State Result(int32_t response);

  // SERVICE CALLBACKS

  // Enable/Disable Onboard Controller
  bool EnableCtl(const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
                      std::shared_ptr<std_srvs::srv::SetBool::Response> response);

  // GENERAL MESSAGE CALLBACKS

  // Called when the internal state changes
  void UpdateCallback(FSM::State const& state, FSM::Event const& event);

  // Called when a pose estimate is available
  void EkfCallback(const std::shared_ptr<ff_msgs::msg::EkfState> state);

  // Called when localization has a new pose data
  void PoseCallback(const std::shared_ptr<geometry_msgs::msg::PoseStamped> truth);

  // Called when localization has new twist data
  void TwistCallback(const std::shared_ptr<geometry_msgs::msg::TwistStamped> truth);

  // Called when management updates inertial info
  void InertiaCallback(const std::shared_ptr<geometry_msgs::msg::InertiaStamped> inertia);

  // Called when the choreographer updates flight modes
  void FlightModeCallback(const std::shared_ptr<ff_msgs::msg::FlightMode> mode);

  // Command GNC directly, bypassing the action-base dinterface
  void SetpointCallback(const std::shared_ptr<ff_msgs::msg::ControlState> command);

  // Used to feed segments
  void TimerCallback();

  // ACTION CLIENT

  // Called when a new goal arrives
  void GoalCallback(std::shared_ptr<const ff_msgs::action::Control::Goal> goal);

  // Called when a goal is preempted
  void PreemptCallback();

  // Called when a goal is cancelled
  void CancelCallback();

  // ORIGINAL GNC FUNCTIONS

  // Update control to take a new setpoint
  bool Command(uint8_t const mode,
    ff_msgs::msg::ControlCommand const& poseVel = ff_msgs::msg::ControlCommand());

  // Step control forward
  bool Step(ros::Time curr_time);

  // Read the control parameters from the LUA config file
  void ReadParams(void);

  // Simple extension to allow NODELET_* logging calls
  std::string getName();

 private:
  // Proxy to gnc
  Control controller_;

  std::mutex mutex_cmd_msg_, mutex_segment_;

  rclcpp::Subscription<geometry_msgs::msg::InertiaStamped>::SharedPtr inertia_sub_;
  rclcpp::Subscription<ff_msgs::msg::FlightMode>::SharedPtr flight_mode_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr twist_sub_;
  rclcpp::Subscription<ff_msgs::msg::EkfState>::SharedPtr ekf_sub_;
  rclcpp::Subscription<ff_msgs::msg::ControlState>::SharedPtr command_sub_;
  rclcpp::Publisher<ff_msgs::msg::FamCommand>::SharedPtr ctl_pub_;
  rclcpp::Publisher<ff_msgs::msg::ControlState>::SharedPtr traj_pub_;
  rclcpp::Publisher<ff_msgs::msg::Segment>::SharedPtr segment_pub_;
  rclcpp::Publisher<ff_msgs::msg::Segment>::SharedPtr progress_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr enable_srv_;
  ff_util::FreeFlyerTimer timer_, config_timer_;

  ff_util::FreeFlyerActionServer<ff_msgs::action::Control> action_;
  ff_util::FSM fsm_;
  ff_util::Segment segment_;
  ff_util::Segment::iterator setpoint_;
  ff_msgs::msg::ControlCommand command_;
  rclcpp::Clock::SharedPtr clock_;

  config_reader::ConfigReader config_;

  uint8_t mode_;
  ControlState state_;

  std::string name_;
  bool inertia_received_;
  bool control_enabled_;
  bool flight_enabled_;
  bool use_truth_;
  float stopping_vel_thresh_squared_;
  float stopping_omega_thresh_squared_;

  float GetCommand(ControlCommand* cmd, ros::Time tim);
};

}  // end namespace ctl

#endif  // CTL_CTL_ROS_H_
