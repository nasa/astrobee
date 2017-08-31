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

#ifndef GNC_ROS_WRAPPER_CTL_H_
#define GNC_ROS_WRAPPER_CTL_H_

// Autocode includes
#include <gnc_autocode/ctl.h>

// For includes
#include <ros/ros.h>

// Standard messages
#include <geometry_msgs/Inertia.h>
#include <geometry_msgs/PoseStamped.h>

// FSW actions
#include <ff_msgs/ControlAction.h>
#include <ff_msgs/ControlCommand.h>
#include <ff_msgs/EkfState.h>
#include <ff_msgs/SetBool.h>
#include <ff_msgs/SetInertia.h>

// Libraries to handle flight config and segment processing
#include <ff_util/ff_flight.h>
#include <ff_util/ff_action.h>

// Libraries to read LIA config file
#include <config_reader/config_reader.h>

// STL includes
#include <string>
#include <vector>
#include <mutex>
#include <memory>

namespace gnc_ros_wrapper {
/**
 * @brief Controller implementation using GNC module
 * @details Controller implementation using GNC module
 */
class Ctl {
 public:
  /**
   * Ctl allocate, register, initialize model
   */
  explicit Ctl(cmc_msg * cmc, ros::NodeHandle* nh, std::string const& name);
  /**
   * destruct model
   */
  ~Ctl();

  /* Added by Andrew */

  // Terminate execution in either IDLE or STOP mode
  void Terminate(uint8_t mode);

  // Complete the  current control with a given response code
  void Complete(int32_t response);

  // Called when a timer has called back to progress control to next setpoint
  void ControlTimerCallback(const ros::TimerEvent & event);

  // If we have a ground truth
  void TruthPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& truth);

  // Action server callbacks
  void GoalCallback(ff_msgs::ControlGoalConstPtr const& control_goal);
  void PreemptCallback();
  void CancelCallback();

  // Read tolerances from the flight mode file
  bool ChangeFlightMode(const std::string &name);

  /* Originally in GNC */

  // Update control to take a new setpoint
  void UpdateControl(const ff_msgs::ControlCommand & poseVel);

  // Run control, taking as input the output of the EKF.
  void Step(kfl_msg* kfl);

  ex_time_msg* GetTimeMsg(void) {return &gnc_.time_;}
  ctl_msg* GetCtlMsg(void) {return &gnc_.ctl_;}
  cmd_msg* GetCmdMsg(void) {return &gnc_.cmd_;}
  int GetFanSpeed(void) {return gnc_.cmc_->speed_gain_cmd;}

  // Read the ctl parameters from the config file
  void ReadParams(config_reader::ConfigReader* config);

  // Push the flight mode specific parameters and inertial
  void UpdateParams();

  // This allws use to use the NODELET_* ros messages
  std::string getName();

 protected:
  bool ControlEnableService(ff_msgs::SetBool::Request& req, ff_msgs::SetBool::Response& res);
  bool SetInertiaService(ff_msgs::SetInertia::Request& req, ff_msgs::SetInertia::Response& res);
  void EnableControl(bool enable);

 private:
  // Proxy to gnc
  gnc_autocode::GncCtlAutocode gnc_;        // Proxy for simulink
  bool ctl_enabled_;                        // Is control enabled?
  std::string name_;                        // Node name
  //  ROS stuff
  ros::Subscriber truth_pose_sub_, truth_twist_sub_, truth_accel_sub_;
  ros::Publisher ctl_pub_, traj_pub_, segment_pub_, progress_pub_;
  ros::ServiceServer enable_srv_;           // Enable control service
  ros::ServiceServer inertia_srv_;          // Set inertia service
  ros::Timer timer_c_;                      // Control timer
  ff_util::FreeFlyerActionServer<ff_msgs::ControlAction> action_;
  // Context
  ff_util::GeneralConfig general_;          // General options - control rate, etc.
  ff_util::InertiaConfig inertia_;          // Inertial config
  ff_util::FlightMode flight_mode_;         // Flight mode
  std::mutex mutex_flight_mode_, mutex_inertia_, mutex_general_;
  // Current segment being processed
  ff_util::Segment segment_;
  ff_util::Segment::iterator setpoint_;
  ff_msgs::ControlFeedback feedback_;
  // Resource locking
  std::mutex mutex_segment_setpoint_, mutex_cmd_msg_, mutex_env_msg_;
};

}  // end namespace gnc_ros_wrapper

#endif  // GNC_ROS_WRAPPER_CTL_H_
