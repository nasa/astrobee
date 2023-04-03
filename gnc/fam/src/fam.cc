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

#include <fam/fam.h>
#include <msg_conversions/msg_conversions.h>
#include <ff_common/ff_names.h>
#include <ff_hw_msgs/PmcCommand.h>

#include <Eigen/QR>

// parameters fam_force_allocation_module_P are set in
//  matlab/code_generation/fam_force_allocation_module_ert_rtw/fam_force_allocation_module_data.c

namespace fam {

Fam::Fam(ros::NodeHandle* nh) : inertia_received_(false) {
  // config_.AddFile("gnc.config");
  // config_.AddFile("geometry.config");
  // ReadParams();
  // config_timer_ = nh->createTimer(ros::Duration(1), [this](ros::TimerEvent e) {
  //     config_.CheckFilesUpdated(std::bind(&Fam::ReadParams, this));}, false, true);

  pt_fam_.Initialize("fam");

  pmc_pub_ = nh->advertise<ff_hw_msgs::PmcCommand>(TOPIC_HARDWARE_PMC_COMMAND, 1);

  // Subscribe to the flight mode to be notified of speed gain command
  flight_mode_sub_ = nh->subscribe(
    TOPIC_MOBILITY_FLIGHT_MODE, 1, &Fam::FlightModeCallback, this);
  inertia_sub_ = nh->subscribe(
    TOPIC_MOBILITY_INERTIA, 1, &Fam::InertiaCallback, this);

  ctl_sub_ = nh->subscribe(TOPIC_GNC_CTL_COMMAND, 5,
    &Fam::CtlCallBack, this, ros::TransportHints().tcpNoDelay());
}

Fam::~Fam() {}

void Fam::CtlCallBack(const ff_msgs::FamCommand & c) {
  input_.body_force_cmd =  msg_conversions::ros_to_eigen_vector(c.wrench.force).cast<float>();
  input_.body_torque_cmd =  msg_conversions::ros_to_eigen_vector(c.wrench.torque).cast<float>();

  Step();
}

void Fam::FlightModeCallback(const ff_msgs::FlightMode::ConstPtr& mode) {
  std::lock_guard<std::mutex> lock(mutex_speed_);
  input_.speed_gain_cmd = mode->speed;
}

void Fam::InertiaCallback(const geometry_msgs::InertiaStamped::ConstPtr& inertia) {
  std::lock_guard<std::mutex> lock(mutex_mass_);
  center_of_mass_ = msg_conversions::ros_to_eigen_vector(inertia->inertia.com).cast<float>();
  fam_.UpdateCOM(center_of_mass_);
  inertia_received_ = true;
}

void Fam::Step() {
  if (!inertia_received_) {
    ROS_DEBUG_STREAM_THROTTLE(10, "FAM step waiting for inertia.");
    return;
  }

  // Step the FAM simulink code
  pt_fam_.Tick();
  uint8_t speed_cmd[2];
  Eigen::Matrix<float, 12, 1> servo_pwm_cmd;
  {
    std::lock_guard<std::mutex> lock_mass(mutex_mass_);
    std::lock_guard<std::mutex> lock(mutex_speed_);
    fam_.Step(input_, speed_cmd, servo_pwm_cmd);
  }
  pt_fam_.Tock();

  // Send the PMC command
  static ff_hw_msgs::PmcCommand pmc;
  pmc.header.stamp = ros::Time::now();
  pmc.header.frame_id = "body";
  pmc.goals.resize(2);
  pmc.goals[0].motor_speed = speed_cmd[0];
  pmc.goals[1].motor_speed = speed_cmd[1];
  for (int i = 0; i < 6; i++) {
    pmc.goals[0].nozzle_positions[i] = (unsigned char)servo_pwm_cmd[i];
    pmc.goals[1].nozzle_positions[i] = (unsigned char)servo_pwm_cmd[6 + i];
  }
  pmc_pub_.publish<ff_hw_msgs::PmcCommand>(pmc);

  pt_fam_.Send();
}

// void Fam::ReadParams(void) {
//   if (!config_.ReadFiles()) {
//     ROS_ERROR("Failed to read config files.");
//     return;
//   }
// }

}  // end namespace fam
