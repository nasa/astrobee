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

#include <Eigen/QR>

// parameters fam_force_allocation_module_P are set in
//  matlab/code_generation/fam_force_allocation_module_ert_rtw/fam_force_allocation_module_data.c

using std::placeholders::_1;

namespace fam {

Fam::Fam(NodeHandle & nh) : inertia_received_(false) {
  // config_.AddFile("gnc.config");
  // config_.AddFile("geometry.config");
  // ReadParams();
  // config_timer_ = nh->createTimer(ros::Duration(1), [this](ros::TimerEvent e) {
  //     config_.CheckFilesUpdated(std::bind(&Fam::ReadParams, this));}, false, true);

  clock_ = nh->get_clock();

  pmc_pub_ = FF_CREATE_PUBLISHER(nh, ff_hw_msgs::msg::PmcCommand, TOPIC_HARDWARE_PMC_COMMAND, 1);

  // Subscribe to the flight mode to be notified of speed gain command
  flight_mode_sub_ = FF_CREATE_SUBSCRIBER(nh, ff_msgs::msg::FlightMode,
    TOPIC_MOBILITY_FLIGHT_MODE, 1,
    std::bind(&Fam::FlightModeCallback, this, std::placeholders::_1));
  inertia_sub_ = FF_CREATE_SUBSCRIBER(nh, geometry_msgs::msg::InertiaStamped,
    TOPIC_MOBILITY_INERTIA, 1,
    std::bind(&Fam::InertiaCallback, this, std::placeholders::_1));

  ctl_sub_ = FF_CREATE_SUBSCRIBER(nh, ff_msgs::msg::FamCommand,
    TOPIC_GNC_CTL_COMMAND, 5,
    std::bind(&Fam::CtlCallBack, this, std::placeholders::_1));
}

Fam::~Fam() {}

void Fam::CtlCallBack(const std::shared_ptr<ff_msgs::msg::FamCommand> c) {
  input_.body_force_cmd =  msg_conversions::ros_to_eigen_vector(c->wrench.force).cast<float>();
  input_.body_torque_cmd =  msg_conversions::ros_to_eigen_vector(c->wrench.torque).cast<float>();

  Step();
}

void Fam::FlightModeCallback(const std::shared_ptr<ff_msgs::msg::FlightMode> mode) {
  std::lock_guard<std::mutex> lock(mutex_speed_);
  input_.speed_gain_cmd = mode->speed;
}

void Fam::InertiaCallback(const std::shared_ptr<geometry_msgs::msg::InertiaStamped> inertia) {
  std::lock_guard<std::mutex> lock(mutex_mass_);
  center_of_mass_ = msg_conversions::ros_to_eigen_vector(inertia->inertia.com).cast<float>();
  fam_.UpdateCOM(center_of_mass_);
  inertia_received_ = true;
}

void Fam::Step() {
  if (!inertia_received_) {
    return;
  }

  // Step the FAM simulink code
  uint8_t speed_cmd[2];
  Eigen::Matrix<float, 12, 1> servo_pwm_cmd;
  {
    std::lock_guard<std::mutex> lock_mass(mutex_mass_);
    std::lock_guard<std::mutex> lock(mutex_speed_);
    fam_.Step(input_, speed_cmd, servo_pwm_cmd);
  }

  // Send the PMC command
  static ff_hw_msgs::msg::PmcCommand pmc;
  pmc.header.stamp = clock_->now();
  pmc.header.frame_id = "body";
  pmc.goals.resize(2);
  pmc.goals[0].motor_speed = speed_cmd[0];
  pmc.goals[1].motor_speed = speed_cmd[1];
  for (int i = 0; i < 6; i++) {
    pmc.goals[0].nozzle_positions[i] = (unsigned char)servo_pwm_cmd[i];
    pmc.goals[1].nozzle_positions[i] = (unsigned char)servo_pwm_cmd[6 + i];
  }
  pmc_pub_->publish<ff_hw_msgs::msg::PmcCommand>(pmc);
}

// void Fam::ReadParams(void) {
//   if (!config_.ReadFiles()) {
//     ROS_ERROR("Failed to read config files.");
//     return;
//   }
// }

}  // end namespace fam
