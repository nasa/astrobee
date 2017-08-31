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

#include <gnc_ros_wrapper/fam.h>
#include <ff_util/ff_names.h>
#include <ff_hw_msgs/PmcCommand.h>

// parameters fam_force_allocation_module_P are set in
//  matlab/code_generation/fam_force_allocation_module_ert_rtw/fam_force_allocation_module_data.c

namespace gnc_ros_wrapper {

Fam::Fam(ros::NodeHandle* nh) {
  pmc_pub_ = nh->advertise<ff_hw_msgs::PmcCommand>(TOPIC_HARDWARE_PMC_COMMAND, 1);
}

Fam::~Fam() {}

void Fam::Step(ex_time_msg* ex_time, cmd_msg* cmd, ctl_msg* ctl) {
  gnc_.Step(ex_time, cmd, ctl);

  ff_hw_msgs::PmcCommand pmc;
  pmc.header.stamp = ros::Time::now();
  pmc.header.frame_id = "body";
  pmc.states.resize(2);
  pmc.states[0].motor_speed = gnc_.act_.act_impeller_speed_cmd[0];
  pmc.states[1].motor_speed = gnc_.act_.act_impeller_speed_cmd[1];
  std::copy(gnc_.act_.act_servo_pwm_cmd, gnc_.act_.act_servo_pwm_cmd + 6,
      pmc.states[0].nozzle_positions.c_array());
  std::copy(gnc_.act_.act_servo_pwm_cmd + 6, gnc_.act_.act_servo_pwm_cmd + 12,
      pmc.states[1].nozzle_positions.c_array());

  pmc_pub_.publish<ff_hw_msgs::PmcCommand>(pmc);
}

void Fam::ReadParams(config_reader::ConfigReader* config) {
  gnc_.ReadParams(config);
}

}  // end namespace gnc_ros_wrapper
