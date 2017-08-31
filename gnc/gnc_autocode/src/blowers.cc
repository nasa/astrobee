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

#include <gnc_autocode/blowers.h>

#include <assert.h>

namespace gnc_autocode {

GncBlowersAutocode::GncBlowersAutocode(void) {
  // allocate blower 1 and check for memory allocation error
  blower1_ = bpm_blower_1_propulsion_module(&states_[0].battery_voltage,
    states_[0].omega_B_ECI_B, &states_[0].impeller_cmd, states_[0].servo_cmd, states_[0].center_of_mass,
    &states_[0].impeller_current, states_[0].servo_current, states_[0].torque_B,
    states_[0].force_B, &states_[0].motor_speed, states_[0].nozzle_theta, &states_[0].meas_motor_speed);
  assert(blower1_);
  assert(rtmGetErrorStatus(blower1_) == NULL);
  // allocate blower 2 and check for memory allocation error
  blower2_ = bpm_blower_2_propulsion_module(&states_[1].battery_voltage,
    states_[1].omega_B_ECI_B, &states_[1].impeller_cmd, states_[1].servo_cmd, states_[1].center_of_mass,
    &states_[1].impeller_current, states_[1].servo_current, states_[1].torque_B,
    states_[1].force_B, &states_[1].motor_speed, states_[1].nozzle_theta, &states_[1].meas_motor_speed);
  assert(blower2_);
  assert(rtmGetErrorStatus(blower2_) == NULL);
  // Initialize the blowers
  Initialize();
}

void GncBlowersAutocode::Initialize(void) {
  bpm_blower_1_propulsion_module_initialize(blower1_, &states_[0].battery_voltage,
    states_[0].omega_B_ECI_B, &states_[0].impeller_cmd, states_[0].servo_cmd, states_[0].center_of_mass,
    &states_[0].impeller_current, states_[0].servo_current, states_[0].torque_B,
    states_[0].force_B, &states_[0].motor_speed, states_[0].nozzle_theta, &states_[0].meas_motor_speed);
  bpm_blower_2_propulsion_module_initialize(blower2_, &states_[1].battery_voltage,
    states_[1].omega_B_ECI_B, &states_[1].impeller_cmd, states_[1].servo_cmd, states_[1].center_of_mass,
    &states_[1].impeller_current, states_[1].servo_current, states_[1].torque_B,
    states_[1].force_B, &states_[1].motor_speed, states_[1].nozzle_theta, &states_[1].meas_motor_speed);
}

GncBlowersAutocode::~GncBlowersAutocode() {
  bpm_blower_1_propulsion_module_terminate(blower1_);
  bpm_blower_2_propulsion_module_terminate(blower2_);
}

void GncBlowersAutocode::Step(void) {
  bpm_blower_1_propulsion_module_step(blower1_, states_[0].battery_voltage,
    states_[0].omega_B_ECI_B, states_[0].impeller_cmd, states_[0].servo_cmd, states_[0].center_of_mass,
    &states_[0].impeller_current, states_[0].servo_current, states_[0].torque_B,
    states_[0].force_B, &states_[0].motor_speed, states_[0].nozzle_theta, &states_[0].meas_motor_speed);
  bpm_blower_2_propulsion_module_step(blower2_, states_[1].battery_voltage,
    states_[1].omega_B_ECI_B, states_[1].impeller_cmd, states_[1].servo_cmd, states_[1].center_of_mass,
    &states_[1].impeller_current, states_[1].servo_current, states_[1].torque_B,
    states_[1].force_B, &states_[1].motor_speed, states_[1].nozzle_theta, &states_[1].meas_motor_speed);
}

void GncBlowersAutocode::SetAngularVelocity(float x, float y, float z) {
  states_[0].omega_B_ECI_B[0] = x;
  states_[0].omega_B_ECI_B[1] = y;
  states_[0].omega_B_ECI_B[2] = z;
  states_[1].omega_B_ECI_B[0] = x;
  states_[1].omega_B_ECI_B[1] = y;
  states_[1].omega_B_ECI_B[2] = z;
}

void GncBlowersAutocode::SetBatteryVoltage(float voltage) {
  states_[0].battery_voltage = voltage;
  states_[1].battery_voltage = voltage;
}

}  // namespace gnc_autocode

