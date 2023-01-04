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

#ifndef GNC_AUTOCODE_BLOWERS_H_
#define GNC_AUTOCODE_BLOWERS_H_

extern "C" {
#include <bpm_blower_1_propulsion_module.h>
#include <bpm_blower_2_propulsion_module.h>
}

#include <Eigen/Dense>

namespace gnc_autocode {

struct  GncBlowerState{
  float battery_voltage;
  float omega_B_ECI_B[3];
  unsigned char impeller_cmd;
  float servo_cmd[6];
  float center_of_mass[3];
  float impeller_current;
  float servo_current[6];
  float torque_B[3];
  float force_B[3];
  float motor_speed;
  float nozzle_theta[6];
  float meas_motor_speed;
};

class GncBlowersAutocode {
 public:
  GncBlowersAutocode();
  ~GncBlowersAutocode();
  virtual void Initialize();
  virtual void Step();
  virtual void SetAngularVelocity(float x, float y, float z);
  virtual void SetBatteryVoltage(float voltage);

  // This is just a thin wrapper with a step function
  RT_MODEL_bpm_blower_1_propuls_T *blower1_;
  RT_MODEL_bpm_blower_2_propuls_T *blower2_;

  // States of our two blowers
  GncBlowerState states_[2];
};

}  // end namespace gnc_autocode

#endif  // GNC_AUTOCODE_BLOWERS_H_
