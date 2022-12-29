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

class PID {
 public:
  void Initialize(float Kp, float Ki, float Kd, float out_max, float out_min);
  float Run(float x);

 private:
  float kp, ki, kd, omax, omin;
  float last;
  float integral;
};

class Blower {
 public:
  explicit Blower(bool is_pmc1);
  void Step(int impeller_cmd, Eigen::Matrix<float, 6, 1> & servo_cmd, Eigen::Vector3f & omega, float voltage);
  Eigen::Vector3f Force() {return force_B_;}
  Eigen::Vector3f Torque() {return torque_B_;}
  float ImpellerCurrent() {return impeller_current_;}
  Eigen::Matrix<float, 6, 1> NozzlesTheta() {return nozzles_theta_;}
  Eigen::Matrix<float, 6, 1> ServoCurrent() {return servo_current_;}
  float MotorSpeed() {return last_speed_;}

 private:
  void ServoModel(const Eigen::Matrix<float, 6, 1> & servo_cmd, Eigen::Matrix<float, 6, 1> & nozzle_theta,
                  Eigen::Matrix<float, 6, 1> & nozzle_area);
  void ImpellerModel(int speed_cmd, float voltage, float* motor_current, float* motor_torque);
  float Lookup(int table_size, const float lookup[], const float breakpoints[], float value);
  Eigen::Matrix<float, 6, 1> Aerodynamics(const Eigen::Matrix<float, 6, 1> & nozzle_area);
  void BlowerBodyDynamics(const Eigen::Vector3f & omega_body, Eigen::Matrix<float, 6, 1> & nozzle_thrusts,
                          float motor_torque);

  Eigen::Matrix<float, 6, 1> prev_omega_, motor_thetas_, backlash_theta_, discharge_coeff_;
  Eigen::Matrix<float, 6, 3> nozzle_offsets_, nozzle_orientations_;
  Eigen::Vector3f center_of_mass_, impeller_orientation_;
  PID servo_pids_[6], speed_pid_;
  float prev_speed_rate_;
  float zero_thrust_area_;
  // outputs
  Eigen::Vector3f force_B_, torque_B_;
  Eigen::Matrix<float, 6, 1> nozzles_theta_, servo_current_;
  float impeller_speed_, last_speed_;
  float impeller_current_;

  // generated in bpm_blower_propulsion_module_prep.m
  static const float AREA_LOOKUP_INPUT[];
  static const float CDP_LOOKUP_OUTPUT[];
  static const int LOOKUP_TABLE_SIZE;
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
  Blower b1, b2;
};

}  // end namespace gnc_autocode

#endif  // GNC_AUTOCODE_BLOWERS_H_
