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

#ifndef PMC_PMC_SIM_H_
#define PMC_PMC_SIM_H_

#include <Eigen/Dense>

namespace pmc {

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
  Eigen::Matrix<float, 6, 1> Aerodynamics(const Eigen::Matrix<float, 6, 1> & nozzle_area);
  void BlowerBodyDynamics(const Eigen::Vector3f & omega_body, Eigen::Matrix<float, 6, 1> & nozzle_thrusts,
                          float motor_torque);

  Eigen::Matrix<float, 6, 1> prev_omega_, servo_thetas_, backlash_theta_, discharge_coeff_;
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
};

class PMCSim {
 public:
  PMCSim();
  virtual void Step();
  virtual void SetAngularVelocity(float x, float y, float z);
  virtual void SetBatteryVoltage(float voltage);

  void SetImpellerCmd(int blower, unsigned char cmd) {impeller_cmd_[blower] = cmd;}
  unsigned char ImpellerCmd(int blower) {return impeller_cmd_[blower];}
  void SetServoCmd(int blower, int index, float value) {servo_cmd_[blower][index] = value;}
  float MotorSpeed(int blower) {return (blower == 0) ? b1.MotorSpeed() : b2.MotorSpeed();}
  Eigen::Vector3f Force() {return b1.Force() + b2.Force();}
  Eigen::Vector3f Torque() {return b1.Torque() + b2.Torque();}

 private:
  Blower b1, b2;
  Eigen::Vector3f omega_;
  float voltage_;
  unsigned char impeller_cmd_[2];
  Eigen::Matrix<float, 6, 1> servo_cmd_[2];
};

}  // end namespace pmc

#endif  // PMC_PMC_SIM_H_
