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

#include "pmc/pmc_sim.h"
#include "pmc/shared.h"

namespace pmc {

void PID::Initialize(float Kp, float Ki, float Kd, float out_max, float out_min) {
  kp = Kp;
  ki = Ki;
  kd = Kd;
  omax = out_max;
  omin = out_min;
  last = integral = 0.0;
}

float PID::Run(float x) {
  float d = kd * x - last;
  float out = kp * x + ki * integral + d;
  last += d * PMCConstants::DT;  // kind of confused here but matches simmulink
  integral += x * PMCConstants::DT;
  if (out < omin)
    out = omin;
  if (out > omax)
    out = omax;
  return out;
}

Blower::Blower(bool is_pmc1) {
  prev_omega_ << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  backlash_theta_ << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  servo_thetas_ << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  prev_speed_rate_ = impeller_speed_ = 0.0f;
  for (int i = 0; i < 6; i++)
    servo_pids_[i].Initialize(5.0, 1.5, 0.8, 6.0, -6.0);
  speed_pid_.Initialize(0.2, 0.1, 0.05, 16.6, -16.6);
  if (is_pmc1) {
    discharge_coeff_ = PMCConstants::discharge_coeff1;
    zero_thrust_area_ = PMCConstants::zero_thrust_area[0];
    nozzle_offsets_ = PMCConstants::nozzle_offsets1;
    nozzle_orientations_ = PMCConstants::nozzle_orientations1;
    impeller_orientation_ = PMCConstants::impeller_orientation1;
  } else {
    discharge_coeff_ = PMCConstants::discharge_coeff2;
    zero_thrust_area_ = PMCConstants::zero_thrust_area[1];
    nozzle_offsets_ = PMCConstants::nozzle_offsets2;
    nozzle_orientations_ = PMCConstants::nozzle_orientations2;
    impeller_orientation_ = PMCConstants::impeller_orientation2;
  }

  center_of_mass_ << 0.0, 0.0, 0.0;  // hmmm.... this seems wrong but was used originally in the simulink
}

void Blower::ServoModel(const Eigen::Matrix<float, 6, 1> & servo_cmd, Eigen::Matrix<float, 6, 1> & nozzle_theta,
                        Eigen::Matrix<float, 6, 1> & nozzle_area) {
  const float bpm_servo_max_theta = PMCConstants::abp_nozzle_gear_ratio *
                                    (PMCConstants::abp_nozzle_max_open_angle - PMCConstants::abp_nozzle_min_open_angle);
  //  [-] Conversion between servo PWM command (0-100) and resulting angle (0-90)
  const float bpm_servo_pwm2angle = bpm_servo_max_theta / 255.0;
  // backlash box
  for (int i = 0; i < 6; i++) {
    if (servo_thetas_[i] < backlash_theta_[i] - PMCConstants::backlash_half_width)
      backlash_theta_[i] = servo_thetas_[i] + PMCConstants::backlash_half_width;
    else if (servo_thetas_[i] > backlash_theta_[i] + PMCConstants::backlash_half_width)
      backlash_theta_[i] = servo_thetas_[i] - PMCConstants::backlash_half_width;
  }
  Eigen::Matrix<float, 6, 1> voltage,
    err = bpm_servo_pwm2angle * servo_cmd - backlash_theta_ * PMCConstants::servo_motor_gear_ratio;
  for (int i = 0; i < 6; i++)
    voltage[i] = servo_pids_[i].Run(err[i]);
  servo_current_ = (voltage - PMCConstants::servo_motor_k * prev_omega_) * (1.0 / PMCConstants::servo_motor_r);
  servo_thetas_ += 1.0 / 62.5 * prev_omega_;
  servo_thetas_ = servo_thetas_.cwiseMin(bpm_servo_max_theta / PMCConstants::servo_motor_gear_ratio).cwiseMax(0.0);
  // torque - viscous friction
  Eigen::Matrix<float, 6, 1> motor_torque =
    PMCConstants::servo_motor_k * servo_current_ - PMCConstants::servo_motor_friction * prev_omega_;
  prev_omega_ += (1.0 / PMCConstants::servo_motor_gearbox_inertia) / 62.5 * motor_torque;
  nozzle_theta = backlash_theta_ * PMCConstants::servo_motor_gear_ratio * 1.0 / PMCConstants::abp_nozzle_gear_ratio;
  for (int i = 0; i < 6; i++) {
    //  * nozzle_flap_count * nozzle_width
    nozzle_area[i] =
      (PMCConstants::abp_nozzle_intake_height -
       cos(PMCConstants::abp_nozzle_min_open_angle + nozzle_theta[i]) * PMCConstants::abp_nozzle_flap_length) *
      2 * PMCConstants::nozzle_widths[i];
  }
}

void Blower::ImpellerModel(int speed_cmd, float voltage, float* motor_current, float* motor_torque) {
  const float max_change = 200*2*M_PI/60.0 * PMCConstants::DT;
  float speed = speed_cmd / PMCConstants::impeller_speed2pwm;
  if (speed > prev_speed_rate_ + max_change)
    speed = prev_speed_rate_ + max_change;
  else if (speed < prev_speed_rate_ - max_change)
    speed = prev_speed_rate_ - max_change;
  prev_speed_rate_ = speed;

  speed = speed - impeller_speed_;
  float v = speed_pid_.Run(speed);
  v = std::max(-voltage, std::min(voltage, v));

  *motor_current = (v - impeller_speed_ / PMCConstants::imp_motor_speed_k) / PMCConstants::imp_motor_r;
  *motor_torque =
    (*motor_current) * PMCConstants::imp_motor_torque_k - PMCConstants::imp_motor_friction_coeff * impeller_speed_;

  last_speed_ = impeller_speed_;
  // in simulink, drag torque would be subtracted from motor, but set to 0
  // noise is also set to 0
  const float impeller_inertia = 0.001;
  impeller_speed_ += *motor_torque / impeller_inertia / 62.5;
}

Eigen::Matrix<float, 6, 1> Blower::Aerodynamics(const Eigen::Matrix<float, 6, 1> & nozzle_area) {
  float area = zero_thrust_area_ + (discharge_coeff_.array() * nozzle_area.array()).sum();
  float cdp = Lookup(PMCConstants::AREA_LOOKUP_TABLE_SIZE, PMCConstants::CDP_LOOKUP_OUTPUT,
                     PMCConstants::AREA_LOOKUP_INPUT, area);
  float delta_p = impeller_speed_ * impeller_speed_ * cdp * PMCConstants::impeller_diameter *
                  PMCConstants::impeller_diameter * PMCConstants::air_density;
  // 1.25 is thrust_error_scale_factor
  return 2 * delta_p * discharge_coeff_.array() * discharge_coeff_.array() * nozzle_area.array() * 1.25;
  // note: currently in simulink noise is set to zero
}

void Blower::Step(int impeller_cmd, Eigen::Matrix<float, 6, 1> & servo_cmd, Eigen::Vector3f & omega, float voltage) {
  Eigen::Matrix<float, 6, 1> nozzle_area;
  float motor_torque;
  ServoModel(servo_cmd, nozzles_theta_, nozzle_area);
  ImpellerModel(impeller_cmd, voltage, &impeller_current_, &motor_torque);
  Eigen::Matrix<float, 6, 1> nozzle_thrust = Aerodynamics(nozzle_area);
  BlowerBodyDynamics(omega, nozzle_thrust, motor_torque);
}

void Blower::BlowerBodyDynamics(const Eigen::Vector3f & omega_body, Eigen::Matrix<float, 6, 1> & nozzle_thrusts,
    float motor_torque) {
  Eigen::Matrix<float, 3, 6> thrust2force, thrust2torque;
  // latch nozzle thrust matrices
  Eigen::Matrix<float, 6, 3> nozzle_moment_arm = nozzle_offsets_ - center_of_mass_.transpose().replicate(6, 1);
  for (int i = 0; i < 6; i++)
    thrust2torque.col(i) = -nozzle_moment_arm.row(i).cross(nozzle_orientations_.row(i)).transpose();

  thrust2force = -nozzle_orientations_.transpose();

  Eigen::Vector3f momentum_B = impeller_orientation_ * PMCConstants::impeller_inertia * impeller_speed_;

  force_B_ = thrust2force * nozzle_thrusts;
  torque_B_ = impeller_orientation_ * -motor_torque + omega_body.cross(momentum_B) + thrust2torque * nozzle_thrusts;
}

PMCSim::PMCSim(void) : b1(true), b2(false), omega_(0, 0, 0), voltage_(0.0f) {
  for (int i = 0; i < 2; i++) {
    impeller_cmd_[i] = 0;
    servo_cmd_[i] << 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f;
  }
}

void PMCSim::Step(void) {
  b1.Step(impeller_cmd_[0], servo_cmd_[0], omega_, voltage_);
  b2.Step(impeller_cmd_[1], servo_cmd_[1], omega_, voltage_);
}

void PMCSim::SetAngularVelocity(float x, float y, float z) {
  omega_ << x, y, z;
}

void PMCSim::SetBatteryVoltage(float voltage) {
  voltage_ = voltage;
}

}  // namespace pmc
