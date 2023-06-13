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

#ifndef PMC_SHARED_H_
#define PMC_SHARED_H_

#include <Eigen/Dense>
#include <limits>

namespace pmc {

// constants
class PMCConstants {
 public:
  static constexpr float DT = 1.0 / 62.5;
  static constexpr float units_inches_to_meters = 0.0254;
  static const Eigen::Matrix<float, 6, 1> discharge_coeff1, discharge_coeff2;
  // [m] Position vector of the nozzle locations in the body frame
  static const Eigen::Matrix<float, 6, 3> nozzle_offsets1, nozzle_offsets2;
  //  [unit vec] PM1: Pointing Direction of each nozzle
  static const Eigen::Matrix<float, 6, 3> nozzle_orientations1, nozzle_orientations2;
  //  [m^2] If air is leaking from the plenum, the
  // '0 thrust' position (all nozzles closed) will have an effective open area
  static const Eigen::Vector3f impeller_orientation1, impeller_orientation2;
  static const Eigen::Matrix<float, 6, 1> nozzle_widths;  //  [m]
  static constexpr float zero_thrust_area[2] = {0.0044667, 0.0042273};
  //  [rad]      THEORETICAL max angle that a nozzle can open to
  static constexpr float abp_nozzle_max_open_angle = 79.91 * M_PI / 180.0;
  //  [rad]      Min angle that the nozzle can close to
  static constexpr float abp_nozzle_min_open_angle = 15.68 * M_PI / 180.0;
  static constexpr float abp_nozzle_flap_length = 0.5353 * units_inches_to_meters;  //  [m]
  static constexpr float abp_nozzle_intake_height = 0.5154 * units_inches_to_meters;  //  [m]
  static constexpr float abp_nozzle_gear_ratio = 0.5;
  // bpm_servo_peak_torque*bpm_servo_motor_gear_ratio/(bpm_servo_peak_curr); [Nm/A] Servo motor torque constant
  static constexpr float servo_motor_k = 0.2893 * 0.01 / 1.5;
  // bpm_servo_max_voltage/bpm_servo_peak_curr; %[ohm] Servo internal resistance
  static constexpr float servo_motor_r = 6.0 / 1.5;
  // bpm_imp_noload_curr*bpm_imp_motor_torque_k/bpm_imp_noload_speed ;  %[Nm*s] Viscous Friction
  static constexpr float servo_motor_friction = 3e-7;
  //  %[kg*m^2] (Tuned in analysis_servo_motor_test.m) Servo gearbox inertia.
  static constexpr float servo_motor_gearbox_inertia = .000000025;
  static constexpr float servo_motor_gear_ratio = 1.0 / 100.0;
  // dc_motor_model
  static constexpr float imp_motor_speed_k = 374*2*M_PI/60;  // [(rad/s)/V] Impeller Speed constant
  static constexpr float imp_motor_r = 1.2;  // %[ohm] Impeller Motor internal resistance
  static constexpr float imp_motor_torque_k = 0.0255;  // [Nm/A] Impeller Torque constant
  // noload_curr * torque_k / noload_speed [Nm*s] Viscous Friction
  static constexpr float imp_motor_friction_coeff = 0.144 * imp_motor_torque_k / (4380*2*M_PI/60.0);
  static constexpr float backlash_half_width = 1.0 * M_PI / 180.0 / 2;
  static constexpr float impeller_speed2pwm = 0.792095;  //  [CNT/(rad/sec)] Converts impeller speeds into PWM values
  static constexpr float impeller_diameter = 5.5 * units_inches_to_meters;  // [m]
  static constexpr float air_density =
    1.2;  //  [kg/m^3]   Air density inside of the ISS (some places in code this was 1.225?)
  static constexpr float impeller_inertia = 0.001;
  // generated in bpm_blower_propulsion_module_prep.m
  static const float AREA_LOOKUP_INPUT[];
  static const float CDP_LOOKUP_OUTPUT[];
  static const int AREA_LOOKUP_TABLE_SIZE;
};

float Lookup(int table_size, const float lookup[], const float breakpoints[], float value);

template <class MatT>
Eigen::Matrix<typename MatT::Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> pseudoInverse(
  const MatT& mat, typename MatT::Scalar tolerance = typename MatT::Scalar{std::numeric_limits<double>::epsilon()}) {
  typedef typename MatT::Scalar Scalar;
  auto svd = mat.jacobiSvd(Eigen::ComputeFullU | Eigen::ComputeFullV);
  const auto& singularValues = svd.singularValues();
  Eigen::Matrix<Scalar, MatT::ColsAtCompileTime, MatT::RowsAtCompileTime> singularValuesInv(mat.cols(), mat.rows());
  singularValuesInv.setZero();
  for (unsigned int i = 0; i < singularValues.size(); ++i) {
    if (singularValues(i) > tolerance) {
      singularValuesInv(i, i) = Scalar {1} / singularValues(i);
    } else {
      singularValuesInv(i, i) = Scalar {0};
    }
  }
  return svd.matrixV() * singularValuesInv * svd.matrixU().adjoint();
}

}  // end namespace pmc

#endif  // PMC_SHARED_H_
