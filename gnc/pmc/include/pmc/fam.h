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

#ifndef PMC_FAM_H_
#define PMC_FAM_H_

#include <Eigen/Dense>

namespace pmc {

struct FamInput {
  Eigen::Vector3f body_force_cmd;
  Eigen::Vector3f body_torque_cmd;
  Eigen::Vector3f center_of_mass;
  uint8_t speed_gain_cmd;
};

class Fam {
 public:
  void Step(const FamInput& in, uint8_t* speed_cmd, Eigen::Matrix<float, 12, 1> & servo_pwm_cmd);
  void UpdateCOM(const Eigen::Vector3f & com);
 protected:
  void CalcThrustMatrices(const Eigen::Vector3f & force, const Eigen::Vector3f & torque,
    Eigen::Matrix<float, 6, 1>& pmc1_nozzle_thrusts, Eigen::Matrix<float, 6, 1>& pmc2_nozzle_thrusts);
  float ComputePlenumDeltaPressure(float impeller_speed, Eigen::Matrix<float, 6, 1> & discharge_coeff,
                                  const Eigen::Matrix<float, 6, 1> & nozzle_thrusts);
  void CalcPMServoCmd(bool is_pmc1, uint8_t speed_gain_cmd, const Eigen::Matrix<float, 6, 1> & nozzle_thrusts,
    uint8_t & impeller_speed_cmd, Eigen::Matrix<float, 6, 1> & servo_pwm_cmd,
    Eigen::Matrix<float, 6, 1> & nozzle_theta_cmd,
    Eigen::Matrix<float, 6, 1> & normalized_pressure_density, Eigen::Matrix<float, 6, 1> & command_area_per_nozzle);

  Eigen::Matrix<float, 3, 12> thrust2force_, thrust2torque_;
  Eigen::Matrix<float, 12, 6> forcetorque2thrust_;

  static const float IMPELLER_SPEEDS[];
  // this lookup table was computed in fam_force_allocation_module_prep.m. We do not expect the values to
  // change so we have just copied them. it is a function of air density and the propeller properties.
  static constexpr int THRUST_LOOKUP_SIZE = 316;
  static const float THRUST_LOOKUP_BREAKPOINTS[];
  static const float THRUST_LOOKUP_CDP[];
};

}  // end namespace pmc

#endif  // PMC_FAM_H_
