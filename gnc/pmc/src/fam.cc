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

#include <ros/console.h>
#include "pmc/fam.h"
#include "pmc/shared.h"

namespace pmc {

const float Fam::IMPELLER_SPEEDS[3] = {2000 * 2 * M_PI / 60, 2500 * 2 * M_PI / 60, 2800 * 2 * M_PI / 60};
const float Fam::THRUST_LOOKUP_CDP[316] =
    {0.0825, 0.08262563, 0.08274699, 0.08286414, 0.08297713, 0.08308605, 0.08319096, 0.08329192, 0.083389, 0.08348226,
    0.08357175, 0.08365756, 0.08373973, 0.08381831, 0.08389337, 0.08396497, 0.08403317, 0.084098, 0.08415954,
    0.08421783, 0.08427292, 0.08432487, 0.08437373, 0.08441953, 0.08446234, 0.08450221, 0.08453917, 0.08457327,
    0.08460455, 0.08463307, 0.08465886, 0.08468197, 0.08470242, 0.08472028, 0.08473557, 0.08474834, 0.08475861,
    0.08476643, 0.08477184, 0.08477487, 0.08477554, 0.08477391, 0.08477, 0.08476384, 0.08475546, 0.08474489,
    0.08473217, 0.08471732, 0.08470037, 0.08468134, 0.08466027, 0.08463717, 0.08461209, 0.08458503, 0.08455602,
    0.08452509, 0.08449226, 0.08445755, 0.08442097, 0.08438256, 0.08434232, 0.08430029, 0.08425647, 0.0842109,
    0.08416356, 0.0841145, 0.08406372, 0.08401124, 0.08395708, 0.08390123, 0.08384374, 0.08378459, 0.08372381,
    0.0836614, 0.08359738, 0.08353176, 0.08346455, 0.08339575, 0.08332538, 0.08325344, 0.08317994, 0.08310489,
    0.0830283, 0.08295017, 0.0828705, 0.0827893, 0.08270658, 0.08262234, 0.08253659, 0.08244931, 0.08236053,
    0.08227024, 0.08217844, 0.08208514, 0.08199033, 0.08189403, 0.08179621, 0.08169689, 0.08159607, 0.08149374,
    0.0813899, 0.08128455, 0.0811777, 0.08106932, 0.08095943, 0.08084802, 0.08073508, 0.08062062, 0.08050462,
    0.08038708, 0.080268, 0.08014737, 0.08002519, 0.07990145, 0.07977614, 0.07964926, 0.0795208, 0.07939076,
    0.07925911, 0.07912587, 0.07899102, 0.07885455, 0.07871646, 0.07857673, 0.07843536, 0.07829233, 0.07814765,
    0.07800129, 0.07785325, 0.07770353, 0.0775521, 0.07739896, 0.07724411, 0.07708752, 0.07692919, 0.07676911,
    0.07660726, 0.07644365, 0.07627825, 0.07611104, 0.07594204, 0.07577121, 0.07559855, 0.07542405, 0.0752477,
    0.07506947, 0.07488938, 0.07470739, 0.0745235, 0.0743377, 0.07414997, 0.0739603, 0.07376868, 0.0735751,
    0.07337955, 0.07318202, 0.07298248, 0.07278094, 0.07257736, 0.07237177, 0.07216411, 0.0719544, 0.07174262,
    0.07152876, 0.07131281, 0.07109474, 0.07087456, 0.07065225, 0.0704278, 0.0702012, 0.06997244, 0.0697415,
    0.06950837, 0.06927305, 0.06903552, 0.06879577, 0.06855379, 0.06830958, 0.06806312, 0.06781439, 0.0675634,
    0.06731013, 0.06705457, 0.06679671, 0.06653654, 0.06627406, 0.06600925, 0.06574212, 0.06547263, 0.06520081,
    0.06492662, 0.06465007, 0.06437115, 0.06408985, 0.06380615, 0.06352008, 0.0632316, 0.06294072, 0.06264743,
    0.06235172, 0.0620536, 0.06175305, 0.06145008, 0.06114467, 0.06083682, 0.06052653, 0.0602138, 0.05989863,
    0.05958102, 0.05926095, 0.05893843, 0.05861346, 0.05828604, 0.05795617, 0.05762384, 0.05728908, 0.05695185,
    0.05661217, 0.05627006, 0.05592549, 0.05557849, 0.05522906, 0.05487716, 0.05452287, 0.05416614, 0.05380698,
    0.05344541, 0.05308145, 0.05271507, 0.05234631, 0.05197515, 0.05160162, 0.05122572, 0.05084746, 0.05046686,
    0.0500839, 0.04969862, 0.04931103, 0.04892113, 0.04852895, 0.04813449, 0.04773775, 0.04733878, 0.04693756,
    0.04653414, 0.04612852, 0.0457207, 0.04531071, 0.04489859, 0.04448432, 0.04406797, 0.04364951, 0.043229,
    0.04280642, 0.04238184, 0.04195525, 0.0415267, 0.04109619, 0.04066373, 0.04022939, 0.03979318, 0.03935514,
    0.03891524, 0.03847359, 0.03803019, 0.03758504, 0.03713819, 0.03668968, 0.03623956, 0.03578782, 0.03533453,
    0.0348797, 0.03442341, 0.03396563, 0.03350645, 0.03304591, 0.03258402, 0.03212083, 0.03165638, 0.03119075,
    0.03072393, 0.03025598, 0.02978694, 0.02931687, 0.02884581, 0.02837384, 0.02790096, 0.02742722, 0.02695271,
    0.02647745, 0.02600148, 0.0255249, 0.02504772, 0.02457004, 0.02409187, 0.0236133, 0.02313438, 0.02265514,
    0.02217567, 0.02169603, 0.0212163, 0.02073649, 0.02025672, 0.01977702, 0.01929744, 0.01881814, 0.01833903,
    0.01786034, 0.01738204, 0.01690428, 0.01642704, 0.01595047, 0.0154746, 0.01499952, 0.01452534, 0.01405202};
const float Fam::THRUST_LOOKUP_BREAKPOINTS[316] =
    {-1.726016e-05, -1.722989e-05, -1.719863e-05, -1.716642e-05, -1.713327e-05, -1.70992e-05,
    -1.706422e-05, -1.702835e-05, -1.699161e-05, -1.695402e-05, -1.691559e-05, -1.687634e-05, -1.683629e-05,
    -1.679544e-05, -1.675383e-05, -1.671145e-05, -1.666833e-05, -1.662449e-05, -1.657993e-05, -1.653467e-05,
    -1.648872e-05, -1.64421e-05, -1.639483e-05, -1.63469e-05, -1.629834e-05, -1.624916e-05, -1.619938e-05,
    -1.614899e-05, -1.609802e-05, -1.604648e-05, -1.599437e-05, -1.594171e-05, -1.588852e-05, -1.583479e-05,
    -1.578054e-05, -1.572579e-05, -1.567054e-05, -1.561479e-05, -1.555857e-05, -1.550188e-05, -1.544472e-05,
    -1.538712e-05, -1.532907e-05, -1.527058e-05, -1.521167e-05, -1.515234e-05, -1.50926e-05, -1.503245e-05,
    -1.497191e-05, -1.491098e-05, -1.484967e-05, -1.478799e-05, -1.472595e-05, -1.466354e-05, -1.460077e-05,
    -1.453767e-05, -1.447422e-05, -1.441043e-05, -1.434632e-05, -1.428188e-05, -1.421712e-05, -1.415206e-05,
    -1.408669e-05, -1.402101e-05, -1.395504e-05, -1.388878e-05, -1.382222e-05, -1.375539e-05, -1.368828e-05,
    -1.36209e-05, -1.355324e-05, -1.348532e-05, -1.341714e-05, -1.33487e-05, -1.328001e-05, -1.321107e-05,
    -1.314188e-05, -1.307245e-05, -1.300278e-05, -1.293287e-05, -1.286272e-05, -1.279235e-05, -1.272175e-05,
    -1.265092e-05, -1.257987e-05, -1.250861e-05, -1.243712e-05, -1.236542e-05, -1.22935e-05, -1.222138e-05,
    -1.214905e-05, -1.207651e-05, -1.200377e-05, -1.193082e-05, -1.185768e-05, -1.178433e-05, -1.171079e-05,
    -1.163706e-05, -1.156313e-05, -1.1489e-05, -1.141469e-05, -1.134019e-05, -1.12655e-05, -1.119062e-05,
    -1.111556e-05, -1.104032e-05, -1.096488e-05, -1.088927e-05, -1.081348e-05, -1.073751e-05, -1.066136e-05,
    -1.058503e-05, -1.050852e-05, -1.043184e-05, -1.035498e-05, -1.027794e-05, -1.020074e-05, -1.012335e-05,
    -1.00458e-05, -9.968076e-06, -9.890177e-06, -9.812112e-06, -9.733873e-06, -9.655467e-06, -9.576891e-06,
    -9.498148e-06, -9.419237e-06, -9.340158e-06, -9.260914e-06, -9.181501e-06, -9.101926e-06, -9.022184e-06,
    -8.942282e-06, -8.862211e-06, -8.781981e-06, -8.701585e-06, -8.621029e-06, -8.540313e-06, -8.459433e-06,
    -8.378394e-06, -8.297197e-06, -8.21584e-06, -8.134323e-06, -8.052652e-06, -7.970822e-06, -7.888837e-06,
    -7.806695e-06, -7.724401e-06, -7.641954e-06, -7.559351e-06, -7.476597e-06, -7.393692e-06, -7.310637e-06,
    -7.227431e-06, -7.144081e-06, -7.060581e-06, -6.976937e-06, -6.893148e-06, -6.809215e-06, -6.72514e-06,
    -6.640923e-06, -6.556566e-06, -6.472074e-06, -6.387444e-06, -6.302679e-06, -6.217779e-06, -6.132747e-06,
    -6.047586e-06, -5.962296e-06, -5.876881e-06, -5.791341e-06, -5.705679e-06, -5.619894e-06, -5.533992e-06,
    -5.447974e-06, -5.361841e-06, -5.275595e-06, -5.189244e-06, -5.102784e-06, -5.01622e-06, -4.929554e-06,
    -4.84279e-06, -4.75593e-06, -4.668978e-06, -4.581933e-06, -4.494807e-06, -4.407594e-06, -4.3203e-06,
    -4.23293e-06, -4.145489e-06, -4.057978e-06, -3.970398e-06, -3.882762e-06, -3.795063e-06, -3.707311e-06,
    -3.619512e-06, -3.531663e-06, -3.443777e-06, -3.355851e-06, -3.267894e-06, -3.179912e-06, -3.091906e-06,
    -3.003885e-06, -2.91585e-06, -2.827807e-06, -2.739765e-06, -2.651725e-06, -2.563695e-06, -2.475683e-06,
    -2.387691e-06, -2.299727e-06, -2.211795e-06, -2.123905e-06, -2.036061e-06, -1.94827e-06, -1.860541e-06,
    -1.772879e-06, -1.68529e-06, -1.597783e-06, -1.510364e-06, -1.423045e-06, -1.335829e-06, -1.248723e-06,
    -1.161742e-06, -1.074888e-06, -9.881705e-07, -9.015994e-07, -8.151846e-07, -7.289327e-07, -6.428545e-07,
    -5.569591e-07, -4.712565e-07, -3.85754e-07, -3.004643e-07, -2.153974e-07, -1.305634e-07, -4.597132e-08,
    3.836612e-08, 1.224362e-07, 2.062316e-07, 2.897377e-07, 3.729447e-07, 4.558406e-07, 5.384145e-07,
    6.206483e-07, 7.025319e-07, 7.840572e-07, 8.652069e-07, 9.459663e-07, 1.026325e-06, 1.106265e-06,
    1.185778e-06, 1.264842e-06, 1.343449e-06, 1.421578e-06, 1.49922e-06, 1.576352e-06, 1.652964e-06,
    1.729039e-06, 1.804556e-06, 1.879501e-06, 1.953857e-06, 2.027606e-06, 2.10073e-06, 2.173206e-06,
    2.245024e-06, 2.316159e-06, 2.386592e-06, 2.456303e-06, 2.525276e-06, 2.593483e-06, 2.660908e-06,
    2.727526e-06, 2.793316e-06, 2.858257e-06, 2.922321e-06, 2.985491e-06, 3.047737e-06, 3.109039e-06,
    3.169364e-06, 3.228694e-06, 3.286999e-06, 3.344252e-06, 3.400427e-06, 3.455491e-06, 3.509414e-06,
    3.562171e-06, 3.613729e-06, 3.664055e-06, 3.713118e-06, 3.760882e-06, 3.807314e-06, 3.852379e-06,
    3.896036e-06, 3.938252e-06, 3.978987e-06, 4.018196e-06, 4.055845e-06, 4.091884e-06, 4.126272e-06,
    4.158959e-06, 4.1899e-06, 4.219043e-06, 4.246336e-06, 4.271728e-06, 4.295159e-06, 4.31657e-06, 4.335897e-06,
    4.353081e-06, 4.36805e-06, 4.380732e-06, 4.391055e-06, 4.398935e-06, 4.404289e-06, 4.407031e-06, 4.407059e-06};

void Fam::UpdateCOM(const Eigen::Vector3f & com) {
  // latch nozzle thrust matrices
  Eigen::Matrix<float, 12, 3> fam_P_nozzle_B_B, fam_nozzle_orientations;
  fam_P_nozzle_B_B << PMCConstants::nozzle_offsets1, PMCConstants::nozzle_offsets2;
  fam_nozzle_orientations << PMCConstants::nozzle_orientations1, PMCConstants::nozzle_orientations2;

  Eigen::Matrix<float, 12, 3> nozzle_moment_arm = fam_P_nozzle_B_B - com.transpose().replicate(12, 1);
  for (int i = 0; i < 12; i++)
    thrust2torque_.col(i) = -nozzle_moment_arm.row(i).cross(fam_nozzle_orientations.row(i)).transpose();

  thrust2force_ = -fam_nozzle_orientations.transpose();
  Eigen::Matrix<float, 6, 12> thrust2forcetorque;
  thrust2forcetorque << thrust2force_, thrust2torque_;
  forcetorque2thrust_ = Eigen::MatrixXf::Identity(12, 6);
  Eigen::JacobiSVD<Eigen::MatrixXf> svd(thrust2forcetorque, Eigen::ComputeThinU | Eigen::ComputeThinV);
  forcetorque2thrust_ = svd.solve(forcetorque2thrust_);
}

void Fam::CalcThrustMatrices(const Eigen::Vector3f & force, const Eigen::Vector3f & torque,
    Eigen::Matrix<float, 6, 1>& pmc1_nozzle_thrusts, Eigen::Matrix<float, 6, 1>& pmc2_nozzle_thrusts) {
  Eigen::Matrix<float, 6, 1> forcetorque;
  forcetorque << force, torque;
  Eigen::Matrix<float, 12, 1> thrust_per_nozzle = forcetorque2thrust_ * forcetorque;
  const int fam_nozzle_dirs[][4] = {{1, 2,  7,  8}, {3, 4,  9, 10}, {5, 6, 11, 12}};
  // prevents negative thrust from being commanded by adding the min value below zero to the null vectors
  // negative thrust occurs because the inverse matrix distributes thrust accross the
  // nozzles aligned in an axis equally, even if they are pointed in opposite directions
  for (int i = 0; i < 3; i++) {
    float min_thrust = 0.0;
    for (int j = 0; j < 4; j++) {
      float t = thrust_per_nozzle(fam_nozzle_dirs[i][j] - 1);
      if (t < min_thrust)
        min_thrust = t;
    }
    if (min_thrust < 0) {
      for (int j = 0; j < 4; j++)
        thrust_per_nozzle(fam_nozzle_dirs[i][j] - 1) -= min_thrust;
    }
  }
  pmc1_nozzle_thrusts = thrust_per_nozzle.head<6>();
  pmc2_nozzle_thrusts = thrust_per_nozzle.tail<6>();
}

float Fam::ComputePlenumDeltaPressure(float impeller_speed, Eigen::Matrix<float, 6, 1> & discharge_coeff,
                                     const Eigen::Matrix<float, 6, 1> & nozzle_thrusts) {
  // lookup_data_tables
  float total_thrust = (nozzle_thrusts.array() / discharge_coeff.array()).sum();
  float cdp = Lookup(Fam::THRUST_LOOKUP_SIZE, Fam::THRUST_LOOKUP_CDP, Fam::THRUST_LOOKUP_BREAKPOINTS,
                     total_thrust / (impeller_speed * impeller_speed));
  return impeller_speed * impeller_speed * PMCConstants::impeller_diameter * PMCConstants::impeller_diameter *
         PMCConstants::air_density * cdp;
}

void Fam::CalcPMServoCmd(bool is_pmc1, uint8_t speed_gain_cmd, const Eigen::Matrix<float, 6, 1> & nozzle_thrusts,
    uint8_t & impeller_speed_cmd, Eigen::Matrix<float, 6, 1> & servo_pwm_cmd,
    Eigen::Matrix<float, 6, 1> & nozzle_theta_cmd,
    Eigen::Matrix<float, 6, 1> & normalized_pressure_density, Eigen::Matrix<float, 6, 1> & command_area_per_nozzle) {
  if (speed_gain_cmd > 3)
    ROS_FATAL("Invalid speed command %d.", speed_gain_cmd);

  Eigen::Matrix<float, 6, 1> discharge_coeff =
    is_pmc1 ? PMCConstants::discharge_coeff1 : PMCConstants::discharge_coeff2;

  float impeller_speed = 0.0f, plenum_delta_pressure = 0.0f;
  impeller_speed_cmd = 0;
  if (speed_gain_cmd != 0) {
    impeller_speed = Fam::IMPELLER_SPEEDS[speed_gain_cmd - 1];
    impeller_speed_cmd = static_cast<uint8_t>(PMCConstants::impeller_speed2pwm * impeller_speed + 0.5);
    plenum_delta_pressure = ComputePlenumDeltaPressure(impeller_speed, discharge_coeff, nozzle_thrusts);
  }
  normalized_pressure_density = 2 * (discharge_coeff.array() * discharge_coeff.array()).matrix() *
                                      plenum_delta_pressure;
  command_area_per_nozzle = (nozzle_thrusts.array() / normalized_pressure_density.array()).matrix();
  const int nozzle_flap_count = 2;
  Eigen::Matrix<float, 6, 1> openings = (PMCConstants::abp_nozzle_intake_height - (1.0 / nozzle_flap_count) *
      command_area_per_nozzle.array() / PMCConstants::nozzle_widths.array()).matrix() /
      PMCConstants::abp_nozzle_flap_length;
  const float min_open_cos = cos(PMCConstants::abp_nozzle_max_open_angle);
  const float max_open_cos = cos(PMCConstants::abp_nozzle_min_open_angle);
  for (int i = 0; i < 6; i++) {
    if (openings(i) < min_open_cos || std::isnan(openings[i]))
      openings[i] = min_open_cos;
    if (openings(i) > max_open_cos)
      openings[i] = max_open_cos;
    nozzle_theta_cmd[i] = acos(openings[i]) - PMCConstants::abp_nozzle_min_open_angle;
  }
  const int servo_max_pwm = 255;
  const int servo_min_pwm = 0;
  //    %[-] Conversion between servo angle (not nozzle angle) command and resulting servo PWM
  servo_pwm_cmd = static_cast<float>(servo_max_pwm - servo_min_pwm) /
                  (PMCConstants::abp_nozzle_max_open_angle - PMCConstants::abp_nozzle_min_open_angle) *
                  nozzle_theta_cmd;
}

void Fam::Step(const FamInput& in, uint8_t* speed_cmd, Eigen::Matrix<float, 12, 1> & servo_pwm_cmd) {
  Eigen::Matrix<float, 3, 12> thrust2force_B, thrust2torque_B;
  Eigen::Matrix<float, 6, 1> pmc1_nozzle_thrusts, pmc2_nozzle_thrusts;
  CalcThrustMatrices(
                     in.body_force_cmd, in.body_torque_cmd,
                     pmc1_nozzle_thrusts, pmc2_nozzle_thrusts);
  Eigen::Matrix<float, 6, 1> nozzle_theta, normalized_pressure_density, command_area_per_nozzle;
  Eigen::Matrix<float, 6, 1> servo_pwm;
  CalcPMServoCmd(true, in.speed_gain_cmd, pmc1_nozzle_thrusts, speed_cmd[0], servo_pwm, nozzle_theta,
      normalized_pressure_density, command_area_per_nozzle);
  servo_pwm_cmd.head<6>() = servo_pwm;
  CalcPMServoCmd(false, in.speed_gain_cmd, pmc2_nozzle_thrusts, speed_cmd[1], servo_pwm, nozzle_theta,
      normalized_pressure_density, command_area_per_nozzle);
  servo_pwm_cmd.tail<6>() = servo_pwm;
  // computes actual output force and torque, but currently not published
  // Eigen::Matrix<float, 12, 1> temp = (command_area_per_nozzle.array() * normalized_pressure_density.array());
  // Eigen::Vector3f predicted_force = thrust2force_B * temp;
  // Eigen::Vector3f predicted_torque = thrust2torque_B * temp;
}
}  // namespace pmc

