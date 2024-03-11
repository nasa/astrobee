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

#ifndef CTL_CTL_H_
#define CTL_CTL_H_

#include <Eigen/Dense>

namespace config_reader {
  class ConfigReader;
}

namespace constants {
const unsigned int ase_status_converged = 0U;
const unsigned int ctl_idle_mode = 0U;
const unsigned int ctl_stopping_mode = 1U;
const unsigned int ctl_stopped_mode = 3U;
}  // namespace constants

namespace ctl {

struct ControlState {
  Eigen::Vector3f est_P_B_ISS_ISS;
  Eigen::Quaternionf est_quat_ISS2B;
  Eigen::Vector3f est_V_B_ISS_ISS;
  Eigen::Vector3f est_omega_B_ISS_B;

  Eigen::Matrix<float, 3, 3> inertia;

  // configuration
  Eigen::Vector3f att_kp;
  Eigen::Vector3f att_ki;
  Eigen::Vector3f omega_kd;
  Eigen::Vector3f pos_kp;
  Eigen::Vector3f pos_ki;
  Eigen::Vector3f vel_kd;

  float mass;
  uint8_t est_confidence;
};

struct ControlCommand {
  Eigen::Vector3f P_B_ISS_ISS;
  Eigen::Quaternionf quat_ISS2B;
  Eigen::Vector3f V_B_ISS_ISS;
  Eigen::Vector3f A_B_ISS_ISS;
  Eigen::Vector3f omega_B_ISS_ISS;
  Eigen::Vector3f alpha_B_ISS_ISS;

  uint8_t mode;
};

struct ControlOutput {
  Eigen::Vector3f body_force_cmd;
  Eigen::Vector3f body_torque_cmd;
  Eigen::Vector3f body_accel_cmd;
  Eigen::Vector3f body_alpha_cmd;
  Eigen::Vector3f pos_err;
  Eigen::Vector3f pos_err_int;
  Eigen::Vector3f att_err;
  Eigen::Vector3f att_err_int;

  Eigen::Vector3f traj_pos;
  Eigen::Quaternionf traj_quat;
  Eigen::Vector3f traj_vel;
  Eigen::Vector3f traj_accel;
  Eigen::Vector3f traj_omega;
  Eigen::Vector3f traj_alpha;

  float att_err_mag;

  float traj_error_pos;
  float traj_error_att;
  float traj_error_vel;
  float traj_error_omega;

  uint8_t ctl_status;
};

class Control {
 public:
  Control(void);

  virtual void Initialize(void);
  virtual void Step(float time_delta, ControlState & state, ControlCommand & cmd, ControlOutput* out);
  virtual void ReadParams(config_reader::ConfigReader* config);

 private:
  int mode_cmd_;
  bool stopped_mode_;
  Eigen::Vector3f prev_filter_vel_;
  Eigen::Vector3f prev_filter_omega_;
  int prev_mode_cmd_[5];  // for the 4 ticks required  to switch to stopped; newest val at index 0
  Eigen::Vector3f prev_position_;
  Eigen::Quaternionf prev_att_;
  Eigen::Vector3f linear_integrator_;
  Eigen::Vector3f rotational_integrator_;

  bool FilterThreshold(Eigen::Vector3f vec, float threshhold, Eigen::Vector3f & previous);
  float ButterWorthFilter(float input, float delay_val, float* sum_out);
  float QuatError(Eigen::Quaternionf cmd, Eigen::Quaternionf actual);
  void UpdateCtlStatus(const ControlState & state, ControlOutput* out);
  Eigen::Vector3f SafeDivide(const Eigen::Vector3f & num, const Eigen::Vector3f & denom);
  void FindPosErr(const ControlState & state, const ControlCommand & cmd, ControlOutput* out);
  Eigen::Vector3f DiscreteTimeIntegrator(const Eigen::Vector3f input, Eigen::Vector3f & accumulator,
                                         uint8_t ctl_status, float upper_limit, float lower_limit);
  void FindBodyForceCmd(const ControlState & state, ControlOutput* out);
  Eigen::Vector3f RotateVectorAtoB(Eigen::Vector3f, Eigen::Quaternionf);
  Eigen::Vector3f SaturateVector(Eigen::Vector3f, float limit);
  void FindBodyAlphaTorqueCmd(const ControlState & state, ControlOutput* out);
  void FindAttErr(const ControlState & state, ControlOutput* out);

  void ForwardTrajectory(float time_delta, const ControlState & state, const ControlCommand & cmd,
                         ControlOutput* out);
  void UpdateMode(const ControlState & state, const ControlCommand & cmd);
  void UpdatePrevious(const ControlState & state);

  Eigen::Matrix<float, 4, 4> OmegaMatrix(Eigen::Vector3f input);

  Eigen::Vector3f tun_accel_gain;
  Eigen::Vector3f tun_alpha_gain;
  float tun_ctl_stopping_omega_thresh;
  float tun_ctl_stopping_vel_thresh;
  float tun_ctl_stopped_pos_thresh;
  float tun_ctl_stopped_quat_thresh;
  float tun_ctl_pos_sat_upper;
  float tun_ctl_pos_sat_lower;
  float tun_ctl_linear_force_limit;
  float tun_ctl_att_sat_upper;
  float tun_ctl_att_sat_lower;
};
}  // end namespace ctl

#endif  // CTL_CTL_H_

