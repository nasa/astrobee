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
#include <ros/static_assert.h>
#include <ros/platform.h>
#include <stdlib.h>
#include <ros/assert.h>
#include <gnc_autocode/ctl.h>
#include <config_reader/config_reader.h>
#include <msg_conversions/msg_conversions.h>
#include <assert.h>
#include <ctl_tunable_funcs.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <iostream>
#include <sstream>
#include <string>
#include <cstring>
#include <cmath>

namespace gnc_autocode {

void NormalizeQuaternion(Eigen::Quaternionf & out) {
  // enfore positive scalar
  if (out.w() < 0) {
    out.coeffs() = -out.coeffs();
  }
  // out.normalize();

  float mag = sqrt(static_cast<double>(out.x() * out.x() + out.y() * out.y() + out.z() * out.z() + out.w() * out.w()));
  if (mag > 1e-7) {
    out.x() = out.x() / mag; out.y() = out.y() / mag; out.z() = out.z() / mag; out.w() = out.w() / mag;
  }
}


Control::Control(void) {
  Initialize();
}

void Control::Step(float time_delta, ControlState & state, ControlCommand & cmd, ControlOutput* out) {
  //  command shaper
  ForwardTrajectory(time_delta, state, cmd, out);

  //  cex_control_executive
  UpdateMode(state, cmd);
  UpdateCtlStatus(state, out);

  // clc_closed_loop_controller
  // Linear Control
  FindPosErr(state, cmd, out);
  FindBodyForceCmd(state, out);

  // Rotational Control
  FindAttErr(state, out);
  FindBodyAlphaTorqueCmd(state, out);

  UpdatePrevious(state);
}

// Defined in Indirect Kalman Filter for 3d attitude Estimation - Trawn, Roumeliotis eq 63
Eigen::Matrix<float, 4, 4> Control::OmegaMatrix(Eigen::Vector3f input) {
  Eigen::Matrix<float, 4, 4> out;
  out(0, 0) = 0;
  out(1, 0) = -input[2];
  out(2, 0) = input[1];
  out(3, 0) = -input[0];

  out(0, 1) = input[2];
  out(1, 1) = 0;
  out(2, 1) = -input[0];
  out(3, 1) = -input[1];

  out(0, 2) = -input[1];
  out(1, 2) =input[0];
  out(2, 2) = 0;
  out(3, 2) = -input[2];

  out(0, 3) = input[0];
  out(1, 3) = input[1];
  out(2, 3) = input[2];
  out(3, 3) = 0;

  return out;
}

void Control::ForwardTrajectory(float time_delta, const ControlState & state, const ControlCommand & cmd,
                                ControlOutput* out) {
  // forward integrate
  out->traj_pos = cmd.P_B_ISS_ISS + cmd.V_B_ISS_ISS * time_delta +
                  0.5 * cmd.A_B_ISS_ISS * time_delta * time_delta;
  out->traj_vel = cmd.V_B_ISS_ISS + cmd.A_B_ISS_ISS * time_delta;
  out->traj_accel = cmd.A_B_ISS_ISS;
  out->traj_alpha = cmd.alpha_B_ISS_ISS;
  out->traj_omega = cmd.omega_B_ISS_ISS + cmd.alpha_B_ISS_ISS * time_delta;

  Eigen::Matrix<float, 4, 4> omega_omega = OmegaMatrix(cmd.omega_B_ISS_ISS);
  Eigen::Matrix<float, 4, 4> omega_alpha = OmegaMatrix(cmd.alpha_B_ISS_ISS);

  Eigen::Matrix<float, 4, 4> a = 0.5 * time_delta * (0.5 * time_delta * omega_alpha + omega_omega);
  a = a.exp();
  a += (1.0/48) * time_delta * time_delta * time_delta * (omega_alpha * omega_omega - omega_omega * omega_alpha);
  out->traj_quat.coeffs() = a * cmd.quat_ISS2B.coeffs();
  NormalizeQuaternion(out->traj_quat);

  out->traj_error_pos =   (out->traj_pos - state.est_P_B_ISS_ISS).norm();
  out->traj_error_vel =   (out->traj_vel - state.est_V_B_ISS_ISS).norm();
  out->traj_error_omega = (out->traj_omega - state.est_omega_B_ISS_B).norm();

  out->traj_error_att = QuatError(out->traj_quat, state.est_quat_ISS2B);
}

void Control::FindAttErr(const ControlState & state, ControlOutput* out) {
  Eigen::Quaternionf q_cmd = out->traj_quat;
  if (stopped_mode_)
    q_cmd = prev_att_;

  Eigen::Quaternionf q_out = state.est_quat_ISS2B.conjugate() * q_cmd;

  NormalizeQuaternion(q_out);

  out->att_err_mag = 2 * acos(static_cast<double>(q_out.w()));
  out->att_err << q_out.x(), q_out.y(), q_out.z();

  Eigen::Vector3f Ki_rot = SafeDivide(state.att_ki, state.omega_kd);
  Eigen::Vector3f in = (out->att_err.array() * Ki_rot.array()).matrix();
  out->att_err_int = DiscreteTimeIntegrator(in, rotational_integrator_, out->ctl_status, tun_ctl_att_sat_upper,
                                               tun_ctl_att_sat_lower);
}

void Control::FindBodyAlphaTorqueCmd(const ControlState & state, ControlOutput* out) {
  Eigen::Vector3f cmd_omega = out->traj_omega;
  Eigen::Vector3f cmd_alpha = out->traj_alpha;
  if (stopped_mode_) {
    cmd_omega.setZero();
    cmd_alpha.setZero();
  }

  Eigen::Vector3f rate_error = -state.est_omega_B_ISS_B;
  if (out->ctl_status > 1) {
    Eigen::Vector3f Kp_rot = SafeDivide(state.att_kp, state.omega_kd);
    rate_error = cmd_omega + out->att_err_int + (Kp_rot.array() * out->att_err.array()).matrix() -
                 state.est_omega_B_ISS_B;
  }

  auto Kd_rot = state.omega_kd.array() * state.inertia.diagonal().array();
  rate_error = (rate_error.array() * Kd_rot).matrix();

  out->body_alpha_cmd = state.inertia.inverse() * rate_error;

  if (out->ctl_status == 0) {
    out->body_torque_cmd.setZero();
  } else {
    cmd_alpha = (tun_alpha_gain.array() * cmd_alpha.array()).matrix();

    out->body_torque_cmd = state.inertia * cmd_alpha + rate_error -
                           (state.inertia * state.est_omega_B_ISS_B).cross(state.est_omega_B_ISS_B);
  }
}

void Control::FindBodyForceCmd(const ControlState & state, ControlOutput* out) {
  if (out->ctl_status == 0) {
    out->body_force_cmd.setZero();
    out->body_accel_cmd.setZero();
    return;
  }
  Eigen::Vector3f target_vel = out->traj_vel;
  Eigen::Vector3f target_accel = out->traj_accel;
  if (stopped_mode_) {
    target_vel.setZero();
    target_accel.setZero();
  }

  Eigen::Vector3f Kp_lin = SafeDivide(state.pos_kp, state.vel_kd);
  Eigen::Vector3f v = Eigen::Vector3f::Zero() - state.est_V_B_ISS_ISS;
  if (out->ctl_status > 1) {
    // find desired velocity from position error
    v += target_vel + (Kp_lin.array() * out->pos_err.array()).matrix() + out->pos_err_int;
  }
  Eigen::Vector3f a = RotateVectorAtoB(v, state.est_quat_ISS2B);
  a = (a.array() * state.vel_kd.array()).matrix();

  Eigen::Vector3f b = RotateVectorAtoB((target_accel.array() * tun_accel_gain.array()).matrix(),
                                       state.est_quat_ISS2B);

  out->body_force_cmd = SaturateVector(state.mass * (a + b), tun_ctl_linear_force_limit);
  out->body_accel_cmd = out->body_force_cmd / state.mass;
}

Eigen::Vector3f Control::SaturateVector(Eigen::Vector3f v, float limit) {
  float mag = v.norm();

  if (mag < limit) {
    return v;
  } else {
    return limit / mag * v;
  }
}

Eigen::Vector3f Control::RotateVectorAtoB(const Eigen::Vector3f v, const Eigen::Quaternionf q) {
  return q.normalized().conjugate().toRotationMatrix() * v;
}

Eigen::Vector3f Control::DiscreteTimeIntegrator(const Eigen::Vector3f input, Eigen::Vector3f & accumulator,
                                    uint8_t ctl_status, float upper_limit, float lower_limit) {
  Eigen::Vector3f output;
  output.setZero();
  if (ctl_status <= 1) {
    accumulator.setZero();
    return output;
  }
  accumulator += input / 62.5;
  output = accumulator;
  for (int i = 0; i < 3; i++) {
    if (output[i] > upper_limit) {
      output[i] = upper_limit;
    } else if (output[i] < lower_limit) {
      output[i] = lower_limit;
    }
  }
  return output;
}

void Control::FindPosErr(const ControlState & state, const ControlCommand & cmd, ControlOutput* out) {
  Eigen::Vector3f target = out->traj_pos;
  if (stopped_mode_)
    target = prev_position_;
  out->pos_err = target - state.est_P_B_ISS_ISS;

  Eigen::Vector3f Ki_lin = SafeDivide(state.pos_ki, state.vel_kd);
  out->pos_err_int = DiscreteTimeIntegrator((Ki_lin.array() * out->pos_err.array()).matrix(),
                   linear_integrator_, out->ctl_status, tun_ctl_pos_sat_upper, tun_ctl_pos_sat_lower);
}

Eigen::Vector3f Control::SafeDivide(const Eigen::Vector3f & num, const Eigen::Vector3f & denom) {
  Eigen::Vector3f out;
  for (int i = 0; i < 3; i++) {
    out[i] = (denom[i] == 0 ? 0 : num[i] / denom[i]);
  }
  return out;
}


void Control::UpdateCtlStatus(const ControlState & state, ControlOutput* out) {
  float pos_err = (prev_position_ - state.est_P_B_ISS_ISS).squaredNorm();
  float quat_err = QuatError(state.est_quat_ISS2B, prev_att_);

  // is tun_ctl_stopped_pos_thresh supposed to be squared?
  if (((pos_err > tun_ctl_stopped_pos_thresh) || (abs(quat_err) > tun_ctl_stopped_quat_thresh)) &&
      (mode_cmd_ == constants::ctl_stopped_mode)) {
    out->ctl_status = constants::ctl_stopping_mode;
  } else {
    if (stopped_mode_) {
      out->ctl_status = constants::ctl_stopped_mode;
    } else {
      out->ctl_status = mode_cmd_;
    }
  }
}

// update the previous as the last part of the step if it is not in stopped mode
void Control::UpdatePrevious(const ControlState & state) {
  if (!stopped_mode_) {
    prev_position_ = state.est_P_B_ISS_ISS;
    prev_att_ = state.est_quat_ISS2B;
  }
}

// the quaternian_error1 block that performs q_cmd - q_actual * q_error
float Control::QuatError(const Eigen::Quaternionf cmd, const Eigen::Quaternionf actual) {
  Eigen::Quaternion<float> out = actual.conjugate() * cmd;
  NormalizeQuaternion(out);
  return fabs(static_cast<float>(acos(static_cast<double>(out.w())))) * 2;
}

float Control::ButterWorthFilter(float input, float delay_val, float* sum_out) {
  const long double butterworth_gain_1 = 0.0031317642291927056;
  const long double butterworth_gain_2 = -0.993736471541614597;
  float tmp_out = input * butterworth_gain_1;
  float previous_gain = delay_val * butterworth_gain_2;
  tmp_out = tmp_out - previous_gain;
  float output = tmp_out + delay_val;
  *sum_out += output * output;
  return tmp_out;
}

bool Control::FilterThreshold(Eigen::Vector3f vec, float threshold, Eigen::Vector3f & previous) {
  float sum = 0;
  for (int i = 0; i < 3; i++)
    previous[i] = ButterWorthFilter(vec[i], previous[i], &sum);

  return sum < threshold;
}

void Control::UpdateMode(const ControlState & state, const ControlCommand & cmd) {
  if (state.est_confidence != constants::ase_status_converged) {
    mode_cmd_ = constants::ctl_idle_mode;
  } else {
    mode_cmd_ = cmd.mode;
  }

  // shift exisitng elements to the right
  for (int i = 3; i >= 0; i--)
    prev_mode_cmd_[i + 1] = prev_mode_cmd_[i];
  prev_mode_cmd_[0] = mode_cmd_;

  // need to run these outside of if condition to make sure that they are being ran every cycle
  bool vel_below_threshold = FilterThreshold(state.est_V_B_ISS_ISS, tun_ctl_stopping_vel_thresh,
                                             prev_filter_vel_);
  bool omega_below_threshold =  FilterThreshold(state.est_omega_B_ISS_B, tun_ctl_stopping_omega_thresh,
                                                prev_filter_omega_);

  stopped_mode_ = false;
  if (vel_below_threshold && omega_below_threshold) {
    // weird that this doesn't use current command
    if ((prev_mode_cmd_[4] == constants::ctl_stopping_mode) && (prev_mode_cmd_[4] == prev_mode_cmd_[3]) &&
        (prev_mode_cmd_[3] == prev_mode_cmd_[2]) && (prev_mode_cmd_[2] == prev_mode_cmd_[1]))
      stopped_mode_ = true;
  }
}

void Control::Initialize(void) {
  mode_cmd_ = 0;
  stopped_mode_ = false;
  prev_filter_vel_.setZero();
  prev_filter_omega_.setZero();
  prev_mode_cmd_[0] = 0;
  prev_mode_cmd_[1] = 0;
  prev_mode_cmd_[2] = 0;
  prev_mode_cmd_[3] = 0;
  prev_position_.setZero();
  prev_att_.x() = 0; prev_att_.y() = 0; prev_att_.z() = 0; prev_att_.w() = 0;
  linear_integrator_.setZero();
  rotational_integrator_.setZero();
}

void Control::ReadParams(config_reader::ConfigReader* config) {
  Eigen::Vector3d temp;
  if (!msg_conversions::config_read_vector(config, "tun_accel_gain", &temp))
    ROS_FATAL("Unspecified tun_accel_gain.");
  tun_accel_gain = temp.cast<float>();
  if (!msg_conversions::config_read_vector(config, "tun_alpha_gain", &temp))
    ROS_FATAL("Unspecified tun_alpha_gain.");
  tun_alpha_gain = temp.cast<float>();
  if (!config->GetReal("tun_ctl_stopped_pos_thresh", &tun_ctl_stopped_pos_thresh))
    ROS_FATAL("Unspecified tun_ctl_stopped_pos_thresh.");
  if (!config->GetReal("tun_ctl_stopped_quat_thresh", &tun_ctl_stopped_quat_thresh))
    ROS_FATAL("Unspecified tun_ctl_stopped_quat_thresh.");
  if (!config->GetReal("tun_ctl_stopping_omega_thresh", &tun_ctl_stopping_omega_thresh))
    ROS_FATAL("Unspecified tun_ctl_stopping_omega_thresh.");
  if (!config->GetReal("tun_ctl_stopping_vel_thresh", &tun_ctl_stopping_vel_thresh))
    ROS_FATAL("Unspecified tun_ctl_stopping_vel_thresh.");
  if (!config->GetReal("tun_ctl_att_sat_lower", &tun_ctl_att_sat_lower))
    ROS_FATAL("Unspecified tun_ctl_att_sat_lower.");
  if (!config->GetReal("tun_ctl_att_sat_upper", &tun_ctl_att_sat_upper))
    ROS_FATAL("Unspecified tun_ctl_att_sat_upper.");
  if (!config->GetReal("tun_ctl_linear_force_limit", &tun_ctl_linear_force_limit))
    ROS_FATAL("Unspecified tun_ctl_linear_force_limit.");
  if (!config->GetReal("tun_ctl_pos_sat_lower", &tun_ctl_pos_sat_lower))
    ROS_FATAL("Unspecified tun_ctl_pos_sat_lower.");
  if (!config->GetReal("tun_ctl_pos_sat_upper", &tun_ctl_pos_sat_upper))
    ROS_FATAL("Unspecified tun_ctl_pos_sat_upper.");
}

}  // end namespace gnc_autocode
