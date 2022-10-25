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

/*
 About: This file contains code for the old Simulink controller and my new C++ controller. I kept the old controller functions in for testing. To test, I run both controllers in the 
 step function. To ensure they are running the same, I copy the input values before I run a controller. Then, I run the Simulink controller step, copy the values after, 
 revert to the orignal values, and then run my controller. Then, I compare the values. Testing is done in the simulator.
  

  Status: not all of the conversion is correct. The bug I am still working on is in the FindQuatError function which is implementing the quaternion_error block.
    The new output is similar to old one: they are the same for some of the time, but then mine and the old trade off being a certain value and 0. For example, mine will be 0 and the 
    old will be 0.5 and then mine will be 0.5 and the old will be 0. This seems like a simple fix, but I couln't find the fix by the end of my internship.  
    There are a few values that rely on this function working, so these values are wrong and the values that use those are wrong as well.  
*/

#include <ros/console.h>
#include <ros/static_assert.h>
#include <ros/platform.h>
#include <stdlib.h>
#include <ros/assert.h>
#include <gnc_autocode/ctl.h>
#include <config_reader/config_reader.h>
#include <assert.h>
#include <ctl_tunable_funcs.h>
#include <Eigen/Dense>
#include<unsupported/Eigen/MatrixFunctions>
#include<iostream>
#include<sstream>
#include<string>
#include<cstring>
#include<cmath>

namespace gnc_autocode {

void NormalizeQuaternion(Eigen::Quaternionf & out) {
  // enfore positive scalar
  if (out.w() < 0) {
    out.coeffs() = -out.coeffs();  // coeffs is a vector (x,y,z,w)
  }
  // out.normalize();

  float mag = sqrt(static_cast<double>(out.x() * out.x() + out.y() * out.y() + out.z() * out.z() + out.w() * out.w()));
  if (mag > 1e-7) {
    out.x() = out.x() / mag; out.y() = out.y() / mag; out.z() = out.z() / mag; out.w() = out.w() / mag;
  }
}


GncCtlAutocode::GncCtlAutocode(void) {
  // TODO(bcoltin): remove
  constants::tun_accel_gain << 1.0f, 1.0f, 1.0f;
  prev_filter_vel[0] = 0;
  prev_filter_vel[1] = 0;
  prev_filter_vel[2] = 0;
  prev_filter_omega[0] = 0;
  prev_filter_omega[1] = 0;
  prev_filter_omega[2] = 0;
  prev_mode_cmd[0] = 0;
  prev_mode_cmd[1] = 0;
  prev_mode_cmd[2] = 0;
  prev_mode_cmd[3] = 0;
  prev_position[0] = 0;
  prev_position[1] = 0;
  prev_position[2] = 0;
  prev_att[0] = 0;
  prev_att[1] = 0;
  prev_att[2] = 0;
  prev_att[3] = 0;
  linear_integrator[0] = 0;
  linear_integrator[1] = 0;
  linear_integrator[2] = 0;
  rotational_integrator[0] = 0;
  rotational_integrator[1] = 0;
  rotational_integrator[2] = 0;

  /****from Simulink Controller*****/
  controller_ = ctl_controller0(&ctl_input_, &cmd_, &ctl_);
  assert(controller_);
  assert(rtmGetErrorStatus(controller_) == NULL);
}

GncCtlAutocode::~GncCtlAutocode() {
  /****from Simulink Controller*****/
  ctl_controller0_terminate(controller_);
}



void GncCtlAutocode::Step(void) {
// copy of values before
  ctl_input_msg before_ctl_input_;
  cmd_msg before_cmd_;
  ctl_msg before_ctl_;
  memcpy(&before_ctl_input_, &ctl_input_, sizeof(ctl_input_));
  memcpy(&before_cmd_, &cmd_, sizeof(cmd_));
  memcpy(&before_ctl_, &ctl_, sizeof(ctl_));

/****from Simulink Controller*****/
  ctl_controller0_step(controller_, &ctl_input_, &cmd_, &ctl_);


// copy of what it is after Simulink controller
  ctl_input_msg after_ctl_input_;
  cmd_msg after_cmd_;
  ctl_msg after_ctl_;
  memcpy(&after_ctl_input_, &ctl_input_, sizeof(ctl_input_));
  memcpy(&after_cmd_, &cmd_, sizeof(cmd_));
  memcpy(&after_ctl_, &ctl_, sizeof(ctl_));

  // revert back to before Simulink
  memcpy(&ctl_input_, &before_ctl_input_, sizeof(before_ctl_input_));
  memcpy(&cmd_, &before_cmd_, sizeof(before_cmd_));
  memcpy(&ctl_, &before_ctl_, sizeof(before_ctl_));

/*****C++ Implementation*****/
  /*****command_shaper*****/
  CmdSelector();
  GenerateCmdPath();
  GenerateCmdAttitude();
  FindTrajErrors(after_cmd_.traj_quat);
  PublishCmdInput();

  /*****cex_control_executive*****/
  UpdateModeCmd();
  UpdateStoppedMode();
  FindPosError();
  FindQuatError(ctl_input_.est_quat_ISS2B, prev_att, quat_err, dummy);  // Fix this fxn
  UpdatePosAndQuat();
  UpdateCtlStatus();

  BypassShaper();

/*****clc_closed_loop_controller*****/
  VariablesTransfer();

  // Linear Control
  UpdateLinearPIDVals();
  FindPosErr();
  FindLinearIntErr();
  FindBodyForceCmd();

  // Rotational Control
  UpdateRotationalPIDVals();
  FindAttErr();  // same as FindQuatError(CMD_Quat_ISS2B, ctl_input_.est_quat_ISS2B, att_err_mag, att_err) // Finds
                 // att_err_mag and att_err
  UpdateRotateIntErr();
  FindBodyAlphaCmd();
  FindBodyTorqueCmd();

  UpdatePrevious();  // this might need to go later

  /*Publish to ctl_msg */
  VarToCtlMsg();

  /***** Comparison Tests *****/
  TestFloats("traj_error_pos", ctl_.traj_error_pos, after_ctl_.traj_error_pos, 0.00001);  // correct
  TestTwoArrays("traj_quat", cmd_.traj_quat, after_cmd_.traj_quat, 4, 0.00001);  // correct
  TestFloats("traj_error_vel", ctl_.traj_error_vel, after_ctl_.traj_error_vel, 0.00001);  // correct
  TestFloats("traj_error_omega", ctl_.traj_error_omega, after_ctl_.traj_error_omega, 0.00001);  // correct
  TestFloats("traj_error_att", ctl_.traj_error_att, after_ctl_.traj_error_att, 0.001);  // correct

  TestFloats("ctl_status", ctl_.ctl_status, after_ctl_.ctl_status, 0);  // correct
  TestTwoArrays("pos_err", ctl_.pos_err, after_ctl_.pos_err, 3, 0.00001);  // correct
  TestTwoArrays("pos_err_int", ctl_.pos_err_int, after_ctl_.pos_err_int, 3, 0.00001);  // correct

  TestTwoArrays("body_force_cmd", ctl_.body_force_cmd, after_ctl_.body_force_cmd, 3, 0.001);  // correct
  TestTwoArrays("body_accel_cmd", ctl_.body_accel_cmd, after_ctl_.body_accel_cmd, 3, 0.0001);

  TestFloats("att_err_mag", ctl_.att_err_mag, after_ctl_.att_err_mag, 0.001);
  TestTwoArrays("att_err", ctl_.att_err, after_ctl_.att_err, 3, 0.000002);
  TestTwoArrays("body_alpha_cmd", ctl_.body_alpha_cmd, after_ctl_.body_alpha_cmd, 3, 0.0001);  // correct
  // is solved TestTwoArrays(ctl_.body_torque_cmd, after_ctl_.body_torque_cmd, 3, 0.000002);
  // tbd when body_alpha_cmd is fixed

  // revert back to Simulink after my controller
  memcpy(&ctl_input_, &after_ctl_input_, sizeof(after_ctl_input_));
  memcpy(&cmd_, &after_cmd_, sizeof(after_cmd_));
  memcpy(&ctl_, &after_ctl_, sizeof(after_ctl_));
}

/*Command Shaper */
void GncCtlAutocode::PublishCmdInput() {
  cmd_.cmd_timestamp_sec = cmd_timestamp_sec;
  cmd_.cmd_timestamp_nsec = cmd_timestamp_nsec;
  cmd_.cmd_mode = ctl_input_.ctl_mode_cmd;
  cmd_.speed_gain_cmd = ctl_input_.speed_gain_cmd;
  cmd_.cmd_B_inuse = cmd_B_inuse;
  for (int i = 0; i < 3; i++) {
    cmd_.traj_pos[i] = traj_pos[i];
    cmd_.traj_vel[i] = traj_vel[i];
    cmd_.traj_accel[i] = traj_accel[i];
    cmd_.traj_omega[i] = traj_omega[i];
    cmd_.traj_alpha[i] = traj_alpha[i];
    cmd_.traj_quat[i] = traj_quat[i];
  }
  cmd_.traj_quat[3] = traj_quat[3];  // since it is size 4
}

void GncCtlAutocode::FindTrajErrors(float traj_q[4]) {
  float traj_error_pos_vec[3];
  float traj_error_vel_vec[3];
  float traj_error_omega_vec[3];
  for (int i = 0; i < 3; i++) {
    traj_error_pos_vec[i] = traj_pos[i] - ctl_input_.est_P_B_ISS_ISS[i];
    traj_error_vel_vec[i] = traj_vel[i] - ctl_input_.est_V_B_ISS_ISS[i];
    traj_error_omega_vec[i] = traj_omega[i] - ctl_input_.est_omega_B_ISS_B[i];
  }

  // magnitude of the vectors
  traj_error_pos = sqrt(pow(traj_error_pos_vec[0], 2) + pow(traj_error_pos_vec[1], 2) + pow(traj_error_pos_vec[2], 2));
  traj_error_vel = sqrt(pow(traj_error_vel_vec[0], 2) + pow(traj_error_vel_vec[1], 2) + pow(traj_error_vel_vec[2], 2));
  traj_error_omega =
    sqrt(pow(traj_error_omega_vec[0], 2) + pow(traj_error_omega_vec[1], 2) + pow(traj_error_omega_vec[2], 2));

  FindQuatError(traj_q, ctl_input_.est_quat_ISS2B, traj_error_att, dummy);
}



void GncCtlAutocode::GenerateCmdAttitude() {
  if (state_cmd_switch_out) {
    for (int i = 0; i < 3; i++) {
      traj_alpha[i] = ctl_input_.cmd_state_a.alpha_B_ISS_B[i];
      traj_omega[i] = ctl_input_.cmd_state_a.omega_B_ISS_B[i] + (ctl_input_.cmd_state_a.alpha_B_ISS_B[i] * time_delta);
    }
  } else {  // false so cmd B
    for (int i = 0; i < 3; i++) {
      traj_alpha[i] = ctl_input_.cmd_state_b.alpha_B_ISS_B[i];
      traj_omega[i] = ctl_input_.cmd_state_b.omega_B_ISS_B[i] + (ctl_input_.cmd_state_b.alpha_B_ISS_B[i] * time_delta);
    }
  }

  if (state_cmd_switch_out)
    FindTrajQuat(ctl_input_.cmd_state_a.omega_B_ISS_B, ctl_input_.cmd_state_a.alpha_B_ISS_B,
                 ctl_input_.cmd_state_a.quat_ISS2B);
  else
    FindTrajQuat(ctl_input_.cmd_state_b.omega_B_ISS_B, ctl_input_.cmd_state_b.alpha_B_ISS_B,
                 ctl_input_.cmd_state_b.quat_ISS2B);
}

void GncCtlAutocode::FindTrajQuat(float omega_B_ISS_B[3], float alpha_B_ISS_B[3], float quat_ISS2B[4]) {
  Eigen::Quaternion<float> quat_state_cmd;
  quat_state_cmd.x() = quat_ISS2B[0];
  quat_state_cmd.y() = quat_ISS2B[1];
  quat_state_cmd.z() = quat_ISS2B[2];
  quat_state_cmd.w() = quat_ISS2B[3];

  Eigen::Matrix<float, 4, 4> omega_omega = OmegaMatrix(omega_B_ISS_B);
  Eigen::Matrix<float, 4, 4> omega_alpha = OmegaMatrix(alpha_B_ISS_B);

  Eigen::Matrix<float, 4, 4> a = 0.5 * time_delta * (0.5 * time_delta * omega_alpha + omega_omega);

  a = a.exp();
  a += (1.0/48) * time_delta * time_delta * time_delta * (omega_alpha * omega_omega - omega_omega * omega_alpha);
  Eigen::Quaternion<float> out;
  out.coeffs() = a * quat_state_cmd.coeffs();

  NormalizeQuaternion(out);
  traj_quat[0] = out.x();
  traj_quat[1] = out.y();
  traj_quat[2] = out.z();
  traj_quat[3] = out.w();
}
// Defined in Indirect Kalman Filter for 3d attitude Estimation - Trawn, Roumeliotis eq 63
Eigen::Matrix<float, 4, 4> GncCtlAutocode::OmegaMatrix(float input[3]) {
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

void GncCtlAutocode::GenerateCmdPath() {
  float test[3];
  if (state_cmd_switch_out) {  // true is A
    for (int i = 0; i < 3; i++) {
      traj_pos[i] = ctl_input_.cmd_state_a.P_B_ISS_ISS[i] + (ctl_input_.cmd_state_a.V_B_ISS_ISS[i] * time_delta) +
                    (0.5 * ctl_input_.cmd_state_a.A_B_ISS_ISS[i] * time_delta * time_delta);
      traj_vel[i] = ctl_input_.cmd_state_a.V_B_ISS_ISS[i] + (ctl_input_.cmd_state_a.A_B_ISS_ISS[i] * time_delta);
      traj_accel[i] = ctl_input_.cmd_state_a.A_B_ISS_ISS[i];
    }
  } else {  // false so cmd B
      for (int i = 0; i < 3; i++) {
        traj_pos[i] = ctl_input_.cmd_state_b.P_B_ISS_ISS[i] + (ctl_input_.cmd_state_b.V_B_ISS_ISS[i] * time_delta) +
                      (0.5 * ctl_input_.cmd_state_b.A_B_ISS_ISS[i] * time_delta * time_delta);

        traj_vel[i] = ctl_input_.cmd_state_b.V_B_ISS_ISS[i] + (ctl_input_.cmd_state_b.A_B_ISS_ISS[i] * time_delta);
        traj_accel[i] = ctl_input_.cmd_state_b.A_B_ISS_ISS[i];
    }
  }
}

void GncCtlAutocode::CmdSelector() {
  float curr_sec = ctl_input_.current_time_sec;
  float curr_nsec = ctl_input_.current_time_nsec;

  if ((ctl_input_.cmd_state_b.timestamp_sec > curr_sec) ||
      ((ctl_input_.cmd_state_b.timestamp_sec == curr_sec) && (curr_nsec < ctl_input_.cmd_state_b.timestamp_nsec))) {
    state_cmd_switch_out = true;
    time_delta =
      (curr_sec - ctl_input_.cmd_state_a.timestamp_sec) + ((curr_nsec - ctl_input_.cmd_state_a.timestamp_nsec) * 1E-9);
    cmd_timestamp_sec = ctl_input_.cmd_state_a.timestamp_sec;
    cmd_timestamp_nsec = ctl_input_.cmd_state_a.timestamp_nsec;
  } else {
    state_cmd_switch_out = false;
    time_delta =
      (curr_sec - ctl_input_.cmd_state_b.timestamp_sec) + ((curr_nsec - ctl_input_.cmd_state_b.timestamp_nsec) * 1E-9);
    cmd_timestamp_sec = ctl_input_.cmd_state_b.timestamp_sec;
    cmd_timestamp_nsec = ctl_input_.cmd_state_b.timestamp_nsec;
  }

  cmd_B_inuse = !state_cmd_switch_out;
}

/* Testing functions */
void GncCtlAutocode::TestFloats(const char* name, const float new_float, const float old_float, float tolerance) {
  float difference = old_float - new_float;
  float perc_difference = difference / old_float;
    if (fabs(difference) > tolerance) {
     ROS_ERROR("%s New: %f, Old: %f, Difference: %f", name, new_float, old_float, difference);
  }
}

void GncCtlAutocode::TestTwoArrays(const char* name, const float new_array[],
                                   const float old_array[], int length, float tolerance) {
  for (int i = 0; i < length; i++) {
    float difference = old_array[i] - new_array[i];
    float perc_difference = difference / old_array[i];
    if (fabs(difference) > tolerance) {
      std::string p1, p2;
      for (int i = 0; i < length; i++) {
        p1 += " " + std::to_string(new_array[i]);
        p2 += " " + std::to_string(old_array[i]);
      }
      ROS_ERROR("%s New: %s, Old: %s", name, p1.c_str(), p2.c_str());
    }
  }
}

void GncCtlAutocode::VarToCtlMsg() {
  ctl_.pos_err[0] = pos_err_outport.x();
  ctl_.pos_err[1] = pos_err_outport.y();
  ctl_.pos_err[2] = pos_err_outport.z();
  ctl_.pos_err_int[0] = linear_int_err.x();
  ctl_.pos_err_int[1] = linear_int_err.y();
  ctl_.pos_err_int[2] = linear_int_err.z();
  for (int i = 0; i < 3; i++) {
    ctl_.body_force_cmd[i] = body_force_cmd[i];
    ctl_.body_accel_cmd[i] = body_accel_cmd[i];

    ctl_.body_torque_cmd[i] = body_torque_cmd[i];
    ctl_.body_alpha_cmd[i] = body_alpha_cmd[i];
    ctl_.att_err[i] = att_err[i];

    ctl_.att_err_int[i] = rotate_int_err[i];
  }
  ctl_.att_err_mag = att_err_mag;
  ctl_.ctl_status = ctl_status;
  ctl_.traj_error_pos = traj_error_pos;
  ctl_.traj_error_att = traj_error_att;
  ctl_.traj_error_vel = traj_error_vel;
  ctl_.traj_error_omega = traj_error_omega;
}

/*****clc_closed_loop_controller functions*****/
void GncCtlAutocode::FindAttErr() {
  Eigen::Quaternion<float> q_cmd;
  Eigen::Quaternion<float> q_actual;
  q_cmd.x() = CMD_Quat_ISS2B[0];
  q_cmd.y() = CMD_Quat_ISS2B[1];
  q_cmd.z() = CMD_Quat_ISS2B[2];
  q_cmd.w() = CMD_Quat_ISS2B[3];

  q_actual.x() = ctl_input_.est_quat_ISS2B[0];
  q_actual.y() = ctl_input_.est_quat_ISS2B[1];
  q_actual.z() = ctl_input_.est_quat_ISS2B[2];
  q_actual.w() = ctl_input_.est_quat_ISS2B[3];

  Eigen::Quaternion<float> q_out = q_actual.conjugate() * q_cmd;

  //  ROS_ERROR("newc %g %g %g %g", q_cmd.x(), q_cmd.y(), q_cmd.z(), q_cmd.w());
  //  ROS_ERROR("newa %g %g %g %g", q_actual.x(), q_actual.y(), q_actual.z(), q_actual.w());
  //  ROS_ERROR("newerr %g %g %g %.20g", q_out.x(), q_out.y(), q_out.z(), q_out.w());
  NormalizeQuaternion(q_out);

  //  ROS_ERROR("newerr %g %g %g %.20g", q_out.x(), q_out.y(), q_out.z(), q_out.w());
  att_err_mag = 2 * acos(static_cast<double>(q_out.w()));
  att_err[0] = q_out.x();
  att_err[1] = q_out.y();
  att_err[2] = q_out.z();
}

void GncCtlAutocode::FindBodyTorqueCmd() {
  if (ctl_status != 0) {
    for (int i = 0; i < 3; i++) {
      body_torque_cmd[i] = 0;
    }
  } else {
    // feed forward accel
    float ang_accel_feed[3];
    Eigen::Vector3f alpha;
    alpha << CMD_Alpha_B_ISS_B[0], CMD_Alpha_B_ISS_B[1], CMD_Alpha_B_ISS_B[2];
    Eigen::Vector3f omega;
    omega << ctl_input_.est_omega_B_ISS_B[0], ctl_input_.est_omega_B_ISS_B[1], ctl_input_.est_omega_B_ISS_B[2];

    Eigen::Vector3f torque = inertia * alpha + rate_error - (inertia * omega).cross(omega);

    for (int i = 0; i < 3; i++) {
      body_torque_cmd[i] = torque[i];
    }
  }
}

void GncCtlAutocode::CrossProduct(float vecA[3], float vecB[3], float vecOut[3]) {
  vecOut[0] = vecA[1] * vecB[2] - vecA[2] * vecB[1];
  vecOut[1] = -(vecA[0] * vecB[2] - vecA[2] * vecB[0]);
  vecOut[2] = vecA[0] * vecB[1] - vecA[1] * vecB[0];
}
void GncCtlAutocode::FindBodyAlphaCmd() {
  rate_error = AngAccelHelper();
  inertia << ctl_input_.inertia_matrix[0], ctl_input_.inertia_matrix[1], ctl_input_.inertia_matrix[2],
             ctl_input_.inertia_matrix[3], ctl_input_.inertia_matrix[4], ctl_input_.inertia_matrix[5],
             ctl_input_.inertia_matrix[6], ctl_input_.inertia_matrix[7], ctl_input_.inertia_matrix[8];

  Eigen::Vector3f v = inertia.inverse() * rate_error;
  for (int i = 0; i < 3; i++)
    body_alpha_cmd[i] = v[i];
}

Eigen::Vector3f GncCtlAutocode::AngAccelHelper() {
  float temp[3];
  if (ctl_status <= 1) {
    for (int i = 0; i < 3; i++) {
      rate_error[i] = -ctl_input_.est_omega_B_ISS_B[i];
    }
  } else {
    for (int i = 0; i < 3; i++) {
      rate_error[i] =
        CMD_Omega_B_ISS_B[i] + rotate_int_err[i] + (Kp_rot[i] * att_err[i]) - ctl_input_.est_omega_B_ISS_B[i];
      temp[i] = rotate_int_err[i] + (Kp_rot[i] * att_err[i]);
    }
  }
  //  ROS_ERROR("newa %g %g %g", CMD_Omega_B_ISS_B[0], CMD_Omega_B_ISS_B[1], CMD_Omega_B_ISS_B[2]);
  //  ROS_ERROR("newb %g %g %g", temp[0], temp[1], temp[2]);

  for (int i = 0; i < 3; i++) {
    rate_error[i] *= Kd_rot[i];
  }
  //  ROS_ERROR("newr %g %g %g", rate_error[0], rate_error[1], rate_error[2]);
  return rate_error;
}

void GncCtlAutocode::UpdateRotateIntErr() {
  Eigen::Vector3f in; in << att_err[0] * Ki_rot[0], att_err[1] * Ki_rot[1], att_err[2] * Ki_rot[2];
  Eigen::Vector3f out = discreteTimeIntegrator(in, rotational_integrator, constants::tun_ctl_att_sat_upper,
                                            constants::tun_ctl_att_sat_lower);
  rotate_int_err[0] = out.x();
  rotate_int_err[1] = out.y();
  rotate_int_err[2] = out.z();
}

void GncCtlAutocode::UpdateRotationalPIDVals() {
  // reshape
  float inertia_vec[3] = {ctl_input_.inertia_matrix[0], ctl_input_.inertia_matrix[4], ctl_input_.inertia_matrix[8]};

  for (int i = 0; i < 3; i++) {
    Kp_rot[i] = SafeDivide(ctl_input_.att_kp[i], ctl_input_.omega_kd[i]);
    Ki_rot[i] = SafeDivide(ctl_input_.att_ki[i], ctl_input_.omega_kd[i]);
    Kd_rot[i] = ctl_input_.omega_kd[i] * inertia_vec[i];
  }
}

void GncCtlAutocode::FindBodyForceCmd() {
  if (ctl_status == 0) {
    for (int i = 0; i < 3; i++) {
      body_force_cmd[i] = 0.0;
      body_accel_cmd[i] = 0.0;
    }
    return;
  }
  Eigen::Vector3f v; v << 0.0f, 0.0f, 0.0f;
  if (ctl_status > 1) {
    // find desired velocity from position error
    v = CMD_V_B_ISS_ISS + (Kp_lin.array() * pos_err_outport.array()).matrix() + linear_int_err;
  }
  Eigen::Vector3f temp = (Kp_lin.array() * pos_err_outport.array()).matrix() + linear_int_err;
  //  ROS_ERROR("new2a %g %g %g", linear_int_err[0], linear_int_err[1], linear_int_err[2]);
  //  ROS_ERROR("new2a %g %g %g", CMD_V_B_ISS_ISS[0], CMD_V_B_ISS_ISS[1], CMD_V_B_ISS_ISS[2]);
  //  ROS_ERROR("new2b %g %g %g", temp[0], temp[1], temp[2]);
  Eigen::Vector3f est_V_B_ISS_ISS;
  est_V_B_ISS_ISS << ctl_input_.est_V_B_ISS_ISS[0], ctl_input_.est_V_B_ISS_ISS[1], ctl_input_.est_V_B_ISS_ISS[2];
  //  ROS_ERROR("new2c %g %g %g", est_V_B_ISS_ISS[0], est_V_B_ISS_ISS[1], est_V_B_ISS_ISS[2]);
  v -= est_V_B_ISS_ISS;

  float velo_err_tmp[3];
  float feed_accel_tmp[3];
  Eigen::Quaternionf est_quat_ISS2B;
  est_quat_ISS2B.x() = ctl_input_.est_quat_ISS2B[0];
  est_quat_ISS2B.y() = ctl_input_.est_quat_ISS2B[1];
  est_quat_ISS2B.z() = ctl_input_.est_quat_ISS2B[2];
  est_quat_ISS2B.w() = ctl_input_.est_quat_ISS2B[3];
  Eigen::Vector3f a = RotateVectorAtoB(v, est_quat_ISS2B);
  //  ROS_ERROR("newt %g %g %g", v[0], v[1], v[2]);
  //  ROS_ERROR("newq %g %g %g %g", est_quat_ISS2B.x(), est_quat_ISS2B.y(), est_quat_ISS2B.z(), est_quat_ISS2B.w());
  //  ROS_ERROR("new2 %g %g %g", a[0], a[1], a[2]);
  a = ctl_input_.mass * (a.array() * Kd_lin.array()).matrix();

  float accel_err_tmp[3];
  Eigen::Vector3f b = RotateVectorAtoB((CMD_A_B_ISS_ISS.array() * constants::tun_accel_gain.array()).matrix(),
                                       est_quat_ISS2B);
  v = CMD_A_B_ISS_ISS.array() * constants::tun_accel_gain.array();
  //  ROS_ERROR("newa %g %g %g", v[0], v[1], v[2]);
  //  ROS_ERROR("newa1 %g %g %g", CMD_A_B_ISS_ISS[0], CMD_A_B_ISS_ISS[1], CMD_A_B_ISS_ISS[2]);
  //  ROS_ERROR("new1 %g %g %g", b[0], b[1], b[2]);
  b *= ctl_input_.mass;
  Eigen::Vector3f t = a + b;
  //  ROS_ERROR("newt %g %g %g", t[0], t[1], t[2]);

  Eigen::Vector3f out = SaturateVector(a + b, constants::tun_ctl_linear_force_limit);
  for (int i = 0; i < 3; i++) {
    body_force_cmd[i] = out[i];
    body_accel_cmd[i] = body_force_cmd[i] / ctl_input_.mass;
  }
}

Eigen::Vector3f GncCtlAutocode::SaturateVector(Eigen::Vector3f v, float limit) {
  float mag = v.norm();

  if (mag < limit) {
    return v;
  } else {
    return limit / mag * v;
  }
}

Eigen::Vector3f GncCtlAutocode::RotateVectorAtoB(const Eigen::Vector3f v, const Eigen::Quaternionf q) {
  return q.normalized().conjugate().toRotationMatrix() * v;
}

// 3x3 multiply by 1x3
void GncCtlAutocode::MatrixMultiplication3x1(float three[3][3], float one[3], float output[3]) {
  // set output to all 0's
  for (int i = 0; i < 3; i++) {
    output[i]= 0;
    }
    for (int i = 0; i < 3; i++) {
      for (int j = 0; j < 3; j++) {
        output[i] += three[i][j] * one[i];
      }
    }
}

// 4x4 multiply by 1x4
void GncCtlAutocode::MatrixMultiplication4x1(float four[4][4], float one[4], float output[4]) {
  // set output to all 0's
  for (int i = 0; i < 4; i++) {
    output[i]= 0;
    }
    for (int i = 0; i < 4; i++) {
      for (int j = 0; j < 4; j++) {
        output[i] += four[i][j] * one[i];
      }
    }
}

void GncCtlAutocode::MatrixMultiplication3x3(float inputA[3][3], float inputB[3][3], float output[3][3]) {
  // make output all zeros
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      output[i][j] = 0;
    }
  }

  for (int row = 0; row < 3; row++) {
    for (int col = 0; col < 3; col++) {
      for (int k = 0; k < 3; k++) {
        output[row][col] +=inputA[row][k] * inputB[k][col];
      }
    }
  }
}

void GncCtlAutocode::MatrixMultiplication4x4(float inputA[4][4], float inputB[4][4], float output[4][4]) {
  // make output all zeros
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 4; j++) {
      output[i][j] = 0;
    }
  }

  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      for (int k = 0; k < 4; k++) {
        output[row][col] +=inputA[row][k] * inputB[k][col];
      }
    }
  }
}
void GncCtlAutocode::SkewSymetricMatrix(const float input[3], float output[3][3]) {
  // from simulink diagram
  output[0][0] = 0;
  output[1][0] = input[2];
  output[2][0] = -input[1];
  output[0][1] = -input[2];
  output[1][1] = 0;
  output[2][1] = input[0];
  output[0][2] = input[1];
  output[1][2] = -input[0];
  output[2][2] = 0;
}

void GncCtlAutocode::FindLinearIntErr() {
  linear_int_err = discreteTimeIntegrator((Ki_lin.array() * pos_err_outport.array()).matrix(),
                   linear_integrator, constants::tun_ctl_pos_sat_upper, constants::tun_ctl_pos_sat_lower);
}

Eigen::Vector3f GncCtlAutocode::discreteTimeIntegrator(Eigen::Vector3f input, float accumulator[3], float upper_limit,
                                            float lower_limit) {
  Eigen::Vector3f output; output << 0.0f, 0.0f, 0.0f;
  if (ctl_status <= 1) {
    for (int i = 0; i < 3; i++) {
      accumulator[0] = 0;
    }
    return output;
  }
  for (int i = 0; i < 3; i++) {
    accumulator[i] += (input[i] / 62.5);
    output[i] = accumulator[i];
    if (output[i] > upper_limit) {
      output[i] = upper_limit;
    } else if (output[i] < lower_limit) {
      output[i] = lower_limit;
    }
  }
  return output;
}

// shouldn't be needed but keeps naming consistant with simulink
void GncCtlAutocode::VariablesTransfer() {
  for (int i = 0; i < 3; i++) {
    CMD_Quat_ISS2B[i] = att_command[i];
    CMD_Omega_B_ISS_B[i] = omega_command[i];
    CMD_Alpha_B_ISS_B[i] = alpha_command[i];
  }
  CMD_Quat_ISS2B[3] = att_command[3];  // since quat has size 4
}

void GncCtlAutocode::FindPosErr() {
  Eigen::Vector3f t;
  t << ctl_input_.est_P_B_ISS_ISS[0], ctl_input_.est_P_B_ISS_ISS[1], ctl_input_.est_P_B_ISS_ISS[2];
  pos_err_outport = CMD_P_B_ISS_ISS - t;
}

void GncCtlAutocode::UpdateLinearPIDVals() {
  Kp_lin.x() = SafeDivide(ctl_input_.pos_kp[0], ctl_input_.vel_kd[0]);
  Kp_lin.y() = SafeDivide(ctl_input_.pos_kp[1], ctl_input_.vel_kd[1]);
  Kp_lin.z() = SafeDivide(ctl_input_.pos_kp[2], ctl_input_.vel_kd[2]);
  Ki_lin.x() = SafeDivide(ctl_input_.pos_ki[0], ctl_input_.vel_kd[0]);
  Ki_lin.y() = SafeDivide(ctl_input_.pos_ki[1], ctl_input_.vel_kd[1]);
  Ki_lin.z() = SafeDivide(ctl_input_.pos_ki[2], ctl_input_.vel_kd[2]);
  Kd_lin.x() = ctl_input_.vel_kd[0];
  Kd_lin.y() = ctl_input_.vel_kd[1];
  Kd_lin.z() = ctl_input_.vel_kd[2];
}

float GncCtlAutocode::SafeDivide(float num, float denom) {
  if (denom == 0) {
    return 0;
  } else {
    return num / denom;
  }
}

/*****cex_control_executive functions *****/
void GncCtlAutocode::BypassShaper() {
  if (stopped_mode) {
    CMD_V_B_ISS_ISS << 0.0f, 0.0f, 0.0f;
    CMD_A_B_ISS_ISS << 0.0f, 0.0f, 0.0f;
    for (int i = 0; i < 3; i++) {
      omega_command[i] = 0;
      alpha_command[i] = 0;
    }
  } else {
    for (int i = 0; i < 3; i++) {
      CMD_V_B_ISS_ISS[i] = cmd_.traj_vel[i];
      CMD_A_B_ISS_ISS[i] = cmd_.traj_accel[i];
      omega_command[i] = cmd_.traj_omega[i];
      alpha_command[i] = cmd_.traj_alpha[i];
    }
  }
}


void GncCtlAutocode::UpdateCtlStatus() {
  if (CtlStatusSwitch()) {
    ctl_status = constants::ctl_stopping_mode;
  } else {
    if (stopped_mode) {
      ctl_status = constants::ctl_stopped_mode;
    } else {
      ctl_status = mode_cmd;
    }
  }
}

// determines if still in stopping; called by UpdateCtlStatus
bool GncCtlAutocode::CtlStatusSwitch() {
  // find sum of squares
  float pos_sum = 0;
  for (int i = 0; i < 3; i++) {
    float tmp = pos_err_parameter[i] * pos_err_parameter[i];
    pos_sum = pos_sum + tmp;
  }

  if (((pos_sum > constants::tun_ctl_stopped_pos_thresh) || (abs(quat_err) > constants::tun_ctl_stopped_quat_thresh)) &&
      (mode_cmd == constants::ctl_stopped_mode)) {
    return true;
  }
    return false;
}

// update the previous as the last part of the step if it is not in stopped mode
void GncCtlAutocode::UpdatePrevious() {
  if (!stopped_mode) {
    for (int i = 0; i < 3; i++) {
      prev_position[i] =  ctl_input_.est_P_B_ISS_ISS[i];
      prev_att[i] = ctl_input_.est_quat_ISS2B[i];
    }
    prev_att[3] = ctl_input_.est_quat_ISS2B[3];  // this has 1 more element thatn pos.
  }
}

void GncCtlAutocode::FindPosError() {
  for (int i = 0; i < 3; i++) {
    pos_err_parameter[i] = prev_position[i] - ctl_input_.est_P_B_ISS_ISS[i];
  }
}

// the quaternian_error1 block that performs q_cmd - q_actual * q_error
// Simulink q_cmd is of format x,y,z,w
void GncCtlAutocode::FindQuatError(float q_cmd[4], float q_actual[4], float& output_scalar, float output_vec[3]) {
  Eigen::Quaternion<float> cmd;
  cmd.x() = q_cmd[0];
  cmd.y() = q_cmd[1];
  cmd.z() = q_cmd[2];
  cmd.w() = q_cmd[3];

  Eigen::Quaternion<float> actual;
  actual.x() = q_actual[0];
  actual.y() = q_actual[1];
  actual.z() = q_actual[2];
  actual.w() = q_actual[3];

  //  ROS_ERROR("newc %g %g %g %g", cmd.x(), cmd.y(), cmd.z(), cmd.w());
  //  ROS_ERROR("newina %g %g %g %g", actual.x(), actual.y(), actual.z(), actual.w());
  Eigen::Quaternion<float> out = actual.conjugate() * cmd;
  //  ROS_ERROR("newa %g %g %g %g", out.x(), out.y(), out.z(), out.w());
  NormalizeQuaternion(out);

  output_vec[0] = out.x();
  output_vec[1] = out.y();
  output_vec[2] = out.z();

  output_scalar = fabs(acos(static_cast<double>(out.w()))) * 2;
}

// updates the position and attitude command
void GncCtlAutocode::UpdatePosAndQuat() {
  if (stopped_mode) {
    CMD_P_B_ISS_ISS.x() = prev_position[0];
    CMD_P_B_ISS_ISS.y() = prev_position[1];
    CMD_P_B_ISS_ISS.z() = prev_position[2];
    for (int i = 0; i < 3; i++) {
      att_command[i] = prev_att[i];
    }
    att_command[3] = prev_att[3];
  } else {
    CMD_P_B_ISS_ISS.x() = cmd_.traj_pos[0];
    CMD_P_B_ISS_ISS.y() = cmd_.traj_pos[1];
    CMD_P_B_ISS_ISS.z() = cmd_.traj_pos[2];
    for (int i = 0; i < 3; i++) {
      att_command[i] = cmd_.traj_quat[i];
    }
    att_command[3] = cmd_.traj_quat[3];
  }
}

/*stopped mode is true when velocity and omega are below thresholds and
 mode_cmd is stopping for 4 cycles */
void GncCtlAutocode::UpdateStoppedMode() {
  float velocity[3];
  float omega[3];
  for (int i = 0; i < 3; i++) {
    velocity[i]= ctl_input_.est_V_B_ISS_ISS[i];
    omega[i] = ctl_input_.est_omega_B_ISS_B[i];
  }

  // need to run these outside of if condition to make sure that they are being ran every cycle
  bool vel_below_threshold = BelowThreshold(velocity, constants::tun_ctl_stopping_vel_thresh, prev_filter_vel);
  bool omega_below_threshold =  BelowThreshold(omega, constants::tun_ctl_stopping_omega_thresh, prev_filter_omega);
  bool cmd_make = CmdModeMakeCondition();
  if (vel_below_threshold && omega_below_threshold && cmd_make) {
    stopped_mode = true;
  } else {
    stopped_mode = false;
  }
}
/*Butterworth filter implementation */
float GncCtlAutocode::ButterWorthFilter(float input, float& delay_val) {
  float tmp_out = input * constants::butterworth_gain_1;
  float previous_gain = delay_val * constants::butterworth_gain_2;
  tmp_out = tmp_out - previous_gain;
  float output = tmp_out + delay_val;
  delay_val = tmp_out;
  return output;
}
/*determine if velocity (linear or angular) values are less than threshhold
  retruns true if it is less than the threshhold*/
bool GncCtlAutocode::BelowThreshold(float velocity[], float threshhold, float previous[3]) {
  float filter_out;
  float sum = 0;
  for (int i = 0; i < 3; i++) {
    filter_out =
      GncCtlAutocode::ButterWorthFilter(velocity[i], previous[i]);  // previous[i] gets updated in this function
    filter_out = filter_out * filter_out;  // square the value
    sum = sum + filter_out;                // sum all of the values
  }

  if (sum < threshhold) {
    return true;
  }
  return false;
}

/*Determines if make conditions is met: if mode_cmd equals ctl_stopping_mode for 4 previous times*/
bool GncCtlAutocode::CmdModeMakeCondition() {
  // shift exisitng elements to the right
  prev_mode_cmd[4] = prev_mode_cmd[3];
  prev_mode_cmd[3] = prev_mode_cmd[2];
  prev_mode_cmd[2] = prev_mode_cmd[1];
  prev_mode_cmd[1] = prev_mode_cmd[0];
  // update index 0 with new value
  prev_mode_cmd[0] = mode_cmd;
  // Note: since i am shifting first, i don't account for the newest value in the comparison
  // check if they all equal ctl_stopping_mode
  if ((prev_mode_cmd[4] == constants::ctl_stopping_mode) && (prev_mode_cmd[4] == prev_mode_cmd[3]) &&
      (prev_mode_cmd[3] == prev_mode_cmd[2]) && (prev_mode_cmd[2] == prev_mode_cmd[1])) {
    return true;
  }
  return false;
}

/* triggers IDLE if est_confidence is 0; idles if diverged */
void GncCtlAutocode::UpdateModeCmd() {
  if (ctl_input_.est_confidence != constants::ase_status_converged) {
    mode_cmd = constants::ctl_idle_mode;
  } else {
    // mode_cmd = cmd_.cmd_mode;
    mode_cmd = ctl_input_.ctl_mode_cmd;
  }
}

void GncCtlAutocode::Initialize(void) {
  /****from Simulink Controller*****/
  ctl_controller0_initialize(controller_, &ctl_input_, &cmd_, &ctl_);
}

void GncCtlAutocode::ReadParams(config_reader::ConfigReader* config) {
  /****from Simulink Controller*****/
  ctl_ReadParams(config, controller_);
}

}  // end namespace gnc_autocode
