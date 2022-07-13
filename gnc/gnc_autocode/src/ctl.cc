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
#include <gnc_autocode/ctl.h>
#include <config_reader/config_reader.h>
#include <assert.h>
#include <ctl_tunable_funcs.h>
#include <Eigen/Dense>



namespace gnc_autocode {




GncCtlAutocode::GncCtlAutocode(void) {
  prev_filter[0] = 0;
  prev_filter[1] = 0;
  prev_filter[2] = 0;
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
}

GncCtlAutocode::~GncCtlAutocode() {
}



void GncCtlAutocode::Step(void) {
  // auto* ctl_input = &ctl_input_;
  // auto cmd = &cmd_;
  // auto ctl = &ctl_; //this is the output
  // auto myVar = ctl_input->est_quat_ISS2B;
  // auto var = constants::ase_status_converged;
  // auto var1 = ctl_input_.est_confidence;

  UpdateModeCmd();
  UpdateStoppedMode();
  UpdatePosAndQuat();

  for (int i = 0; i < 3; i++) {
    pos_err[i] = prev_position[i] - ctl_input_.est_P_B_ISS_ISS[i];
  }

  // update the previous as the last part of the step if it is not in stopped mode
  for (int i = 0; i < 3; i++) {
    if (!stopped_mode) {
      prev_position[i] =  ctl_input_.est_P_B_ISS_ISS[i];
      prev_att[i] = ctl_input_.est_quat_ISS2B[i];
    }
  }
}

void GncCtlAutocode::Initialize(void) {
}




void GncCtlAutocode::ReadParams(config_reader::ConfigReader* config) {
}

// the quaternian_error1 block that performs q_cmd - q_actual * q_error
// Simulink q_cmd is of format x,y,z,w
void GncCtlAutocode::QuaternianError(float q_cmd[4], float q_actual[4]) {
  Eigen::Quaternion<float> cmd;
  cmd.w() = q_cmd[3];
  cmd.x() = q_cmd[0];
  cmd.y() = q_cmd[1];
  cmd.z() = q_cmd[2];

  Eigen::Quaternion<float> actual;
  actual.w() = q_actual[3];
  actual.x() = q_actual[0];
  actual.y() = q_actual[1];
  actual.z() = q_actual[2];

  Eigen::Quaternion<float> out = cmd.inverse() * actual;

  // enfore positive scalar
  if (out.w() < 0) {
    out.coeffs() = -out.coeffs();  // coeffs is a vector (x,y,z,w)
  }
  out.normalize();

  quat_err = acos(out.w()) * 2;
}

// updates the position and attitude command
void GncCtlAutocode::UpdatePosAndQuat() {
  if (stopped_mode) {
    for (int i = 0; i < 3; i++) {
      position_command[i] = prev_position[i];
      att_command[i] = prev_att[i];
    }
  } else {
    for (int i = 0; i < 3; i++) {
      position_command[i] = cmd_.traj_pos[i];
      att_command[i] = cmd_.traj_quat[i];
    }
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

  if (BelowThreshold(velocity, constants::tun_ctl_stopping_vel_thresh) &&
      BelowThreshold(omega, constants::tun_ctl_stopping_omega_thresh) && CmdModeMakeCondition()) {
    stopped_mode = true;
  } else {
    stopped_mode = false;
  }
}
/*Butterworth filter implementation */
float GncCtlAutocode::ButterWorthFilter(float input, float previous) {
  float output = input * constants::butterworth_gain_1;
  float previous_gain = previous * constants::butterworth_gain_2;
  output = output - previous_gain;
  output = output + previous;
  return output;
}
/*determine if velocity (linear or angular) values are less than threshhold
  retruns true if it is less than the threshhold*/
bool GncCtlAutocode::BelowThreshold(float velocity[], float threshhold) {
  float filter_out;
  float sum = 0;
  for (int i = 0; i < 3; i++) {
    filter_out = GncCtlAutocode::ButterWorthFilter(velocity[i], prev_filter[i]);
    prev_filter[i] = filter_out;           // update prev_filter after it is used for the calculation
    filter_out = filter_out * filter_out;  // square the value
    sum = sum + filter_out;                // sum all of the values
  }

  if (sum < threshhold) {
    return true;
  }
  return false;
}

/*Determines if make conditions is met: if mode_cmd equals ctl_stopping_mode for 4 times in a row*/
bool GncCtlAutocode::CmdModeMakeCondition() {
  // shift exisitng elements to the right
  prev_mode_cmd[3] = prev_mode_cmd[2];
  prev_mode_cmd[2] = prev_mode_cmd[1];
  prev_mode_cmd[1] = prev_mode_cmd[0];
  // update index 0 with new value
  prev_mode_cmd[0] = mode_cmd;

  // check if they all equal ctl_stopping_mode
  if ((prev_mode_cmd[3] == constants::ctl_stopping_mode) && (prev_mode_cmd[3] == prev_mode_cmd[2]) &&
      (prev_mode_cmd[2] == prev_mode_cmd[1]) && (prev_mode_cmd[1] == prev_mode_cmd[0])) {
    return true;
  }
  return false;
}

/* triggers IDLE if est_confidence is 0; idles if diverged */
void GncCtlAutocode::UpdateModeCmd() {
  if (ctl_input_.est_confidence != constants::ase_status_converged) {
    mode_cmd = constants::ctl_idle_mode;
  } else {
    mode_cmd = cmd_.cmd_mode;
  }
}

}  // end namespace gnc_autocode
