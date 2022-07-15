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
  prev_att[3] = 0;
  linear_integrator[0] = 0;
  linear_integrator[1] = 0;
  linear_integrator[2] = 0;
  rotational_integrator[0] = 0;
  rotational_integrator[1] = 0;
  rotational_integrator[2] = 0;
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

/*****cex_control_executive*****/
  UpdateModeCmd();
  UpdateStoppedMode();
  UpdatePosAndQuat();
  FindPosError();
  FindQuatError(ctl_input_.est_quat_ISS2B, prev_att, quat_err, dummy);
  UpdatePrevious();  // this might need to go later
  UpdateCtlStatus();
  BypassShaper();

/*****clc_closed_loop_controller*****/
  VariablesTransfer();
  // Linear Control

  UpdateLinearPIDVals();
  FindPosErr();
  FindLinearIntErr();  // I'll want to check this
  FindBodyForceCmd();
  FindBodyAccelCmd();

  // Rotational Control
  UpdateRotationalPIDVals();
  FindQuatError(CMD_Quat_ISS2B, ctl_input_.est_quat_ISS2B, att_err_mag, att_err);  // Finds att_err_mag and att_err
  UpdateRotateIntErr();
}








/*****clc_closed_loop_controller functions*****/
void GncCtlAutocode::FindBodyAlphaCmd()
{
  float rate_error[3];
  AngAccelHelper(rate_error);
  //make 1d array into matrix like expecting
  float i_matrix[3][3] = { ctl_input_.inertia_matrix[0],  ctl_input_.inertia_matrix[1], ctl_input_.inertia_matrix[2],
                    ctl_input_.inertia_matrix[3], ctl_input_.inertia_matrix[4], ctl_input_.inertia_matrix[5],
                    ctl_input_.inertia_matrix[6], ctl_input_.inertia_matrix[7], ctl_input_.inertia_matrix[8]};
  MatrixMultiplication3x1(i_matrix, rate_error, body_alpha_cmd);

}
void GncCtlAutocode::AngAccelHelper(float rate_error[3])
{
  if (ctl_status <=1)
  {
    for (int i = 0; i < 3; i++)
    {
      rate_error[i] = -ctl_input_.est_omega_B_ISS_B[i];
    }
  }
  else{
    for(int i = 0; i < 3; i++)
    {
      rate_error[i] = CMD_Omega_B_ISS_B[i] + rotate_int_err[i] + (Kp_rot[i]*att_err[i]) - ctl_input_.est_omega_B_ISS_B[i];
    }
  }


  for (int i = 0; i< 3; i++)
  {
    rate_error[i] *= Kd_rot[i];
  }
}


void GncCtlAutocode::UpdateRotateIntErr()
{
  float input[3] = {att_err[0] * Ki_rot[0], att_err[1] * Ki_rot[1], att_err[2] * Ki_rot[2]};
  discreteTimeIntegrator(input, rotate_int_err, rotational_integrator, constants::tun_ctl_att_sat_upper,
                                            constants::tun_ctl_att_sat_lower);
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

void GncCtlAutocode::FindBodyAccelCmd() {
  for (int i = 0; i < 3; i++) {
    body_accel_cmd[i] = body_force_cmd[i] / ctl_input_.mass;
  }
}

void GncCtlAutocode::FindBodyForceCmd() {
  if (ctl_status != 0) {
    float vec[3];
    if (ctl_status <= 1) {
      for (int i = 0; i < 3; i++) {
        vec[i] = 0 - ctl_input_.est_V_B_ISS_ISS[i];
      }
    } else {
      // find desired velocity from position error
      float des_vel[3];
      for (int i = 0; i < 3; i++) {
        des_vel[i] = (Kp_lin[i] * pos_err_outport[i]) + linear_int_err[i];
        vec[i] = CMD_V_B_ISS_ISS[i] + des_vel[i] - ctl_input_.est_V_B_ISS_ISS[i];
      }
    }

    float velo_err_tmp[3];
    float feed_accel_tmp[3];
    RotateVectorAtoB(vec, ctl_input_.est_quat_ISS2B, velo_err_tmp);

    float a_b_gain[3] = {CMD_A_B_ISS_ISS[0] * constants::tun_accel_gain[0],
                         CMD_A_B_ISS_ISS[1] * constants::tun_accel_gain[1],
                         CMD_A_B_ISS_ISS[2] * constants::tun_accel_gain[2]};
    float accel_err_tmp[3];
    RotateVectorAtoB(a_b_gain, ctl_input_.est_quat_ISS2B, accel_err_tmp);

    float input_u[3] = { 0, 0, 0};
    for (int i = 0; i < 3; i++) {
      velo_err_tmp[i] *= Kd_lin[i];
      accel_err_tmp[i] *= ctl_input_.mass;
      input_u[i] = velo_err_tmp[i] + accel_err_tmp[i];
    }

    float saturated_output_u[3];
    SaturateVector(input_u, constants::tun_ctl_linear_force_limit, saturated_output_u);
    for (int i = 0; i < 3; i++) {
      body_force_cmd[i] = saturated_output_u[i];
    }
  } else {
    for (int i = 0; i < 3; i++) {
      body_force_cmd[i] = 0;
    }
  }
}

void GncCtlAutocode::SaturateVector(const float u[3], float limit, float output[3]) {
  // find vector magnitude
  float mag = sqrt(pow(u[0], 2) + pow(u[1], 2) + pow(u[2], 2));

  if (mag < limit) {
    for (int i = 0; i < 3; i++) {
      output[i] = u[i];
    }
  } else {
    for (int i = 0; i < 3; i++) {
      output[i] = u[i] / mag * limit;
    }
  }
}

void GncCtlAutocode::RotateVectorAtoB(float v[3], float q[4], float output[3]) {
  float tmp_output[3][3];
  QuaternionToDCM(q, tmp_output);

  // 3x3 multiply by 1x3
  MatrixMultiplication3x1(tmp_output, v, output);
  
}

 // 3x3 multiply by 1x3
void GncCtlAutocode::MatrixMultiplication3x1(float three[3][3], float one[3], float output[3])
{
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

void GncCtlAutocode::QuaternionToDCM(float input_quat[4], float output[3][3]) {
  float quat_w = input_quat[3];
  float U = ((quat_w * quat_w) * 2) - 1;  // U in Simulink diagram
  float scalar_matrix[3][3];

  // set scalar_matrix to all 0's
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      scalar_matrix[i][j] = 0;
    }
  }
  scalar_matrix[0][0] = U;
  scalar_matrix[1][1] = U;
  scalar_matrix[2][2] = U;

  float subtract_matrix[3][3];
  float quat_vals[3] = {input_quat[0], input_quat[1], input_quat[2]};
  SkewSymetricMatrix(quat_vals, subtract_matrix);
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      subtract_matrix[i][j] = subtract_matrix[i][j] * 2 * quat_w;
    }
  }
  /******Probable source of error*****/
  float vals_matrix[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      vals_matrix[i][j] = input_quat[i];
    }
  }

  // find transpose of vals_matrix
  float transpose[3][3];
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      transpose[j][i] = vals_matrix[i][j];
    }
  }

  float vals_output[3][3];
  MatrixMultiplication3x3(vals_matrix, transpose, vals_output);
  // element wise gain of 2 and add/sub
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      vals_output[i][j] *= 2;
      output[i][j] = scalar_matrix[i][j] - subtract_matrix[i][j] + vals_output[i][j];
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
void GncCtlAutocode::SkewSymetricMatrix(const float input[3], float output[3][3]) {
  // from simulink diagram
  output[0][0] = 0;
  output[1][0] = input[2];
  output[2][0] = -input[1];
  output[0][1] = -input[2];
  output[1][1] = 0;
  output[2][1] = input[0];
  output[0][2] =input[1];
  output[1][2] = -input[0];
  output[2][2] = 0;
}

void GncCtlAutocode::FindLinearIntErr() {
  float input[3];
  for (int i = 0; i < 3; i++) {
    input[i] = Ki_lin[i] * pos_err_outport[i];
  }
  float output[3];
  discreteTimeIntegrator(input, output, linear_integrator, constants::tun_ctl_pos_sat_upper,
                         constants::tun_ctl_pos_sat_lower);
  for (int i = 0; i < 3; i++) {
    linear_int_err[i] = output[i];
  }
}

void GncCtlAutocode::discreteTimeIntegrator(float input[3], float output[3], float accumulator[3], float upper_limit,
                                            float lower_limit) {
  if (ctl_status <= 1) {
    for (int i = 0; i < 3; i++) {
      output[i] = 0;
    }
    return;
  }
  for (int i = 0; i < 3; i++) {
    accumulator[i] += input[i];
    output[i] = accumulator[i];
    if (output[i] > upper_limit) {
      output[i] = upper_limit;
    } else if (output[i] < lower_limit) {
      output[i] = lower_limit;
    }
  }
}

// shouldn't be needed but keeps naming consistant with simulink
void GncCtlAutocode::VariablesTransfer() {
  for (int i = 0; i < 3; i++) {
    CMD_P_B_ISS_ISS[i] = position_command[i];
    CMD_V_B_ISS_ISS[i] = velocity_command[i];
    CMD_A_B_ISS_ISS[i] = accel_command[i];
    CMD_Quat_ISS2B[i] = att_command[i];
    CMD_Omega_B_ISS_B[i] = omega_command[i];
    CMD_Alpha_B_ISS_B[i] = alpha_command[i];
  }
  CMD_Quat_ISS2B[3] = att_command[3];  // since quat has size 4
}

void GncCtlAutocode::FindPosErr() {
  for (int i = 0; i < 3; i++) {
    pos_err_outport[i] = CMD_P_B_ISS_ISS[i] - ctl_input_.est_P_B_ISS_ISS[i];
  }
}

void GncCtlAutocode::UpdateLinearPIDVals() {
  for (int i = 0; i < 3; i++) {
    Kp_lin[i] = SafeDivide(ctl_input_.pos_kp[i], ctl_input_.vel_kd[i]);
    Ki_lin[i] = SafeDivide(ctl_input_.pos_ki[i], ctl_input_.vel_kd[i]);
    Kd_lin[i] = ctl_input_.vel_kd[i] * ctl_input_.mass;
  }
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
    for (int i = 0; i < 3; i++) {
      velocity_command[i] = 0;
      omega_command[i] = 0;
      accel_command[i] = 0;
      alpha_command[i] = 0;
    }
  } else {
    for (int i = 0; i < 3; i++) {
      velocity_command[i] = cmd_.traj_vel[i];
      omega_command[i] = cmd_.traj_omega[i];
      accel_command[i] = cmd_.traj_accel[i];
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

  output_vec[0] = out.x();
  output_vec[1] = out.y();
  output_vec[2] = out.z();

  output_scalar = acos(out.w()) * 2;
}

// updates the position and attitude command
void GncCtlAutocode::UpdatePosAndQuat() {
  if (stopped_mode) {
    for (int i = 0; i < 3; i++) {
      position_command[i] = prev_position[i];
      att_command[i] = prev_att[i];
    }
    att_command[3] = prev_att[3];
  } else {
    for (int i = 0; i < 3; i++) {
      position_command[i] = cmd_.traj_pos[i];
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


void GncCtlAutocode::Initialize(void) {
}




void GncCtlAutocode::ReadParams(config_reader::ConfigReader* config) {
}
}  // end namespace gnc_autocode
