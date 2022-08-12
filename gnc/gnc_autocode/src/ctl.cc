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




GncCtlAutocode::GncCtlAutocode(void) {
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
  // copy of what it is before
  ctl_input_msg before_ctl_input_;
  cmd_msg before_cmd_;
  ctl_msg before_ctl_;
  memcpy(&before_ctl_input_, &ctl_input_, sizeof(ctl_input_));
  memcpy(&before_cmd_, &cmd_, sizeof(cmd_));
  memcpy(&before_ctl_, &ctl_, sizeof(ctl_));
  // BeforeSimulink(before_ctl_input_, before_cmd_, before_ctl_);

  /****from Simulink Controller*****/
  ctl_controller0_step(controller_, &ctl_input_, &cmd_, &ctl_);


  // copy of what it is after Simulink controller
  ctl_input_msg after_ctl_input_;
  cmd_msg after_cmd_;
  ctl_msg after_ctl_;
  memcpy(&after_ctl_input_, &ctl_input_, sizeof(ctl_input_));
  memcpy(&after_cmd_, &cmd_, sizeof(cmd_));
  memcpy(&after_ctl_, &ctl_, sizeof(ctl_));

  // AfterSimulink(after_ctl_input_, after_cmd_, after_ctl_);

  memcpy(&ctl_input_, &before_ctl_input_, sizeof(before_ctl_input_));
  memcpy(&cmd_, &before_cmd_, sizeof(before_cmd_));
  memcpy(&ctl_, &before_ctl_, sizeof(before_ctl_));

  // RevertBackToBeforeSimulink(before_ctl_input_, before_cmd_, before_ctl_);


  /*****command_shaper*****/
  CmdSelector();
  GenerateCmdPath();
  GenerateCmdAttitude();
  FindTrajErrors();
  PublishCmdInput();

  /*****cex_control_executive*****/
  UpdateModeCmd();
  UpdateStoppedMode();
  FindPosError();
  FindQuatError(ctl_input_.est_quat_ISS2B, prev_att, quat_err, dummy);
  UpdatePosAndQuat();

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
  //FindQuatError(CMD_Quat_ISS2B, ctl_input_.est_quat_ISS2B, att_err_mag, att_err);  // Finds att_err_mag and att_err
  FindAttErr();
  UpdateRotateIntErr();
  FindBodyAlphaCmd();
  FindBodyTorqueCmd();

  UpdatePrevious();  // this might need to go later

  /*Publish to ctl_msg */
  VarToCtlMsg();

  // comparison tests

  //Test_Ctl_Status();

 TestTwoArrays(ctl_.pos_err, after_ctl_.pos_err, 3, 0.000002);


/* Test for pos_err */
  // float difference0 = ctl_.pos_err[0] - after_ctl_.pos_err[0];
  // float perc_difference0 = difference0 / after_ctl_.pos_err[0];
  //   if (fabs(difference0) > 0.000001f){
  //   ROS_ERROR("************ERROR************\n ******************************************\n");
  // }
  // ROS_ERROR("idx:0 New: %f, Old: %f, Difference: %f", ctl_.pos_err[0], after_ctl_.pos_err[0], difference0);

  // float difference1 = ctl_.pos_err[1] - after_ctl_.pos_err[1];
  // float perc_difference1 = difference1 / after_ctl_.pos_err[1];
  //   if (fabs(difference1) > 0.000001f){
  //   ROS_ERROR("************ERROR************\n ******************************************\n");
  // }
  // ROS_ERROR("idx:1 New: %f, Old: %f, Difference: %f", ctl_.pos_err[1], after_ctl_.pos_err[1], difference1);

  // float difference2 = ctl_.pos_err[2] - after_ctl_.pos_err[2];
  // float perc_difference2 = difference2 / after_ctl_.pos_err[2];
  //   if (fabs(difference2) > 0.000001f){
  //   ROS_ERROR("************ERROR************\n ******************************************\n");
  // }
  // ROS_ERROR("idx:2 New: %f, Old: %f, Difference: %f", ctl_.pos_err[2], after_ctl_.pos_err[2], difference2);

  /*Test for Linear/Pos Int Err*/
  // ROS_ERROR("ctl_status: %d",  ctl_.ctl_status);
  // double difference0 = ctl_.pos_err_int[0] - after_ctl_.pos_err_int[0];
  //   if (fabs(difference0) > 0.000001f){
  //   ROS_ERROR("************ERROR************\n ******************************************\n");
  // }
  // ROS_ERROR("idx:0 New: %f, Old: %f, Difference: %f", ctl_.pos_err_int[0], after_ctl_.pos_err_int[0], difference0);

  // double difference1 = ctl_.pos_err_int[1] - after_ctl_.pos_err_int[1];
  //   if (fabs(difference1) > 0.000001f){
  //   ROS_ERROR("************ERROR************\n ******************************************\n");
  // }
  // ROS_ERROR("idx:1 New: %f, Old: %f, Difference: %f", ctl_.pos_err_int[1], after_ctl_.pos_err_int[1], difference1);

  // double difference2 = ctl_.pos_err_int[2] - after_ctl_.pos_err_int[2];
  //   if (fabs(difference2) > 0.000001f){
  //   ROS_ERROR("************ERROR************\n ******************************************\n");
  // }
  // ROS_ERROR("idx:2 New: %f, Old: %f, Difference: %f", ctl_.pos_err_int[2], after_ctl_.pos_err_int[2], difference2);

  /*Test for Body Force CMD*/
  // double difference0 = ctl_.body_force_cmd[0] - after_ctl_.body_force_cmd[0];
  //     if (fabs(difference0) > 0.000020f){
  //     ROS_ERROR("************ERROR************\n ******************************************\n");
  //   }
  //   ROS_ERROR("idx:0 New: %f, Old: %f, Difference: %f", ctl_.body_force_cmd[0], after_ctl_.body_force_cmd[0],
  //   difference0);

  //   double difference1 = ctl_.body_force_cmd[1] - after_ctl_.body_force_cmd[1];
  //     if (fabs(difference1) > 0.000020f){
  //     ROS_ERROR("************ERROR************\n ******************************************\n");
  //   }
  //   ROS_ERROR("idx:1 New: %f, Old: %f, Difference: %f", ctl_.body_force_cmd[1], after_ctl_.body_force_cmd[1],
  //   difference1);

  //   double difference2 = ctl_.body_force_cmd[2] - after_ctl_.body_force_cmd[2];
  //     if (fabs(difference2) > 0.000020f){
  //     ROS_ERROR("************ERROR************\n ******************************************\n");
  //   }
  //   ROS_ERROR("idx:2 New: %f, Old: %f, Difference: %f", ctl_.body_force_cmd[2], after_ctl_.body_force_cmd[2],
  //   difference2);

/* Test for body accel cmd */
// double difference0 = ctl_.body_accel_cmd[0] - after_ctl_.body_accel_cmd[0];
//       if (fabs(difference0) > 0.000020f){
//       ROS_ERROR("************ERROR************\n ******************************************\n");
//     }
//     ROS_ERROR("idx:0 New: %f, Old: %f, Difference: %f", ctl_.body_accel_cmd[0], after_ctl_.body_accel_cmd[0],
//     difference0);

//     double difference1 = ctl_.body_accel_cmd[1] - after_ctl_.body_accel_cmd[1];
//       if (fabs(difference1) > 0.000020f){
//       ROS_ERROR("************ERROR************\n ******************************************\n");
//     }
//     ROS_ERROR("idx:1 New: %f, Old: %f, Difference: %f", ctl_.body_accel_cmd[1], after_ctl_.body_accel_cmd[1],
//     difference1);

//     double difference2 = ctl_.body_accel_cmd[2] - after_ctl_.body_accel_cmd[2];
//       if (fabs(difference2) > 0.000020f){
//       ROS_ERROR("************ERROR************\n ******************************************\n");
//     }
//     ROS_ERROR("idx:2 New: %f, Old: %f, Difference: %f", ctl_.body_accel_cmd[2], after_ctl_.body_accel_cmd[2],
//     difference2);

// /* Test for body torque cmd */
// double difference0 = ctl_.body_torque_cmd[0] - after_ctl_.body_torque_cmd[0];
//       if (fabs(difference0) > 0.000020f){
//       ROS_ERROR("************ERROR************\n ******************************************\n");
//     }
//     ROS_ERROR("idx:0 New: %f, Old: %f, Difference: %f", ctl_.body_torque_cmd[0], after_ctl_.body_torque_cmd[0],
//     difference0);

//     double difference1 = ctl_.body_torque_cmd[1] - after_ctl_.body_torque_cmd[1];
//       if (fabs(difference1) > 0.000020f){
//       ROS_ERROR("************ERROR************\n ******************************************\n");
//     }
//     ROS_ERROR("idx:1 New: %f, Old: %f, Difference: %f", ctl_.body_torque_cmd[1], after_ctl_.body_torque_cmd[1],
//     difference1);

//     double difference2 = ctl_.body_torque_cmd[2] - after_ctl_.body_torque_cmd[2];
//       if (fabs(difference2) > 0.000020f){
//       ROS_ERROR("************ERROR************\n ******************************************\n");
//     }
//     ROS_ERROR("idx:2 New: %f, Old: %f, Difference: %f", ctl_.body_torque_cmd[2], after_ctl_.body_torque_cmd[2],
//     difference2);

/* Test for body alpha cmd */
// double difference0 = ctl_.body_alpha_cmd[0] - after_ctl_.body_alpha_cmd[0];
// if (fabs(difference0) > 0.000020f) {
//   ROS_ERROR("************ERROR************\n ******************************************\n");
// }
//     ROS_ERROR("idx:0 New: %f, Old: %f, Difference: %f", ctl_.body_alpha_cmd[0], after_ctl_.body_alpha_cmd[0],
//     difference0);

//     double difference1 = ctl_.body_alpha_cmd[1] - after_ctl_.body_alpha_cmd[1];
//     if (fabs(difference1) > 0.000020f) {
//       ROS_ERROR("************ERROR************\n ******************************************\n");
//     }
//     ROS_ERROR("idx:1 New: %f, Old: %f, Difference: %f", ctl_.body_alpha_cmd[1], after_ctl_.body_alpha_cmd[1],
//     difference1);

//     double difference2 = ctl_.body_alpha_cmd[2] - after_ctl_.body_alpha_cmd[2];
//     if (fabs(difference2) > 0.000020f) {
//       ROS_ERROR("************ERROR************\n ******************************************\n");
//     }
//     ROS_ERROR("idx:2 New: %f, Old: %f, Difference: %f", ctl_.body_alpha_cmd[2], after_ctl_.body_alpha_cmd[2],
//     difference2);

  /*Test for att_Err_mag*/
//    double difference0 = ctl_.att_err_mag - after_ctl_.att_err_mag;
// if (fabs(difference0) > 0.000020f) {
//   ROS_ERROR("************ERROR************\n ******************************************\n");
// }
//     ROS_ERROR(" New: %f, Old: %f, Difference: %f", ctl_.att_err_mag, after_ctl_.att_err_mag,
//     difference0);

  // std::string str = std::to_string(att_command[2]);
  //   const char *mine = str.c_str();
  // ROS_ERROR("Att_command New:%s ", mine);

  /***** Test for Body Accel Cmd *****/
  // double difference0 = ctl_.body_accel_cmd[0] - after_ctl_.body_accel_cmd[0];
  //   if (fabs(difference0) > 0.000001f){
  //   ROS_ERROR("************ERROR************\n ******************************************\n");
  // }
  // ROS_ERROR("idx:0 New: %f, Old: %f, Difference: %f", ctl_.body_accel_cmd[0], after_ctl_.body_accel_cmd[0],
  // difference0);

  // double difference1 = ctl_.body_accel_cmd[1] - after_ctl_.body_accel_cmd[1];
  //   if (fabs(difference1) > 0.000001f){
  //   ROS_ERROR("************ERROR************\n ******************************************\n");
  // }
  // ROS_ERROR("idx:1 New: %f, Old: %f, Difference: %f", ctl_.body_accel_cmd[1], after_ctl_.body_accel_cmd[1],
  // difference1);

  // double difference2 = ctl_.body_accel_cmd[2] - after_ctl_.body_accel_cmd[2];
  //   if (fabs(difference2) > 0.000001f){
  //   ROS_ERROR("************ERROR************\n ******************************************\n");
  // }
  // ROS_ERROR("idx:2 New: %f, Old: %f, Difference: %f", ctl_.body_accel_cmd[2], after_ctl_.body_accel_cmd[2],
  // difference2);

  /*****Traj Error Pos*****/
  // double pos_diff = ctl_.traj_error_pos - after_ctl_.traj_error_pos;
  //  if (fabs(pos_diff) > 0.000002f){
  //     ROS_ERROR("************ERROR************\n ******************************************\n");
  //   }
  //   ROS_ERROR("Traj_error_pos New: %f, Old: %f, Difference: %f",  ctl_.traj_error_pos, after_ctl_.traj_error_pos,
  //   pos_diff);

  /*****Traj Error Att*****/
  // double att_diff = ctl_.traj_error_att - after_ctl_.traj_error_att;
  // if (fabs(att_diff) > 0.000002f) {
  //   ROS_ERROR("************ERROR************\n ******************************************");
  // }
  //   ROS_ERROR("Traj_error_att New: %f, Old: %f, Difference: %f",  ctl_.traj_error_att, after_ctl_.traj_error_att,
  //   att_diff);
    // ROS_ERROR("my traj_qaut: %f %f %f %f", traj_quat[0], traj_quat[1], traj_quat[2], traj_quat[3]);
    // ROS_ERROR("ol traj_quat: %f %f %f %f", after_cmd_.traj_quat[0], after_cmd_.traj_quat[1], after_cmd_.traj_quat[2], after_cmd_.traj_quat[3]);

   

    /*****Traj Error Vel*****/
    // double vel_diff = ctl_.traj_error_vel - after_ctl_.traj_error_vel;
    // if (fabs(vel_diff) > 0.000002f) {
    //   ROS_ERROR("************ERROR************\n ******************************************\n");
    // }
    // ROS_ERROR("Traj_error_vel New: %f, Old: %f, Difference: %f", ctl_.traj_error_vel, after_ctl_.traj_error_vel,
    //           vel_diff);

    /*****Traj Error Omega*****/
    // double omega_diff = ctl_.traj_error_omega - after_ctl_.traj_error_omega;
    // if (fabs(omega_diff) > 0.000002f) {
    //   ROS_ERROR("************ERROR************\n ******************************************\n");
    // }
    // ROS_ERROR("Traj_error_omega New: %f, Old: %f, Difference: %f", ctl_.traj_error_omega,
    // after_ctl_.traj_error_omega,
    //           omega_diff);

    // revert back to Simulink after my controller
    // RevertBackToAfterSimulink(after_ctl_input_, after_cmd_, after_ctl_);
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

void GncCtlAutocode::FindTrajErrors() {
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

  // FindQuatError(traj_quat, ctl_input_.est_quat_ISS2B, traj_error_att, dummy);
  FindTrajErrAtt();
}

void GncCtlAutocode::FindTrajErrAtt()
{
  Eigen::Quaternion<float> cmd;
  cmd.w() = traj_quat[3];
  cmd.x() = traj_quat[0];
  cmd.y() = traj_quat[1];
  cmd.z() = traj_quat[2];

  Eigen::Quaternion<float> actual;
  actual.w() = ctl_input_.est_quat_ISS2B[3];
  actual.x() = ctl_input_.est_quat_ISS2B[0];
  actual.y() = ctl_input_.est_quat_ISS2B[1];
  actual.z() = ctl_input_.est_quat_ISS2B[2];

  Eigen::Quaternion<float> out = actual.inverse() * cmd;

  // enfore positive scalar
  if (out.w() < 0) {
    // out.coeffs() = -out.coeffs();  // coeffs is a vector (x,y,z,w)
    out.x() = -out.x();
    out.y() = -out.y();
    out.z() = -out.z();
    out.w() = -out.w();
  }

  double mag = sqrt(pow(out.x(), 2) + pow(out.y(), 2) + pow(out.z(), 2) + pow(out.w(), 2));
  double thresh = 1E-7;
  // if (mag > thresh) {
  //   out.normalize();
  // }

  float output_vec[3];
  output_vec[0] = out.x();
  output_vec[1] = out.y();
  output_vec[2] = out.z();

  traj_error_att = fabs(acos(out.w())) * 2;
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

  FindTrajQuat();
}

void GncCtlAutocode::FindTrajQuat() {
  float omega_omega[4][4];
  float omega_alpha[4][4];
  float quat_state_cmd[4];
  if (state_cmd_switch_out) {
    CreateOmegaMatrix(ctl_input_.cmd_state_a.omega_B_ISS_B, omega_omega);
    CreateOmegaMatrix(ctl_input_.cmd_state_a.alpha_B_ISS_B, omega_alpha);
    for (int i = 0; i < 4; i++) {
      quat_state_cmd[i] = ctl_input_.cmd_state_a.quat_ISS2B[i];
    }

  } else {
    CreateOmegaMatrix(ctl_input_.cmd_state_b.omega_B_ISS_B, omega_omega);
    CreateOmegaMatrix(ctl_input_.cmd_state_b.alpha_B_ISS_B, omega_alpha);
    for (int i = 0; i < 4; i++) {
      quat_state_cmd[i] = ctl_input_.cmd_state_b.quat_ISS2B[i];
    }
  }
  // for (int row = 0; row < 4; row++) {
  //   for (int col = 0; col < 4; col++) {
  //     ROS_ERROR("omega omega, %d, %d: %f", row, col,  omega_omega[row][col]);
  //   }
  // }
  // ROS_ERROR("omegaB: %f", ctl_input_.cmd_state_b.omega_B_ISS_B[0]);

  // element wise multiplication and addition
  float average_omega_matrix[4][4];
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      average_omega_matrix[row][col] = (0.5 * omega_alpha[row][col] *time_delta) + omega_omega[row][col];
      average_omega_matrix[row][col] =  average_omega_matrix[row][col] * 0.5 * time_delta;
      // ROS_ERROR("omega matrix, %d, %d: %f", row, col, average_omega_matrix[row][col]);
    }
  }


  Eigen::Matrix<float, 4, 4> MatrixA;
  MatrixA << average_omega_matrix[0][0], average_omega_matrix[0][1], average_omega_matrix[0][2],
    average_omega_matrix[0][3], average_omega_matrix[1][0], average_omega_matrix[1][1], average_omega_matrix[1][2],
    average_omega_matrix[1][3], average_omega_matrix[2][0], average_omega_matrix[2][1], average_omega_matrix[2][2],
    average_omega_matrix[2][3], average_omega_matrix[3][0], average_omega_matrix[3][1], average_omega_matrix[3][2],
    average_omega_matrix[3][3];

  MatrixA = MatrixA.exp();

  // repopulate the 2d array
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      average_omega_matrix[row][col] = MatrixA(row, col);
    }
  }

  float bottom_sum_1[4][4];
  float bottom_sum_2[4][4];
  MatrixMultiplication4x4(omega_alpha, omega_omega, bottom_sum_1);
  MatrixMultiplication4x4(omega_omega, omega_alpha, bottom_sum_2);

  // subtract the 2 matrices and then multiply then add
  float bottom_sum_input[4][4];
  float sum_output[4][4];
  for (int row = 0; row < 4; row++) {
    for (int col = 0; col < 4; col++) {
      bottom_sum_input[row][col] = bottom_sum_1[row][col] - bottom_sum_2[row][col];
      bottom_sum_input[row][col] = bottom_sum_input[row][col] * (1/48) *time_delta * time_delta * time_delta;
      sum_output[row][col] = bottom_sum_input[row][col] + average_omega_matrix[row][col];
    }
  }

  float matrix_mult_out[4];
  MatrixMultiplication4x1(sum_output, quat_state_cmd, matrix_mult_out);

  Eigen::Quaternion<float> out;
  out.x() = matrix_mult_out[0];
  out.y() = matrix_mult_out[1];
  out.z() = matrix_mult_out[2];
  out.w() = matrix_mult_out[3];

  // enfore positive scalar
  if (out.w() < 0) {
    out.coeffs() = -out.coeffs();  // coeffs is a vector (x,y,z,w)
  }
  float mag = sqrt(pow(matrix_mult_out[0], 2) + pow(matrix_mult_out[1], 2) + pow(matrix_mult_out[2], 2) +
                   pow(matrix_mult_out[3], 2));
  if (mag < 0) {
    out.normalize();
  }

// for (int row = 0; row < 4; row++) {
//    ROS_ERROR("traj_quatt, %d,  %f", row, out.x());

//   }


  traj_quat[0] = out.x();
  traj_quat[1] = out.y();
  traj_quat[2] = out.z();
  traj_quat[3] = out.w();
}
// Defined in Indirect Kalman Filter for 3d attitude Estimation - Trawn, Roumeliotis eq 63
void GncCtlAutocode::CreateOmegaMatrix(float input[3], float output[4][4]) {
  output[0][0] = 0;
  output[1][0] = -input[2];
  output[2][0] = input[1];
  output[3][0] = -input[0];

  output[0][1] = input[2];
  output[1][1] = 0;
  output[2][1] = -input[0];
  output[3][1] = -input[1];

  output[0][2] = -input[1];
  output[1][2] =input[0];
  output[2][2] = 0;
  output[3][2] = -input[2];

  output[0][3] = input[0];
  output[1][3] = input[1];
  output[2][3] = input[2];
  output[3][3] = 0;
}

void GncCtlAutocode::GenerateCmdPath() {
  float test[3];
  if (state_cmd_switch_out) {  // true is A
    for (int i = 0; i < 3; i++) {
      traj_pos[i] = ctl_input_.cmd_state_a.P_B_ISS_ISS[i] + (ctl_input_.cmd_state_a.V_B_ISS_ISS[i] * time_delta) +
                    (0.5 * ctl_input_.cmd_state_a.A_B_ISS_ISS[i] * time_delta * time_delta);
      traj_vel[i] = ctl_input_.cmd_state_a.V_B_ISS_ISS[i] + (ctl_input_.cmd_state_a.A_B_ISS_ISS[i] * time_delta);
      traj_accel[i] = ctl_input_.cmd_state_b.A_B_ISS_ISS[i];
    }
  } else {  // false so cmd B
      for (int i = 0; i < 3; i++) {
        traj_pos[i] = ctl_input_.cmd_state_b.P_B_ISS_ISS[i] + (ctl_input_.cmd_state_b.V_B_ISS_ISS[i] * time_delta) +
                      (0.5 * ctl_input_.cmd_state_b.A_B_ISS_ISS[i] * time_delta * time_delta);

        traj_vel[i] = ctl_input_.cmd_state_b.V_B_ISS_ISS[i] + (ctl_input_.cmd_state_b.A_B_ISS_ISS[i] * time_delta);
        traj_accel[i] = ctl_input_.cmd_state_b.A_B_ISS_ISS[i];
    }
  }
  //   ROS_ERROR("I get out of CMD path");
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
void GncCtlAutocode::TestTwoArrays(const float new_array[], const float old_array[], int length, float tolerance)
{
  for(int i = 0; i < length; i++)
  {
    float difference = old_array[i] - new_array[i];
    float perc_difference = difference / old_array[i];
    if (fabs(difference) > tolerance){
      ROS_ERROR("************ERROR************\n ******************************************");
  }
    ROS_ERROR("Idx: %i New: %f, Old: %f Difference: %f", i, new_array[i], old_array[i], difference);
  
  }
}
void GncCtlAutocode::RevertBackToAfterSimulink(ctl_input_msg& after_ctl_input_, cmd_msg& after_cmd_,
                                               ctl_msg& after_ctl_) {
  for (int i = 0; i < 4; i++) {
      ctl_input_.est_quat_ISS2B[i] = after_ctl_input_.est_quat_ISS2B[i];
    }
    for (int i = 0; i < 3; i++) {
      ctl_input_.est_omega_B_ISS_B[i] = after_ctl_input_.est_omega_B_ISS_B[i];
      ctl_input_.est_V_B_ISS_ISS[i] = after_ctl_input_.est_V_B_ISS_ISS[i];
      ctl_input_.est_P_B_ISS_ISS[i] = after_ctl_input_.est_P_B_ISS_ISS[i];
      ctl_input_.att_kp[i] = after_ctl_input_.att_kp[i];
      ctl_input_.att_ki[i] = after_ctl_input_.att_ki[i];
      ctl_input_.omega_kd[i] = after_ctl_input_.omega_kd[i];
      ctl_input_.pos_kp[i] = after_ctl_input_.pos_kp[i];
      ctl_input_.pos_ki[i] = after_ctl_input_.pos_ki[i];
      ctl_input_.vel_kd[i] = after_ctl_input_.vel_kd[i];
    }
  ctl_input_.est_confidence = after_ctl_input_.est_confidence;
  // ctl_input_.cmd_state_a = after_ctl_input_.cmd_state_a;
  // ctl_input_.cmd_state_b = after_ctl_input_.cmd_state_b;
  ctl_input_.ctl_mode_cmd = after_ctl_input_.ctl_mode_cmd;
  ctl_input_.current_time_sec = after_ctl_input_.current_time_sec;
  ctl_input_.current_time_nsec = after_ctl_input_.current_time_nsec;

  ctl_input_.speed_gain_cmd = after_ctl_input_.speed_gain_cmd;
  ctl_input_. mass = after_ctl_input_. mass;

  // copy the cmd_state_ a and b for ctl_input
  ctl_input_.cmd_state_a.timestamp_sec =  after_ctl_input_.cmd_state_a.timestamp_sec;
  ctl_input_.cmd_state_b.timestamp_sec =  after_ctl_input_.cmd_state_b.timestamp_sec;


  ctl_input_.cmd_state_a.timestamp_nsec =  after_ctl_input_.cmd_state_a.timestamp_nsec;
  ctl_input_.cmd_state_b.timestamp_nsec =  after_ctl_input_.cmd_state_b.timestamp_nsec;

  for (int i = 0; i < 3; i++) {
    ctl_input_.cmd_state_a.P_B_ISS_ISS[i] =  after_ctl_input_.cmd_state_a.P_B_ISS_ISS[i];
    ctl_input_.cmd_state_b.P_B_ISS_ISS[i] =  after_ctl_input_.cmd_state_b.P_B_ISS_ISS[i];

    ctl_input_.cmd_state_a.V_B_ISS_ISS[i] =  after_ctl_input_.cmd_state_a.V_B_ISS_ISS[i];
    ctl_input_.cmd_state_b.V_B_ISS_ISS[i] =  after_ctl_input_.cmd_state_b.V_B_ISS_ISS[i];

    ctl_input_.cmd_state_a.A_B_ISS_ISS[i] =  after_ctl_input_.cmd_state_a.A_B_ISS_ISS[i];
    ctl_input_.cmd_state_b.A_B_ISS_ISS[i] =  after_ctl_input_.cmd_state_b.A_B_ISS_ISS[i];

    ctl_input_.cmd_state_a.omega_B_ISS_B[i] =  after_ctl_input_.cmd_state_a.omega_B_ISS_B[i];
    ctl_input_.cmd_state_b.omega_B_ISS_B[i] =  after_ctl_input_.cmd_state_b.omega_B_ISS_B[i];

    ctl_input_.cmd_state_a.alpha_B_ISS_B[i] =  after_ctl_input_.cmd_state_a.alpha_B_ISS_B[i];
    ctl_input_.cmd_state_b.alpha_B_ISS_B[i] =  after_ctl_input_.cmd_state_b.alpha_B_ISS_B[i];
  }

  for (int i = 0; i < 4; i++) {
    ctl_input_.cmd_state_a.quat_ISS2B[i] =  after_ctl_input_.cmd_state_a.quat_ISS2B[i];
    ctl_input_.cmd_state_b.quat_ISS2B[i] =  after_ctl_input_.cmd_state_b.quat_ISS2B[i];
  }

  for (int i = 0; i < 9; i++) {
    ctl_input_.inertia_matrix[i] = after_ctl_input_.inertia_matrix[i];
  }

  cmd_.cmd_timestamp_sec = after_cmd_.cmd_timestamp_sec;
  cmd_.cmd_timestamp_nsec = after_cmd_.cmd_timestamp_nsec;
  cmd_.cmd_mode = after_cmd_.cmd_mode;
  cmd_.speed_gain_cmd = after_cmd_.speed_gain_cmd;
  cmd_.cmd_B_inuse = after_cmd_.cmd_B_inuse;

  for (int i = 0; i < 3; i++) {
    cmd_.traj_pos[i] = after_cmd_.traj_pos[i];
    cmd_.traj_vel[i] = after_cmd_.traj_vel[i];
    cmd_.traj_accel[i] = after_cmd_.traj_accel[i];
    cmd_.traj_omega[i] = after_cmd_.traj_omega[i];
    cmd_.traj_alpha[i] = after_cmd_.traj_alpha[i];
  }

  for (int i = 0; i < 4; i++) {
    cmd_.traj_quat[i] = after_cmd_.traj_quat[i];
  }

  for (int i = 0; i < 3; i++) {
    ctl_.body_force_cmd[i] = after_ctl_.body_force_cmd[i];
    ctl_.body_accel_cmd[i] = after_ctl_.body_accel_cmd[i];
    ctl_.pos_err[i] = after_ctl_.pos_err[i];
    ctl_.pos_err_int[i] = after_ctl_.pos_err_int[i];
    ctl_.body_torque_cmd[i] = after_ctl_.body_torque_cmd[i];
    ctl_.body_alpha_cmd[i] = after_ctl_.body_alpha_cmd[i];
    ctl_.att_err[i] = after_ctl_.att_err[i];
    ctl_.att_err_int[i] = after_ctl_.att_err_int[i];
  }
  ctl_.att_err_mag = after_ctl_.att_err_mag;
  ctl_.ctl_status = after_ctl_.ctl_status;
  ctl_.traj_error_pos = after_ctl_.traj_error_pos;
  ctl_.traj_error_att = after_ctl_.traj_error_att;
  ctl_.traj_error_vel = after_ctl_.traj_error_vel;
  ctl_.traj_error_omega = after_ctl_.traj_error_omega;
}
void GncCtlAutocode::RevertBackToBeforeSimulink(ctl_input_msg& before_ctl_input_, cmd_msg& before_cmd_,
                                                ctl_msg& before_ctl_) {
  for (int i = 0; i < 4; i++) {
    ctl_input_.est_quat_ISS2B[i] = before_ctl_input_.est_quat_ISS2B[i];
  }
  for (int i = 0; i < 3; i++) {
    ctl_input_.est_omega_B_ISS_B[i] = before_ctl_input_.est_omega_B_ISS_B[i];
    ctl_input_.est_V_B_ISS_ISS[i] = before_ctl_input_.est_V_B_ISS_ISS[i];
    ctl_input_.est_P_B_ISS_ISS[i] = before_ctl_input_.est_P_B_ISS_ISS[i];
    ctl_input_.att_kp[i] = before_ctl_input_.att_kp[i];
    ctl_input_.att_ki[i] = before_ctl_input_.att_ki[i];
    ctl_input_.omega_kd[i] = before_ctl_input_.omega_kd[i];
    ctl_input_.pos_kp[i] = before_ctl_input_.pos_kp[i];
    ctl_input_.pos_ki[i] = before_ctl_input_.pos_ki[i];
    ctl_input_.vel_kd[i] = before_ctl_input_.vel_kd[i];
  }
  ctl_input_.est_confidence = before_ctl_input_.est_confidence;
  // ctl_input_.cmd_state_a = before_ctl_input_.cmd_state_a;
  // ctl_input_.cmd_state_b = before_ctl_input_.cmd_state_b;
  ctl_input_.ctl_mode_cmd = before_ctl_input_.ctl_mode_cmd;
  ctl_input_.current_time_sec = before_ctl_input_.current_time_sec;
  ctl_input_.current_time_nsec = before_ctl_input_.current_time_nsec;

  ctl_input_.speed_gain_cmd = before_ctl_input_.speed_gain_cmd;
  ctl_input_. mass = before_ctl_input_. mass;

  // copy the cmd_state_ a and b for ctl_input
  ctl_input_.cmd_state_a.timestamp_sec = before_ctl_input_.cmd_state_a.timestamp_sec;
  ctl_input_.cmd_state_b.timestamp_sec = before_ctl_input_.cmd_state_b.timestamp_sec;


  ctl_input_.cmd_state_a.timestamp_nsec = before_ctl_input_.cmd_state_a.timestamp_nsec;
  ctl_input_.cmd_state_b.timestamp_nsec = before_ctl_input_.cmd_state_b.timestamp_nsec;

  for (int i = 0; i < 3; i++) {
    ctl_input_.cmd_state_a.P_B_ISS_ISS[i] = before_ctl_input_.cmd_state_a.P_B_ISS_ISS[i];
    ctl_input_.cmd_state_b.P_B_ISS_ISS[i] = before_ctl_input_.cmd_state_b.P_B_ISS_ISS[i];

    ctl_input_.cmd_state_a.V_B_ISS_ISS[i] = before_ctl_input_.cmd_state_a.V_B_ISS_ISS[i];
    ctl_input_.cmd_state_b.V_B_ISS_ISS[i] = before_ctl_input_.cmd_state_b.V_B_ISS_ISS[i];

    ctl_input_.cmd_state_a.A_B_ISS_ISS[i] = before_ctl_input_.cmd_state_a.A_B_ISS_ISS[i];
    ctl_input_.cmd_state_b.A_B_ISS_ISS[i] = before_ctl_input_.cmd_state_b.A_B_ISS_ISS[i];

    ctl_input_.cmd_state_a.omega_B_ISS_B[i] = before_ctl_input_.cmd_state_a.omega_B_ISS_B[i];
    ctl_input_.cmd_state_b.omega_B_ISS_B[i] = before_ctl_input_.cmd_state_b.omega_B_ISS_B[i];

    ctl_input_.cmd_state_a.alpha_B_ISS_B[i] = before_ctl_input_.cmd_state_a.alpha_B_ISS_B[i];
    ctl_input_.cmd_state_b.alpha_B_ISS_B[i] = before_ctl_input_.cmd_state_b.alpha_B_ISS_B[i];
  }

  for (int i = 0; i < 4; i++) {
    ctl_input_.cmd_state_a.quat_ISS2B[i] = before_ctl_input_.cmd_state_a.quat_ISS2B[i];
    ctl_input_.cmd_state_b.quat_ISS2B[i] = before_ctl_input_.cmd_state_b.quat_ISS2B[i];
  }

  for (int i = 0; i < 9; i++) {
    ctl_input_.inertia_matrix[i] = before_ctl_input_.inertia_matrix[i];
  }

  cmd_.cmd_timestamp_sec = before_cmd_.cmd_timestamp_sec;
  cmd_.cmd_timestamp_nsec = before_cmd_.cmd_timestamp_nsec;
  cmd_.cmd_mode = before_cmd_.cmd_mode;
  cmd_.speed_gain_cmd = before_cmd_.speed_gain_cmd;
  cmd_.cmd_B_inuse = before_cmd_.cmd_B_inuse;

  for (int i = 0; i < 3; i++) {
    cmd_.traj_pos[i] = before_cmd_.traj_pos[i];
    cmd_.traj_vel[i] = before_cmd_.traj_vel[i];
    cmd_.traj_accel[i] = before_cmd_.traj_accel[i];
    cmd_.traj_omega[i] = before_cmd_.traj_omega[i];
    cmd_.traj_alpha[i] = before_cmd_.traj_alpha[i];
  }

  for (int i = 0; i < 4; i++) {
    cmd_.traj_quat[i] = before_cmd_.traj_quat[i];
  }

  for (int i = 0; i < 3; i++) {
    ctl_.body_force_cmd[i] = before_ctl_.body_force_cmd[i];
    ctl_.body_accel_cmd[i] = before_ctl_.body_accel_cmd[i];
    ctl_.pos_err[i] = before_ctl_.pos_err[i];
    ctl_.pos_err_int[i] = before_ctl_.pos_err_int[i];
    ctl_.body_torque_cmd[i] = before_ctl_.body_torque_cmd[i];
    ctl_.body_alpha_cmd[i] = before_ctl_.body_alpha_cmd[i];
    ctl_.att_err[i] = before_ctl_.att_err[i];
    ctl_.att_err_int[i] = before_ctl_.att_err_int[i];
  }
  ctl_.att_err_mag = before_ctl_.att_err_mag;
  ctl_.ctl_status = before_ctl_.ctl_status;
  ctl_.traj_error_pos = before_ctl_.traj_error_pos;
  ctl_.traj_error_att = before_ctl_.traj_error_att;
  ctl_.traj_error_vel = before_ctl_.traj_error_vel;
  ctl_.traj_error_omega = before_ctl_.traj_error_omega;
}

void GncCtlAutocode::AfterSimulink(ctl_input_msg& after_ctl_input_, cmd_msg& after_cmd_, ctl_msg& after_ctl_) {
  for (int i = 0; i < 4; i++) {
    after_ctl_input_.est_quat_ISS2B[i] = ctl_input_.est_quat_ISS2B[i];
  }
  for (int i = 0; i < 3; i++) {
    after_ctl_input_.est_omega_B_ISS_B[i] = ctl_input_.est_omega_B_ISS_B[i];
    after_ctl_input_.est_V_B_ISS_ISS[i] = ctl_input_.est_V_B_ISS_ISS[i];
    after_ctl_input_.est_P_B_ISS_ISS[i] = ctl_input_.est_P_B_ISS_ISS[i];
    after_ctl_input_.att_kp[i] = ctl_input_.att_kp[i];
    after_ctl_input_.att_ki[i] = ctl_input_.att_ki[i];
    after_ctl_input_.omega_kd[i] = ctl_input_.omega_kd[i];
    after_ctl_input_.pos_kp[i] = ctl_input_.pos_kp[i];
    after_ctl_input_.pos_ki[i] = ctl_input_.pos_ki[i];
    after_ctl_input_.vel_kd[i] = ctl_input_.vel_kd[i];
  }
  after_ctl_input_.est_confidence = ctl_input_.est_confidence;
  // after_ctl_input_.cmd_state_a = ctl_input_.cmd_state_a;
  // after_ctl_input_.cmd_state_b = ctl_input_.cmd_state_b;
  after_ctl_input_.ctl_mode_cmd = ctl_input_.ctl_mode_cmd;
  after_ctl_input_.current_time_sec = ctl_input_.current_time_sec;
  after_ctl_input_.current_time_nsec = ctl_input_.current_time_nsec;

  after_ctl_input_.speed_gain_cmd = ctl_input_.speed_gain_cmd;
  after_ctl_input_. mass = ctl_input_. mass;

  // copy the cmd_state_ a and b for ctl_input
  after_ctl_input_.cmd_state_a.timestamp_sec = ctl_input_.cmd_state_a.timestamp_sec;
  after_ctl_input_.cmd_state_b.timestamp_sec = ctl_input_.cmd_state_b.timestamp_sec;


  after_ctl_input_.cmd_state_a.timestamp_nsec = ctl_input_.cmd_state_a.timestamp_nsec;
  after_ctl_input_.cmd_state_b.timestamp_nsec = ctl_input_.cmd_state_b.timestamp_nsec;

  for (int i = 0; i < 3; i++) {
    after_ctl_input_.cmd_state_a.P_B_ISS_ISS[i] = ctl_input_.cmd_state_a.P_B_ISS_ISS[i];
    after_ctl_input_.cmd_state_b.P_B_ISS_ISS[i] = ctl_input_.cmd_state_b.P_B_ISS_ISS[i];

    after_ctl_input_.cmd_state_a.V_B_ISS_ISS[i] = ctl_input_.cmd_state_a.V_B_ISS_ISS[i];
    after_ctl_input_.cmd_state_b.V_B_ISS_ISS[i] = ctl_input_.cmd_state_b.V_B_ISS_ISS[i];

    after_ctl_input_.cmd_state_a.A_B_ISS_ISS[i] = ctl_input_.cmd_state_a.A_B_ISS_ISS[i];
    after_ctl_input_.cmd_state_b.A_B_ISS_ISS[i] = ctl_input_.cmd_state_b.A_B_ISS_ISS[i];

    after_ctl_input_.cmd_state_a.omega_B_ISS_B[i] = ctl_input_.cmd_state_a.omega_B_ISS_B[i];
    after_ctl_input_.cmd_state_b.omega_B_ISS_B[i] = ctl_input_.cmd_state_b.omega_B_ISS_B[i];

    after_ctl_input_.cmd_state_a.alpha_B_ISS_B[i] = ctl_input_.cmd_state_a.alpha_B_ISS_B[i];
    after_ctl_input_.cmd_state_b.alpha_B_ISS_B[i] = ctl_input_.cmd_state_b.alpha_B_ISS_B[i];
  }

  for (int i = 0; i < 4; i++) {
    after_ctl_input_.cmd_state_a.quat_ISS2B[i] = ctl_input_.cmd_state_a.quat_ISS2B[i];
    after_ctl_input_.cmd_state_b.quat_ISS2B[i] = ctl_input_.cmd_state_b.quat_ISS2B[i];
  }
  for (int i = 0; i < 9; i++) {
    after_ctl_input_.inertia_matrix[i] = ctl_input_.inertia_matrix[i];
  }

  after_cmd_.cmd_timestamp_sec = cmd_.cmd_timestamp_sec;
  after_cmd_.cmd_timestamp_nsec = cmd_.cmd_timestamp_nsec;
  after_cmd_.cmd_mode = cmd_.cmd_mode;
  after_cmd_.speed_gain_cmd = cmd_.speed_gain_cmd;
  after_cmd_.cmd_B_inuse = cmd_.cmd_B_inuse;

  for (int i = 0; i < 3; i++) {
    after_cmd_.traj_pos[i] = cmd_.traj_pos[i];
    after_cmd_.traj_vel[i] = cmd_.traj_vel[i];
    after_cmd_.traj_accel[i] = cmd_.traj_accel[i];
    after_cmd_.traj_omega[i] = cmd_.traj_omega[i];
    after_cmd_.traj_alpha[i] = cmd_.traj_alpha[i];
  }

  for (int i = 0; i < 4; i++) {
    after_cmd_.traj_quat[i] = cmd_.traj_quat[i];
  }

  for (int i = 0; i < 3; i++) {
    after_ctl_.body_force_cmd[i] = ctl_.body_force_cmd[i];
    after_ctl_.body_accel_cmd[i] = ctl_.body_accel_cmd[i];
    after_ctl_.pos_err[i] = ctl_.pos_err[i];
    after_ctl_.pos_err_int[i] = ctl_.pos_err_int[i];
    after_ctl_.body_torque_cmd[i] = ctl_.body_torque_cmd[i];
    after_ctl_.body_alpha_cmd[i] = ctl_.body_alpha_cmd[i];
    after_ctl_.att_err[i] = ctl_.att_err[i];
    after_ctl_.att_err_int[i] = ctl_.att_err_int[i];
  }
  after_ctl_.att_err_mag = ctl_.att_err_mag;
  after_ctl_.ctl_status = ctl_.ctl_status;
  after_ctl_.traj_error_pos = ctl_.traj_error_pos;
  after_ctl_.traj_error_att = ctl_.traj_error_att;
  after_ctl_.traj_error_vel = ctl_.traj_error_vel;
  after_ctl_.traj_error_omega = ctl_.traj_error_omega;
}
void GncCtlAutocode::BeforeSimulink(ctl_input_msg& before_ctl_input_, cmd_msg& before_cmd_, ctl_msg& before_ctl_) {
  for (int i = 0; i < 4; i++) {
    before_ctl_input_.est_quat_ISS2B[i] = ctl_input_.est_quat_ISS2B[i];
  }
  for (int i = 0; i < 3; i++) {
    before_ctl_input_.est_omega_B_ISS_B[i] = ctl_input_.est_omega_B_ISS_B[i];
    before_ctl_input_.est_V_B_ISS_ISS[i] = ctl_input_.est_V_B_ISS_ISS[i];
    before_ctl_input_.est_P_B_ISS_ISS[i] = ctl_input_.est_P_B_ISS_ISS[i];
    before_ctl_input_.att_kp[i] = ctl_input_.att_kp[i];
    before_ctl_input_.att_ki[i] = ctl_input_.att_ki[i];
    before_ctl_input_.omega_kd[i] = ctl_input_.omega_kd[i];
    before_ctl_input_.pos_kp[i] = ctl_input_.pos_kp[i];
    before_ctl_input_.pos_ki[i] = ctl_input_.pos_ki[i];
    before_ctl_input_.vel_kd[i] = ctl_input_.vel_kd[i];
  }
  before_ctl_input_.est_confidence = ctl_input_.est_confidence;

  before_ctl_input_.ctl_mode_cmd = ctl_input_.ctl_mode_cmd;
  before_ctl_input_.current_time_sec = ctl_input_.current_time_sec;
  before_ctl_input_.current_time_nsec = ctl_input_.current_time_nsec;

  before_ctl_input_.speed_gain_cmd = ctl_input_.speed_gain_cmd;
  before_ctl_input_. mass = ctl_input_. mass;

  // copy the cmd_state_ a and b for ctl_input
  before_ctl_input_.cmd_state_a.timestamp_sec = ctl_input_.cmd_state_a.timestamp_sec;
  before_ctl_input_.cmd_state_b.timestamp_sec = ctl_input_.cmd_state_b.timestamp_sec;


  before_ctl_input_.cmd_state_a.timestamp_nsec = ctl_input_.cmd_state_a.timestamp_nsec;
  before_ctl_input_.cmd_state_b.timestamp_nsec = ctl_input_.cmd_state_b.timestamp_nsec;

  for (int i = 0; i < 3; i++) {
    before_ctl_input_.cmd_state_a.P_B_ISS_ISS[i] = ctl_input_.cmd_state_a.P_B_ISS_ISS[i];
    before_ctl_input_.cmd_state_b.P_B_ISS_ISS[i] = ctl_input_.cmd_state_b.P_B_ISS_ISS[i];

    before_ctl_input_.cmd_state_a.V_B_ISS_ISS[i] = ctl_input_.cmd_state_a.V_B_ISS_ISS[i];
    before_ctl_input_.cmd_state_b.V_B_ISS_ISS[i] = ctl_input_.cmd_state_b.V_B_ISS_ISS[i];

    before_ctl_input_.cmd_state_a.A_B_ISS_ISS[i] = ctl_input_.cmd_state_a.A_B_ISS_ISS[i];
    before_ctl_input_.cmd_state_b.A_B_ISS_ISS[i] = ctl_input_.cmd_state_b.A_B_ISS_ISS[i];

    before_ctl_input_.cmd_state_a.omega_B_ISS_B[i] = ctl_input_.cmd_state_a.omega_B_ISS_B[i];
    before_ctl_input_.cmd_state_b.omega_B_ISS_B[i] = ctl_input_.cmd_state_b.omega_B_ISS_B[i];

    before_ctl_input_.cmd_state_a.alpha_B_ISS_B[i] = ctl_input_.cmd_state_a.alpha_B_ISS_B[i];
    before_ctl_input_.cmd_state_b.alpha_B_ISS_B[i] = ctl_input_.cmd_state_b.alpha_B_ISS_B[i];
  }

  for (int i = 0; i < 4; i++) {
    before_ctl_input_.cmd_state_a.quat_ISS2B[i] = ctl_input_.cmd_state_a.quat_ISS2B[i];
    before_ctl_input_.cmd_state_b.quat_ISS2B[i] = ctl_input_.cmd_state_b.quat_ISS2B[i];
  }

  for (int i = 0; i < 9; i++) {
    before_ctl_input_.inertia_matrix[i] = ctl_input_.inertia_matrix[i];
  }

  before_cmd_.cmd_timestamp_sec = cmd_.cmd_timestamp_sec;
  before_cmd_.cmd_timestamp_nsec = cmd_.cmd_timestamp_nsec;
  before_cmd_.cmd_mode = cmd_.cmd_mode;
  before_cmd_.speed_gain_cmd = cmd_.speed_gain_cmd;
  before_cmd_.cmd_B_inuse = cmd_.cmd_B_inuse;

  for (int i = 0; i < 3; i++) {
    before_cmd_.traj_pos[i] = cmd_.traj_pos[i];
    before_cmd_.traj_vel[i] = cmd_.traj_vel[i];
    before_cmd_.traj_accel[i] = cmd_.traj_accel[i];
    before_cmd_.traj_omega[i] = cmd_.traj_omega[i];
    before_cmd_.traj_alpha[i] = cmd_.traj_alpha[i];
  }

  for (int i = 0; i < 4; i++) {
    before_cmd_.traj_quat[i] = cmd_.traj_quat[i];
  }

  for (int i = 0; i < 3; i++) {
    before_ctl_.body_force_cmd[i] = ctl_.body_force_cmd[i];
    before_ctl_.body_accel_cmd[i] = ctl_.body_accel_cmd[i];
    before_ctl_.pos_err[i] = ctl_.pos_err[i];
    before_ctl_.pos_err_int[i] = ctl_.pos_err_int[i];
    before_ctl_.body_torque_cmd[i] = ctl_.body_torque_cmd[i];
    before_ctl_.body_alpha_cmd[i] = ctl_.body_alpha_cmd[i];
    before_ctl_.att_err[i] = ctl_.att_err[i];
    before_ctl_.att_err_int[i] = ctl_.att_err_int[i];
  }
  before_ctl_.att_err_mag = ctl_.att_err_mag;
  before_ctl_.ctl_status = ctl_.ctl_status;
  before_ctl_.traj_error_pos = ctl_.traj_error_pos;
  before_ctl_.traj_error_att = ctl_.traj_error_att;
  before_ctl_.traj_error_vel = ctl_.traj_error_vel;
  before_ctl_.traj_error_omega = ctl_.traj_error_omega;
}
void GncCtlAutocode::VarToCtlMsg() {
  for (int i = 0; i < 3; i++) {
    ctl_.body_force_cmd[i] = body_force_cmd[i];
    ctl_.body_accel_cmd[i] = body_accel_cmd[i];
    ctl_.pos_err[i] = pos_err_outport[i];
    ctl_.pos_err_int[i] = linear_int_err[i];

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
void GncCtlAutocode::FindAttErr()
{
  float cmd[4];
  float actual[4];
  for(int i = 0; i < 4; i++)
  {
    cmd[i] = CMD_Quat_ISS2B[i];
    actual[i] = ctl_input_.est_quat_ISS2B[i];
  }

  Eigen::Quaternion<float> q_cmd;
  Eigen::Quaternion<float> q_actual;
//trying with orientation w, x, y, z
  q_cmd.w() = cmd[0];
  q_cmd.x() = cmd[1];
  q_cmd.y() = cmd[2];
  q_cmd.z() = cmd[3];

  q_actual.w() = actual[0];
  q_actual.x() = actual[1];
  q_actual.y() = actual[2];
  q_actual.z() = actual[3];

  Eigen::Quaternion<float> q_inverse_actual;
  q_inverse_actual = q_actual.inverse();

   Eigen::Quaternion<float> q_out;
   q_out = q_inverse_actual * q_cmd;

//*Note if this doesn't work, try to use the z() element since that is what the notation would be 
   if (q_out.w() < 0)
   {
    q_out.coeffs() = -q_out.coeffs(); 
   }

   float mag = sqrt((q_out.w() * q_out.w()) + (q_out.x() * q_out.x()) + (q_out.y() * q_out.y()) + (q_out.z() * q_out.z()));
   if (double(mag) > 1E-7)
   {
    q_out.normalize();
   }

   att_err_mag = 2 * acos(q_out.w());



  
}



void GncCtlAutocode::FindBodyTorqueCmd() {
  if (ctl_status != 0) {
    for (int i = 0; i < 3; i++) {
      body_torque_cmd[i] = 0;
    }
  } else {
    // feed forward accel
    float ang_accel_feed[3];
    MatrixMultiplication3x1(i_matrix, CMD_Alpha_B_ISS_B, ang_accel_feed);  // the gain is just 1.0

    // feed forward linearization
    float for_linearization[3];
    MatrixMultiplication3x1(i_matrix, ctl_input_.est_omega_B_ISS_B, for_linearization);
    float feed_linearization[3];
    CrossProduct(for_linearization, ctl_input_.est_omega_B_ISS_B, feed_linearization);

    for (int i = 0; i < 3; i++) {
      body_torque_cmd[i] = ang_accel_feed[i] + rate_error[i] - feed_linearization[i];
    }
  }
}

void GncCtlAutocode::CrossProduct(float vecA[3], float vecB[3], float vecOut[3]) {
  vecOut[0] = vecA[1] * vecB[2] - vecA[2] * vecB[1];
  vecOut[1] = -(vecA[0] * vecB[2] - vecA[2] * vecB[0]);
  vecOut[2] = vecA[0] * vecB[1] - vecA[1] * vecB[0];
}
void GncCtlAutocode::FindBodyAlphaCmd() {
  AngAccelHelper(rate_error);
  // make 1d array into matrix like expecting
  i_matrix[0][0] = ctl_input_.inertia_matrix[0];
  i_matrix[0][1] = ctl_input_.inertia_matrix[1];
  i_matrix[0][2] = ctl_input_.inertia_matrix[2];

  i_matrix[1][0] = ctl_input_.inertia_matrix[3];
  i_matrix[1][1] = ctl_input_.inertia_matrix[4];
  i_matrix[1][2] = ctl_input_.inertia_matrix[5];

  i_matrix[2][0] = ctl_input_.inertia_matrix[6];
  i_matrix[2][1] = ctl_input_.inertia_matrix[7];
  i_matrix[2][2] = ctl_input_.inertia_matrix[8];

  MatrixMultiplication3x1(i_matrix, rate_error, body_alpha_cmd);
}

void GncCtlAutocode::AngAccelHelper(float rate_error[3]) {
  if (ctl_status <= 1) {
    for (int i = 0; i < 3; i++) {
      rate_error[i] = -ctl_input_.est_omega_B_ISS_B[i];
    }
  } else {
    for (int i = 0; i < 3; i++) {
      rate_error[i] =
        CMD_Omega_B_ISS_B[i] + rotate_int_err[i] + (Kp_rot[i] * att_err[i]) - ctl_input_.est_omega_B_ISS_B[i];
    }
  }

  for (int i = 0; i < 3; i++) {
    rate_error[i] *= Kd_rot[i];
  }
}

void GncCtlAutocode::UpdateRotateIntErr() {
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
      accumulator[0] = 0;
    }
    return;
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
  std::string str1 = std::to_string(CMD_P_B_ISS_ISS[0]);
  const char *old = str1.c_str();

  std::string str2 = std::to_string(ctl_input_.est_P_B_ISS_ISS[0]);
  const char *old1 = str2.c_str();
    // ROS_ERROR("Mine: first:%s", old);



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
// float inverse_actual[4];

// for (int i = 0; i < 3; i++)  {
//   inverse_actual[i] = -q_actual[i];
// }
// inverse_actual[3] = q_actual[3];

// // quat multiplication
// Eigen::Quaternion<float> q1;
// q1.x() = q_actual[0];
// q1.y() = q_actual[1];
// q1.z() = q_actual[2];
// q1.w() = q_actual[3];

// Eigen::Quaternion<float> q2;
// q2.x() = q_cmd[0];
// q2.y() = q_cmd[1];
// q2.z() = q_cmd[2];
// q2.w() = q_cmd[3];

// Eigen::Quaternion<float> out;
// out = q1 * q2;

// float mult_out[4];
// mult_out[0] = out.x();
// mult_out[1] = out.y();
// mult_out[2] = out.z();
// mult_out[3] = out.w();

// if (mult_out[3] < 0) {
//   for (int i = 0; i < 4; i++) {
//     mult_out[i] = -mult_out[i];
//   }
// }

// double mag = sqrt((mult_out[0] * mult_out[0]) + (mult_out[1] * mult_out[1]) + (mult_out[2] * mult_out[2]) +
//                   (mult_out[3] * mult_out[3]));
// if (mag > 1E-7) {
//   for (int i = 0; i < 4; i++) {
//     mult_out[i] = mult_out[i] / mag;
//   }
// }

// for (int i = 0; i < 3; i++) {
//   output_vec[i] = mult_out[i];
// }
// if (mult_out[3] > 1.0F) {
//   mult_out[3] = 1.0F;
// } else if (mult_out[3] < -1.0F) {
//   mult_out[3] = -1.0F;
// }
// output_scalar = fabs(acos(mult_out[3])) * 2;





/*Break to old */
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

Eigen::Quaternion<float> out = actual.inverse() * cmd;

// enfore positive scalar
if (out.w() < 0) {
  // out.coeffs() = -out.coeffs();  // coeffs is a vector (x,y,z,w)
  out.x() = -out.x();
  out.y() = -out.y();
  out.z() = -out.z();
  out.w() = -out.w();
}

double mag = sqrt(pow(out.x(), 2) + pow(out.y(), 2) + pow(out.z(), 2) + pow(out.w(), 2));
double thresh = 1E-7;
if (mag > thresh) {
  out.normalize();
}

output_vec[0] = out.x();
output_vec[1] = out.y();
output_vec[2] = out.z();

output_scalar = fabs(acos(out.w())) * 2;
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

  // need to run these outside of if condition to make sure that they are being ran every cycle
  bool vel_below_threshold = BelowThreshold(velocity, constants::tun_ctl_stopping_vel_thresh, prev_filter_vel);
  bool omega_below_threshold =  BelowThreshold(omega, constants::tun_ctl_stopping_omega_thresh, prev_filter_omega);
  bool cmd_make = CmdModeMakeCondition();
  if (vel_below_threshold && omega_below_threshold && cmd_make) {
    stopped_mode = true;
  } else {
    stopped_mode = false;
  }
  // ROS_ERROR("stopped mode: %i", stopped_mode);
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
