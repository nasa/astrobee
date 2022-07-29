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

#ifndef GNC_AUTOCODE_CTL_H_
#define GNC_AUTOCODE_CTL_H_

extern "C" {
#include <ctl_controller0.h>
}

namespace config_reader {
  class ConfigReader;
}

// constants from here:
// https://github.com/nasa/astrobee/blob/master/gnc/matlab/code_generation/ctl_controller0_ert_rtw/ctl_controller0_data.cpp
namespace constants {
const unsigned int ase_status_converged = 0U;
const unsigned int ctl_idle_mode = 0U;
const unsigned int ctl_stopping_mode = 1U;
const unsigned int ctl_stopped_mode = 3U;
const long double butterworth_gain_1 = 0.0031317642291927056;
const long double butterworth_gain_2 = -0.993736471541614597;
const auto tun_ctl_stopping_omega_thresh = 0.0004F;
const auto tun_ctl_stopping_vel_thresh = 0.0004F;
const auto tun_ctl_stopped_pos_thresh = 0.1F;
const auto tun_ctl_stopped_quat_thresh = 0.174533F;
const auto tun_ctl_pos_sat_upper = 0.1F;
const auto tun_ctl_pos_sat_lower = -0.1F;
const float tun_accel_gain[3] = {1.0F, 1.0F, 1.0F};
const float tun_ctl_linear_force_limit = 100.0F;
const float tun_ctl_att_sat_upper = 0.5F;
const float tun_ctl_att_sat_lower = -0.5F;
}  // namespace constants

namespace gnc_autocode {

class GncCtlAutocode {
 public:
  GncCtlAutocode(void);
  ~GncCtlAutocode(void);

  virtual void Initialize(void);
  virtual void Step(void);
  virtual void ReadParams(config_reader::ConfigReader* config);



  RT_MODEL_ctl_controller0_T* controller_;

  // see GN&C_ICD.xlsx in freeflyer_docs for documentation
  // inputs
  ctl_input_msg ctl_input_;

  // outputs
  cmd_msg cmd_;

  ctl_msg ctl_;

 private:
  int mode_cmd;
  bool stopped_mode;
  float prev_filter_vel[3];
  float prev_filter_omega[3];
  int prev_mode_cmd[5];  // for the 4 ticks required  to switch to stopped; newest val at index 0
  float prev_position[3];
  float prev_att[4];
  float pos_err_parameter[3];  // in cex control executive
  float quat_err;
  // Simulink outports
  // cex_control_executive
  float att_command[4];
  float position_command[3];
  float ctl_status;
  float velocity_command[3];
  float omega_command[3];
  float accel_command[3];
  float alpha_command[3];

  // clc_closed_loop controller
  float Kp_lin[3];
  float Ki_lin[3];
  float Kd_lin[3];
  float Kp_rot[3];
  float Ki_rot[3];
  float Kd_rot[3];
  float linear_int_err[3];
  float body_accel_cmd[3];
  float body_force_cmd[3];
  float pos_err_outport[3];  // in closed loop controller
  float CMD_P_B_ISS_ISS[3];
  float CMD_V_B_ISS_ISS[3];
  float CMD_A_B_ISS_ISS[3];
  float CMD_Quat_ISS2B[4];
  float CMD_Omega_B_ISS_B[3];
  float CMD_Alpha_B_ISS_B[3];
  float linear_integrator[3];
  float rotational_integrator[3];  // accumulator
  float linear_int_error[3];
  float att_err_mag;
  float att_err[3];
  float dummy[3];
  float rotate_int_err[3];
  float body_alpha_cmd[3];
  float i_matrix[3][3];
  float rate_error[3];  // helper in rot control
  float body_torque_cmd[3];

  bool BelowThreshold(float velocity[], float threshhold, float previous[3]);
  void UpdateModeCmd(void);
  float ButterWorthFilter(float input, float& delay_val);
  bool CmdModeMakeCondition();
  void UpdateStoppedMode();
  void UpdatePosAndQuat();
  void FindPosError();
  void UpdatePrevious();
  void FindQuatError(float q_cmd[4], float q_actual[4], float& output, float output_vec[3]);
  void UpdateCtlStatus();
  bool CtlStatusSwitch();
  void BypassShaper();
  // clc_closed_loop controller
  void VariablesTransfer();
  float SafeDivide(float num, float denom);
  void UpdateLinearPIDVals();
  void FindPosErr();
  void discreteTimeIntegrator(float input[3], float output[3], float accumulator[3], float upper_limit,
                              float lower_limit);
  void FindLinearIntErr();
  void FindBodyForceCmd();
  void SkewSymetricMatrix(const float input[3], float output[3][3]);
  void QuaternionToDCM(float input_quat[4], float output[3][3]);
  void RotateVectorAtoB(float v[3], float q[4], float output[3]);
  void MatrixMultiplication3x3(float inputA[3][3], float inputB[3][3], float output[3][3]);
  void SaturateVector(const float u[3], float limit, float output[3]);
  void FindBodyAccelCmd();
  void UpdateRotationalPIDVals();
  void UpdateRotateIntErr();
  void MatrixMultiplication3x1(float three[3][3], float one[3], float output[3]);
  void FindBodyAlphaCmd();
  void AngAccelHelper(float rate_error[3]);
  void FindBodyTorqueCmd();
  void CrossProduct(float vecA[3], float vecB[3], float vecOut[3]);

  void VarToCtlMsg();

  // for testing
  void BeforeSimulink(ctl_input_msg &before_ctl_input_, cmd_msg &before_cmd_, ctl_msg &before_ctl_);
  void AfterSimulink(ctl_input_msg &after_ctl_input_, cmd_msg &after_cmd_, ctl_msg &after_ctl_);
  void RevertBackToBeforeSimulink(ctl_input_msg &before_ctl_input_, cmd_msg &before_cmd_, ctl_msg &before_ctl_);
  void RevertBackToAfterSimulink(ctl_input_msg &after_ctl_input_, cmd_msg &after_cmd_, ctl_msg &after_ctl_);
};
}  // end namespace gnc_autocode

#endif  // GNC_AUTOCODE_CTL_H_

