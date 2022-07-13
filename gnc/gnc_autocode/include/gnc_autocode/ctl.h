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
}




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
  float prev_filter[3];
  int prev_mode_cmd[4];  // for the 4 ticks required  to swtich to stopped; newest val at index 0
  float prev_position[3];
  float prev_att[4];
  float pos_err[3];
  float quat_err;

  bool BelowThreshold(float velocity[], float threshhold);
  void UpdateModeCmd(void);
  float ButterWorthFilter(float input, float previous);
  bool CmdModeMakeCondition();
  void UpdateStoppedMode();
  void UpdatePosAndQuat();
  void FindPosError();
  void UpdatePrevious();
  void FindQuatError(float q_cmd[4], float q_actual[4]);
  void UpdateCtlStatus();
  bool CtlStatusSwitch();

  // Simulink outports
  float att_command[3];
  float position_command[3];
  float ctl_status;
  float velocity_command[3];
  float omega_command[3];
  float accel_command[3];
  float alpha_command[3];
};
}  // end namespace gnc_autocode

#endif  // GNC_AUTOCODE_CTL_H_
