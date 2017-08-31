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

#include <gnc_autocode/sim.h>

#include <assert.h>

#include <sim_tunable_funcs.h>

namespace gnc_autocode {

GncSimAutocode::GncSimAutocode(void) {
  // allocate model data
  sim_ = sim_model_lib0(&act_msg_, &cmc_in_msg_, &optical_msg_, &hand_msg_, &cmc_out_msg_, &imu_msg_,
                        &env_msg_, &bpm_msg_, &reg_pulse_, &landmark_msg_, &ar_tag_msg_, &ex_time_msg_);
  // check for memory allocation error
  assert(sim_);
  // check status of model registration
  assert(rtmGetErrorStatus(sim_) == NULL);

  Initialize();
}

void GncSimAutocode::Initialize(void) {
  // initialize model
  sim_model_lib0_initialize(sim_, &act_msg_, &cmc_in_msg_, &optical_msg_, &hand_msg_, &cmc_out_msg_, &imu_msg_,
                            &env_msg_, &bpm_msg_, &reg_pulse_, &landmark_msg_, &ar_tag_msg_, &ex_time_msg_);
}

GncSimAutocode::~GncSimAutocode() {
  sim_model_lib0_terminate(sim_);
}

void GncSimAutocode::ReadParams(config_reader::ConfigReader* config) {
  sim_ReadParams(config,  sim_);
}

void GncSimAutocode::Step(void) {
  // This is copied from the auto code. Copy it again if GNC updates as there
  // are magical divisors in here. Yikes!
  sim_model_lib0_step0(sim_, &act_msg_, &cmc_in_msg_, &optical_msg_, &hand_msg_,  &cmc_out_msg_, &imu_msg_,
                       &env_msg_, &bpm_msg_, &reg_pulse_, &landmark_msg_, &ar_tag_msg_, &ex_time_msg_);

  static bool OverrunFlags[5] = { 0, 0, 0, 0, 0 };
  static bool eventFlags[5] = { 0, 0, 0, 0, 0 };
  static int taskCounter[5] = { 0, 0, 0, 0, 0 };
  int i;

  /* Check base rate for overrun */
  if (OverrunFlags[0]) {
    rtmSetErrorStatus(sim_, "Overrun");
    return;
  }

  OverrunFlags[0] = true;

  /*
   * For a bare-board target (i.e., no operating system), the
   * following code checks whether any subrate overruns,
   * and also sets the rates that need to run this time step.
   */
  for (i = 1; i < 5; i++) {
    if (taskCounter[i] == 0) {
      if (eventFlags[i]) {
        OverrunFlags[0] = false;
        OverrunFlags[i] = true;

        /* Sampling too fast */
        rtmSetErrorStatus(sim_, "Overrun");
        return;
      }

      eventFlags[i] = true;
    }
  }

  taskCounter[1]++;
  if (taskCounter[1] == 4) {
    taskCounter[1]= 0;
  }

  taskCounter[2]++;
  if (taskCounter[2] == 10) {
    taskCounter[2]= 0;
  }

  taskCounter[3]++;
  if (taskCounter[3] == 13) {
    taskCounter[3]= 0;
  }

  taskCounter[4]++;
  if (taskCounter[4] == 31) {
    taskCounter[4]= 0;
  }

  /* Indicate task for base rate complete */
  OverrunFlags[0] = false;

  /* Step the model for any subrate */
  for (i = 1; i < 5; i++) {
    /* If task "i" is running, don't run any lower priority task */
    if (OverrunFlags[i]) {
      return;
    }

    if (eventFlags[i]) {
      OverrunFlags[i] = true;

      /* Step the model for subrate "i" */
      switch (i) {
        case 1 :
          sim_model_lib0_step1(sim_);
          break;
        case 2 :
          sim_model_lib0_step2(sim_);
          break;
        case 3 :
          sim_model_lib0_step3(sim_);
          break;
        case 4 :
          sim_model_lib0_step4(sim_);
          break;
      }

      /* Indicate task complete for sample time "i" */
      OverrunFlags[i] = false;
      eventFlags[i] = false;
    }
  }
}

}  // namespace gnc_autocode

