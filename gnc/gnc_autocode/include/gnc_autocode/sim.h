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

#ifndef GNC_AUTOCODE_SIM_H_
#define GNC_AUTOCODE_SIM_H_

extern "C" {
#include <sim_model_lib0.h>
}

namespace config_reader {
  class ConfigReader;
}

namespace gnc_autocode {
class GncSimAutocode {
 public:
  GncSimAutocode();
  ~GncSimAutocode();
  virtual void Initialize();
  virtual void Step();
  virtual void ReadParams(config_reader::ConfigReader* config);

  // This is just a thin wrapper with a step function
  RT_MODEL_sim_model_lib0_T* sim_;

  // state variable
  env_msg env_msg_;

  // input variables
  act_msg act_msg_;
  cmc_msg cmc_in_msg_;

  // output variables
  ex_time_msg ex_time_msg_;
  cvs_registration_pulse reg_pulse_;
  cvs_landmark_msg landmark_msg_;
  cvs_landmark_msg ar_tag_msg_;
  cvs_optical_flow_msg optical_msg_;
  cvs_handrail_msg hand_msg_;
  cmc_msg cmc_out_msg_;
  imu_msg imu_msg_;
  bpm_msg bpm_msg_;
};
}  // end namespace gnc_autocode

#endif  // GNC_AUTOCODE_SIM_H_
