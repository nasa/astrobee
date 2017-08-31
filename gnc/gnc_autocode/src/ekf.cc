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

#include <gnc_autocode/ekf.h>

#include <config_reader/config_reader.h>
#include <assert.h>

#include <est_tunable_funcs.h>

namespace gnc_autocode {

GncEkfAutocode::GncEkfAutocode(cmc_msg * cmc) : cmc_(cmc) {
  int s = 15 + 6 + 6 * ASE_OF_NUM_AUG;
  P_ = static_cast<float*>(malloc(sizeof(float) * s * s));
  assert(P_);
  est_ = est_estimator(&vis_, &reg_, &of_, &hand_, &imu_, cmc_, quat_, &kfl_, P_);
  assert(est_);
  assert(rtmGetErrorStatus(est_) == NULL);
  Initialize();
}

GncEkfAutocode::~GncEkfAutocode() {
  est_estimator_terminate(est_);
  free(P_);
}

void GncEkfAutocode::Step() {
  est_estimator_step(est_, &vis_, &reg_, &of_, &hand_, &imu_, cmc_, quat_, &kfl_, P_);
}

void GncEkfAutocode::Initialize() {
  // initialize model
  est_estimator_initialize(est_, &vis_, &reg_, &of_, &hand_, &imu_, cmc_, quat_, &kfl_, P_);
}

void GncEkfAutocode::ReadParams(config_reader::ConfigReader* config) {
  est_ReadParams(config,  est_);
}

}  // end namespace gnc_autocode
