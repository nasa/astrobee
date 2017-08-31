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

#include <gnc_autocode/fam.h>

#include <config_reader/config_reader.h>
#include <assert.h>

#include <fam_tunable_funcs.h>

namespace gnc_autocode {

GncFamAutocode::GncFamAutocode() {
  fam_ = fam_force_allocation_module(&time_, &cmd_, &ctl_, &cmc_, &act_);
  assert(fam_);
  assert(rtmGetErrorStatus(fam_) == NULL);
  Initialize();
}

GncFamAutocode::~GncFamAutocode() {
  fam_force_allocation_module_terminate(fam_);
}

void GncFamAutocode::Step(ex_time_msg* ex_time, cmd_msg* cmd, ctl_msg* ctl) {
  fam_force_allocation_module_step(fam_, ex_time, cmd, ctl, &cmc_, &act_);
}

void GncFamAutocode::Initialize() {
  // initialize model
  fam_force_allocation_module_initialize(fam_, &time_, &cmd_, &ctl_, &cmc_, &act_);
}

void GncFamAutocode::ReadParams(config_reader::ConfigReader* config) {
  fam_ReadParams(config,  fam_);
}

}  // end namespace gnc_autocode
