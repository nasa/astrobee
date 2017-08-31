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

namespace gnc_autocode {

GncCtlAutocode::GncCtlAutocode(cmc_msg * cmc) : cmc_(cmc) {
  controller_ = ctl_controller0(&kfl_, cmc_, &time_, &env_, &cmd_, &ctl_);
  assert(controller_);
  assert(rtmGetErrorStatus(controller_) == NULL);
}

GncCtlAutocode::~GncCtlAutocode() {
  ctl_controller0_terminate(controller_);
}

void GncCtlAutocode::Step(kfl_msg* kfl) {
  ctl_controller0_step(controller_, kfl, cmc_, &time_, &env_, &cmd_, &ctl_);
}

void GncCtlAutocode::Initialize() {
  // initialize model
  ctl_controller0_initialize(controller_, &kfl_, cmc_, &time_, &env_, &cmd_, &ctl_);
}

void GncCtlAutocode::ReadParams(config_reader::ConfigReader* config) {
  ctl_ReadParams(config, controller_);
}

}  // end namespace gnc_autocode
