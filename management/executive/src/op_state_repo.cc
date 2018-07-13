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

#include "executive/op_state_repo.h"

namespace executive {
// eager initialization of repo
OpStateRepoPtr OpStateRepo::instance_(new OpStateRepo);

void OpStateRepo::SetExec(Executive *const exec) {
  ready_->SetExec(exec);
  plan_exec_->SetExec(exec);
  teleop_->SetExec(exec);
  auto_return_->SetExec(exec);
  fault_->SetExec(exec);
}

OpStateRepo::OpStateRepo() {
  ready_.reset(new OpStateReady("ready", ff_msgs::OpState::READY));
  plan_exec_.reset(new OpStatePlanExec("plan execution",
        ff_msgs::OpState::PLAN_EXECUTION));
  teleop_.reset(new OpStateTeleop("teleoperation",
        ff_msgs::OpState::TELEOPERATION));
  auto_return_.reset(new OpStateAutoReturn("auto return",
        ff_msgs::OpState::AUTO_RETURN));
  fault_.reset(new OpStateFault("fault", ff_msgs::OpState::FAULT));
}
}  // namespace executive
