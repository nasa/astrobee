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

#ifndef EXECUTIVE_OP_STATE_REPO_H_
#define EXECUTIVE_OP_STATE_REPO_H_

#include <ros/ros.h>

#include "executive/op_state.h"
#include "executive/op_state_auto_return.h"
#include "executive/op_state_fault.h"
#include "executive/op_state_plan_exec.h"
#include "executive/op_state_ready.h"
#include "executive/op_state_teleop.h"

#include "ff_msgs/OpState.h"

namespace executive {
/**
 * OpStateRepo singleton class contains all available states,
 * knowledge of avail transitions are to be
 * held in respective states
 *
 * When new states are defined, they must be added here
 * with an accessor method
 */
typedef std::unique_ptr<OpStateRepo> OpStateRepoPtr;
class OpStateRepo {
 public:
  static OpStateRepo* Instance() {return instance_.get();}

  void SetExec(Executive *const exec);

  // get state ptrs
  OpState* ready() {return ready_.get();}
  OpState* plan_exec() {return plan_exec_.get();}
  OpState* teleop() {return teleop_.get();}
  OpState* auto_return() {return auto_return_.get();}
  OpState* fault() {return fault_.get();}

 private:
  OpStateRepo();
  static OpStateRepoPtr instance_;
  // all avail states
  OpStatePtr ready_, plan_exec_, teleop_, auto_return_, fault_;

  OpStateRepo (const OpStateRepo&) = delete;
  OpStateRepo& operator= (const OpStateRepo&) = delete;
};
}  // namespace executive
#endif  // EXECUTIVE_OP_STATE_REPO_H_
