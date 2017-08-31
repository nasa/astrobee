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

#include <traj_opt_basic/trajectory.h>
#include <vector>

namespace traj_opt {

decimal_t Trajectory::getExecuteTime() const {
  if (exec_t <= 0)
    return getTotalTime();
  else
    return exec_t;
}

bool Trajectory::getCommand(decimal_t t, uint num_derivatives, MatD &data) {
  // check input
  if (dim_ < 1) return false;

  // allocate data
  data = MatD::Zero(dim_, num_derivatives + 1);
  // evaluate and pack
  for (uint i = 0; i <= num_derivatives; i++) {
    VecD block;
    this->evaluate(t, i, block);
    data.block(0, i, dim_, 1) = block;
  }
  return true;
}

}  // namespace traj_opt
