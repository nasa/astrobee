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

#ifndef TRAJ_OPT_BASIC_TRAJECTORY_H_
#define TRAJ_OPT_BASIC_TRAJECTORY_H_

// package includes
#include <traj_opt_basic/traj_data.h>
#include <traj_opt_basic/types.h>

// library includes
#include <boost/smart_ptr/shared_ptr.hpp>

// STL includes
#include <iostream>
#include <vector>

namespace traj_opt {

class Trajectory {
 public:
  virtual ~Trajectory() {}
  virtual bool evaluate(decimal_t t, uint derr, VecD &out) const = 0;
  virtual decimal_t getTotalTime() const = 0;
  virtual decimal_t getCost() = 0;
  // execute time
  decimal_t getExecuteTime() const;

  // returns a matrix (dim X num_derivatives + 1) of the trajectory evalutated
  // at time t
  bool getCommand(decimal_t t, uint num_derivatives, MatD &data);

  void setDim(uint ndim) { dim_ = ndim; }
  void setExecuteTime(decimal_t t) { exec_t = t; }

  virtual TrajData serialize() = 0;

 protected:
  decimal_t exec_t{-1.0};  // duration of execute time
  int dim_{0};
};
}  // namespace traj_opt
#endif  // TRAJ_OPT_BASIC_TRAJECTORY_H_
