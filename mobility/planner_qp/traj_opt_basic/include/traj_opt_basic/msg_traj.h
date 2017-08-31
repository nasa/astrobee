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

#ifndef TRAJ_OPT_BASIC_MSG_TRAJ_H_
#define TRAJ_OPT_BASIC_MSG_TRAJ_H_

#include <traj_opt_basic/polynomial_basis.h>
#include <traj_opt_basic/traj_data.h>
#include <traj_opt_basic/trajectory.h>
#include <vector>

namespace traj_opt {
class MsgTrajectory : public Trajectory {
 public:
  explicit MsgTrajectory(const TrajData &traj);

  bool evaluate(decimal_t t, uint derr, VecD &out) const override;
  decimal_t getTotalTime() const override;
  decimal_t getCost() override;
  TrajData serialize() override;

 protected:
  TrajData traj_;
  std::vector<std::vector<boost::shared_ptr<Poly>>> polyies_;
  std::vector<std::vector<std::vector<boost::shared_ptr<Poly>>>> derrives_;
  uint num_secs_;
  std::vector<decimal_t> dts;
  uint deg_;
};
}  // namespace traj_opt
#endif  // TRAJ_OPT_BASIC_MSG_TRAJ_H_
