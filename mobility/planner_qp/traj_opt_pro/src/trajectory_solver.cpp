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

#include <traj_opt_pro/trajectory_solver.h>
#include <utility>
#include <vector>

namespace traj_opt {

TrajectorySolver::TrajectorySolver() {}

bool TrajectorySolver::solveTrajectory(
    const std::vector<Waypoint> &waypnts, const std::vector<MatD> &A,
    const std::vector<VecD> &b, const std::vector<decimal_t> &ds,
    decimal_t epsilon, boost::shared_ptr<Vec3Vec> points, decimal_t upsilon) {
  std::cout << "Using base class, doing nothing" << std::endl;
  return false;
}

bool TrajectorySolver::trajectoryStatus() { return trajectory_solved_; }
boost::shared_ptr<Trajectory> TrajectorySolver::getTrajectory() {
  return traj_;
}
bool TrajectorySolver::adjustTimes(decimal_t epsilon) { return false; }

void TrajectorySolver::setParams(decimal_t v_max, decimal_t a_max,
                                 decimal_t j_max, int time_its,
                                 decimal_t time_eps) {
  v_max_ = v_max;
  a_max_ = a_max;
  j_max_ = j_max;
  time_its_ = time_its;
  time_eps_ = time_eps;
}

std::pair<Eigen::VectorXi, MatD> Waypoint::getIndexForm()
    const {  // just makes bookkeeping easier.
  int n = static_cast<int>(use_pos) + static_cast<int>(use_vel) +
          static_cast<int>(use_acc) + static_cast<int>(use_jrk);
  MatD form = MatD(pos.rows(), n);
  Eigen::VectorXi ind = Eigen::VectorXi(n);
  int j = 0;
  if (use_pos) {
    form.block(0, j, pos.rows(), pos.cols()) = pos;
    ind(j) = 0;
    j++;
  }
  if (use_vel) {
    form.block(0, j, vel.rows(), vel.cols()) = vel;
    ind(j) = 1;
    j++;
  }
  if (use_acc) {
    form.block(0, j, acc.rows(), acc.cols()) = acc;
    ind(j) = 2;
    j++;
  }
  if (use_jrk) {
    form.block(0, j, jrk.rows(), jrk.cols()) = jrk;
    ind(j) = 3;
    j++;
  }
  return std::pair<Eigen::VectorXi, MatD>(ind, form);
}

}  // namespace traj_opt
