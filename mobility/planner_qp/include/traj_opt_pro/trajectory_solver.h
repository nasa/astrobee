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

#ifndef TRAJ_OPT_PRO_TRAJECTORY_SOLVER_H_
#define TRAJ_OPT_PRO_TRAJECTORY_SOLVER_H_

#include <traj_opt_basic/trajectory.h>

#include <boost/smart_ptr/shared_ptr.hpp>
#include <iostream>
#include <utility>
#include <vector>

namespace traj_opt {

struct Waypoint {
  int knot_id{0};  // 0: first, 1: second, -1: last -2: second to last, etc
  VecD pos{VecD::Zero(4, 1)};
  VecD vel{VecD::Zero(4, 1)};
  VecD acc{VecD::Zero(4, 1)};
  VecD jrk{VecD::Zero(4, 1)};
  bool use_pos{false};
  bool use_vel{false};
  bool use_acc{false};
  bool use_jrk{false};

  // stack constraint into matrix form
  std::pair<Eigen::VectorXi, MatD> getIndexForm() const;
};

class TrajectorySolver {
 public:
  TrajectorySolver();
  virtual bool solveTrajectory(
      const std::vector<Waypoint> &waypnts, const std::vector<MatD> &A,
      const std::vector<VecD> &b, const std::vector<decimal_t> &ds,
      decimal_t epsilon = 0,
      boost::shared_ptr<Vec3Vec> points = boost::shared_ptr<Vec3Vec>(),
      decimal_t upsilon = 0);

  virtual bool trajectoryStatus();
  virtual boost::shared_ptr<Trajectory> getTrajectory();

  virtual bool adjustTimes(decimal_t epsilon);
  virtual void setParams(decimal_t v_max, decimal_t a_max, decimal_t j_max,
                         int time_its, decimal_t time_eps);

  virtual bool checkMax(decimal_t r) { return false; }

 protected:
  std::vector<Waypoint> waypoints_;
  bool trajectory_solved_;
  boost::shared_ptr<Trajectory> traj_;

  // parameters
  decimal_t v_max_, a_max_, j_max_, time_eps_;
  int time_its_;
};
}  // namespace traj_opt
#endif  // TRAJ_OPT_PRO_TRAJECTORY_SOLVER_H_
