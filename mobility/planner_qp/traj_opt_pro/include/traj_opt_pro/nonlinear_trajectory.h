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

#include <traj_opt_pro/nonlinear_solver.h>
#include <traj_opt_pro/polynomial_basis.h>
#include <traj_opt_pro/trajectory_solver.h>
#include <boost/algorithm/clamp.hpp>
#include <map>
#include <tuple>
#include <utility>
#include <vector>

#ifndef TRAJ_OPT_PRO_NONLINEAR_TRAJECTORY_H_
#define TRAJ_OPT_PRO_NONLINEAR_TRAJECTORY_H_

namespace traj_opt {

typedef std::vector<Variable *> NLTimes;
typedef std::vector<Variable *> NLPoly;
typedef std::vector<NLPoly> NLSpline;
typedef std::vector<NLSpline> NLTraj;

// to maintain same tensor ordering of (dim,segment) for ball confinement
typedef std::vector<Variable *> NLChain;  // (xc,yc,zc,radius)
typedef std::vector<NLChain> NLBeads;

class AxbConstraint;
class BallConstraint;

class SymbolicPoly {
  typedef std::tuple<Variable *, Variable *, int>
      PolyTriple;  // for linear polys in c
  typedef std::tuple<Variable *, Variable *, Variable *, int>
      PolyQuad;  // for quadratic polys in c

 public:
  SymbolicPoly() {}  // null constructor
                     // form of a*coeff*t^n
  SymbolicPoly(Variable *coeff, Variable *time, int n, decimal_t a);
  // form of a*coeff0*coeff1*t^n
  SymbolicPoly(Variable *coeff0, Variable *coeff1, Variable *time, int n,
               decimal_t a);
  SymbolicPoly &operator+=(const SymbolicPoly &rhs);
  friend SymbolicPoly operator*(decimal_t lhs, const SymbolicPoly &rhs);
  decimal_t evaluate();
  // for these derrivaties assume only linear terms
  ETV gradient(int u_id);
  ETV hessian();
  // for these derrivaties assume only quadratic terms
  ETV quad_gradient(int u_id);
  ETV quad_gradient();
  ETV quad_hessian();

  SymbolicPoly square();  // returns the square the linear parts
  void add(const SymbolicPoly &rhs);
  friend std::ostream &operator<<(std::ostream &os, const SymbolicPoly &poly);

 private:
  std::map<PolyTriple, decimal_t> poly_map;
  std::map<PolyQuad, decimal_t> quad_map;
};

class PolyCost;
class TimeBound;
class NonlinearTrajectory : public Trajectory {
 public:
  // standard contruction
  NonlinearTrajectory(
      const std::vector<Waypoint> &waypoints,
      const std::vector<std::pair<MatD, VecD> > &cons, int deg = 7,
      int min_dim = 3, boost::shared_ptr<std::vector<decimal_t> > ds =
                           boost::shared_ptr<std::vector<decimal_t> >(),
      boost::shared_ptr<VecDVec> path = boost::shared_ptr<VecDVec>());
  // nonconvex pointcloud test
  NonlinearTrajectory(const std::vector<Waypoint> &waypoints,
                      const Vec3Vec &points, int segs, decimal_t dt);
  virtual ~NonlinearTrajectory() {}

  decimal_t getTotalTime() const;
  bool evaluate(decimal_t t, uint derr, VecD &out) const;
  decimal_t getCost();
  TrajData serialize();
  bool isSolved() { return solved_; }
  Vec4Vec getBeads();
  void scaleTime(decimal_t ratio);

 protected:
  void allocate_poly(
      boost::shared_ptr<std::vector<decimal_t> > ds =
          boost::shared_ptr<std::vector<decimal_t> >(),
      boost::shared_ptr<VecDVec> path = boost::shared_ptr<VecDVec>());
  void allocate_beads();
  void link_sections();  // note, requires endpoint basis
  void add_boundary(
      const std::vector<Waypoint> &waypoints);  // add boundary values
  void add_Axb(const std::vector<std::pair<MatD, VecD> > &cons);
  void addCloudConstraint(const Vec3Vec &points);
  void addPosTime();  // ensures time segments are >
  void addTimeBound(boost::shared_ptr<std::vector<decimal_t> > ds =
                        boost::shared_ptr<std::vector<decimal_t> >());
  void make_convex(boost::shared_ptr<std::vector<decimal_t> > ds);
  NonlinearSolver solver{10000};
  NLTraj traj;
  NLTimes times;
  NLBeads beads;

  boost::shared_ptr<PolyCost> cost;
  boost::shared_ptr<BasisTransformer> basisT;
  // sizes
  // dim_ in parent
  int seg_, deg_;
  BasisBundlePro basis;
  bool solved_{false};
  friend class AxbConstraint;
  friend class BallConstraint;
};
class PolyCost : public CostFunction {
 public:
  PolyCost(const NLTraj &traj, const NLTimes &times, BasisBundlePro &basis,
           int min_dim);
  decimal_t evaluate();
  ETV gradient();
  ETV hessian();

 private:
  SymbolicPoly poly;
  MatD cost_v_;
  Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic> cost_n_;
  void init_constants();

 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace traj_opt
#endif  // TRAJ_OPT_PRO_NONLINEAR_TRAJECTORY_H_
