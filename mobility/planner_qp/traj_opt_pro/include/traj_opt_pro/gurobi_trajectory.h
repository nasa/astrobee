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

#ifndef TRAJ_OPT_PRO_GUROBI_TRAJECTORY_H_
#define TRAJ_OPT_PRO_GUROBI_TRAJECTORY_H_

#include <traj_opt_pro/polynomial_basis.h>
#include <traj_opt_pro/trajectory_solver.h>

#include <memory>
#include <vector>

class GRBLinExpr;
class GRBQuadExpr;
class GRBVar;
class GRBModel;

namespace traj_opt {

enum { TRAJ_X = 0, TRAJ_Y = 1, TRAJ_Z = 2, TRAJ_PSI = 3 };

class TrajSection1D {
 public:
  TrajSection1D(GRBModel *model_, uint n_p_, uint k_r_, decimal_t dt_,
                boost::shared_ptr<BasisBundlePro> basis);
  ~TrajSection1D();

  decimal_t evaluate(decimal_t t, uint derr) const;

  // gurobi contr functions
  void getContr(decimal_t x, uint derr, GRBLinExpr &expr);
  void getLinCost(GRBLinExpr &expr, uint i);
  void getCost(GRBQuadExpr &cost);

  // unpacks optimized coefficients
  void recoverVars(decimal_t ratio = 1.0);
  void recoverVarsF(decimal_t ratio = 1.0);

  decimal_t getCostDerr();
  void setDt(decimal_t t) {
    // rescale coefficients
    //    for (auto &co : coeffs)
    //      co *= std::pow(dt / t, k_r);
    dt = t;
  }
  Poly getBoostPoly();

  uint n_p;      // dimension of underlying basis representation
  uint k_r;      // order being minimized i.e. 4 = snap
  decimal_t dt;  // durration of segment

  boost::shared_ptr<BasisBundlePro> basis;
  // list of bases // makethis global singleton?

  std::vector<GRBVar> coeffs_var;

  std::vector<decimal_t> coeffs;

 protected:
  bool generated;  // whether or not trajectory has been generated

  // gurobi model
  GRBModel *model;
};

// Trajectory section in 4D

class TrajSection4D {
 public:
  TrajSection4D(GRBModel *model, uint n_p_, uint k_r_, decimal_t dt_,
                boost::shared_ptr<BasisBundlePro> basis_,
                boost::shared_ptr<BasisTransformer> basisT_,
                boost::shared_ptr<BasisTransformer> basisD_);
  ~TrajSection4D();

  void evaluate(decimal_t t, uint derr, VecD &out) const;

  void getContr(decimal_t x, uint derr, uint dim,
                GRBLinExpr &expr);  // dim 0,1,2,3 for x,y,z,psi
  void getCost(GRBQuadExpr &cost);

  decimal_t getCostDerr();
  // unpacks optimized coefficients
  void recoverVars(decimal_t ratio = 1.0);
  void recoverVarsF(decimal_t ratio = 1.0);
  void setDt(decimal_t dt);

  std::vector<boost::shared_ptr<TrajSection1D>> secs;
  void addPathCost(const MatD &A, const VecD &b, double epsilon);
  void addLineCost(const Vec3 &p0, const Vec3 &p1, double upsilon);
  void getControl(uint point_id, uint dim, GRBLinExpr &expr, bool derr);
  void addVisionCost(const MatD &norm_pnts, decimal_t weight_loss, VecD &grad);
  // private:
  std::vector<GRBVar> const_var;

  GRBModel *model_;
  boost::shared_ptr<BasisTransformer> basisT;
  boost::shared_ptr<BasisTransformer> basisD;
  boost::shared_ptr<BasisBundlePro> basis_;
  int dim{4};
  int n_p;
  decimal_t dt_;
};
enum ConstraintMode { CONTROL, SAMPLE, CHEBY, ELLIPSE };
class LegendreTrajectory : public Trajectory {
 public:
  LegendreTrajectory(GRBModel *model_, const std::vector<decimal_t> dts_,
                     int degree, PolyType polytype,
                     int order = 3);  // way points and
                                      // times each column
                                      // represents a time
                                      // derrivate: column 0
                                      // is position, column
                                      // 1 is velocity, etc.
  // Trajectory(const Trajectory& traj) = delete; // forbid copying because
  // dependent on gurobi model pointer
  ~LegendreTrajectory();

  void addMaximumBound(decimal_t bound,
                       uint derr);  // ex (3.5,1) sets maximum velocities to 3.5
  bool recoverVars(decimal_t ratio = 1.0);
  bool recoverVarsF(decimal_t ratio = 1.0);
  bool evaluate(decimal_t t, uint derr, VecD &out)
      const;  // returns false when out of time range, but returns enpoints

  // get total trajectory length in seconds
  decimal_t getTotalTime() const;

  void addVolumeContraints(const Mat4Vec &constr);  // adds 3D volume constrains
                                                    // to the position of the
                                                    // robot
  // add constraints of the form Ax <= b
  void addAxbConstraints(const std::vector<MatD> &A, const std::vector<VecD> &b,
                         const std::vector<decimal_t> &ds);

  void addWayPointConstrains(const std::vector<Waypoint> &waypnts, bool use_v0);

  decimal_t getCost();

  bool adjustTimes(decimal_t epsilon);

  void adjustTimeToMaxs(decimal_t max_vel, decimal_t max_acc, decimal_t max_jrk,
                        bool use_v0, decimal_t v0);
  VecD check_max_violation(decimal_t max_vel, decimal_t max_acc,
                           decimal_t max_jrk);
  void resetConstr(bool lp = false);
  void setQuadraticCost();
  void setLinearCost();
  void setCost(bool lp);

  virtual TrajData serialize();

  std::vector<boost::shared_ptr<TrajSection4D>> individual_sections;
  void addConvexificationConstraints(const MatD &Aobs, const VecD &bobs,
                                     const MatD &Aenv, const VecD &benv);
  void setSlackCost();
  void setConParam(ConstraintMode mode, int num) {
    con_mode_ = mode;
    num_sample_ = num;
  }
  void resetConstrVision(const MatD &points, decimal_t weight);
  void getJacobian(const MatD &norm_pnts, decimal_t weight_loss, VecD &J);
  decimal_t getCostV(const MatD &points, decimal_t weight);

  bool check_max(decimal_t max_vel, decimal_t max_acc, decimal_t max_jrk);

 private:
  void linkSections();  // links endpoints of trajectories

  GRBModel *model;

  std::vector<decimal_t> dts;
  std::vector<MatD> A_;
  std::vector<VecD> b_;
  std::vector<Waypoint> waypnts_;

  std::vector<decimal_t> old_dts;
  int n_p_;
  int k_r_;

  boost::shared_ptr<BasisTransformer> basisT;
  boost::shared_ptr<BasisTransformer> basisD;
  std::vector<GRBVar> slack_var;

  std::vector<Poly> time_composite;
  bool use_time_composite{false};
  ConstraintMode con_mode_{CONTROL};
  int num_sample_{10};
  int dim{4};
};
}  // namespace traj_opt
#endif  // TRAJ_OPT_PRO_GUROBI_TRAJECTORY_H_
