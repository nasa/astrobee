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

#include <traj_opt_pro/nonlinear_trajectory.h>

#include <utility>
#include <vector>

namespace traj_opt {

void PolyCost::init_constants() {
  cost_n_ = Eigen::Matrix<int, Eigen::Dynamic, Eigen::Dynamic>::Zero(8, 8);
  cost_v_ = MatD::Zero(8, 8);

  // LOL, I guess I should document these. In the mean time, enjoy decyphering
  // them :P
  cost_n_ << -5, -5, -4, -4, -3, -3, -2, -2, -5, -5, -4, -4, -3, -3, -2, -2, -4,
      -4, -3, -3, -2, -2, -1, -1, -4, -4, -3, -3, -2, -2, -1, -1, -3, -3, -2,
      -2, -1, -1, 0, 0, -3, -3, -2, -2, -1, -1, 0, 0, -2, -2, -1, -1, 0, 0, 1,
      1, -2, -2, -1, -1, 0, 0, 1, 1;
  cost_v_ << 1.120000000000000, -1.120000000000000, 0.560000000000000,
      0.560000000000000, 0.100000000000000, -0.100000000000000,
      0.003333333333333, 0.003333333333333, -1.120000000000000,
      1.120000000000000, -0.560000000000000, -0.560000000000000,
      -0.100000000000000, 0.100000000000000, -0.003333333333333,
      -0.003333333333333, 0.560000000000000, -0.560000000000000,
      0.297142857142857, 0.262857142857143, 0.058571428571429,
      -0.041428571428571, 0.002095238095238, 0.001238095238095,
      0.560000000000000, -0.560000000000000, 0.262857142857143,
      0.297142857142857, 0.041428571428571, -0.058571428571429,
      0.001238095238095, 0.002095238095238, 0.100000000000000,
      -0.100000000000000, 0.058571428571429, 0.041428571428571,
      0.014285714285714, -0.005714285714286, 0.000547619047619,
      0.000119047619048, -0.100000000000000, 0.100000000000000,
      -0.041428571428571, -0.058571428571429, -0.005714285714286,
      0.014285714285714, -0.000119047619048, -0.000547619047619,
      0.003333333333333, -0.003333333333333, 0.002095238095238,
      0.001238095238095, 0.000547619047619, -0.000119047619048,
      0.000063492063492, -0.000007936507937, 0.003333333333333,
      -0.003333333333333, 0.001238095238095, 0.002095238095238,
      0.000119047619048, -0.000547619047619, -0.000007936507937,
      0.000063492063492;
  cost_v_ *= 1000.0;
}

NonlinearTrajectory::NonlinearTrajectory(
    const std::vector<Waypoint> &waypoints,
    const std::vector<std::pair<MatD, VecD>> &cons, int deg, int min_dim,
    boost::shared_ptr<std::vector<decimal_t>> ds,
    boost::shared_ptr<VecDVec> path)
    : seg_(cons.size()), deg_(deg), basis(PolyType::ENDPOINT, deg_, min_dim) {
  dim_ = waypoints.front().pos.rows();
  assert(dim_ == cons.front().first.cols());

  auto endpoint_basis = boost::make_shared<EndPointBasis>(deg_);
  auto bezier_basis = boost::make_shared<BezierBasis>(deg_);
  basisT = boost::make_shared<BasisTransformer>(endpoint_basis, bezier_basis);

  // if (ds != NULL) std::cout << "ds " << ds->size() << std::endl;
  // if (path != NULL) std::cout << "path " << path->size() << std::endl;

  // std::cout << "con " << cons.size() << std::endl;

  // setup poly
  allocate_poly(ds, path);
  // equality
  add_boundary(waypoints);
  link_sections();
  // ineq
  add_Axb(cons);
  addPosTime();
  // bound time total
  //      addTimeBound(ds);
  make_convex(ds);
  // cost
  cost = boost::make_shared<PolyCost>(traj, times, basis, min_dim);

  solver.setCost(cost);
  // call solver
  // solved_ = solver.solve(true);
  solved_ = solver.solve(false);
}
NonlinearTrajectory::NonlinearTrajectory(const std::vector<Waypoint> &waypoints,
                                         const Vec3Vec &points, int segs,
                                         decimal_t dt)
    : seg_(segs), deg_(7), basis(PolyType::ENDPOINT, deg_, 3) {
  dim_ = waypoints.front().pos.rows();
  auto endpoint_basis = boost::make_shared<EndPointBasis>(deg_);
  auto bezier_basis = boost::make_shared<BezierBasis>(deg_);
  basisT = boost::make_shared<BasisTransformer>(endpoint_basis, bezier_basis);

  boost::shared_ptr<std::vector<decimal_t>> ds =
      boost::make_shared<std::vector<decimal_t>>(segs, dt);

  // setup poly
  allocate_poly(ds);
  allocate_beads();
  // equality
  add_boundary(waypoints);
  link_sections();
  // ineq
  addCloudConstraint(points);
  addPosTime();
  // bound time total
  //      addTimeBound(ds);
  make_convex(ds);
  // cost
  cost = boost::make_shared<PolyCost>(traj, times, basis, 3);

  solver.setCost(cost);
  // call solver
  solved_ = solver.solve(true);
  // solved_ = solver.solve(false);
}
decimal_t NonlinearTrajectory::getCost() { return cost->evaluate(); }

void NonlinearTrajectory::allocate_poly(
    boost::shared_ptr<std::vector<decimal_t>> ds,
    boost::shared_ptr<VecDVec> path) {
  traj.clear();
  for (int i = 0; i < dim_; i++) {
    NLSpline spline;
    for (int j = 0; j < seg_; j++) {
      NLPoly pi;
      for (int k = 0; k <= deg_; k++) {
        if (path != NULL && j > 0 && k == 0)
          pi.push_back(solver.addVar(path->at(j)(i)));
        else if (path != NULL && j < deg_ && k == 1)
          pi.push_back(solver.addVar(path->at(j + 1)(i)));
        else
          pi.push_back(solver.addVar(2.0));
      }
      spline.push_back(pi);
    }
    traj.push_back(spline);
  }
  for (int j = 0; j < seg_; j++) {
    if (ds != NULL)
      times.push_back(solver.addVar(ds->at(j)));
    else
      times.push_back(solver.addVar());
  }
}
void NonlinearTrajectory::allocate_beads() {
  beads.clear();
  for (int i = 0; i <= dim_; i++) {
    NLChain chain;
    for (int j = 0; j < seg_; j++) chain.push_back(solver.addVar(2.0));
    beads.push_back(chain);
  }
}

void NonlinearTrajectory::link_sections() {
  for (int d = 0; d < dim_; d++) {
    for (int q = 0; q <= 3; q++) {
      for (int j = 0; j < (seg_ - 1); j++) {
        std::vector<EqConstraint::EqPair> con;
        con.push_back(
            EqConstraint::EqPair(traj.at(d).at(j).at(2 * q + 1), 1.0));
        con.push_back(
            EqConstraint::EqPair(traj.at(d).at(j + 1).at(2 * q), -1.0));
        solver.addConstraint(con, 0.0);
      }
    }
  }
}
void NonlinearTrajectory::make_convex(
    boost::shared_ptr<std::vector<decimal_t>> ds) {
  int i = 0;
  for (auto t : times) {
    std::vector<EqConstraint::EqPair> con;
    con.push_back(EqConstraint::EqPair(t, 1.0));
    if (ds != NULL)
      solver.addConstraint(con, ds->at(i));
    else
      solver.addConstraint(con, 1.0);
    i++;
  }
}
void NonlinearTrajectory::add_boundary(const std::vector<Waypoint> &waypoints) {
  for (auto &point : waypoints) {
    int id = point.knot_id;
    std::pair<Eigen::VectorXi, MatD> pair = point.getIndexForm();
    // note we use start of each segment, unless id == seg_
    if (id < 0) id += seg_ + 1;

    for (int c = 0; c < pair.first.rows(); c++) {
      int co = 2 * pair.first(c);
      int idi = id;
      if (id == seg_) {
        co += 1;
        idi--;
      }

      for (int d = 0; d < dim_; d++) {
        EqConstraint::EqPair p(traj.at(d).at(idi).at(co), 1);
        std::vector<EqConstraint::EqPair> con(1, p);
        solver.addConstraint(con, pair.second(d, c));
      }
    }
  }
}

// first need to create Axb class
class AxbConstraint : public IneqConstraint {
  NonlinearTrajectory *traj_;
  int i, j;
  VecD ai_;
  decimal_t bi_;

 public:
  SymbolicPoly poly;
  // i is bezier coeff numer
  // j is trajectory segment
  AxbConstraint(NonlinearTrajectory *traj, int j_, int i_, const VecD &ai,
                decimal_t bi)
      : traj_(traj) {
    j = j_;
    i = i_;
    ai_ = ai;
    bi_ = bi;
    MatD tf = traj_->basisT->getBasisBasisTransform();
    //        std::cout << "tf "<< tf << std::endl;
    // create polynomial object
    for (int d = 0; d < ai.rows(); d++) {
      for (int k = 0; k <= traj_->deg_; k++) {
        if (std::abs(ai_(d)) > 1e-4)  // for numerical robustness
          poly.add(SymbolicPoly(traj->traj.at(d).at(j).at(k),
                                traj_->times.at(j), k / 2, tf(i, k) * ai_(d)));
      }
    }
  }

 private:
  decimal_t evaluate() { return poly.evaluate() - bi_; }
  ETV gradient() { return poly.gradient(var_u->getId()); }
  ETV hessian() { return poly.hessian(); }
};
// first need to create Axb class
class BallConstraint : public IneqConstraint {
  NonlinearTrajectory *traj_;
  int i, j;
  decimal_t rhs{0.0};

 public:
  SymbolicPoly poly;
  // i is bezier coeff numer
  // j is trajectory segment
  // constructor for keeping the trajectory inside the ball
  BallConstraint(NonlinearTrajectory *traj, int j_, int i_) : traj_(traj) {
    j = j_;
    i = i_;
    MatD tf = traj_->basisT->getBasisBasisTransform();
    // create polynomial object
    for (int d = 0; d < traj->dim_; d++) {
      SymbolicPoly subpoly;
      for (int k = 0; k <= traj_->deg_; k++) {
        subpoly.add(SymbolicPoly(traj->traj.at(d).at(j).at(k),
                                 traj_->times.at(j), k / 2, tf(i, k)));
      }
      subpoly.add(
          SymbolicPoly(traj->beads.at(d).at(j), traj_->times.at(j), 0, -1.0));
      poly.add(subpoly.square());
    }
    // add radius
    poly.add(SymbolicPoly(traj->beads.at(traj_->dim_).at(j),
                          traj->beads.at(traj->dim_).at(j), traj_->times.at(j),
                          0, -1.0));
  }
  // constructor for keeping the ball outisde the points
  // j is the segment
  BallConstraint(NonlinearTrajectory *traj, int j_, const Vec3 &point,
                 decimal_t robot_r)
      : traj_(traj) {
    j = j_;
    assert(traj->dim_ <= 3);
    for (int d = 0; d < traj->dim_; d++) {
      // just manually expand (x-c)^2
      poly.add(SymbolicPoly(traj->beads.at(d).at(j), traj->beads.at(d).at(j),
                            traj_->times.at(j), 0, -1.0));  // x^2
      poly.add(SymbolicPoly(traj->beads.at(d).at(j), traj_->times.at(j), 0,
                            2.0 * point(d)));  // -2xc
      rhs += -point(d) * point(d);             // +c^2
    }
    // expand -(r+r_robot_r)^2
    poly.add(SymbolicPoly(traj->beads.at(traj_->dim_).at(j),
                          traj->beads.at(traj_->dim_).at(j), traj_->times.at(j),
                          0, 1.0));  // -r^2

    rhs += robot_r * robot_r;  // -robot_r^2
  }

 private:
  decimal_t evaluate() { return poly.evaluate() + rhs; }
  ETV gradient() {
    ETV gradl = poly.gradient(var_u->getId());
    ETV gradq = poly.quad_gradient(var_u->getId());

    gradl.insert(gradl.end(), gradq.begin(), gradq.end());
    return gradl;
  }
  ETV hessian() {
    ETV hessl = poly.hessian();
    ETV hessq = poly.quad_hessian();

    hessl.insert(hessl.end(), hessq.begin(), hessq.end());
    return hessl;
  }
};
class PosTimeConstraint : public IneqConstraint {
  Variable *v_;

 public:
  explicit PosTimeConstraint(Variable *v) : v_(v) { vars.push_back(v); }
  decimal_t evaluate() { return -1.0 * v_->getVal(); }
  ETV gradient() {
    ET gt = ET(var_u->getId(), v_->getId(), -1.0);
    return ETV(1, gt);
  }
  ETV hessian() { return ETV(); }
};
class TimeBound : public IneqConstraint {
  decimal_t bound_;

 public:
  TimeBound(const std::vector<Variable *> &v, decimal_t upper_bound)
      : IneqConstraint(), bound_(upper_bound) {
    for (auto &vi : v) vars.push_back(vi);
    //        std::copy(v.begin(),v.end(),vars.begin());
  }
  decimal_t evaluate() {
    decimal_t val = -bound_;
    for (auto &v : vars) val += v->getVal();
    return val;
  }
  ETV gradient() {
    ETV gs;
    for (auto &v : vars) {
      ET gt = ET(var_u->getId(), v->getId(), 1.0);
      gs.push_back(gt);
    }
    return gs;
  }
  ETV hessian() { return ETV(); }
};

void NonlinearTrajectory::addPosTime() {
  for (auto &v : times) {
    boost::shared_ptr<IneqConstraint> con =
        boost::make_shared<PosTimeConstraint>(v);
    solver.addConstraint(con);
  }
}
void NonlinearTrajectory::addTimeBound(
    boost::shared_ptr<std::vector<decimal_t>> ds) {
  boost::shared_ptr<IneqConstraint> con;
  if (ds == NULL) {
    con = boost::make_shared<TimeBound>(times, 10.0);
  } else {
    decimal_t T = 0.0;
    for (auto t : *ds) T += t;
    con = boost::make_shared<TimeBound>(times, T);
  }

  solver.addConstraint(con);
}

void NonlinearTrajectory::add_Axb(
    const std::vector<std::pair<MatD, VecD>> &cons) {
  int j = 0;
  for (auto &pair : cons) {
    assert(pair.first.rows() == pair.second.rows());
    for (int i = 0; i < pair.first.rows(); i++) {
      VecD ai = pair.first.block(i, 0, 1, pair.first.cols()).transpose();
      for (uint k = 0; k < traj.at(0).at(j).size(); k++) {
        boost::shared_ptr<AxbConstraint> con =
            boost::make_shared<AxbConstraint>(this, j, k, ai, pair.second(i));
        solver.addConstraint(con);
        //                std::cout  << "con " << con->id << " : " << con->poly
        //                << std::endl;
      }
    }
    j++;
  }
}
decimal_t NonlinearTrajectory::getTotalTime() const {
  decimal_t T = 0;
  for (auto &v : times) T += v->getVal();
  return T;
}
bool NonlinearTrajectory::evaluate(decimal_t t, uint derr, VecD &out) const {
  t = boost::algorithm::clamp(t, 0.0, getTotalTime());
  int j = 0;
  while (j < seg_ && t > times.at(j)->getVal()) {
    t -= times.at(j)->getVal();
    j++;
  }
  j = boost::algorithm::clamp(j, 0, seg_ - 1);
  // decimal_t val=0;
  decimal_t dt = times.at(j)->getVal();
  decimal_t s = t / dt;
  out = VecD::Zero(dim_);
  for (int d = 0; d < dim_; d++) {
    for (int i = 0; i <= deg_; i++) {
      out(d) += basis.getVal(s, 1.0, i, derr) *
                traj.at(d).at(j).at(i)->getVal() * std::pow(dt, i / 2);
    }
  }
  out /= std::pow(times.at(j)->getVal(), derr);
  return true;
}
TrajData NonlinearTrajectory::serialize() {
  // just copy variable feilds to struct
  TrajData trajd(dim_, seg_, deg_);
  for (int d = 0; d < dim_; d++) {
    for (int j = 0; j < seg_; j++) {
      Poly poly;
      for (int k = 0; k <= deg_; k++) {
        boost::shared_ptr<StandardBasis> ptr =
            boost::static_pointer_cast<StandardBasis>(basis.getBasis(0));
        //                 poly +=
        //                 traj.at(d).at(j).at(k)->getVal()/std::pow(times.at(j)->getVal(),int(k/2))*ptr->getPoly(k);
        poly += traj.at(d).at(j).at(k)->getVal() *
                std::pow(times.at(j)->getVal(), k / 2) * ptr->getPoly(k);
      }
      for (int k = 0; k <= deg_; k++) {
        trajd.data.at(d).segs.at(j).coeffs.at(k) = poly[k];
      }
      //            std::cout << "Serialized poly " << poly << std::endl;
      trajd.data.at(d).segs.at(j).dt = times.at(j)->getVal();
      trajd.data.at(d).segs.at(j).basis = PolyType::ENDPOINT;
    }
  }
  return trajd;
}

std::ostream &operator<<(std::ostream &os, const SymbolicPoly &poly) {
  for (auto p : poly.poly_map) {
    Variable *coeff = std::get<0>(p.first);
    Variable *time = std::get<1>(p.first);
    int n = std::get<2>(p.first);
    os << p.second << "*c" << coeff->getId() << "*t" << time->getId() << "^"
       << n << std::endl;
  }
  for (auto &p : poly.quad_map) {
    Variable *coeff0 = std::get<0>(p.first);
    Variable *coeff1 = std::get<1>(p.first);
    Variable *time = std::get<2>(p.first);
    int n = std::get<3>(p.first);
    os << p.second << "*c" << coeff0->getId() << "*c" << coeff1->getId() << "*t"
       << time->getId() << "^" << n << std::endl;
  }
  return os;
}
// polycost
PolyCost::PolyCost(const NLTraj &traj, const NLTimes &times,
                   BasisBundlePro &basis, int min_dim) {
  init_constants();
  int dim = traj.size();
  int segs = times.size();
  int deg = traj.front().front().size();
  for (int d = 0; d < dim; d++) {
    for (int s = 0; s < segs; s++) {
      for (int i = 0; i < deg; i++) {
        for (int j = 0; j <= i; j++) {
          if (i == j)
            poly += SymbolicPoly(traj.at(d).at(s).at(i), traj.at(d).at(s).at(j),
                                 times.at(s), cost_n_(i, j), cost_v_(i, j));
          else  // add symetric elements properly
            poly +=
                SymbolicPoly(traj.at(d).at(s).at(i), traj.at(d).at(s).at(j),
                             times.at(s), cost_n_(i, j), 2.0 * cost_v_(i, j));
        }
      }
    }
  }
  //    std::cout << "Cost " << poly << std::endl;
}
decimal_t PolyCost::evaluate() { return poly.evaluate(); }
ETV PolyCost::gradient() { return poly.quad_gradient(); }
ETV PolyCost::hessian() { return poly.quad_hessian(); }

void NonlinearTrajectory::addCloudConstraint(const Vec3Vec &points) {
  for (int j = 0; j < seg_; j++) {
    for (int k = 0; k <= deg_; k++) {
      boost::shared_ptr<BallConstraint> con =
          boost::make_shared<BallConstraint>(this, j, k);
      solver.addConstraint(con);
    }
    for (auto &p : points) {
      boost::shared_ptr<BallConstraint> con =
          boost::make_shared<BallConstraint>(this, j, p,
                                             0.8);  // hardcode robot_r for now
      solver.addConstraint(con);
    }
  }
}
Vec4Vec NonlinearTrajectory::getBeads() {
  Vec4Vec res(seg_, Vec4::Zero());
  for (int i = 0; i < seg_; i++)
    for (int d = 0; d <= dim_; d++) res.at(i)(d) = beads.at(d).at(i)->getVal();
  return res;
}
void NonlinearTrajectory::scaleTime(decimal_t ratio) {
  for (auto t : times) t->val *= ratio;

  for (int d = 0; d < dim_; d++) {
    for (int j = 0; j < seg_; j++) {
      for (int k = 0; k <= deg_; k++) {
        traj.at(d).at(j).at(k)->val /= std::pow(ratio, k / 2);
      }
    }
  }
}
}  // namespace traj_opt
