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
// move to cpp
#include <algorithm>
#include <map>
#include <utility>
#include <vector>

namespace traj_opt {

void NonlinearSolver::updateInfo(std::vector<Variable *> times) {
  solver_info.gap = duality();
  solver_info.cost = cost->evaluate();
  solver_info.iterations++;
  solver_info.slack.clear();
  for (auto in : ineq_con) solver_info.slack.push_back(in->evaluate());
  solver_info.gap_history.push_back(solver_info.gap);
  solver_info.cost_history.push_back(solver_info.cost);

  int num_v = eq_con.size();
  int num_u = ineq_con.size();
  int total_v = vars.size();
  int num_z = total_v - 2 * num_u - num_v;
  solver_info.var_history.push_back(std::vector<float>());
  solver_info.time_history.push_back(std::vector<float>());
  for (int i = 0; i < num_z; i++)
    solver_info.var_history.back().push_back(vars.at(i).val);
  for (auto &vi : times) solver_info.time_history.back().push_back(vi->val);
}
// outstreams
std::ostream &operator<<(std::ostream &os, const Variable &var) {
  os << "V" << var.id << " : " << var.val << " ";
  return os;
}
std::ostream &operator<<(std::ostream &os, const EqConstraint &var) {
  for (auto &p : var.coeff) os << *(p.first) << " * " << p.second << " + ";
  os << " = " << var.rhs;
  return os;
}
std::ostream &operator<<(std::ostream &os, const ET &trip) {
  os << trip.row() << " , " << trip.col() << " , " << trip.value();
  return os;
}

std::ostream &operator<<(std::ostream &os, const ETV &trip) {
  for (auto &t : trip) os << t << std::endl;
  return os;
}

// Eq solver helpers
ETV EqConstraint::ai() {
  ETV a;
  a.reserve(coeff.size());
  for (auto &p : coeff) a.push_back(ET(var_v->id, p.first->getId(), p.second));
  return a;
}
ET EqConstraint::bi() {
  decimal_t val = -rhs;
  for (auto &p : coeff) val += p.first->val * p.second;
  return ET(var_v->id, 0, val);
}
ETV EqConstraint::audio_video() {
  ETV av;
  for (auto &p : coeff) av.push_back(ET(p.first->id, 0, var_v->val * p.second));
  return av;
}
std::pair<ETV, ET> EqConstraint::get_presolve() {
  ETV a_i;
  ET b_i(id, 0, rhs);
  for (auto &c : coeff) a_i.push_back(ET(id, c.first->id, c.second));
  return std::pair<ETV, ET>(a_i, b_i);
}

// Ineq solver helpers
void IneqConstraint::update_slack() { var_s->val = -1.0 * evaluate(); }

ET IneqConstraint::slack() { return ET(var_u->id, 0, evaluate() + var_s->val); }
ET IneqConstraint::sports_util(decimal_t nu) {
  return ET(var_s->id, 0, var_s->val * var_u->val - nu);
}
ETV IneqConstraint::gradientS() {
  ETV grad = gradient();
  ETV gradS;
  gradS.reserve(grad.size());
  for (auto t : grad) gradS.push_back(ET(t.col(), 0, t.value() * var_u->val));
  return gradS;
}
decimal_t IneqConstraint::linesearch(const VecD &delta, decimal_t max_h) {
  max_h = 1.0;
  decimal_t ui = var_u->val;
  decimal_t dui = delta(var_u->id);
  if (dui > 1e-6) {  // negative dui is fine
    max_h = std::min(ui / dui, max_h);
  }
  decimal_t si = var_s->val;
  decimal_t dsi = delta(var_s->id);
  if (dsi > 1e-6) {  // negative dui is fine
    max_h = std::min(si / dsi, max_h);
  }

  //    if(max_h < 1e-13){
  //        std::cout << "Having trouble with constriant " << id << std::endl;
  //        std::cout << "u: " << ui  << " ,du: " << dui << std::endl;
  //        std::cout << "s: " << si  << " ,ds: " << dsi << std::endl;
  //        // hacky fix
  ////        max_h = 1.0;
  ////        if(dui > 1e-6)
  ////            var_u->val += dui;
  ////        if(dsi > 1e-6)
  ////            var_s->val += dsi;
  //    }

  return max_h;
}

// Nonlinear solver
void NonlinearSolver::addConstraint(boost::shared_ptr<EqConstraint> con) {
  con->id = static_cast<int>(eq_con.size());
  con->var_v = addVar();
  eq_con.push_back(con);
}
void NonlinearSolver::addConstraint(boost::shared_ptr<IneqConstraint> con) {
  con->id = static_cast<int>(ineq_con.size());
  con->var_u = addVar();
  con->var_s = addVar();
  //    con->var_s->val = - con->evaluate();
  ineq_con.push_back(con);
}
void NonlinearSolver::addConstraint(std::vector<EqConstraint::EqPair> con,
                                    decimal_t rhs) {
  boost::shared_ptr<EqConstraint> c = boost::make_shared<EqConstraint>();
  c->coeff = con;
  c->rhs = rhs;
  addConstraint(c);
}

// transposes sparse triple
ETV NonlinearSolver::transpose(const ETV &vec) {
  ETV res;
  res.reserve(vec.size());
  for (auto &v : vec) {
    res.push_back(ET(v.col(), v.row(), v.value()));
  }
  return res;
}
decimal_t NonlinearSolver::duality() {
  decimal_t gap = 0.0;
  int num_u = ineq_con.size();
  for (auto &con : ineq_con) gap += con->var_u->val * con->var_s->val;
  gap /= decimal_t(num_u);
  return gap;
}

bool NonlinearSolver::iterate() {
  // get variable sizes
  // int num_v = eq_con.size();
  int num_u = ineq_con.size();
  int total_v = vars.size();
  // int num_z = total_v - 2*num_u - num_v;

  SpMat M(total_v, total_v);
  SpMat b(total_v, 1);
  ETV coeffs;
  ETV bcoeffs;

  VecD nu = VecD::Zero(num_u);
  // add A
  ETV A;
  for (auto &eq : eq_con) {
    ETV ai = eq->ai();
    //        for(auto &aii:ai)
    //            std::cout << aii.row() << " , " << aii.col() << " : " <<
    //            aii.value() << std::endl;
    //        std::cout << "rhs " << eq->rhs << std::endl;
    A.insert(A.end(), ai.begin(), ai.end());
    bcoeffs.push_back(eq->bi());
  }
  ETV AT = transpose(A);
  coeffs.insert(coeffs.end(), A.begin(), A.end());
  coeffs.insert(coeffs.end(), AT.begin(), AT.end());

  // add G
  ETV G;
  for (auto &ineq : ineq_con) {
    ETV Gi = ineq->gradient();
    G.insert(G.end(), Gi.begin(), Gi.end());
    bcoeffs.push_back(ineq->slack());
    bcoeffs.push_back(ineq->sports_util(nu(ineq->id)));
    // S,I and Z
    coeffs.push_back(
        ET(ineq->var_s->id, ineq->var_u->id, ineq->var_s->val));  // S
    coeffs.push_back(ET(ineq->var_u->id, ineq->var_s->id, 1.0));  // I
    coeffs.push_back(
        ET(ineq->var_s->id, ineq->var_s->id, ineq->var_u->val));  // Z
  }
  ETV GT = transpose(G);
  coeffs.insert(coeffs.end(), G.begin(), G.end());
  coeffs.insert(coeffs.end(), GT.begin(), GT.end());

  // add Cost

  // add to left hand side

  ETV co = cost->hessian();
  coeffs.insert(coeffs.end(), co.begin(), co.end());
  ETV na = cost->gradient();
  bcoeffs.insert(bcoeffs.end(), na.begin(), na.end());
  // end Add Cost

  // add tensor sum terms
  for (auto &ineq : ineq_con) {
    ETV coi = ineq->hessian();
    ETV gu = ineq->gradientS();

    coeffs.insert(coeffs.end(), coi.begin(), coi.end());
    bcoeffs.insert(bcoeffs.end(), gu.begin(), gu.end());
  }
  for (auto &eq : eq_con) {
    ETV av = eq->audio_video();
    bcoeffs.insert(bcoeffs.end(), av.begin(), av.end());
  }

  // pack non summands
  M.setFromTriplets(coeffs.begin(), coeffs.end());
  b.setFromTriplets(bcoeffs.begin(), bcoeffs.end());

  // pass to backend

  //    std::cout << "Frontend took " << tm.toc() << std::endl;
  //    tm.tic();

  // backend bottleneck
  Eigen::SparseLU<SpMat> solver;
  //    Eigen::SparseQR<SpMat,Eigen::COLAMDOrdering<int> > solver;
  //    Eigen::PardisoLU<SpMat> solver;

  //        std::cout << "M: " << M << std::endl;
  //        std::cout << "b: " << b << std::endl;
  //    draw_matrix(M);
  solver.compute(M);
  //    std::cout << "Back end took " << tm.toc() << std::endl;

  if (solver.info() != Eigen::Success) {
    std::cout << "Back end failed" << std::endl;
    //        std::cout << M << std::endl;
    return false;
  }
  VecD delta_x = solver.solve(b);

  //    VecD err = M*delta_x;
  //    err-=b;
  //    std::cout << "err: " << err.transpose() << std::endl;

  // Check error

  //        std::cout << "delta x aff " << delta_x.transpose() << std::endl;

  // do feasible direction search
  //        MatD x = MatD::Zero(total_v,1);
  //        for(uint i =0 ; i <vars.size();i++)
  //            x(i) = vars.at(i)->val;

  // calculate max distance
  decimal_t max_h = 1.0;

  for (auto &con : ineq_con) {
    decimal_t max_i = con->linesearch(delta_x, max_h);
    max_h = std::min(max_i, max_h);
  }
  //    max_h = std::min(max_h,cost_linesearch(delta_x));
  max_h = std::max(max_h, 0.0);
  //    std::cout << "h aff: " << max_h << std::endl;
  // update state or compute affine

  // calculate nu
  decimal_t mu = duality();
  decimal_t mu_aff = 0.0;
  for (auto &con : ineq_con) {
    decimal_t new_s = con->var_s->val - max_h * delta_x(con->var_s->id);
    decimal_t new_u = con->var_u->val - max_h * delta_x(con->var_u->id);
    if (new_s > 0 && new_u > 0) mu_aff += new_s * new_u;
  }
  mu_aff /= decimal_t(ineq_con.size());
  //    std::cout << "mu: " << mu << std::endl;
  // centering param
  decimal_t sigma = std::pow(mu_aff / mu, 3);
  centering_ = sigma;  // save this fime optimization

  //    std::cout << "Sigma " << sigma << std::endl;

  nu = mu * sigma * VecD::Ones(num_u);
  for (uint i = 0; i < ineq_con.size(); i++)
    nu(i) -=
        delta_x(ineq_con.at(i)->var_u->id) * delta_x(ineq_con.at(i)->var_s->id);

  // calculate state update

  // update b
  for (auto &ineq : ineq_con) {
    ET suv = ineq->sports_util(nu(ineq->id));
    b.coeffRef(suv.row(), suv.col()) = suv.value();
  }
  //     std::cout << "updated b: " << b << std::endl;
  delta_x = solver.solve(b);

  // redo line search
  max_h = 1.0;
  for (auto &con : ineq_con) {
    decimal_t max_i = con->linesearch(delta_x, max_h);
    max_h = std::min(max_i, max_h);
    if (max_i == 0)
      std::cout << "out of slack for ineq " << con->id << std::endl;
    //         std::cout << "max_i: " << max_i << std::endl;
  }

  //     // check duality gap reduction
  //     decimal_t ad=0.0,bd=0.0;

  //     for(auto &con:ineq_con){
  //         ad += delta_x(con->var_u->id)*delta_x(con->var_s->id);
  //         bd += delta_x(con->var_u->id)*con->var_s->val;
  //         bd += delta_x(con->var_s->id)*con->var_u->val;
  //     }
  //     std::cout << "duality diff: h ( h * " <<ad << " - " << bd << " )" <<
  //     std::endl;

  //     max_h = std::min(max_h,cost_linesearch(delta_x));
  max_h = std::max(max_h, 0.0);
  max_h *= 0.99;
  //     VecD delta_sub = delta_x.block(0,0,num_z,1);
  //     VecD delta_v = delta_x.block(num_z,0,num_v,1);
  //     VecD delta_us = delta_x.block(num_v+num_z,0,2*num_u,1);

  //      std::cout << "delta_x  " << delta_sub.transpose() << std::endl;
  //      std::cout << "delta_v  " << delta_v.transpose() << std::endl;
  //      std::cout << "delta_us  " << delta_us.transpose() << std::endl;
  //     std::cout << "expected cost diff: " << delta_x.sum()*max_h <<
  //     std::endl;

  // update variables
  for (int i = 0; i < total_v; i++)
    vars.at(i).val -= delta_x(vars.at(i).id) * max_h;
  // robustify against bad search directions
  //     for(uint i =num_z+num_v ; i <total_v;i++){
  //         decimal_t dx = delta_x(vars.at(i).id)*max_h;
  //         if(vars.at(i).val - dx > 0)
  //             vars.at(i).val -= dx;
  //     }

  // central path push
  //     for(auto &con:ineq_con){
  //         con->update_slack();
  //     }
  // check duality gap
  // decimal_t epsilon = 1e-6;
  mu = duality();
  //     std::cout << "max_h: " << max_h << std::endl;
  //     std::cout << "mu: " << mu << std::endl;

  updateInfo();
  return (mu > epsilon_) && (max_h > 1e-15);
}

inline bool contains(const std::map<int, int> &map, int key) {
  auto val = map.find(key);
  return val != map.end();
}
bool NonlinearSolver::iterate_time(std::vector<Variable *> times) {
  decimal_t alpha = 10;  // richer, roy magic parameter maybe pass in

  // check input
  if (times.size() == 0) return true;
  uint num_t = times.size();
  // get gradient + hession
  ETV hess = cost->hessian();
  ETV grad = cost->gradient();

  ETV hess_time;
  hess_time.reserve(hess.size());
  ETV grad_time;
  grad_time.reserve(grad.size());

  // get dimension reduced version
  std::map<int, int> time_inds;
  int i = 0;
  for (auto &v : times) time_inds[v->id] = i++;

  for (auto &triple : hess)
    if (contains(time_inds, triple.row()) && contains(time_inds, triple.col()))
      hess_time.push_back(
          ET(time_inds[triple.row()], time_inds[triple.col()], triple.value()));

  for (auto &triple : grad)
    if (contains(time_inds, triple.row()))
      grad_time.push_back(
          ET(time_inds[triple.row()], triple.col(), triple.value()));

  for (uint c = 0; c < num_t; c++) grad_time.push_back(ET(c, 0, alpha));

  SpMat AE(num_t, num_t);
  SpMat bE(num_t, 1);
  AE.setFromTriplets(hess_time.begin(), hess_time.end());
  bE.setFromTriplets(grad_time.begin(), grad_time.end());

  // solver and iterate
  Eigen::SparseLU<SpMat> solver;
  solver.compute(AE);
  if (solver.info() != Eigen::Success) {
    std::cout << "Time opt back end failed" << std::endl;
    std::cout << "AE " << AE << std::endl;
    std::cout << "bE " << bE << std::endl;
    for (auto &t : times) std::cout << "time id " << t->id << std::endl;

    return false;
  }
  VecD delta_x = solver.solve(bE);

  decimal_t h = 1.0;
  // do line search
  for (int r = 0; r < delta_x.rows(); r++)
    if (delta_x(r) > 0) h = std::min(h, 1.0 / delta_x(r));

  // gamma
  h *= 0.99;

  for (int r = 0; r < delta_x.rows(); r++) times.at(r)->val -= h * delta_x(r);

  // find and adjust equality constraints

  for (auto &eq : eq_con) {
    for (auto &v : eq->coeff) {
      if (contains(time_inds, v.first->id)) {
        ET diff = eq->bi();
        eq->rhs += diff.value();
      }
    }
  }

  return true;
}

bool NonlinearSolver::solve(bool verbose, decimal_t epsilon,
                            std::vector<Variable *> times, int max_iterations) {
  epsilon_ = epsilon;

  decimal_t nu = 0.5;

  int num_v = eq_con.size();
  int num_u = ineq_con.size();
  int total_v = vars.size();
  int num_z = total_v - 2 * num_u - num_v;
  Timer tm;
  Timer tm_t;
  tm_t.tic();

  if (verbose) {
    std::cout << "Solving model with " << total_v << " variables" << std::endl;
    std::cout << "Primary: " << num_z << std::endl;
    std::cout << "Equality: " << num_v << std::endl;
    std::cout << "Inequality: " << num_u << std::endl << std::endl;
    std::cout << "Nu: " << nu << std::endl;
    std::cout << "Max Iterations: " << max_iterations << std::endl << std::endl;
  }
  //    if(!presolved_)
  //        presolve();
  //        specialized_presolve();
  // decimal_t old_cost = cost->evaluate();
  VecD old_x = VecD::Zero(num_z);
  for (int i = 0; i < num_z; i++) old_x(i) = vars.at(i).val;

  // bool costreg=false;
  int its = 0;
  for (int i = 0; i < max_iterations; i++) {
    its = i;
    if (verbose) {
      //            std::cout << "Starting iteration " << i << std::endl;
      //            std::cout << "x: ";
      //            for(int i=0;i<num_z;i++)
      //                std::cout << vars.at(i).val << " ";
      //            std::cout << std::endl;
      //            std::cout << "v: ";
      //            for(auto &v:eq_con)
      //                std::cout << v->var_v->val << " ";
      //            std::cout << std::endl;
      //            std::cout << "u: ";
      //            for(auto &v:ineq_con)
      //                std::cout << v->var_u->val << " ";
      //            std::cout << std::endl;
      //            std::cout << "s: ";
      //            for(auto &v:ineq_con)
      //                std::cout << v->var_s->val << " ";
      //            std::cout << std::endl;
      //            std::cout << "Cost: " << cost->evaluate() << std::endl;
      //            tm.tic();
    }
    if (!iterate()) break;
    if (centering_ < 0.1)  // watterson parameter, this is new and novel!
      iterate_time(times);

    // check cost regression
    //        decimal_t new_cost = cost->evaluate();

    //        std::cout << "Cost diff: " << old_cost - new_cost << std::endl;
    //        if(old_cost <= new_cost){
    //            for(int i=0;i<num_z;i++)
    //               vars.at(i).val = old_x(i);
    //            costreg = true;
    ////            std::cout << "new cost" << new_cost << std::endl;
    ////            std::cout << "old cost" << old_cost << std::endl;
    //            break;
    //        } else{
    //            for(int i=0;i<num_z;i++)
    //                old_x(i) = vars.at(i).val;
    //            old_cost = new_cost;
    //        }

    //        if(verbose)
    //            std::cout << "Iteration took: " << tm.toc() << std::endl;
  }
  decimal_t mu = duality();
  if (verbose) {
    std::cout << "Solver terminated." << std::endl;

    std::cout << "Cost: " << cost->evaluate() << std::endl;
    std::cout << "Gap: " << mu << std::endl;
    std::cout << "Iterations: " << its << std::endl;
  }
  if (verbose)
    std::cout << "Total time: " << tm_t.toc() * 1000.0 << " ms." << std::endl;
  return mu <= epsilon_;
}
bool NonlinearSolver::specialized_presolve() {
  // int num_v = eq_con.size();
  // int num_u = ineq_con.size();
  // int total_v = vars.size();
  // int num_z = total_v - 2*num_u - num_v;
  for (auto &con : eq_con) {
    if (con->coeff.size() == 1) {
      con->coeff.front().first->val = con->rhs / con->coeff.front().second;
    } else if (con->coeff.size() == 2) {
      con->coeff.front().first->val =
          (con->rhs - con->coeff.back().first->val * con->coeff.back().second) /
          con->coeff.front().second;
    } else {
      std::cout << "Problem is incompatible with specialized presolve"
                << std::endl;
      return false;
    }
  }

  // add initial slack
  //    for(auto &con:ineq_con) {
  //        con->update_slack();
  //        con->var_u->val = 1.0/con->var_s->val;
  //    }

  //    for(auto&con:eq_con)
  //        std::cout << *con <<std::endl;

  return true;
}

bool NonlinearSolver::presolve() {
  int num_v = eq_con.size();
  int num_u = ineq_con.size();
  int total_v = vars.size();
  int num_z = total_v - 2 * num_u - num_v;
  ETV A, b;
  // add stuff
  for (auto &con : eq_con) {
    std::pair<ETV, ET> pa = con->get_presolve();
    A.insert(A.end(), pa.first.begin(), pa.first.end());
    b.push_back(pa.second);
  }
  SpMat AE(num_v, num_z);
  SpMat bE(num_v, 1);
  AE.setFromTriplets(A.begin(), A.end());
  bE.setFromTriplets(b.begin(), b.end());

  SpMat ATA = AE * AE.transpose();
  SpMat ATb = bE;

  Eigen::SimplicialLLT<SpMat> solver;

  //    std::cout << "M: " << ATA << std::endl;
  //    std::cout << "b: " << ATb << std::endl;

  solver.compute(ATA);
  MatD z_init = AE.transpose() * solver.solve(ATb);
  //    std::cout << "z_init: " << z_init.transpose() << std::endl;

  // add solved variable values
  for (int i = 0; i < num_z; i++) vars.at(i).val = z_init(i);

  // add initial slack
  for (auto &con : ineq_con) con->update_slack();

  for (auto &con : eq_con) std::cout << *con << std::endl;

  return true;
}
decimal_t NonlinearSolver::cost_linesearch(const VecD &delta) {
  std::vector<decimal_t> old_vars;
  for (auto &v : vars) old_vars.push_back(v.val);
  decimal_t old_cost = cost->evaluate();
  decimal_t max_h = 1.0;
  int max_bisects = 50;

  for (int i = 0; i < max_bisects; i++) {
    for (uint j = 0; j < vars.size(); j++) {
      vars.at(j).val = old_vars.at(j) - max_h * delta(j);
    }
    if (cost->evaluate() < old_cost)
      break;
    else
      max_h /= 2.0;
  }
  if (cost->evaluate() >= old_cost) max_h = 0.0;

  // return iterate
  for (uint j = 0; j < vars.size(); j++) {
    vars.at(j).val = old_vars.at(j);
  }
  return max_h;
}
void NonlinearSolver::draw_matrix(const SpMat &mat) {
  int zoom = 5;
  cv::Mat img = cv::Mat(cv::Size(zoom * mat.rows(), zoom * mat.cols()), CV_8U);
  for (int k = 0; k < mat.outerSize(); ++k)
    for (SpMat::InnerIterator it(mat, k); it; ++it)
      for (int zi = 0; zi < zoom; zi++)
        for (int zj = 0; zj < zoom; zj++) {
          if (it.value() > 1e-7)
            img.at<unsigned char>(zoom * it.row() + zi, zoom * it.col() + zj) =
                255;
          else if (it.value() < -1e-7)
            img.at<unsigned char>(zoom * it.row() + zi, zoom * it.col() + zj) =
                128;
          else
            img.at<unsigned char>(zoom * it.row() + zi, zoom * it.col() + zj) =
                0;
        }
  cv::imshow("Matrix", img);
  cv::waitKey(-1);
}
}  // namespace traj_opt
