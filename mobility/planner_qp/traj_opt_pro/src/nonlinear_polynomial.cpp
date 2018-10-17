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
#include <map>

namespace traj_opt {
SymbolicPoly::SymbolicPoly(Variable *coeff, Variable *time, int n,
                           decimal_t a) {
  PolyTriple p = PolyTriple(coeff, time, n);
  if (std::abs(a) > 1e-11) poly_map[p] = a;
}
SymbolicPoly::SymbolicPoly(Variable *coeff0, Variable *coeff1, Variable *time,
                           int n, decimal_t a) {
  PolyQuad p = PolyQuad(coeff0, coeff1, time, n);
  if (std::abs(a) > 1e-11) quad_map[p] = a;
}
void SymbolicPoly::add(const SymbolicPoly &rhs) {
  for (auto &p : rhs.poly_map) {
    auto it = poly_map.find(p.first);
    if (it == poly_map.end()) {
      poly_map[p.first] = p.second;
    } else {
      poly_map[p.first] = poly_map[p.first] + p.second;
    }
  }
  for (auto &p : rhs.quad_map) {
    auto it = quad_map.find(p.first);
    // check for oposite order
    PolyQuad back = PolyQuad(std::get<1>(p.first), std::get<0>(p.first),
                             std::get<2>(p.first), std::get<3>(p.first));
    auto it2 = quad_map.find(back);
    if (it == quad_map.end()) {
      quad_map[p.first] = p.second;
    } else if (it2 != quad_map.end()) {
      quad_map[back] = quad_map[back] + p.second;
    } else {
      quad_map[p.first] = quad_map[p.first] + p.second;
    }
  }
}
SymbolicPoly &SymbolicPoly::operator+=(const SymbolicPoly &rhs) {
  this->add(rhs);
  return *this;
}
SymbolicPoly operator+(SymbolicPoly lhs, const SymbolicPoly &rhs) {
  lhs += rhs;
  return lhs;
}
SymbolicPoly operator*(decimal_t lhs, const SymbolicPoly &rhs) {
  SymbolicPoly p;
  p.poly_map = rhs.poly_map;
  p.quad_map = rhs.quad_map;
  for (auto &pi : p.poly_map) p.poly_map[pi.first] *= lhs;
  for (auto &pi : p.quad_map) p.quad_map[pi.first] *= lhs;
  return p;
}
decimal_t SymbolicPoly::evaluate() {
  decimal_t val = 0.0;
  for (auto &p : poly_map) {
    Variable *coeff = std::get<0>(p.first);
    Variable *time = std::get<1>(p.first);
    int n = std::get<2>(p.first);
    decimal_t a = p.second;
    val += std::pow(time->getVal(), n) * coeff->getVal() * a;
  }
  for (auto &p : quad_map) {
    Variable *coeff0 = std::get<0>(p.first);
    Variable *coeff1 = std::get<1>(p.first);
    Variable *time = std::get<2>(p.first);
    int n = std::get<3>(p.first);
    decimal_t a = p.second;
    val +=
        std::pow(time->getVal(), n) * coeff0->getVal() * coeff1->getVal() * a;
  }
  //        std::cout << "evaluate " << val << std::endl;
  return val;
}
ETV SymbolicPoly::gradient(int u_id) {
  ETV grad;
  std::map<Variable *, decimal_t> time_grad;
  for (auto &p : poly_map) {
    Variable *coeff = std::get<0>(p.first);
    Variable *time = std::get<1>(p.first);
    int n = std::get<2>(p.first);

    decimal_t dt = decimal_t(n) * std::pow(time->getVal(), n - 1) *
                   coeff->getVal() * p.second;
    if (time_grad.find(time) == time_grad.end())
      time_grad[time] = dt;
    else
      time_grad[time] += dt;

    ET gi = ET(u_id, coeff->getId(), p.second * std::pow(time->getVal(), n));
    grad.push_back(gi);
  }
  for (auto &v : time_grad) {
    ET gt = ET(u_id, v.first->getId(), v.second);
    grad.push_back(gt);
  }
  return grad;
}
ETV SymbolicPoly::hessian() {
  std::map<Variable *, decimal_t> time_hess;
  ETV hess;
  // note for this form, all second derivatives are zero with respect to the
  // coeffiients
  for (auto &p : poly_map) {
    Variable *coeff = std::get<0>(p.first);
    Variable *time = std::get<1>(p.first);
    int n = std::get<2>(p.first);
    // if (n > 1) {
    decimal_t dt = decimal_t(n * (n - 1)) * std::pow(time->getVal(), n - 2) *
                   coeff->getVal() * p.second;
    if (time_hess.find(time) == time_hess.end())
      time_hess[time] = dt;
    else
      time_hess[time] += dt;
    // }
    // if (n > 0) {
    decimal_t c = decimal_t(n) * std::pow(time->getVal(), n - 1) *
                  coeff->getVal() * p.second;
    hess.push_back(ET(coeff->getId(), time->getId(), c));
    hess.push_back(ET(time->getId(), coeff->getId(), c));
    // }
  }
  // d^2g/dtdt
  for (auto &v : time_hess) {
    ET ht = ET(v.first->getId(), v.first->getId(), v.second);
    hess.push_back(ht);
  }
  return hess;
}
ETV SymbolicPoly::quad_gradient() {
  return NonlinearSolver::transpose(
      quad_gradient(0));  // for cost function, gradient is single column
}

ETV SymbolicPoly::quad_gradient(int u_id) {
  ETV grad;
  std::map<Variable *, decimal_t> time_grad;
  std::map<Variable *, decimal_t> coeff_grad;

  for (auto &p : quad_map) {
    Variable *coeff0 = std::get<0>(p.first);
    Variable *coeff1 = std::get<1>(p.first);
    Variable *time = std::get<2>(p.first);
    int n = std::get<3>(p.first);
    if (1) {
      decimal_t dt = decimal_t(n) * std::pow(time->getVal(), n - 1) *
                     coeff0->getVal() * coeff1->getVal() * p.second;
      if (time_grad.find(time) == time_grad.end())
        time_grad[time] = dt;
      else
        time_grad[time] += dt;
    }
    if (coeff0->getId() == coeff1->getId()) {
      if (coeff_grad.find(coeff0) == coeff_grad.end())
        coeff_grad[coeff0] =
            2.0 * coeff0->getVal() * p.second * std::pow(time->getVal(), n);
      else
        coeff_grad[coeff0] +=
            2.0 * coeff0->getVal() * p.second * std::pow(time->getVal(), n);

    } else {
      if (coeff_grad.find(coeff0) == coeff_grad.end())
        coeff_grad[coeff0] =
            coeff1->getVal() * p.second * std::pow(time->getVal(), n);
      else
        coeff_grad[coeff0] +=
            coeff1->getVal() * p.second * std::pow(time->getVal(), n);

      if (coeff_grad.find(coeff1) == coeff_grad.end())
        coeff_grad[coeff1] =
            coeff0->getVal() * p.second * std::pow(time->getVal(), n);
      else
        coeff_grad[coeff1] +=
            coeff0->getVal() * p.second * std::pow(time->getVal(), n);
    }
  }
  for (auto &v : coeff_grad) {
    ET gt = ET(u_id, v.first->getId(), v.second);
    grad.push_back(gt);
  }
  for (auto &v : time_grad) {
    ET gt = ET(u_id, v.first->getId(), v.second);
    grad.push_back(gt);
  }
  return grad;
}
ETV SymbolicPoly::quad_hessian() {
  ETV hess;
  std::map<Variable *, decimal_t> time_hess;
  for (auto &p : quad_map) {
    Variable *coeff0 = std::get<0>(p.first);
    Variable *coeff1 = std::get<1>(p.first);
    Variable *time = std::get<2>(p.first);
    int n = std::get<3>(p.first);
    // d^2/dt^2
    decimal_t dt = decimal_t(n - 1) * decimal_t(n) *
                   std::pow(time->getVal(), n - 2) * coeff0->getVal() *
                   coeff1->getVal() * p.second;
    if (time_hess.find(time) == time_hess.end())
      time_hess[time] = dt;
    else
      time_hess[time] += dt;
    // d^2/dc/dc
    if (coeff0->getId() == coeff1->getId()) {
      ET gi = ET(coeff0->getId(), coeff0->getId(),
                 2.0 * p.second * std::pow(time->getVal(), n));
      hess.push_back(gi);
      // d^2/{dtdc} has 2 terms
      ET gi2 = ET(coeff0->getId(), time->getId(),
                  2.0 * coeff1->getVal() * p.second * decimal_t(n) *
                      std::pow(time->getVal(), n - 1));
      ET gi3 = ET(time->getId(), coeff0->getId(),
                  2.0 * coeff1->getVal() * p.second * decimal_t(n) *
                      std::pow(time->getVal(), n - 1));
      hess.push_back(gi2);
      hess.push_back(gi3);

    } else {
      ET gi0 = ET(coeff0->getId(), coeff1->getId(),
                  p.second * std::pow(time->getVal(), n));
      ET gi1 = ET(coeff1->getId(), coeff0->getId(),
                  p.second * std::pow(time->getVal(), n));

      hess.push_back(gi0);
      hess.push_back(gi1);

      // 4 things  d^2/{dtdc}
      ET gi2 = ET(coeff0->getId(), time->getId(),
                  coeff1->getVal() * p.second * decimal_t(n) *
                      std::pow(time->getVal(), n - 1));
      ET gi3 = ET(time->getId(), coeff0->getId(),
                  coeff1->getVal() * p.second * decimal_t(n) *
                      std::pow(time->getVal(), n - 1));
      ET gi4 = ET(coeff1->getId(), time->getId(),
                  coeff0->getVal() * p.second * decimal_t(n) *
                      std::pow(time->getVal(), n - 1));
      ET gi5 = ET(time->getId(), coeff1->getId(),
                  coeff0->getVal() * p.second * decimal_t(n) *
                      std::pow(time->getVal(), n - 1));

      hess.push_back(gi2);
      hess.push_back(gi3);
      hess.push_back(gi4);
      hess.push_back(gi5);
    }
  }
  for (auto &v : time_hess) {
    ET gt = ET(v.first->getId(), v.first->getId(), v.second);
    hess.push_back(gt);
  }
  return hess;
}
SymbolicPoly SymbolicPoly::square() {
  SymbolicPoly resultant;
  // do the naive thing, can be speed up by factor of 2 with some intellegence.
  for (auto p1 : poly_map) {
    Variable *coeff1 = std::get<0>(p1.first);
    Variable *time1 = std::get<1>(p1.first);
    decimal_t a1 = p1.second;
    for (auto p2 : poly_map) {
      Variable *coeff2 = std::get<0>(p2.first);
      decimal_t a2 = p2.second;
      // function assumptions
      assert(time1 == std::get<1>(p2.first));
      assert(std::get<2>(p1.first) == 0);
      assert(std::get<2>(p2.first) == 0);
      resultant.add(SymbolicPoly(coeff1, coeff2, time1, 0, a1 * a2));
    }
  }
  return resultant;
}
}  // namespace traj_opt
