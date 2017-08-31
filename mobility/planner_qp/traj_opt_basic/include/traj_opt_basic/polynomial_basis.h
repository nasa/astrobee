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

#ifndef TRAJ_OPT_BASIC_POLYNOMIAL_BASIS_H_
#define TRAJ_OPT_BASIC_POLYNOMIAL_BASIS_H_

#include <traj_opt_basic/traj_data.h>
#include <traj_opt_basic/types.h>

#include <boost/make_shared.hpp>
#include <boost/math/special_functions/binomial.hpp>
#include <boost/math/tools/polynomial.hpp>
#include <cassert>
#include <cmath>
#include <memory>
#include <vector>

namespace traj_opt {

// Basis virtual class
// //////////////////////////////////////////////////////////////////////////
class Basis {
 public:
  explicit Basis(uint n_p_);  // dimension of basis
  virtual void differentiate() = 0;
  virtual void integrate() = 0;

  // evaluates ith element of basis at x where x is normalized to 0 to 1
  virtual decimal_t evaluate(decimal_t x, uint i) const = 0;
  // innerproduct bewtween ith and jth basis functions
  virtual decimal_t innerproduct(uint i, uint j) const = 0;

  virtual uint dim();
  const PolyType &type() const { return type_; }
  bool orthogonal() { return orthogonal_; }

 protected:
  uint n_p;
  PolyType type_;
  bool orthogonal_{false};
};

// Poly Calculus Functions
// //////////////////////////////////////////////////////////////////////////

typedef boost::math::tools::polynomial<decimal_t> Poly;

class PolyCalculus {
 public:
  static Poly integrate(const Poly &p);
  // indefinate integral of polynomial with constant 0
  static Poly differentiate(const Poly &p);
  // differentiates polynomial
};

class BasisBundle {  // bundles the basis with its derrivatives
 public:
  // constructs using arbitrary basis
  BasisBundle(PolyType type, uint n_p_, uint k_r_);
  // constructs using legendre
  BasisBundle(uint n_p_, uint k_r_);
  // constructs using bezier
  explicit BasisBundle(int n);
  BasisBundle() {}

  //  ~BasisBundle();
  decimal_t getVal(
      decimal_t x, decimal_t dt, uint coeff,
      int derr) const;  // returns value of basis at value x, with time
                        // dt, basis function coeff, and derrivative
                        // derr
  boost::shared_ptr<Basis> getBasis(int i);

  std::vector<boost::shared_ptr<Basis>> derrivatives;
  std::vector<boost::shared_ptr<Basis>> integrals;

 protected:
  uint n_p, k_r;

  //  std::vector<LegendreBasis> derrivatives;
};

// Polynomial Bases
// A trig basis could be implemented, but we can approximate sin and cos with
// a Lagrange error bound of 1e-6 with a 7th order polynomial over the input
// domain 0 to pi/4.
// //////////////////////////////////////////////////////////////////////////

// 1, t, t^2, ...
class StandardBasis : public Basis {
 public:
  explicit StandardBasis(uint n_p_);
  virtual void differentiate();
  virtual void integrate();
  virtual decimal_t evaluate(decimal_t x, uint coeff) const;
  friend std::ostream &operator<<(std::ostream &os, const StandardBasis &lb);
  virtual decimal_t innerproduct(uint i, uint j) const;
  virtual Poly getPoly(uint i) const;

 protected:
  std::vector<Poly> polys;  // basis polynomials
  uint k_r;
};

}  // namespace traj_opt
#endif  // TRAJ_OPT_BASIC_POLYNOMIAL_BASIS_H_
