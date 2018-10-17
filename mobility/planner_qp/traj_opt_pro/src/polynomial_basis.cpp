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

// Implementation File
// Look at polynomial_basis.h for documentation
#include <traj_opt_pro/polynomial_basis.h>
#include <boost/math/special_functions/factorials.hpp>
#include <boost/pointer_cast.hpp>
#include <boost/range/irange.hpp>

#include <iostream>
#include <stdexcept>
#include <vector>

namespace traj_opt {

Poly PolyCalculusPro::bernstein_polynomial(typename Poly::size_type n,
                                           typename Poly::size_type i) {
  std::vector<decimal_t> t_dat({0, 1});
  Poly t(t_dat.data(), 1);  // t
  std::vector<decimal_t> omt_dat({1, -1});
  Poly omt(omt_dat.data(), 1);  // 1-t

  Poly t_rasied_i(omt_dat.data(), 0);  // t to the ith power
  Poly omt_rasied_ni(t_rasied_i);      // (1-t) to the (n-i)th power

  for (uint c = 0; c < i; c++) t_rasied_i *= t;
  for (uint c = 0; c < n - i; c++) omt_rasied_ni *= omt;

  Poly result(t_rasied_i);
  result *= omt_rasied_ni;
  result *= boost::math::binomial_coefficient<decimal_t>(n, i);
  return result;
}
Poly PolyCalculusPro::chebyshev_polynomial(typename Poly::size_type n) {
  std::vector<decimal_t> omt_dat({1, -1});
  Poly omt(omt_dat.data(), 1);  //  1 - t
  decimal_t one = 1;
  decimal_t zero = 0;

  Poly omt_raised_k(&one, 0);  // (1-t) to the kth power
  if (n == 0) return omt_raised_k;

  Poly result(&zero, 0);

  for (uint k = 0; k <= n; k++) {
    decimal_t coeff = std::pow(-2.0, k) * decimal_t(n) / decimal_t(n + k);
    coeff *= boost::math::binomial_coefficient<decimal_t>(n + k, 2 * k);
    // std::cout << "sub chevy " << coeff*omt_raised_k << std::endl;
    result += coeff * omt_raised_k;
    omt_raised_k *= omt;
  }

  return result;
}
Poly PolyCalculusPro::shifted_legendre(typename Poly::size_type n) {
  // shifted legensdre polynomial of order n
  typename std::vector<decimal_t> v;
  for (typename Poly::size_type k = 0; k <= n; k++) {
    v.push_back(boost::math::binomial_coefficient<decimal_t>(n, k) *
                boost::math::binomial_coefficient<decimal_t>(n + k, k) *
                std::pow(-1.0, k + n));
  }
  Poly result(v.data(), n);
  return result;
}

uint Basis::dim() { return n_p; }

// // switched these constructors to use new generic one
// BasisBundle::BasisBundle(uint n_p_, uint k_r_)
//     : BasisBundle(LEGENDRE, n_p_, k_r_) {}
// BasisBundle::BasisBundle(int n) : BasisBundle(BEZIER, n, 0) {}

LegendreBasis::LegendreBasis(uint n_p_, uint k_r_) : StandardBasis(0) {
  orthogonal_ = true;
  n_p = n_p_;
  k_r = k_r_;
  std::vector<decimal_t> simple;
  simple.push_back(1.0);
  for (uint i = 0; i < k_r; i++) {
    Poly poly(simple.data(), simple.size() - 1);
    polys.push_back(poly);
    simple.back() = 0.0;
    simple.push_back(1.0);
  }
  // does this line break ooqp? yes it does
  if (n_p == k_r) return;
  for (uint p = 0; p <= n_p - k_r; p++) {
    Poly p_cur = PolyCalculusPro::shifted_legendre(p);
    // std::cout << "shifted p " << p_cur << std::endl;
    for (uint i = 0; i < k_r; i++) {
      p_cur = PolyCalculus::integrate(p_cur);
    }
    polys.push_back(p_cur);
  }
}

BezierBasis::BezierBasis(uint n_p_) : StandardBasis(0) {
  n_p = n_p_;  // this was commented out, why?
  type_ = PolyType::BEZIER;
  for (int i = 0; i <= static_cast<int>(n_p); i++)
    polys.push_back(PolyCalculusPro::bernstein_polynomial(n_p, i));
}
ChebyshevBasis::ChebyshevBasis(uint n_p_) : StandardBasis(0) {
  n_p = n_p_;
  type_ = PolyType::CHEBYSHEV;
  std::vector<decimal_t> simple;
  simple.push_back(1.0);
  for (uint i = 0; i <= n_p; i++) {
    polys.push_back(PolyCalculusPro::chebyshev_polynomial(i));
  }

  // std::cout << "Chebyshev "  << *this << std::endl;
}
EndPointBasis::EndPointBasis(uint n_p_) : StandardBasis(0) {
  n_p = n_p_;  // this was commented out, why?
  type_ = PolyType::ENDPOINT;
  traj_opt::MatD coeffs = MatD::Zero(n_p_ + 1, n_p + 1);
  for (int i = 0; i <= static_cast<int>(n_p); i++) {
    if (i % 2 == 0) {
      coeffs(i, i / 2) = boost::math::factorial<decimal_t>(i / 2);

    } else {
      for (int j = 0; j <= static_cast<int>(n_p - i / 2); j++) {
        if (i < 2)
          coeffs(i, j) = 1;
        else
          coeffs(i, j + i / 2) = coeffs(i - 2, j + i / 2) * (j + 1);
      }
    }
  }
  traj_opt::MatD coeffsi = coeffs.inverse();

  for (int i = 0; i <= static_cast<int>(n_p); i++) {
    std::vector<decimal_t> data;
    for (int j = 0; j <= static_cast<int>(n_p); j++) {
      //      data.push_back(coeffsi(i, j));
      if (std::abs(coeffsi(j, i)) > 1e-12) data.push_back(coeffsi(j, i));
      //            data.push_back(coeffsi(i, j));
      else
        data.push_back(0.0);
    }
    polys.push_back(Poly(data.data(), n_p));
  }
  //  std::cout << "Endpoint M " << coeffs << std::endl;
  //  std::cout << "Endpoint Minv " << coeffsi << std::endl;
  //  std::cout << "Basis " << *this << std::endl;

  //  std::cout << "Endpoint np " << n_p << " poly size " << polys.size()
  //  <<std::endl;
}

decimal_t LegendreBasis::innerproduct(uint i, uint j) const {
  if (i != j)
    return 0;
  else
    return StandardBasis::innerproduct(i, j);
}
Poly StandardBasis::getPoly(uint i) const { return polys.at(i); }
StandardBasis::StandardBasis(uint n) : Basis(n) {
  type_ = PolyType::STANDARD;
  if (n == 0) return;
  std::vector<decimal_t> simple;
  simple.push_back(1.0);
  for (uint i = 0; i <= n_p; i++) {
    Poly poly(simple.data(), simple.size() - 1);
    polys.push_back(poly);
    simple.back() = 0.0;
    simple.push_back(1.0);
  }
}
BasisTransformer::BasisTransformer(boost::shared_ptr<StandardBasis> from,
                                   int derr) {
  int n = from->dim();
  boost::shared_ptr<StandardBasis> to = boost::make_shared<StandardBasis>(n);
  *this = BasisTransformer(from, to, derr);
}
BasisTransformer::BasisTransformer(boost::shared_ptr<StandardBasis> from,
                                   boost::shared_ptr<StandardBasis> to,
                                   int derr)
    : to_(to), from_(from) {
  //  assert(to_->dim() == from_->dim());
  n = static_cast<int>(from_->dim()) + 1;
  A = MatD::Zero(n - derr, n);
  //  std::cout << "Rows " << A.rows() << " , " << A.cols() << std::endl;
  for (int i = 0; i < n - derr; i++) {
    Poly pi = from_->getPoly(i + derr);
    for (int j = 0; j < static_cast<int>(pi.size()); j++) {
      //      std::cout << "i,j " << i << " , " << j << std::endl;
      A(i, j + derr) = pi[j];
    }
  }
  B = MatD::Zero(n - derr, n - derr);
  for (int i = 0; i < n - derr; i++) {
    Poly qi = to_->getPoly(i);
    for (int j = 0; j < static_cast<int>(qi.size()); j++) {
      //      std::cout << "i,j " << i << " , " << j << std::endl;
      B(i, j) = qi[j];
    }
  }
  //  Ainv = A.inverse();
  Binv = B.inverse();
  // draw your communitive diagram to see where these come from
  //  basisbasis_ = Binv * A;
  basisbasis_ = (A * Binv).transpose();

  //   std::cout << "Debug A: " << A << std::endl;
  //   std::cout << "Debug B: " << B << std::endl;
  //   std::cout << "Debug Ainv: " << Ainv << std::endl;
  //   std::cout << "Debug Binv: " << Binv << std::endl;
  //   std::cout << "BB " << basisbasis_ << std::endl;
}
const MatD &BasisTransformer::getBasisBasisTransform() {
  //  std::cout << "BB " << basisbasis_ << std::endl;
  return basisbasis_;
}
const MatD &BasisTransformer::getLinearTransform(decimal_t a, decimal_t b) {
  decimal_t data[2];
  data[0] = b;
  data[1] = a;  // remember boost's convention is backward from matlabs
  Poly fac(data, 1);
  decimal_t one = 1;
  Poly base(&one, 0);
  MatD mat = MatD::Zero(n, n);
  for (int i = 0; i < n; i++) {
    for (int j = 0; j < static_cast<int>(base.size()); j++) {
      mat(i, j) = base[j];
    }
    base = base * fac;
  }

  //  std::cout << "Debug mat: " << mat << std::endl;
  scaledtranform_ = Ainv * mat * A;  // more communitive diagrams
  return scaledtranform_;
}
boost::shared_ptr<Basis> BasisBundle::getBasis(int i) {
  if (i >= 0)
    return derrivatives.at(i);
  else
    return integrals.at(-i - 1);
}
BasisBundlePro::BasisBundlePro(PolyType type, uint n_p_, uint k_r_) {
  n_p = n_p_;
  k_r = k_r_;
  // only computer the first 4 derrivatives
  derrivatives.reserve(10);
  // computer the integral too, preferable use i7 to do computering
  integrals.reserve(1);

  for (auto i : boost::irange(0, 11)) {
    boost::shared_ptr<Basis> base;
    //    std::cout << "np " << n_p << std::endl;
    //    std::cout << "np2 " << n_p_ << std::endl;

    if (type == LEGENDRE)
      base = boost::make_shared<LegendreBasis>(n_p, k_r);
    else if (type == STANDARD)
      base = boost::make_shared<StandardBasis>(n_p);
    else if (type == BEZIER)
      base = boost::make_shared<BezierBasis>(n_p);
    else if (type == ENDPOINT)
      base = boost::make_shared<EndPointBasis>(n_p);
    else if (type == CHEBYSHEV)
      base = boost::make_shared<ChebyshevBasis>(n_p);
    else
      throw std::runtime_error("Unknown basis type");

    if (i == 11) {
      base->integrate();
      integrals.push_back(base);
    } else {
      for (int j = 0; j < i; j++) base->differentiate();
      derrivatives.push_back(base);
    }
  }
}
}  // namespace traj_opt
