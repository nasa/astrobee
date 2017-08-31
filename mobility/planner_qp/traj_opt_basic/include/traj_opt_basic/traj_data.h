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

#ifndef TRAJ_OPT_BASIC_TRAJ_DATA_H_
#define TRAJ_OPT_BASIC_TRAJ_DATA_H_

#include <string>
#include <vector>

namespace traj_opt {

enum PolyType {
  STANDARD,
  LEGENDRE,
  WATTERSON,
  BEZIER,
  ENDPOINT,
  LIU,
  CHEBYSHEV
};

struct PolynomialData {
  int degree;
  float dt;
  PolyType basis;
  std::vector<float> coeffs;
  // polynomials are stored with parameterization s \in [0,1]
  // time duration dt is used to evaluate polynomial p(t/dt) for t \in [0,dt]

  // define with and without space allocating constructor
  PolynomialData() {}
  explicit PolynomialData(int d) : degree(d), coeffs(degree + 1) {}
};
struct SplineData {
  int segments;
  float t_total;
  // t_total should equal the sum of dt for each segment
  std::vector<PolynomialData> segs;

  // define with and without space allocating constructor
  SplineData() {}
  SplineData(int deg, int seg)
      : segments(seg), segs(segments, PolynomialData(deg)) {}
};

struct TrajData {
  int dimensions;
  std::vector<SplineData> data;
  std::vector<std::string> dimension_names;
  // dimension_names are optional, but are useful for determining what this
  // trajectory parametrizes
  // ex. dimension_names = {'x','y',z'}

  // define with and without space allocating constructor
  TrajData() {}
  TrajData(int dim, int segs, int deg)
      : dimensions(dim), data(dimensions, SplineData(deg, segs)) {}
};
}  // namespace traj_opt
#endif  // TRAJ_OPT_BASIC_TRAJ_DATA_H_
