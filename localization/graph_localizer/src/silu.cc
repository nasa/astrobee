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

#include <graph_localizer/silu.h>

#include <cmath>

namespace graph_localizer {
double Silu(const double x, gtsam::OptionalJacobian<1, 1> d_silu_d_x) {
  const double silu_x = x / (1.0 + std::exp(-1.0 * x));
  if (d_silu_d_x) {
    *d_silu_d_x << (1.0 + std::exp(-1.0 * x) + x * std::exp(-1.0 * x)) / std::pow(1 + std::exp(-1.0 * x), 2);
  }
  return silu_x;
}
}  // namespace graph_localizer
