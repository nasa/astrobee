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
#ifndef OPTIMIZATION_COMMON_OPTIMIZATION_PARAMS_H_
#define OPTIMIZATION_COMMON_OPTIMIZATION_PARAMS_H_

#include <ceres/solver.h>

namespace optimization_common {
struct OptimizationParams {
  ceres::Solver::Options solver_options;
  bool verbose;
  double huber_loss;
};
}  // namespace optimization_common

#endif  // OPTIMIZATION_COMMON_OPTIMIZATION_PARAMS_H_
