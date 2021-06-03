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
#ifndef GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_PARAMS_H_
#define GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_PARAMS_H_

#include <string>

namespace graph_optimizer {
struct GraphOptimizerParams {
  bool verbose;
  bool fatal_failures;
  bool print_factor_info;
  bool use_ceres_params;
  int max_iterations;
  std::string marginals_factorization;
  bool add_marginal_factors;
  double huber_k;
  int log_rate;
};
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_PARAMS_H_
