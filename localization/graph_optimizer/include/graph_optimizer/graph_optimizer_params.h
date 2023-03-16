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

namespace graph_optimizer {
struct GraphOptimizerParams {
  // Maximum optimization iterations to perform
  int max_iterations;
  // Huber k used for all factors
  double huber_k;
  // Verbose printing for Levenberg-Marquardt optimization
  bool verbose;
  // Log statistics after destruction
  bool log_stats_on_destruction;
  // Log factor graph info and stats after optimization
  bool print_after_optimization;
  // Use ceres-style params for nonlinear optimization
  bool use_ceres_params;
};
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_PARAMS_H_
