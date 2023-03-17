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

#include <graph_optimizer/nonlinear_optimizer_params.h>

#include <string>

namespace graph_optimizer {
struct GraphOptimizerParams {
  // Optimizer to use. Either "nonlinear" or "isam2".
  std::string optimizer;
  // Huber k used for all factors.
  double huber_k;
  // Log statistics after destruction.
  bool log_stats_on_destruction;
  // Log factor graph info and stats after optimization.
  bool print_after_optimization;
  // Params for nonlinear optimizer if used.
  NonlinearOptimizerParams nonlinear_optimizer;
  // Params for isam2 optimizer if used.
  // ISAM2OptimizerParams isam2_optimizer;
};
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_GRAPH_OPTIMIZER_PARAMS_H_
