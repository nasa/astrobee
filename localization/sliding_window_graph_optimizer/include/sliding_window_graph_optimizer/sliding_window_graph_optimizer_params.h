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
#ifndef SLIDING_WINDOW_GRAPH_OPTIMIZER_SLIDING_WINDOW_GRAPH_OPTIMIZER_PARAMS_H_
#define SLIDING_WINDOW_GRAPH_OPTIMIZER_SLIDING_WINDOW_GRAPH_OPTIMIZER_PARAMS_H_

#include <graph_optimizer/graph_optimizer_params.h>

namespace sliding_window_graph_optimizer {
struct SlidingWindowGraphOptimizerParams : public graph_optimizer::GraphOptimizerParams {
  // Add marginal factors to the graph after sliding the window.
  // Marginal factors consist of linearized errors for factors removed after sliding the window.
  bool add_marginal_factors;
  // Slide window before or after optimization. Sliding before optimization
  // ensures a set window size is used during optimization while sliding after
  // allows more nodes and factors to be included.
  bool slide_window_before_optimization;
};
}  // namespace sliding_window_graph_optimizer

#endif  // SLIDING_WINDOW_GRAPH_OPTIMIZER_SLIDING_WINDOW_GRAPH_OPTIMIZER_PARAMS_H_
