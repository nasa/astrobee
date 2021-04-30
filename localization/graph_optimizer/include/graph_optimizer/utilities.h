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

#ifndef GRAPH_OPTIMIZER_UTILITIES_H_
#define GRAPH_OPTIMIZER_UTILITIES_H_

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace graph_optimizer {
template <typename FactorType>
void DeleteFactors(gtsam::NonlinearFactorGraph& graph) {
  int num_removed_factors = 0;
  for (auto factor_it = graph.begin(); factor_it != graph.end();) {
    if (dynamic_cast<FactorType*>(factor_it->get())) {
      factor_it = graph.erase(factor_it);
      ++num_removed_factors;
      continue;
    }
    ++factor_it;
  }
  LogDebug("DeleteFactors: Num removed factors: " << num_removed_factors);
}

template <typename FactorType>
int NumFactors(const gtsam::NonlinearFactorGraph& graph) {
  int num_factors = 0;
  for (const auto& factor : graph) {
    if (dynamic_cast<const FactorType*>(factor.get())) {
      ++num_factors;
    }
  }
  return num_factors;
}

}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_UTILITIES_H_
