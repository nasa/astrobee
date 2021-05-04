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

#ifndef GRAPH_OPTIMIZER_NODE_UPDATER_WITH_PRIORS_H_
#define GRAPH_OPTIMIZER_NODE_UPDATER_WITH_PRIORS_H_

#include <graph_optimizer/node_updater.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace graph_optimizer {
template <typename NodeType, typename NoiseType>
class NodeUpdaterWithPriors : public NodeUpdater {
 public:
  virtual ~NodeUpdaterWithPriors() {}

  virtual void AddInitialValuesAndPriors(const NodeType& node, const NoiseType& noise,
                                         gtsam::NonlinearFactorGraph& graph) = 0;

  virtual void AddPriors(const NodeType& node, const NoiseType& noise, gtsam::NonlinearFactorGraph& factors) = 0;
};
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_NODE_UPDATER_WITH_PRIORS_H_
