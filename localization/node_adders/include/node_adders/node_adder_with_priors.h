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

#ifndef NODE_ADDERS_NODE_ADDER_WITH_PRIORS_H_
#define NODE_ADDERS_NODE_ADDER_WITH_PRIORS_H_

#include <node_adders/node_adder.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace node_adders {
template <typename NodeType, typename NoiseType>
class NodeAdderWithPriors : public NodeAdder {
 public:
  virtual ~NodeAdderWithPriors() {}

  virtual void AddInitialNodesAndPriors(const NodeType& node, const NoiseType& noise,
                                        const localization_common::Time timestamp,
                                        gtsam::NonlinearFactorGraph& graph) = 0;
};
}  // namespace node_adders

#endif  // NODE_ADDERS_NODE_ADDER_WITH_PRIORS_H_
