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

#ifndef NODE_ADDERS_NODE_ADDER_H_
#define NODE_ADDERS_NODE_ADDER_H_

#include <localization_common/time.h>

#include <gtsam/nonlinear/Marginals.h>

namespace node_adders {
// Base class for adding nodes to a graph. A different node adder should be used
// for each node type added to the graph. Nodes can consist of multiple types that are added in unison,
// for example a pose and velocity. See the nodes package for more details.
// Assumes nodes require an initial values and prior factors.
class NodeAdder {
 public:
  virtual ~NodeAdder() {}

  // Adds initial nodes and priors to constrain them during optimization.
  virtual void AddInitialNodesAndPriors(gtsam::NonlinearFactorGraph& graph) = 0;

  // Adds a node using the provided timestamp if possible.
  virtual bool AddNode(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) = 0;

  // Returns whether a node can be added at timestamp or not.
  virtual bool CanAddNode(const localization_common::Time timestamp) const = 0;

  // Returns keys for a node at the timestamp.
  virtual gtsam::KeyVector Keys(const localization_common::Time timestamp) const = 0;
};
}  // namespace node_adders

#endif  // NODE_ADDERS_NODE_ADDER_H_
