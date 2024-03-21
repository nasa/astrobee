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

#ifndef NODE_ADDERS_TIMESTAMPED_NODE_ADDER_MODEL_H_
#define NODE_ADDERS_TIMESTAMPED_NODE_ADDER_MODEL_H_

#include <node_adders/timestamped_node_adder_model_params.h>

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <vector>

namespace node_adders {
// Handles adding priors and relative factors for a given node type.
// Generates these factors for provided timestamps.
// Required by TimestampedNodeAdder.
template <typename NodeType, typename NodesType>
class TimestampedNodeAdderModel {
 public:
  explicit TimestampedNodeAdderModel(const TimestampedNodeAdderModelParams& params) : params_(params) {}
  virtual ~TimestampedNodeAdderModel() = default;
  // Adds prior factors for a given node using provided noise models.
  virtual void AddPriors(const NodeType& node, const std::vector<gtsam::SharedNoiseModel>& noise_models,
                         const localization_common::Time timestamp, const NodesType& nodes,
                         gtsam::NonlinearFactorGraph& factors) const = 0;

  // Adds nodes for timestamp_b and connect it to the node at timestamp_a with relative factors.
  virtual bool AddNodesAndRelativeFactors(const localization_common::Time timestamp_a,
                                          const localization_common::Time timestamp_b, NodesType& nodes,
                                          gtsam::NonlinearFactorGraph& factors) const = 0;

  // Adds relative factors between nodes at timestamp_a and timestamp_b.
  virtual bool AddRelativeFactors(const localization_common::Time timestamp_a,
                                  const localization_common::Time timestamp_b, const NodesType& nodes,
                                  gtsam::NonlinearFactorGraph& factors) const = 0;

  // Returns whether a node can be added at timestamp or not.
  virtual bool CanAddNode(const localization_common::Time timestamp) const = 0;

  // Remove relative adder model factors between nodes
  virtual bool RemoveRelativeFactors(const localization_common::Time timestamp_a,
                                     const localization_common::Time timestamp_b, const NodesType& nodes,
                                     gtsam::NonlinearFactorGraph& factors) const = 0;

 protected:
  // TODO(rsoussan): Add constructor to set these, template on params?
  TimestampedNodeAdderModelParams params_;
};
}  // namespace node_adders

#endif  // NODE_ADDERS_TIMESTAMPED_NODE_ADDER_MODEL_H_
