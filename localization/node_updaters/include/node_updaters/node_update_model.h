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

#ifndef NODE_UPDATERS_NODE_UPDATE_MODEL_H_
#define NODE_UPDATERS_NODE_UPDATE_MODEL_H_

#include <node_updaters/node_update_model_params.h>

#include <gtsam/inference/Key.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <vector>

namespace node_updaters {
// Handles adding priors and relative factors for a given node type.
// Generates these factors for provided timestamps.
template <typename NodeType, typename NodesType>
class NodeUpdateModel {
 public:
  virtual ~NodeUpdateModel() = 0;
  virtual void AddPriors(const NodeType& node, const std::vector<gtsam::SharedNoiseModel>& noise_models,
                         const localization_common::Time timestamp, const NodesType& nodes,
                         gtsam::NonlinearFactorGraph& factors) const = 0;
  virtual bool AddNodesAndRelativeFactors(const localization_common::Time timestamp_a,
                                          const localization_common::Time timestamp_b, NodesType& nodes,
                                          gtsam::NonlinearFactorGraph& factors) const = 0;
  virtual bool AddRelativeFactors(const localization_common::Time timestamp_a,
                                  const localization_common::Time timestamp_b, const NodesType& nodes,
                                  gtsam::NonlinearFactorGraph& factors) const = 0;

  virtual bool CanUpdate(const localization_common::Time timestamp) const = 0;

 protected:
  // TODO(rsoussan): Add constructor to set these, template on params?
  NodeUpdateModelParams params_;
};
}  // namespace node_updaters

#endif  // NODE_UPDATERS_NODE_UPDATE_MODEL_H_
