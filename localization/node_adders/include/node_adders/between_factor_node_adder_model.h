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

#ifndef NODE_ADDERS_BETWEEN_FACTOR_NODE_ADDER_MODEL_H_
#define NODE_ADDERS_BETWEEN_FACTOR_NODE_ADDER_MODEL_H_

#include <node_adders/timestamped_node_adder_model.h>
#include <node_adders/measurement_based_timestamped_node_adder_model.h>
#include <node_adders/utilities.h>
#include <nodes/timestamped_nodes.h>

#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

#include <utility>
#include <vector>

namespace node_adders {
// Node adder model that adds GTSAM between factors as relative factors and GTSAM prior factors as priors
// for a given node type.
template <typename NodeType, typename NodeAdderModelType>
class BetweenFactorNodeAdderModel : public NodeAdderModelType {
 public:
  using NodesType = nodes::TimestampedNodes<NodeType>;
  using Base = NodeAdderModelType;
  explicit BetweenFactorNodeAdderModel(const TimestampedNodeAdderModelParams& params) : Base(params) {}
  virtual ~BetweenFactorNodeAdderModel() = default;
  // Adds prior factors for a given node using provided noise models.
  void AddPriors(const NodeType& node, const std::vector<gtsam::SharedNoiseModel>& noise_models,
                 const localization_common::Time timestamp, const NodesType& nodes,
                 gtsam::NonlinearFactorGraph& factors) const final;

  // Adds nodes for provided timestamps and connects them with between factors.
  bool AddNodesAndRelativeFactors(const localization_common::Time timestamp_a,
                                  const localization_common::Time timestamp_b, NodesType& nodes,
                                  gtsam::NonlinearFactorGraph& factors) const final;

  // Connects nodes at given timestamps with between factors.
  bool AddRelativeFactors(const localization_common::Time timestamp_a, const localization_common::Time timestamp_b,
                          const NodesType& nodes, gtsam::NonlinearFactorGraph& factors) const final;

  bool RemoveRelativeFactors(const localization_common::Time timestamp_a, const localization_common::Time timestamp_b,
                             const NodesType& nodes, gtsam::NonlinearFactorGraph& factors) const final;

 private:
  // Connects nodes at given keys with between factors.
  bool AddRelativeFactors(const gtsam::KeyVector& keys_a, const localization_common::Time timestamp_a,
                          const gtsam::KeyVector& keys_b, const localization_common::Time timestamp_b,
                          gtsam::NonlinearFactorGraph& factors) const;

  // Adds a node at the given timestamp.
  // Needs to be implemented by the child class
  virtual gtsam::KeyVector AddNode(const localization_common::Time timestamp, NodesType& nodes) const = 0;

  // Creates a node with noise at timestamp_b relative to timestamp_a.
  // Needs to be implemented by the child class
  virtual boost::optional<std::pair<NodeType, gtsam::SharedNoiseModel>> RelativeNodeAndNoise(
    const localization_common::Time timestamp_a, const localization_common::Time timestamp_b) const = 0;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
  }
};

// Implementation
template <typename NodeType, typename NodeAdderModelType>
void BetweenFactorNodeAdderModel<NodeType, NodeAdderModelType>::AddPriors(
  const NodeType& node, const std::vector<gtsam::SharedNoiseModel>& noise_models,
  const localization_common::Time timestamp, const NodesType& nodes, gtsam::NonlinearFactorGraph& factors) const {
  const auto keys = nodes.Keys(timestamp);
  if (keys.empty()) {
    LogError("AddPriors: Failed to get keys.");
    return;
  }
  // TODO(rsoussan): Ensure symbols not used by other node adders
  gtsam::PriorFactor<NodeType> prior_factor(keys[0], node, noise_models[0]);
  factors.push_back(prior_factor);
}

template <typename NodeType, typename NodeAdderModelType>
bool BetweenFactorNodeAdderModel<NodeType, NodeAdderModelType>::AddNodesAndRelativeFactors(
  const localization_common::Time timestamp_a, const localization_common::Time timestamp_b, NodesType& nodes,
  gtsam::NonlinearFactorGraph& factors) const {
  if (!nodes.Contains(timestamp_b)) {
    const auto keys = AddNode(timestamp_b, nodes);
    if (keys.empty()) {
      LogError("AddNodesAndRelativeFactors: Failed to add node.");
      return false;
    }
  }

  if (!AddRelativeFactors(timestamp_a, timestamp_b, nodes, factors)) {
    LogError("AddNodesAndRelativeFactors: Failed to add relative factor.");
    return false;
  }
  return true;
}

template <typename NodeType, typename NodeAdderModelType>
bool BetweenFactorNodeAdderModel<NodeType, NodeAdderModelType>::AddRelativeFactors(
  const localization_common::Time timestamp_a, const localization_common::Time timestamp_b, const NodesType& nodes,
  gtsam::NonlinearFactorGraph& factors) const {
  const auto keys_a = nodes.Keys(timestamp_a);
  if (keys_a.empty()) {
    LogError("AddRelativeFactors: Failed to get keys a.");
    return false;
  }
  const auto keys_b = nodes.Keys(timestamp_b);
  if (keys_b.empty()) {
    LogError("AddRelativeFactors: Failed to get keys b.");
    return false;
  }
  if (!AddRelativeFactors(keys_a, timestamp_a, keys_b, timestamp_b, factors)) {
    LogError("AddRelativeFactor: Failed to add relative factor.");
    return false;
  }
  return true;
}

template <typename NodeType, typename NodeAdderModelType>
bool BetweenFactorNodeAdderModel<NodeType, NodeAdderModelType>::AddRelativeFactors(
  const gtsam::KeyVector& keys_a, const localization_common::Time timestamp_a, const gtsam::KeyVector& keys_b,
  const localization_common::Time timestamp_b, gtsam::NonlinearFactorGraph& factors) const {
  const auto relative_node_and_noise = RelativeNodeAndNoise(timestamp_a, timestamp_b);
  if (!relative_node_and_noise) {
    LogError("AddRelativeFactor: Failed to add relative node and noise.");
    return false;
  }

  typename gtsam::BetweenFactor<NodeType>::shared_ptr relative_factor(new gtsam::BetweenFactor<NodeType>(
    keys_a[0], keys_b[0], relative_node_and_noise->first, relative_node_and_noise->second));
  factors.push_back(relative_factor);
  return true;
}

template <typename NodeType, typename NodeAdderModelType>
bool BetweenFactorNodeAdderModel<NodeType, NodeAdderModelType>::RemoveRelativeFactors(
  const localization_common::Time timestamp_a, const localization_common::Time timestamp_b, const NodesType& nodes,
  gtsam::NonlinearFactorGraph& factors) const {
  return RemoveRelativeFactor<gtsam::BetweenFactor<NodeType>>(timestamp_a, timestamp_b, nodes, factors);
}

// Specialization helpers
template <typename MeasurementType, typename NodeType>
using BetweenFactorMeasurementBasedTimestampedNodeAdderModel = BetweenFactorNodeAdderModel<
  NodeType, MeasurementBasedTimestampedNodeAdderModel<MeasurementType, NodeType, nodes::TimestampedNodes<NodeType>>>;

template <typename NodeType>
using BetweenFactorTimestampedNodeAdderModel =
  BetweenFactorNodeAdderModel<NodeType, TimestampedNodeAdderModel<NodeType, nodes::TimestampedNodes<NodeType>>>;
}  // namespace node_adders

#endif  // NODE_ADDERS_BETWEEN_FACTOR_NODE_ADDER_MODEL_H_
