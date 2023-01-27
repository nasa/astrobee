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

#ifndef NODE_UPDATERS_BETWEEN_FACTOR_NODE_UPDATE_MODEL_H_
#define NODE_UPDATERS_BETWEEN_FACTOR_NODE_UPDATE_MODEL_H_

#include <graph_optimizer/timestamped_nodes.h>
#include <localization_common/pose_with_covariance_interpolater.h>
#include <node_updaters/node_update_model.h>

#include <gtsam/inference/Key.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>

#include <utility>
#include <vector>

namespace node_updaters {
template <typename NodeType>
// Simple class that has one node type and adds GTSAM between factors as relative factors to those
// nodes and GTSAM prior factors as priors
class BetweenFactorNodeUpdateModel : public NodeUpdateModel<NodeType, graph_optimizer::TimestampedNodes<NodeType>> {
 public:
  using NodesType = graph_optimizer::TimestampedNodes<NodeType>;
  using Base = NodeUpdateModel<NodeType, NodesType>;
  void AddPriors(const NodeType& node, const std::vector<gtsam::SharedNoiseModel>& noise_models,
                 const localization_common::Time timestamp, const NodesType& nodes,
                 gtsam::NonlinearFactorGraph& factors) final;
  bool AddNodesAndRelativeFactors(const localization_common::Time timestamp_a,
                                  const localization_common::Time timestamp_b, NodesType& nodes,
                                  gtsam::NonlinearFactorGraph& factors) final;
  bool AddRelativeFactors(const localization_common::Time timestamp_a, const localization_common::Time timestamp_b,
                          const NodesType& nodes, gtsam::NonlinearFactorGraph& factors) final;

 private:
  bool AddRelativeFactor(const gtsam::Key key_a, const localization_common::Time timestamp_a, const gtsam::Key key_b,
                         const localization_common::Time timestamp_b, gtsam::NonlinearFactorGraph& factors) const;
  // These functions needs to be implemented by the child class
  virtual boost::optional<gtsam::Key> AddNode(const localization_common::Time timestamp, NodesType& nodes) = 0;
  virtual boost::optional<std::pair<NodeType, gtsam::SharedNoiseModel>> RelativeNodeAndNoise(
    const localization_common::Time timestamp_a, const localization_common::Time timestamp_b) const = 0;

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(pose_interpolater_);
  }

  localization_common::PoseWithCovarianceInterpolater pose_interpolater_;
};

// Implementation
template <typename NodeType>
void BetweenFactorNodeUpdateModel<NodeType>::AddPriors(const NodeType& node,
                                                       const std::vector<gtsam::SharedNoiseModel>& noise_models,
                                                       const localization_common::Time timestamp,
                                                       const NodesType& nodes, gtsam::NonlinearFactorGraph& factors) {
  // TODO(rsoussan): vector now
  const auto key = nodes.Key(timestamp);
  if (!key) {
    LogError("AddPriors: Failed to get key.");
    return;
  }
  // TODO(rsoussan): Ensure symbols not used by other node updaters
  gtsam::PriorFactor<NodeType> prior_factor(*key, node, noise_models[0]);
  factors.push_back(prior_factor);
}

template <typename NodeType>
bool BetweenFactorNodeUpdateModel<NodeType>::AddNodesAndRelativeFactors(const localization_common::Time timestamp_a,
                                                                        const localization_common::Time timestamp_b,
                                                                        NodesType& nodes,
                                                                        gtsam::NonlinearFactorGraph& factors) {
  const auto key_b = AddNode(timestamp_b, nodes);
  if (!key_b) {
    LogError("AddNodesAndRelativeFactors: Failed to add node.");
    return false;
  }
  if (!AddRelativeFactors(timestamp_a, timestamp_b, nodes, factors)) {
    LogError("AddNodesAndRelativeFactors: Failed to add relative factor.");
    return false;
  }
  return true;
}

template <typename NodeType>
bool BetweenFactorNodeUpdateModel<NodeType>::AddRelativeFactors(const localization_common::Time timestamp_a,
                                                                const localization_common::Time timestamp_b,
                                                                const NodesType& nodes,
                                                                gtsam::NonlinearFactorGraph& factors) {
  // TODO(rsoussan): Are these vectors now?
  const auto key_a = nodes.Key(timestamp_a);
  if (!key_a) {
    LogError("AddRelativeFactor: Failed to get key a.");
    return false;
  }
  const auto key_b = nodes.Key(timestamp_b);
  if (!key_b) {
    LogError("AddRelativeFactor: Failed to get key b.");
    return false;
  }
  if (!AddRelativeFactor(*key_a, timestamp_a, *key_b, timestamp_b, factors)) {
    LogError("AddRelativeFactor: Failed to add relative factor.");
    return false;
  }
  return true;
}

template <typename NodeType>
bool BetweenFactorNodeUpdateModel<NodeType>::AddRelativeFactor(const gtsam::Key key_a,
                                                               const localization_common::Time timestamp_a,
                                                               const gtsam::Key key_b,
                                                               const localization_common::Time timestamp_b,
                                                               gtsam::NonlinearFactorGraph& factors) const {
  const auto relative_node_and_noise = RelativeNodeAndNoise(timestamp_a, timestamp_b);
  if (!relative_node_and_noise) {
    LogError("AddRelativeFactor: Failed to relative node and noise.");
  }

  typename gtsam::BetweenFactor<NodeType>::shared_ptr relative_factor(
    new gtsam::BetweenFactor<NodeType>(key_a, key_b, relative_node_and_noise->first, relative_node_and_noise->second));
  factors.push_back(relative_factor);
}
}  // namespace node_updaters

#endif  // NODE_UPDATERS_BETWEEN_FACTOR_NODE_UPDATE_MODEL_H_