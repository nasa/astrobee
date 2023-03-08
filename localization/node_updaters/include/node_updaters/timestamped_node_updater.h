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

#ifndef NODE_UPDATERS_TIMESTAMPED_NODE_UPDATER_H_
#define NODE_UPDATERS_TIMESTAMPED_NODE_UPDATER_H_

#include <graph_optimizer/node_updater_with_priors.h>
#include <graph_optimizer/timestamped_nodes.h>
#include <graph_optimizer/utilities.h>
#include <node_updaters/timestamped_node_update_model.h>
#include <node_updaters/timestamped_node_updater_params.h>

#include <algorithm>
#include <string>
#include <vector>

namespace node_updaters {

// Creates a node updater with timestamp-indexed nodes.
// Enables node updater use with a time-based sliding window graph.
template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
class TimestampedNodeUpdater
    : public graph_optimizer::NodeUpdaterWithPriors<NodeType, std::vector<gtsam::SharedNoiseModel>> {
  using Base = graph_optimizer::NodeUpdaterWithPriors<NodeType, gtsam::SharedNoiseModel>;

 public:
  TimestampedNodeUpdater(
    const TimestampedNodeUpdaterParams<NodeType>& params,
    std::shared_ptr<TimestampedNodesType> nodes = std::make_shared<TimestampedNodesType>(),
    std::shared_ptr<NodeUpdateModelType> node_update_model = std::make_shared<NodeUpdateModelType>());
  TimestampedNodeUpdater() = default;
  virtual ~TimestampedNodeUpdater() = default;

  // Adds initial nodes and priors using default values.
  void AddInitialNodesAndPriors(gtsam::NonlinearFactorGraph& factors);

  // Adds initial nodes and priors using provided noise values and timestamp.
  void AddInitialNodesAndPriors(const NodeType& initial_node,
                                 const std::vector<gtsam::SharedNoiseModel>& initial_noise,
                                 const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final;

  // Adds a node using the provided timestamp if possible.
  // TODO(rsousan): Rename this?
  bool AddNode(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final;

  // Slides the window, removes nodes older than oldest allowed time.
  // Adds priors to the oldest remaining nodes using their marginalized covariances.
  // Also removes any nodes containing any keys in old_keys.
  // Removes any factors in factors containing a removed node.
  bool SlideWindow(const localization_common::Time oldest_allowed_timestamp,
                   const boost::optional<gtsam::Marginals>& marginals, const gtsam::KeyVector& old_keys,
                   const double huber_k, gtsam::NonlinearFactorGraph& factors) final;

  // Returns the node updater type
  // This needs to be specialized
  std::string type() const override;

  // Returns the oldest node time that will remain after SlideWindow is called.
  // Returns boost::none if no nodes exist.
  boost::optional<localization_common::Time> SlideWindowNewOldestTime() const final;

  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time,
                           const gtsam::NonlinearFactorGraph& graph) const final;

  // TODO(rsoussan): Deprecate/Remove this
  boost::optional<gtsam::Key> GetKey(graph_optimizer::KeyCreatorFunction key_creator_function,
                                     const localization_common::Time timestamp) const final;

  boost::optional<localization_common::Time> OldestTimestamp() const final;

  boost::optional<localization_common::Time> LatestTimestamp() const final;

  // Returns whether a node can be added at timestamp or not.
  bool CanAddNode(const localization_common::Time timestamp) const final;

  const TimestampedNodesType& nodes() const { return *nodes_; }

 protected:
  std::shared_ptr<NodeUpdateModelType> node_update_model_;

 private:
  void RemovePriors(const gtsam::KeyVector& old_keys, gtsam::NonlinearFactorGraph& factors);
  bool AddLatestNodesAndRelativeFactors(const localization_common::Time timestamp,
                                        gtsam::NonlinearFactorGraph& factors);
  bool SplitOldRelativeFactor(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors);
  bool RemoveFactors(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors);

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(nodes_);
    ar& BOOST_SERIALIZATION_NVP(params_);
  }

  // TODO(rsoussan): do these need to be shared ptrs?
  std::shared_ptr<TimestampedNodesType> nodes_;
  TimestampedNodeUpdaterParams<NodeType> params_;
};

// Implementation
template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
TimestampedNodeUpdater<NodeType, TimestampedNodesType, NodeUpdateModelType>::TimestampedNodeUpdater(
  const TimestampedNodeUpdaterParams<NodeType>& params, std::shared_ptr<TimestampedNodesType> nodes,
  std::shared_ptr<NodeUpdateModelType> node_update_model)
    : params_(params), nodes_(nodes), node_update_model_(node_update_model) {}

template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
void TimestampedNodeUpdater<NodeType, TimestampedNodesType, NodeUpdateModelType>::AddInitialNodesAndPriors(
  gtsam::NonlinearFactorGraph& factors) {
  AddInitialNodesAndPriors(params_.start_node, params_.start_noise_models, params_.starting_time, factors);
}

template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
void TimestampedNodeUpdater<NodeType, TimestampedNodesType, NodeUpdateModelType>::AddInitialNodesAndPriors(
  const NodeType& initial_node, const std::vector<gtsam::SharedNoiseModel>& initial_noise,
  const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
  nodes_->Add(timestamp, initial_node);
  node_update_model_->AddPriors(initial_node, initial_noise, timestamp, *nodes_, factors);
}

template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
bool TimestampedNodeUpdater<NodeType, TimestampedNodesType, NodeUpdateModelType>::SlideWindow(
  const localization_common::Time oldest_allowed_timestamp, const boost::optional<gtsam::Marginals>& marginals,
  const gtsam::KeyVector& old_keys, const double huber_k, gtsam::NonlinearFactorGraph& factors) {
  nodes_->RemoveOldNodes(oldest_allowed_timestamp);
  if (params_.add_priors) {
    // Add prior to oldest pose using covariances from last round of
    // optimization
    const auto oldest_node = nodes_->OldestNode();
    const auto oldest_timestamp = nodes_->OldestTimestamp();
    if (!oldest_node || !oldest_timestamp) {
      LogError("SlideWindow: Failed to get oldest node and timestamp.");
      return false;
    }

    // Make sure priors are removed before adding new ones
    RemovePriors(old_keys, factors);
    if (marginals) {
      const auto keys = nodes_->Keys(*oldest_timestamp);
      if (keys.empty()) {
        LogError("SlideWindow: Failed to get oldest keys.");
        return false;
      }

      std::vector<gtsam::SharedNoiseModel> prior_noise_models;
      for (const auto& key : keys) {
        const auto prior_noise = graph_optimizer::Robust(
          gtsam::noiseModel::Gaussian::Covariance(marginals->marginalCovariance(key)), huber_k);
        prior_noise_models.emplace_back(prior_noise);
      }
      node_update_model_->AddPriors(*oldest_node, prior_noise_models, *oldest_timestamp, *nodes_, factors);
    } else {
      // TODO(rsoussan): Add seperate marginal fallback sigmas instead of relying on starting prior noise
      node_update_model_->AddPriors(*oldest_node, params_.start_noise_models, *oldest_timestamp, *nodes_, factors);
    }
  }

  return true;
}

template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
std::string TimestampedNodeUpdater<NodeType, TimestampedNodesType, NodeUpdateModelType>::type() const {
  static_assert(sizeof(NodeType) == std::size_t(-1), "This needs to be specialized by template class.");
}

template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
boost::optional<localization_common::Time>
TimestampedNodeUpdater<NodeType, TimestampedNodesType, NodeUpdateModelType>::SlideWindowNewOldestTime() const {
  // TODO(rsoussan): Generalize this with CombinedNavStateGraphValues
  if (nodes_->empty()) {
    LogDebug("SlideWindowOldestTime: No states in map.");
    return boost::none;
  }

  const size_t size = nodes_->size();
  if (size <= params_.min_num_states) {
    LogDebug("SlideWindowOldestTime: Not enough states to remove.");
    return boost::none;
  }

  const double total_duration = nodes_->Duration();
  LogDebug("SlideWindowOldestTime: Starting total num states: " << nodes_->size());
  LogDebug("SlideWindowOldestTime: Starting total duration is " << total_duration);
  const localization_common::Time ideal_oldest_allowed_state =
    std::max(0.0, *(nodes_->LatestTimestamp()) - params_.ideal_duration);

  int num_states_to_be_removed = 0;
  // Ensures that new oldest time is consistent with a number of states <= max_num_states
  // and >= min_num_states.
  // Assumes min_num_states < max_num_states.
  for (const auto& timestamp : nodes_->Timestamps()) {
    ++num_states_to_be_removed;
    const int new_num_states = size - num_states_to_be_removed;
    if (new_num_states > params_.max_num_states) continue;
    if (new_num_states <= params_.min_num_states) return timestamp;
    if (timestamp >= ideal_oldest_allowed_state) return timestamp;
  }

  // Shouldn't occur
  return boost::none;
}

template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
gtsam::KeyVector TimestampedNodeUpdater<NodeType, TimestampedNodesType, NodeUpdateModelType>::OldKeys(
  const localization_common::Time oldest_allowed_time, const gtsam::NonlinearFactorGraph& graph) const {
  return nodes_->OldKeys(oldest_allowed_time);
}

// TODO(rsoussan): Change this interface
template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
boost::optional<gtsam::Key> TimestampedNodeUpdater<NodeType, TimestampedNodesType, NodeUpdateModelType>::GetKey(
  graph_optimizer::KeyCreatorFunction key_creator_function, const localization_common::Time timestamp) const {
  // TODO(rsoussan): This is temp standin, this function should be removed altogether
  return nodes_->Keys(timestamp)[0];
}

template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
boost::optional<localization_common::Time>
TimestampedNodeUpdater<NodeType, TimestampedNodesType, NodeUpdateModelType>::OldestTimestamp() const {
  return nodes_->OldestTimestamp();
}

template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
boost::optional<localization_common::Time>
TimestampedNodeUpdater<NodeType, TimestampedNodesType, NodeUpdateModelType>::LatestTimestamp() const {
  return nodes_->LatestTimestamp();
}

template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
bool TimestampedNodeUpdater<NodeType, TimestampedNodesType, NodeUpdateModelType>::CanAddNode(
  const localization_common::Time timestamp) const {
  return node_update_model_->CanAddNode(timestamp);
}

template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
void TimestampedNodeUpdater<NodeType, TimestampedNodesType, NodeUpdateModelType>::RemovePriors(
  const gtsam::KeyVector& old_keys, gtsam::NonlinearFactorGraph& factors) {
  int removed_factors = 0;
  for (auto factor_it = factors.begin(); factor_it != factors.end();) {
    bool erase_factor = false;
    const auto prior_factor = dynamic_cast<gtsam::PriorFactor<NodeType>*>(factor_it->get());
    if (prior_factor) {
      // Erase factor if it contains an old key
      for (const auto& old_key : old_keys) {
        if (prior_factor->key() == old_key) {
          erase_factor = true;
          factor_it = factors.erase(factor_it);
          ++removed_factors;
        }
      }
      if (!erase_factor) {
        ++factor_it;
      }
    }
  }
  LogDebug("RemovePriors: Erase " << removed_factors << " factors.");
}

template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
bool TimestampedNodeUpdater<NodeType, TimestampedNodesType, NodeUpdateModelType>::AddNode(
  const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
  if (nodes_->Contains(timestamp)) {
    LogDebug(
      "Update: Node exists at "
      "timestamp, nothing to do.");
    return true;
  }

  const auto latest_timestamp = nodes_->LatestTimestamp();
  if (!latest_timestamp) {
    LogError("Update: Failed to get latest timestamp.");
    return false;
  }

  if (timestamp > *latest_timestamp) {
    LogDebug("Update: Adding latest node and relative factor.");
    return AddLatestNodesAndRelativeFactors(timestamp, factors);
  } else {
    LogDebug("Update: Splitting old relative factor.");
    return SplitOldRelativeFactor(timestamp, factors);
  }
}

template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
bool TimestampedNodeUpdater<NodeType, TimestampedNodesType, NodeUpdateModelType>::AddLatestNodesAndRelativeFactors(
  const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
  const auto timestamp_a = nodes_->LatestTimestamp();
  if (!timestamp_a) {
    LogError("AddLatestNodeAndRelativeFactor: Failed to get latest timestamp.");
    return false;
  }
  return node_update_model_->AddNodesAndRelativeFactors(*timestamp_a, timestamp, *nodes_, factors);
}

template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
bool TimestampedNodeUpdater<NodeType, TimestampedNodesType, NodeUpdateModelType>::SplitOldRelativeFactor(
  const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
  const auto timestamp_bounds = nodes_->LowerAndUpperBoundTimestamps(timestamp);
  if (!timestamp_bounds.first || !timestamp_bounds.second) {
    LogError("SplitOldRelativeFactor: Failed to get upper and lower bound timestamp.");
    return false;
  }

  const localization_common::Time lower_bound_time = *(timestamp_bounds.first);
  const localization_common::Time upper_bound_time = *(timestamp_bounds.second);

  if (timestamp < lower_bound_time || timestamp > upper_bound_time) {
    LogError("SplitOldRelativeFactor: Timestamp is not within bounds of existing timestamps.");
    return false;
  }

  const bool removed_old_factors = RemoveFactors(timestamp, factors);
  if (!removed_old_factors) {
    LogError(
      "SplitOldRelativeFactor: Failed to remove "
      "old factors.");
    return false;
  }
  if (!node_update_model_->AddNodesAndRelativeFactors(lower_bound_time, timestamp, *nodes_, factors)) {
    LogError("SplitOldRelativeFactor: Failed to add first relative node and factor.");
    return false;
  }
  if (!node_update_model_->AddRelativeFactors(timestamp, upper_bound_time, *nodes_, factors)) {
    LogError("SplitOldRelativeFactor: Failed to add second relative factor.");
    return false;
  }
}

template <typename NodeType, typename TimestampedNodesType, typename NodeUpdateModelType>
bool TimestampedNodeUpdater<NodeType, TimestampedNodesType, NodeUpdateModelType>::RemoveFactors(
  const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
  const auto keys = nodes_->Keys(timestamp);
  if (keys.empty()) {
    LogError("RemoveFactors: Failed to get keys.");
    return false;
  }

  bool removed_factor = false;
  for (const auto& key : keys) {
    for (auto factor_it = factors.begin(); factor_it != factors.end();) {
      if ((*factor_it)->find(key) != std::end((*factor_it)->keys())) {
        factors.erase(factor_it);
        removed_factor = true;
      } else {
        ++factor_it;
      }
    }
  }
  return removed_factor;
}
}  // namespace node_updaters

#endif  // NODE_UPDATERS_TIMESTAMPED_NODE_UPDATER_H_
