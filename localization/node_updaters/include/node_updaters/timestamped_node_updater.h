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
#include <node_updaters/node_update_model.h>
#include <node_updaters/timestamped_node_updater_params.h>

#include <algorithm>

namespace node_updaters {
template <typename NodeType, typename NodeUpdateModelType>
using Base = graph_optimizer::NodeUpdaterWithPriors<NodeType, gtsam::SharedNoiseModel>;
using TimestampedNodes = graph_optimizer::TimestampedNodes<NodeType>;
class TimestampedNodeUpdater : public Base {
 public:
  explicit TimestampedNodeUpdater(std::shared_ptr<TimestampedNodes> nodes);
  TimestampedNodeUpdater() = default;
  virtual ~TimestampedNodeUpdater() = default;

  void AddInitialValuesAndPriors(gtsam::NonlinearFactorGraph& factors);

  void AddInitialValuesAndPriors(const NodeType& initial_node, const gtsam::SharedNoiseModel& initial_noise,
                                 const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final;

  void AddPriors(const NodeType& node, const gtsam::SharedNoiseModel& noise, const localization_common::Time timestamp,
                 gtsam::NonlinearFactorGraph& factors) final;

  // TODO(rsousan): Rename this?
  bool Update(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final;

  bool SlideWindow(const localization_common::Time oldest_allowed_timestamp,
                   const boost::optional<gtsam::Marginals>& marginals, const gtsam::KeyVector& old_keys,
                   const double huber_k, gtsam::NonlinearFactorGraph& factors) final;

  // This needs to be specialized
  graph_optimizer::NodeUpdaterType type() const final;

  boost::optional<localization_common::Time> SlideWindowNewOldestTime() const final;

  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time,
                           const gtsam::NonlinearFactorGraph& graph) const final;

  // TODO(rsoussan): Remove key creator function
  boost::optional<gtsam::Key> GetKey(graph_optimizer::KeyCreatorFunction key_creator_function,
                                     const localization_common::Time timestamp) const final;

  boost::optional<localization_common::Time> OldestTimestamp() const final;

  boost::optional<localization_common::Time> LatestTimestamp() const final;

 private:
  void RemovePriors(const gtsam::KeyVector& old_keys, gtsam::NonlinearFactorGraph& factors);
  bool AddLatestNodeAndRelativeFactor(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors);
  boost::optional<gtsam::Key> LatestKey();
  bool AddNodeAndRelativeFactor(const localization_common::Time timestamp_a,
                                const localization_common::Time timestamp_b, gtsam::NonlinearFactorGraph& factors);
  bool AddRelativeFactor(const localization_common::Time timestamp_a, const localization_common::Time timestamp_b,
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

  std::shared_ptr<TimestampedNodes> nodes_;
  std::shared_ptr<NodeUpdateModelType> node_update_model_;
  TimestampedNodeUpdaterParams params_;
};

// Implementation
template <typename NodeType>
TimestampedNodeUpdater(std::shared_ptr<TimestampedNodes> nodes, std::shared_ptr<NodeUpdateModelType> node_update_model)
    : nodes_(nodes), node_update_model_(node_update_model) {}

template <typename NodeType>
void TimestampedNodeUpdater<NodeType>::AddInitialValuesAndPriors(gtsam::NonlinearFactorGraph& factors) {
  AddInitialValuesAndPriors(params_.start_node, params_.start_noise, params_.starting_time, factors);
}

template <typename NodeType>
void TimestampedNodeUpdater<NodeType>::AddInitialValuesAndPriors(const NodeType& initial_node,
                                                                 const gtsam::SharedNoiseModel& initial_noise,
                                                                 const lc::Time timestamp,
                                                                 gtsam::NonlinearFactorGraph& factors) {
  nodes_->Add(timestamp, initial_node);
  AddPriors(initial_node, initial_noise, timestamp, factors);
}

template <typename NodeType>
void TimestampedNodeUpdater<NodeType>::AddPriors(const NodeType& node, const gtsam::SharedNoiseModel& noise,
                                                 const lc::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
  const auto key = nodes_->Key(timestamp);
  if (!key) {
    LogError("AddPriors: Failed to get key.");
    return;
  }
  // TODO(rsoussan): Ensure symbols not used by other node updaters
  gtsam::PriorFactor<NodeType> prior_factor(*key, node, noise);
  factors.push_back(prior_factor);
}

template <typename NodeType>
bool TimestampedNodeUpdater<NodeType>::SlideWindow(const lc::Time oldest_allowed_timestamp,
                                                   const boost::optional<gtsam::Marginals>& marginals,
                                                   const gtsam::KeyVector& old_keys, const double huber_k,
                                                   gtsam::NonlinearFactorGraph& factors) {
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
      const auto key = nodes_->Key(*oldest_timestamp);
      if (!key) {
        LogError("SlideWindow: Failed to get oldest key.");
        return false;
      }
      const auto prior_noise =
        go::Robust(gtsam::noiseModel::Gaussian::Covariance(marginals->marginalCovariance(*key)), huber_k);
      AddPriors(*oldest_node, prior_noise, *oldest_timestamp, factors);
    } else {
      // TODO(rsoussan): Add seperate marginal fallback sigmas instead of relying on starting prior noise
      AddPriors(*oldest_node, params_.start_noise, *oldest_timestamp, factors);
    }
  }

  return true;
}

template <typename NodeType>
go::NodeUpdaterType TimestampedNodeUpdater<NodeType>::type() const {
  static_assert(sizeof(T) == std::size_t(-1), "This needs to be specialized by template class.");
}

boost::optional<lc::Time> TimestampedNodeUpdater<NodeType>::SlideWindowNewOldestTime() const {
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
  const lc::Time ideal_oldest_allowed_state = std::max(0.0, *(nodes_->LatestTimestamp()) - params_.ideal_duration);

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

template <typename NodeType>
gtsam::KeyVector TimestampedNodeUpdater<NodeType>::OldKeys(const lc::Time oldest_allowed_time,
                                                           const gtsam::NonlinearFactorGraph& graph) const {
  return nodes_->OldKeys(oldest_allowed_time);
}

// TODO(rsoussan): Change this interface
template <typename NodeType>
boost::optional<gtsam::Key> TimestampedNodeUpdater<NodeType>::GetKey(go::KeyCreatorFunction key_creator_function,
                                                                     const lc::Time timestamp) const {
  return nodes_->Key(timestamp);
}

template <typename NodeType>
boost::optional<lc::Time> TimestampedNodeUpdater<NodeType>::OldestTimestamp() const {
  return nodes_->OldestTimestamp();
}

template <typename NodeType>
boost::optional<lc::Time> TimestampedNodeUpdater<NodeType>::LatestTimestamp() const {
  return nodes_->LatestTimestamp();
}

template <typename NodeType>
void TimestampedNodeUpdater<NodeType>::RemovePriors(const gtsam::KeyVector& old_keys,
                                                    gtsam::NonlinearFactorGraph& factors) {
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

template <typename NodeType>
bool TimestampedNodeUpdater<NodeType>::Update(const lc::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
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
    return AddLatestNodeAndRelativeFactor(timestamp, factors);
  } else {
    LogDebug("Update: Splitting old relative factor.");
    return SplitOldRelativeFactor(timestamp, factors);
  }
}

template <typename NodeType>
bool TimestampedNodeUpdater<NodeType>::AddNodeAndRelativeFactor(const lc::Time timestamp_a, const lc::Time timestamp_b,
                                                                gtsam::NonlinearFactorGraph& factors) {
  const auto key_b = node_update_model_->AddNode(timestamp_b);
  if (!key_b) {
    LogError("AddNodeAndRelativeFactor: Failed to add node.");
    return false;
  }
  if (!AddRelativeFactor(timestamp_a, timestamp_b, factors)) {
    LogError("AddNodeAndRelativeFactor: Failed to add relative factor.");
    return false;
  }
  return true;
}

template <typename NodeType>
bool TimestampedNodeUpdater<NodeType>::AddRelativeFactor(const lc::Time timestamp_a, const lc::Time timestamp_b,
                                                         gtsam::NonlinearFactorGraph& factors) {
  const auto key_a = nodes_->Key(timestamp_a);
  if (!key_a) {
    LogError("AddRelativeFactor: Failed to get key a.");
    return false;
  }
  const auto key_b = nodes_->Key(timestamp_b);
  if (!key_b) {
    LogError("AddRelativeFactor: Failed to get key b.");
    return false;
  }
  if (!node_update_model_->AddRelativeFactor(*key_a, timestamp_a, *key_b, timestamp_b, factors)) {
    LogError("AddRelativeFactor: Failed to add relative factor.");
    return false;
  }
  return true;
}

template <typename NodeType>
bool TimestampedNodeUpdater<NodeType>::AddLatestNodeAndRelativeFactor(const lc::Time timestamp,
                                                                      gtsam::NonlinearFactorGraph& factors) {
  const auto timestamp_a = nodes_->LatestTimestamp();
  if (!timestamp_a) {
    LogError("AddLatestNodeAndRelativeFactor: Failed to get latest timestamp.");
    return false;
  }
  return AddNodeAndRelativeFactor(*timestamp_a, timestamp, factors);
}

// TODO(rsoussan): add this to timestamped nodes!
template <typename NodeType>
boost::optional<gtsam::Key> TimestampedNodeUpdater<NodeType>::LatestKey() {
  const auto latest_timestamp = nodes_->LatestTimestamp();
  if (!latest_timestamp) return boost::none;
  return nodes_->Key(*latest_timestamp);
}

template <typename NodeType>
bool TimestampedNodeUpdater<NodeType>::SplitOldRelativeFactor(const lc::Time timestamp,
                                                              gtsam::NonlinearFactorGraph& factors) {
  const auto timestamp_bounds = nodes_->LowerAndUpperBoundTimestamps(timestamp);
  if (!timestamp_bounds.first || !timestamp_bounds.second) {
    LogError("SplitOldRelativeFactor: Failed to get upper and lower bound timestamp.");
    return false;
  }

  const lc::Time lower_bound_time = *(timestamp_bounds.first);
  const lc::Time upper_bound_time = *(timestamp_bounds.second);

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
  if (!AddNodeAndRelativeFactor(lower_bound_time, timestamp, factors)) {
    LogError("SplitOldRelativeFactor: Failed to add first relative node and factor.");
    return false;
  }
  if (!AddRelativeFactor(timestamp, upper_bound_time, factors)) {
    LogError("SplitOldRelativeFactor: Failed to add second relative factor.");
    return false;
  }
}

template <typename NodeType>
bool TimestampedNodeUpdater<NodeType>::RemoveFactors(const lc::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
  const auto key = nodes_->Key(timestamp);
  if (!key) {
    LogError("RemoveFactors: Failed to get key.");
    return false;
  }

  bool removed_factor = false;
  for (auto factor_it = factors.begin(); factor_it != factors.end();) {
    if ((*factor_it)->find(*key) != std::end((*factor_it)->keys())) {
      factors.erase(factor_it);
      removed_factor = true;
    } else {
      ++factor_it;
    }
  }
  return removed_factor;
}
}  // namespace node_updaters

#endif  // NODE_UPDATERS_TIMESTAMPED_NODE_UPDATER_H_
