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

#ifndef NODE_ADDERS_TIMESTAMPED_NODE_ADDER_H_
#define NODE_ADDERS_TIMESTAMPED_NODE_ADDER_H_

#include <localization_common/utilities.h>
#include <node_adders/sliding_window_node_adder.h>
#include <node_adders/timestamped_node_adder_model.h>
#include <node_adders/timestamped_node_adder_params.h>
#include <nodes/timestamped_nodes.h>

#include <algorithm>
#include <vector>

namespace node_adders {

// Sliding window node adder using timestamp-indexed nodes.
// Generates functions that adds nodes, relative factors, splits old factors, and so on.
// Uses the provided node adder model to accomplish these.
template <typename NodeType, typename TimestampedNodesType, typename NodeAdderModelType>
class TimestampedNodeAdder : public SlidingWindowNodeAdder {
  using Base = SlidingWindowNodeAdder;

 public:
  // Construct using nodes. Creates timestamped nodes interally.
  TimestampedNodeAdder(const TimestampedNodeAdderParams<NodeType>& params,
                       const typename NodeAdderModelType::Params& node_adder_model_params,
                       std::shared_ptr<nodes::Values> values);

  // Construct using already constructed timestamped nodes.
  TimestampedNodeAdder(const TimestampedNodeAdderParams<NodeType>& params,
                       const typename NodeAdderModelType::Params& node_adder_model_params,
                       std::shared_ptr<TimestampedNodesType> timestamped_nodes);

  // For serialization only
  TimestampedNodeAdder() = default;

  virtual ~TimestampedNodeAdder() = default;

  void AddInitialNodesAndPriors(gtsam::NonlinearFactorGraph& factors) final;

  // Adds initial nodes and priors using provided noise values and timestamp.
  void AddInitialNodesAndPriors(const NodeType& initial_node, const std::vector<gtsam::SharedNoiseModel>& initial_noise,
                                const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors);

  bool AddNode(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) final;

  // Slides the window, removes nodes older than oldest allowed time.
  // Adds priors to the oldest remaining nodes using their marginalized covariances
  // and removes old priors containing any key in old keys if param use_priors is true.
  // Note: Old factor removal (other than starting priors) is handled in the graph optimizer.
  // The oldest allowed timestamp is also determing by the graph optimizer based on
  // the ideal oldest allowed timestamps of each node adder used in the graph.
  bool SlideWindow(const localization_common::Time oldest_allowed_timestamp,
                   const boost::optional<const gtsam::Marginals&>& marginals, const gtsam::KeyVector& old_keys,
                   const double huber_k, gtsam::NonlinearFactorGraph& factors) override;

  // Returns the oldest node time that should remain after SlideWindow is called.
  // Calculated using params to ensure the set min/max node limits are enforced
  // and the ideal duration is enforced otherwise.
  // Returns boost::none if no nodes exist or too few nodes exist.
  // If the min/max limits are enforced and the total duration of nodes is still less than
  // the ideal duration, the new oldest time is set to 0.
  // Used by the graph optimizer to compute the new oldest time for all node adders
  // in the graph.
  boost::optional<localization_common::Time> SlideWindowNewStartTime() const final;

  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time,
                           const gtsam::NonlinearFactorGraph& graph) const final;

  boost::optional<localization_common::Time> StartTime() const final;

  boost::optional<localization_common::Time> EndTime() const final;

  bool CanAddNode(const localization_common::Time timestamp) const final;

  gtsam::KeyVector Keys(const localization_common::Time timestamp) const final;

  const TimestampedNodesType& nodes() const { return *nodes_; }

  std::shared_ptr<const TimestampedNodesType> nodes_ptr() { return nodes_; }

  NodeAdderModelType& node_adder_model() { return node_adder_model_; }

 private:
  void RemovePriors(const gtsam::KeyVector& old_keys, gtsam::NonlinearFactorGraph& factors);
  bool AddNewNodesAndRelativeFactors(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors);
  bool SplitOldRelativeFactor(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors);

  // Serialization function
  friend class boost::serialization::access;
  template <class Archive>
  void serialize(Archive& ar, const unsigned int file_version) {
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(Base);
    ar& BOOST_SERIALIZATION_NVP(nodes_);
    ar& BOOST_SERIALIZATION_NVP(node_adder_model_);
    ar& BOOST_SERIALIZATION_NVP(params_);
  }

  TimestampedNodeAdderParams<NodeType> params_;
  NodeAdderModelType node_adder_model_;
  std::shared_ptr<TimestampedNodesType> nodes_;
};

// Implementation
template <typename NodeType, typename TimestampedNodesType, typename NodeAdderModelType>
TimestampedNodeAdder<NodeType, TimestampedNodesType, NodeAdderModelType>::TimestampedNodeAdder(
  const TimestampedNodeAdderParams<NodeType>& params,
  const typename NodeAdderModelType::Params& node_adder_model_params, std::shared_ptr<nodes::Values> values)
    : params_(params),
      nodes_(std::make_shared<TimestampedNodesType>(values)),
      node_adder_model_(node_adder_model_params) {}

template <typename NodeType, typename TimestampedNodesType, typename NodeAdderModelType>
TimestampedNodeAdder<NodeType, TimestampedNodesType, NodeAdderModelType>::TimestampedNodeAdder(
  const TimestampedNodeAdderParams<NodeType>& params,
  const typename NodeAdderModelType::Params& node_adder_model_params,
  std::shared_ptr<TimestampedNodesType> timestamped_nodes)
    : params_(params), nodes_(timestamped_nodes), node_adder_model_(node_adder_model_params) {}

template <typename NodeType, typename TimestampedNodesType, typename NodeAdderModelType>
void TimestampedNodeAdder<NodeType, TimestampedNodesType, NodeAdderModelType>::AddInitialNodesAndPriors(
  gtsam::NonlinearFactorGraph& factors) {
  AddInitialNodesAndPriors(params_.start_node, params_.start_noise_models, params_.starting_time, factors);
}

template <typename NodeType, typename TimestampedNodesType, typename NodeAdderModelType>
void TimestampedNodeAdder<NodeType, TimestampedNodesType, NodeAdderModelType>::AddInitialNodesAndPriors(
  const NodeType& initial_node, const std::vector<gtsam::SharedNoiseModel>& initial_noise,
  const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
  nodes_->Add(timestamp, initial_node);
  node_adder_model_.AddPriors(initial_node, initial_noise, timestamp, *nodes_, factors);
  // Store initial node as measurement so subsequent measurements can be computed relative to this
}

template <typename NodeType, typename TimestampedNodesType, typename NodeAdderModelType>
bool TimestampedNodeAdder<NodeType, TimestampedNodesType, NodeAdderModelType>::SlideWindow(
  const localization_common::Time oldest_allowed_timestamp, const boost::optional<const gtsam::Marginals&>& marginals,
  const gtsam::KeyVector& old_keys, const double huber_k, gtsam::NonlinearFactorGraph& factors) {
  if (oldest_allowed_timestamp <= *(StartTime())) {
    LogDebug("SlideWindow: Oldest allowed timestamp older than current start time, nothing to do.");
    return true;
  }
  nodes_->RemoveOldNodes(oldest_allowed_timestamp);
  if (params_.add_priors) {
    // Add prior to oldest pose using covariances from last round of
    // optimization
    const auto oldest_node = nodes_->OldestNode();
    const auto start_time = StartTime();
    if (!oldest_node || !start_time) {
      LogError("SlideWindow: Failed to get oldest node and timestamp.");
      return false;
    }

    // Make sure priors are removed before adding new ones
    RemovePriors(old_keys, factors);
    if (marginals) {
      const auto keys = nodes_->Keys(*start_time);
      if (keys.empty()) {
        LogError("SlideWindow: Failed to get oldest keys.");
        return false;
      }

      std::vector<gtsam::SharedNoiseModel> prior_noise_models;
      for (const auto& key : keys) {
        // If covariance doesn't exist yet (can happen if the new start node hasn't been optimized yet)
        // revert to the initial start noise for the prior.
        try {
          const auto prior_noise = localization_common::Robust(
            gtsam::noiseModel::Gaussian::Covariance(marginals->marginalCovariance(key)), huber_k);
          prior_noise_models.emplace_back(prior_noise);
        } catch (...) {
          prior_noise_models = params_.start_noise_models;
          break;
        }
      }
      node_adder_model_.AddPriors(*oldest_node, prior_noise_models, *start_time, *nodes_, factors);
    } else {
      // TODO(rsoussan): Add seperate marginal fallback sigmas instead of relying on starting prior noise
      node_adder_model_.AddPriors(*oldest_node, params_.start_noise_models, *start_time, *nodes_, factors);
    }
  }

  return true;
}

template <typename NodeType, typename TimestampedNodesType, typename NodeAdderModelType>
boost::optional<localization_common::Time>
TimestampedNodeAdder<NodeType, TimestampedNodesType, NodeAdderModelType>::SlideWindowNewStartTime() const {
  if (nodes_->empty()) {
    LogDebug("SlideWindowNewStartTime: No states in map.");
    return boost::none;
  }

  const int size = nodes_->size();
  if (size <= params_.min_num_states) {
    LogDebug("SlideWindowNewStartTime: Not enough states to remove.");
    return boost::none;
  }

  const double total_duration = nodes_->Duration();
  LogDebug("SlideWindowNewStartTime: Starting total num states: " << nodes_->size());
  LogDebug("SlideWindowNewStartTime: Starting total duration is " << total_duration);
  const localization_common::Time ideal_oldest_allowed_state = std::max(0.0, *(EndTime()) - params_.ideal_duration);

  // Find oldest timestamp that first prioritizes that the graph has at least min_num_states.
  // Second priority, find the optimal oldest timestamp that additionally ensures the graph does not have more than
  // max_num_states. Last priortity, find the optimial oldest timestamp that is also closest to the ideal oldest
  // timestamp (and thus ensures the graph is <= the ideal duration).
  int num_states_to_be_removed = 0;
  for (const auto& timestamp : nodes_->Timestamps()) {
    const int new_num_states = size - num_states_to_be_removed++;
    // First priority
    if (new_num_states <= params_.min_num_states) return timestamp;
    // Second priority
    if (new_num_states > params_.max_num_states) continue;
    // Last priority
    if (timestamp >= ideal_oldest_allowed_state) return timestamp;
  }

  // If the constraints aren't able to be satisfied, an error was made
  // when setting the graph duration or state limit params
  LogError("SlideWindowNewStartTime: Invalid sliding window params set, max num states: "
           << params_.max_num_states << ", min num states: " << params_.min_num_states
           << ", ideal duration: " << params_.ideal_duration);
  return boost::none;
}

template <typename NodeType, typename TimestampedNodesType, typename NodeAdderModelType>
gtsam::KeyVector TimestampedNodeAdder<NodeType, TimestampedNodesType, NodeAdderModelType>::OldKeys(
  const localization_common::Time oldest_allowed_time, const gtsam::NonlinearFactorGraph& graph) const {
  return nodes_->OldKeys(oldest_allowed_time);
}

template <typename NodeType, typename TimestampedNodesType, typename NodeAdderModelType>
boost::optional<localization_common::Time>
TimestampedNodeAdder<NodeType, TimestampedNodesType, NodeAdderModelType>::StartTime() const {
  return nodes_->OldestTimestamp();
}

template <typename NodeType, typename TimestampedNodesType, typename NodeAdderModelType>
boost::optional<localization_common::Time>
TimestampedNodeAdder<NodeType, TimestampedNodesType, NodeAdderModelType>::EndTime() const {
  return nodes_->LatestTimestamp();
}

template <typename NodeType, typename TimestampedNodesType, typename NodeAdderModelType>
bool TimestampedNodeAdder<NodeType, TimestampedNodesType, NodeAdderModelType>::CanAddNode(
  const localization_common::Time timestamp) const {
  return node_adder_model_.CanAddNode(timestamp);
}

template <typename NodeType, typename TimestampedNodesType, typename NodeAdderModelType>
gtsam::KeyVector TimestampedNodeAdder<NodeType, TimestampedNodesType, NodeAdderModelType>::Keys(
  const localization_common::Time timestamp) const {
  return nodes_->Keys(timestamp);
}

template <typename NodeType, typename TimestampedNodesType, typename NodeAdderModelType>
void TimestampedNodeAdder<NodeType, TimestampedNodesType, NodeAdderModelType>::RemovePriors(
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
    } else {
      ++factor_it;
    }
  }
  LogDebug("RemovePriors: Erase " << removed_factors << " factors.");
}

template <typename NodeType, typename TimestampedNodesType, typename NodeAdderModelType>
bool TimestampedNodeAdder<NodeType, TimestampedNodesType, NodeAdderModelType>::AddNode(
  const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
  if (nodes_->Contains(timestamp)) {
    LogDebug(
      "Adder: Node exists at "
      "timestamp, nothing to do.");
    return true;
  }

  const auto end_time = EndTime();
  if (!end_time) {
    LogError("Adder: Failed to get end timestamp.");
    return false;
  }
  if (timestamp > *end_time) {
    LogDebug("Adder: Adding new nodes and relative factors.");
    return AddNewNodesAndRelativeFactors(timestamp, factors);
  } else {
    LogDebug("Adder: Splitting old relative factor.");
    return SplitOldRelativeFactor(timestamp, factors);
  }
}

template <typename NodeType, typename TimestampedNodesType, typename NodeAdderModelType>
bool TimestampedNodeAdder<NodeType, TimestampedNodesType, NodeAdderModelType>::AddNewNodesAndRelativeFactors(
  const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
  const auto timestamp_a = EndTime();
  if (!timestamp_a) {
    LogError("AddNewNodesAndRelativeFactor: Failed to get end timestamp.");
    return false;
  }
  return node_adder_model_.AddNodesAndRelativeFactors(*timestamp_a, timestamp, *nodes_, factors);
}

template <typename NodeType, typename TimestampedNodesType, typename NodeAdderModelType>
bool TimestampedNodeAdder<NodeType, TimestampedNodesType, NodeAdderModelType>::SplitOldRelativeFactor(
  const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& factors) {
  const auto timestamp_bounds = nodes_->LowerAndUpperBoundTimestamps(timestamp);
  // TODO(rsoussan): print upper and lower bound! print timestamp!
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

  const bool removed_old_factors =
    node_adder_model_.RemoveRelativeFactors(lower_bound_time, upper_bound_time, *nodes_, factors);
  if (!removed_old_factors) {
    LogError(
      "SplitOldRelativeFactor: Failed to remove "
      "old factors.");
    return false;
  }
  if (!node_adder_model_.AddNodesAndRelativeFactors(lower_bound_time, timestamp, *nodes_, factors)) {
    LogError("SplitOldRelativeFactor: Failed to add first relative node and factor.");
    return false;
  }
  if (!node_adder_model_.AddRelativeFactors(timestamp, upper_bound_time, *nodes_, factors)) {
    LogError("SplitOldRelativeFactor: Failed to add second relative factor.");
    return false;
  }
  return true;
}
}  // namespace node_adders

#endif  // NODE_ADDERS_TIMESTAMPED_NODE_ADDER_H_
