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

#ifndef GRAPH_OPTIMIZER_TIMESTAMPED_NODES_H_
#define GRAPH_OPTIMIZER_TIMESTAMPED_NODES_H_

#include <graph_optimizer/nodes.h>
#include <localization_common/time.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/optional.hpp>
#include <boost/serialization/serialization.hpp>

#include <map>
#include <utility>
#include <vector>

namespace graph_optimizer {
template <typename NodeType>
class TimestampedNodes {
 public:
  explicit TimestampedNodes(std::shared_ptr<Nodes> nodes);
  // For serialization only
  TimestampedNodes() {}

  boost::optional<gtsam::Key> Add(const localization_common::Time timestamp, const NodeType& node);

  bool Remove(const localization_common::Time timestamp);

  boost::optional<NodeType> Node(const localization_common::Time timestamp) const;

  boost::optional<gtsam::Key> Key(const localization_common::Time timestamp) const;

  size_t size() const;

  bool empty() const;

  boost::optional<localization_common::Time> OldestTimestamp() const;

  boost::optional<NodeType> OldestNode() const;

  boost::optional<localization_common::Time> LatestTimestamp() const;

  boost::optional<NodeType> LatestNode() const;

  // Return lower and upper bounds.  Equal values are set as upper bound only.
  std::pair<boost::optional<localization_common::Time>, boost::optional<localization_common::Time>>
  LowerAndUpperBoundTimestamps(const localization_common::Time timestamp) const;

  // Return lower and upper bounds.  Equal values are set as upper bound only.
  std::pair<boost::optional<NodeType>, boost::optional<NodeType>> LowerAndUpperBoundNodes(
    const localization_common::Time timestamp) const;

  boost::optional<localization_common::Time> ClosestTimestamp(const localization_common::Time timestamp) const;

  boost::optional<NodeType> ClosestNode(const localization_common::Time timestamp) const;

  boost::optional<localization_common::Time> LowerBoundOrEqualTimestamp(
    const localization_common::Time timestamp) const;

  boost::optional<NodeType> LowerBoundOrEqualNode(const localization_common::Time timestamp) const;

  std::vector<localization_common::Time> OldTimestamps(const localization_common::Time oldest_allowed_timestamp) const;

  std::vector<NodeType> OldNodes(const localization_common::Time oldest_allowed_timestamp) const;

  gtsam::KeyVector OldKeys(const localization_common::Time timestamp) const;

  int RemoveOldNodes(const localization_common::Time oldest_allowed_timestamp);

  std::vector<localization_common::Time> Timestamps() const;

  double Duration() const;

  bool Contains(const localization_common::Time timestamp) const;

 private:
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/);

  std::shared_ptr<Nodes> nodes_;
  std::map<localization_common::Time, gtsam::Key> timestamp_key_map_;
};

// Implementation
template <typename NodeType>
TimestampedNodes<NodeType>::TimestampedNodes(std::shared_ptr<Nodes> nodes) : nodes_(std::move(nodes)) {}

template <typename NodeType>
boost::optional<gtsam::Key> TimestampedNodes<NodeType>::Add(const localization_common::Time timestamp,
                                                            const NodeType& node) {
  if (Contains(timestamp)) return false;
  const auto key = nodes_->Add(node);
  timestamp_key_map_.emplace(timestamp, key);
  return key;
}

template <typename NodeType>
bool TimestampedNodes<NodeType>::Remove(const localization_common::Time timestamp) {
  if (!Contains(timestamp)) return false;
  const auto key = timestamp_key_map_[timestamp];
  timestamp_key_map_.erase(timestamp);
  nodes_->Remove(key);
  return true;
}

template <typename NodeType>
boost::optional<NodeType> TimestampedNodes<NodeType>::Node(const localization_common::Time timestamp) const {
  if (!Contains(timestamp)) return boost::none;
  const auto key = Key(timestamp);
  return nodes_->Node<NodeType>(*key);
}

template <typename NodeType>
boost::optional<gtsam::Key> TimestampedNodes<NodeType>::Key(const localization_common::Time timestamp) const {
  if (!Contains(timestamp)) return boost::none;
  return timestamp_key_map_.at(timestamp);
}

template <typename NodeType>
size_t TimestampedNodes<NodeType>::size() const {
  return timestamp_key_map_.size();
}

template <typename NodeType>
bool TimestampedNodes<NodeType>::empty() const {
  return timestamp_key_map_.empty();
}

template <typename NodeType>
boost::optional<localization_common::Time> TimestampedNodes<NodeType>::OldestTimestamp() const {
  if (empty()) {
    LogDebug("OldestTimestamp: No timestamps available.");
    return boost::none;
  }
  return timestamp_key_map_.cbegin()->first;
}

template <typename NodeType>
boost::optional<NodeType> TimestampedNodes<NodeType>::OldestNode() const {
  const auto oldest_timestamp = OldestTimestamp();
  if (!oldest_timestamp) return boost::none;
  return Node(*oldest_timestamp);
}

template <typename NodeType>
boost::optional<localization_common::Time> TimestampedNodes<NodeType>::LatestTimestamp() const {
  if (empty()) {
    LogDebug("LatestTimestamp: No timestamps available.");
    return boost::none;
  }
  return timestamp_key_map_.crbegin()->first;
}

template <typename NodeType>
boost::optional<NodeType> TimestampedNodes<NodeType>::LatestNode() const {
  const auto latest_timestamp = LatestTimestamp();
  if (!latest_timestamp) return boost::none;
  return Node(*latest_timestamp);
}

template <typename NodeType>
std::pair<boost::optional<localization_common::Time>, boost::optional<localization_common::Time>>
TimestampedNodes<NodeType>::LowerAndUpperBoundTimestamps(const localization_common::Time timestamp) const {
  if (empty()) {
    LogDebug("LowerAndUpperBoundTimestamps: No timestamps available.");
    return {boost::none, boost::none};
  }

  // lower bound returns first it >= query, call this upper bound
  const auto upper_bound_it = timestamp_key_map_.lower_bound(timestamp);
  if (upper_bound_it == timestamp_key_map_.cend()) {
    LogDebug("LowerAndUpperBoundTimestamps: No upper bound timestamp exists.");
    const localization_common::Time lower_bound_time = (timestamp_key_map_.crbegin())->first;
    return {boost::optional<localization_common::Time>(lower_bound_time), boost::none};
  } else if (upper_bound_it == timestamp_key_map_.cbegin()) {
    LogDebug("LowerAndUpperBoundTimestamps: No lower bound timestamp exists.");
    return {boost::none, boost::optional<localization_common::Time>(upper_bound_it->first)};
  }
  const auto lower_bound_it = std::prev(upper_bound_it);

  return {lower_bound_it->first, upper_bound_it->first};
}

template <typename NodeType>
std::pair<boost::optional<NodeType>, boost::optional<NodeType>> TimestampedNodes<NodeType>::LowerAndUpperBoundNodes(
  const localization_common::Time timestamp) const {
  const auto lower_and_upper_bound_timestamps = LowerAndUpperBoundTimestamps(timestamp);
  boost::optional<NodeType> lower_bound;
  if (!lower_and_upper_bound_timestamps.first)
    lower_bound = boost::none;
  else
    lower_bound = Node(*(lower_and_upper_bound_timestamps.first));
  boost::optional<NodeType> upper_bound;
  if (!lower_and_upper_bound_timestamps.second)
    upper_bound = boost::none;
  else
    upper_bound = Node(*(lower_and_upper_bound_timestamps.second));
  return {lower_bound, upper_bound};
}

template <typename NodeType>
boost::optional<localization_common::Time> TimestampedNodes<NodeType>::LowerBoundOrEqualTimestamp(
  const localization_common::Time timestamp) const {
  const auto lower_and_upper_bound_timestamp = LowerAndUpperBoundTimestamps(timestamp);
  if (!lower_and_upper_bound_timestamp.first && !lower_and_upper_bound_timestamp.second) {
    LogDebug("LowerBoundOrEqualTimestamp: Failed to get lower or upper bound timestamps.");
    return boost::none;
  }

  // Only return upper bound timestamp if it is equal to timestamp
  if (lower_and_upper_bound_timestamp.second && *(lower_and_upper_bound_timestamp.second) == timestamp) {
    return lower_and_upper_bound_timestamp.second;
  }

  return lower_and_upper_bound_timestamp.first;
}

template <typename NodeType>
boost::optional<NodeType> TimestampedNodes<NodeType>::LowerBoundOrEqualNode(
  const localization_common::Time timestamp) const {
  const auto lower_bound_or_equal_timestamp = LowerBoundOrEqualTimestamp(timestamp);
  if (!lower_bound_or_equal_timestamp) return boost::none;
  return Node(*lower_bound_or_equal_timestamp);
}

template <typename NodeType>
std::vector<localization_common::Time> TimestampedNodes<NodeType>::Timestamps() const {
  std::vector<localization_common::Time> timestamps;
  for (const auto& timestamp_key_pair : timestamp_key_map_) {
    timestamps.emplace_back(timestamp_key_pair.first);
  }
  return timestamps;
}

template <typename NodeType>
double TimestampedNodes<NodeType>::Duration() const {
  if (empty()) return 0;
  return (*LatestTimestamp() - *OldestTimestamp());
}

template <typename NodeType>
boost::optional<localization_common::Time> TimestampedNodes<NodeType>::ClosestTimestamp(
  const localization_common::Time timestamp) const {
  if (empty()) {
    LogDebug("ClosestTimestamp: No nodes available.");
    return boost::none;
  }

  const auto lower_and_upper_bound_timestamp = LowerAndUpperBoundTimestamps(timestamp);
  if (!lower_and_upper_bound_timestamp.first && !lower_and_upper_bound_timestamp.second) {
    LogDebug("ClosestTimestamp: Failed to get lower or upper bound timestamp.");
    return boost::none;
  }

  localization_common::Time closest_timestamp;
  if (!lower_and_upper_bound_timestamp.first) {
    closest_timestamp = *(lower_and_upper_bound_timestamp.second);
  } else if (!lower_and_upper_bound_timestamp.second) {
    closest_timestamp = *(lower_and_upper_bound_timestamp.first);
  } else {
    const localization_common::Time lower_bound_timestamp = *(lower_and_upper_bound_timestamp.first);
    const localization_common::Time upper_bound_timestamp = *(lower_and_upper_bound_timestamp.second);
    const double upper_bound_dt = std::abs(timestamp - upper_bound_timestamp);
    const double lower_bound_dt = std::abs(timestamp - lower_bound_timestamp);
    closest_timestamp = (upper_bound_dt < lower_bound_dt) ? upper_bound_timestamp : lower_bound_timestamp;
  }

  return closest_timestamp;
}

template <typename NodeType>
std::vector<localization_common::Time> TimestampedNodes<NodeType>::OldTimestamps(
  const localization_common::Time oldest_allowed_timestamp) const {
  std::vector<localization_common::Time> old_timestamps;
  for (const auto& timestamp_key_pair : timestamp_key_map_) {
    if (timestamp_key_pair.first >= oldest_allowed_timestamp) break;
    old_timestamps.emplace_back(timestamp_key_pair.first);
  }

  return old_timestamps;
}

template <typename NodeType>
std::vector<NodeType> TimestampedNodes<NodeType>::OldNodes(
  const localization_common::Time oldest_allowed_timestamp) const {
  const auto old_timestamps = OldTimestamps(oldest_allowed_timestamp);
  std::vector<NodeType> old_nodes;
  for (const auto old_timestamp : old_timestamps) {
    const auto old_node = Node(old_timestamp);
    if (!old_node) {
      LogError("OldNodes: Failed to get node for timestamp " << std::setprecision(15) << old_timestamp);
      continue;
    }
    old_nodes.emplace_back(*old_node);
  }
  return old_nodes;
}

template <typename NodeType>
gtsam::KeyVector TimestampedNodes<NodeType>::OldKeys(const localization_common::Time oldest_allowed_timestamp) const {
  const auto old_timestamps = OldTimestamps(oldest_allowed_timestamp);
  gtsam::KeyVector old_keys;
  for (const auto old_timestamp : old_timestamps) old_keys.emplace_back(timestamp_key_map_.at(old_timestamp));
  return old_keys;
}

template <typename NodeType>
int TimestampedNodes<NodeType>::RemoveOldNodes(const localization_common::Time oldest_allowed_timestamp) {
  const auto old_timestamps = OldTimestamps(oldest_allowed_timestamp);
  int num_removed_nodes = 0;
  for (const auto old_timestamp : old_timestamps)
    if (Remove(old_timestamp)) ++num_removed_nodes;
  return num_removed_nodes;
}

template <typename NodeType>
boost::optional<NodeType> TimestampedNodes<NodeType>::ClosestNode(const localization_common::Time timestamp) const {
  const auto closest_timestamp = ClosestTimestamp(timestamp);
  if (!closest_timestamp) return boost::none;
  return Node(*closest_timestamp);
}

template <typename NodeType>
bool TimestampedNodes<NodeType>::Contains(const localization_common::Time timestamp) const {
  return timestamp_key_map_.count(timestamp) > 0;
}

template <typename NodeType>
template <class ARCHIVE>
void TimestampedNodes<NodeType>::serialize(ARCHIVE& ar, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(nodes_);
  ar& BOOST_SERIALIZATION_NVP(timestamp_key_map_);
}
}  // namespace graph_optimizer

#endif  // GRAPH_OPTIMIZER_TIMESTAMPED_NODES_H_
