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

#ifndef NODES_TIMESTAMPED_COMBINED_NODES_H_
#define NODES_TIMESTAMPED_COMBINED_NODES_H_

#include <nodes/nodes.h>
#include <localization_common/timestamped_set.h>
#include <localization_common/time.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/optional.hpp>
#include <boost/serialization/serialization.hpp>

#include <map>
#include <utility>
#include <vector>

namespace nodes {
template <typename NodeType, bool CombinedType = true>
// Container for timestamped nodes with multiple values per node.
// Enables a NodeAdder to couple multiple values at the same timestamp that are always added and updated together into a
// single combined node. For example, a visual-intertial NodeAdder may always add and update pose, velocity, and
// IMU-biases together and never individually, so these should be grouped into a combined node rather than treated
// separately. The Add and Node functions must be specialized for the desired combined node. For nodes with a single
// value, such as just a pose or position, use TimestampedNodes, which is a specialization of TimestampedCombinedNodes
// with CombinedType = false.
class TimestampedCombinedNodes {
 public:
  // Nodes should be provided by a graph optimizer and the same nodes
  // should be passed to all timestamped nodes in the graph.
  explicit TimestampedCombinedNodes(std::shared_ptr<Nodes> nodes);

  // For serialization only
  TimestampedCombinedNodes() = default;

  // Add a node at the provided timestamp and return the GTSAM keys that correspond to it.
  // The keys are used in graph factors to connect a factor to a node.
  // Note that a single node can have multiple keys, for example a pose velocity node
  // may have a pose key and velocity key. The ordering of keys is set in the specialized
  // Add(node) function for the NodeType.
  gtsam::KeyVector Add(const localization_common::Time timestamp, const NodeType& node);

  // Returns a node at the provided timestamp if it exists.
  boost::optional<NodeType> Node(const localization_common::Time timestamp) const;

  // Returns a portion of a combined node (or a full non-combined node)
  // with the provided key if it exists.
  // To return a combined node, use Node(timestamp) instead.
  template <typename T>
  boost::optional<T> Node(const gtsam::Key& key) const;

  // Returns the keys for a timestamped node given the timestamp if it exists.
  // If not, an empty key vector is returned.
  gtsam::KeyVector Keys(const localization_common::Time timestamp) const;

  // Removes a node at the provided timestamp if it exists.
  // Returns if a node was successfully removed.
  bool Remove(const localization_common::Time& timestamp);

  // Returns the number of nodes. This does not return the number of values,
  // so if one combined node containing a pose and velocity exists, this will
  // return 1 for example.
  size_t size() const;

  // Returns if there node container is empty.
  bool empty() const;

  // Returns the oldest timestamp of the nodes in the container.
  // Returns boost::none if no nodes exists.
  boost::optional<localization_common::Time> OldestTimestamp() const;

  // Returns the oldest nodes in the container.
  // Returns boost::none if no nodes exists.
  boost::optional<NodeType> OldestNode() const;

  // Returns the latest timestamp of the nodes in the container.
  // Returns boost::none if no nodes exists.
  boost::optional<localization_common::Time> LatestTimestamp() const;

  // Returns the latest node in the container.
  // Returns boost::none if no nodes exists.
  boost::optional<NodeType> LatestNode() const;

  // Returns lower and upper time bounds for the provided timestamp.  Equal values are set as lower and upper bound.
  std::pair<boost::optional<localization_common::Time>, boost::optional<localization_common::Time>>
  LowerAndUpperBoundTimestamps(const localization_common::Time timestamp) const;

  // Return lower and upper node bounds for the provided timestamp.  Equal values are set as lower and upper bound.
  std::pair<boost::optional<NodeType>, boost::optional<NodeType>> LowerAndUpperBoundNodes(
    const localization_common::Time timestamp) const;

  // Returns the closest node in time to the provided timestamp.
  boost::optional<NodeType> ClosestNode(const localization_common::Time timestamp) const;

  // Returns the lower bounded or equal in time node to the provided timestamp.
  boost::optional<NodeType> LowerBoundOrEqualNode(const localization_common::Time timestamp) const;

  // Returns all nodes older than the provided oldest_allowed_timestamp.
  std::vector<NodeType> OldNodes(const localization_common::Time oldest_allowed_timestamp) const;

  // Returns keys for all nodes older than the provied oldest_allowed_timestamp.
  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_timestamp) const;

  // Removes nodes older than the provied timestamp.
  // Returns the number of removed nodes.
  int RemoveOldNodes(const localization_common::Time oldest_allowed_timestamp);

  // Returns a vector containing the timestamps of all nodes in the container.
  // Timestamps are sorted from oldest to latest.
  std::vector<localization_common::Time> Timestamps() const;

  // Returns the total duration of node timestamps in the container.
  double Duration() const;

  // Returns whether the container contains a node at the provided timestamp.
  bool Contains(const localization_common::Time timestamp) const;

 private:
  // Removes a node with the provided keys if it exists.
  bool Remove(const gtsam::KeyVector& keys);

  // Adds a node and returns the keys associated with it.
  gtsam::KeyVector Add(const NodeType& node);

  // Helper function that returns a node with the provied keys and timestamp if it exists.
  boost::optional<NodeType> Node(const gtsam::KeyVector& keys, const localization_common::Time timestamp) const;

  // Helper function that returns a node with the provided timestamped keys if it exists.
  boost::optional<NodeType> Node(const localization_common::TimestampedValue<gtsam::KeyVector>& timestamped_keys) const;

  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/);

  std::shared_ptr<Nodes> nodes_;
  localization_common::TimestampedSet<gtsam::KeyVector> timestamp_keys_map_;
};

// Implementation
template <typename NodeType, bool CombinedType>
TimestampedCombinedNodes<NodeType, CombinedType>::TimestampedCombinedNodes(std::shared_ptr<Nodes> nodes)
    : nodes_(std::move(nodes)) {}

template <typename NodeType, bool CombinedType>
gtsam::KeyVector TimestampedCombinedNodes<NodeType, CombinedType>::Add(const localization_common::Time timestamp,
                                                                       const NodeType& node) {
  if (Contains(timestamp)) return gtsam::KeyVector();
  gtsam::KeyVector keys = Add(node);
  timestamp_keys_map_.Add(timestamp, keys);
  return keys;
}

template <typename NodeType, bool CombinedType>
gtsam::KeyVector TimestampedCombinedNodes<NodeType, CombinedType>::Add(const NodeType& node) {
  static_assert(!CombinedType, "This needs to be specialized for combined types.");
  // Implementation for non-combined type
  const auto key = nodes_->Add(node);
  return {key};
}

template <typename NodeType, bool CombinedType>
boost::optional<NodeType> TimestampedCombinedNodes<NodeType, CombinedType>::Node(
  const gtsam::KeyVector& keys, const localization_common::Time timestamp) const {
  static_assert(!CombinedType, "This needs to be specialized for combined types.");
  // Implementation for non-combined type
  // Assumes keys only has a single key since using non-combined type
  return nodes_->Node<NodeType>(keys[0]);
}

template <typename NodeType, bool CombinedType>
boost::optional<NodeType> TimestampedCombinedNodes<NodeType, CombinedType>::Node(
  const localization_common::Time timestamp) const {
  const auto keys = Keys(timestamp);
  if (keys.empty()) return boost::none;
  return Node(keys, timestamp);
}

template <typename NodeType, bool CombinedType>
boost::optional<NodeType> TimestampedCombinedNodes<NodeType, CombinedType>::Node(
  const localization_common::TimestampedValue<gtsam::KeyVector>& timestamped_keys) const {
  return Node(timestamped_keys.value, timestamped_keys.timestamp);
}

template <typename NodeType, bool CombinedType>
template <typename T>
boost::optional<T> TimestampedCombinedNodes<NodeType, CombinedType>::Node(const gtsam::Key& key) const {
  return nodes_->Node<T>(key);
}

template <typename NodeType, bool CombinedType>
gtsam::KeyVector TimestampedCombinedNodes<NodeType, CombinedType>::Keys(
  const localization_common::Time timestamp) const {
  if (!Contains(timestamp)) return {};
  return (timestamp_keys_map_.Get(timestamp))->value;
}

template <typename NodeType, bool CombinedType>
bool TimestampedCombinedNodes<NodeType, CombinedType>::Remove(const localization_common::Time& timestamp) {
  const auto value = timestamp_keys_map_.Get(timestamp);
  if (!value) return false;
  bool successful_removal = true;
  successful_removal = successful_removal && timestamp_keys_map_.Remove(timestamp);
  successful_removal = successful_removal && Remove(value->value);
  return successful_removal;
}

template <typename NodeType, bool CombinedType>
bool TimestampedCombinedNodes<NodeType, CombinedType>::Remove(const gtsam::KeyVector& keys) {
  return nodes_->Remove(keys);
}

template <typename NodeType, bool CombinedType>
size_t TimestampedCombinedNodes<NodeType, CombinedType>::size() const {
  return timestamp_keys_map_.size();
}

template <typename NodeType, bool CombinedType>
bool TimestampedCombinedNodes<NodeType, CombinedType>::empty() const {
  return timestamp_keys_map_.empty();
}

template <typename NodeType, bool CombinedType>
boost::optional<localization_common::Time> TimestampedCombinedNodes<NodeType, CombinedType>::OldestTimestamp() const {
  const auto oldest = timestamp_keys_map_.Oldest();
  if (!oldest) return boost::none;
  return oldest->timestamp;
}

template <typename NodeType, bool CombinedType>
boost::optional<NodeType> TimestampedCombinedNodes<NodeType, CombinedType>::OldestNode() const {
  const auto oldest = timestamp_keys_map_.Oldest();
  if (!oldest) return boost::none;
  return Node(*oldest);
}

template <typename NodeType, bool CombinedType>
boost::optional<localization_common::Time> TimestampedCombinedNodes<NodeType, CombinedType>::LatestTimestamp() const {
  const auto latest = timestamp_keys_map_.Latest();
  if (!latest) return boost::none;
  return latest->timestamp;
}

template <typename NodeType, bool CombinedType>
boost::optional<NodeType> TimestampedCombinedNodes<NodeType, CombinedType>::LatestNode() const {
  const auto latest = timestamp_keys_map_.Latest();
  if (!latest) return boost::none;
  return Node(*latest);
}

template <typename NodeType, bool CombinedType>
std::pair<boost::optional<localization_common::Time>, boost::optional<localization_common::Time>>
TimestampedCombinedNodes<NodeType, CombinedType>::LowerAndUpperBoundTimestamps(
  const localization_common::Time timestamp) const {
  const auto lower_and_upper_bound = timestamp_keys_map_.LowerAndUpperBound(timestamp);
  boost::optional<localization_common::Time> lower_bound;
  if (!lower_and_upper_bound.first)
    lower_bound = boost::none;
  else
    lower_bound = lower_and_upper_bound.first->timestamp;
  boost::optional<localization_common::Time> upper_bound;
  if (!lower_and_upper_bound.second)
    upper_bound = boost::none;
  else
    upper_bound = lower_and_upper_bound.second->timestamp;
  return {lower_bound, upper_bound};
}

template <typename NodeType, bool CombinedType>
std::pair<boost::optional<NodeType>, boost::optional<NodeType>>
TimestampedCombinedNodes<NodeType, CombinedType>::LowerAndUpperBoundNodes(
  const localization_common::Time timestamp) const {
  const auto lower_and_upper_bound = timestamp_keys_map_.LowerAndUpperBound(timestamp);
  boost::optional<NodeType> lower_bound;
  if (!lower_and_upper_bound.first)
    lower_bound = boost::none;
  else
    lower_bound = Node(*(lower_and_upper_bound.first));
  boost::optional<NodeType> upper_bound;
  if (!lower_and_upper_bound.second)
    upper_bound = boost::none;
  else
    upper_bound = Node(*(lower_and_upper_bound.second));
  return {lower_bound, upper_bound};
}

template <typename NodeType, bool CombinedType>
boost::optional<NodeType> TimestampedCombinedNodes<NodeType, CombinedType>::LowerBoundOrEqualNode(
  const localization_common::Time timestamp) const {
  const auto lower_bound_or_equal = timestamp_keys_map_.LowerBoundOrEqual(timestamp);
  if (!lower_bound_or_equal) return boost::none;
  return Node(*lower_bound_or_equal);
}

template <typename NodeType, bool CombinedType>
std::vector<localization_common::Time> TimestampedCombinedNodes<NodeType, CombinedType>::Timestamps() const {
  return timestamp_keys_map_.Timestamps();
}

template <typename NodeType, bool CombinedType>
double TimestampedCombinedNodes<NodeType, CombinedType>::Duration() const {
  return timestamp_keys_map_.Duration();
}

template <typename NodeType, bool CombinedType>
std::vector<NodeType> TimestampedCombinedNodes<NodeType, CombinedType>::OldNodes(
  const localization_common::Time oldest_allowed_timestamp) const {
  const auto old_values = timestamp_keys_map_.OldValues(oldest_allowed_timestamp);
  std::vector<NodeType> old_nodes;
  for (const auto& old_value : old_values) {
    const auto old_node = Node(old_value);
    if (!old_node) {
      LogError("OldNodes: Failed to get node for keys.");
      continue;
    }
    old_nodes.emplace_back(*old_node);
  }
  return old_nodes;
}

template <typename NodeType, bool CombinedType>
gtsam::KeyVector TimestampedCombinedNodes<NodeType, CombinedType>::OldKeys(
  const localization_common::Time oldest_allowed_timestamp) const {
  const auto old_timestamp_key_sets = timestamp_keys_map_.OldValues(oldest_allowed_timestamp);
  gtsam::KeyVector all_old_keys;
  for (const auto& old_timestamp_key_set : old_timestamp_key_sets) {
    const auto& old_keys = old_timestamp_key_set.value;
    all_old_keys.insert(all_old_keys.end(), old_keys.begin(), old_keys.end());
  }
  return all_old_keys;
}

template <typename NodeType, bool CombinedType>
int TimestampedCombinedNodes<NodeType, CombinedType>::RemoveOldNodes(
  const localization_common::Time oldest_allowed_timestamp) {
  const auto old_values = timestamp_keys_map_.OldValues(oldest_allowed_timestamp);
  timestamp_keys_map_.RemoveOldValues(oldest_allowed_timestamp);
  int num_removed_nodes = 0;
  for (const auto& old_value : old_values)
    if (Remove(old_value.value)) ++num_removed_nodes;
  return num_removed_nodes;
}

template <typename NodeType, bool CombinedType>
boost::optional<NodeType> TimestampedCombinedNodes<NodeType, CombinedType>::ClosestNode(
  const localization_common::Time timestamp) const {
  const auto closest = timestamp_keys_map_.Closest(timestamp);
  if (!closest) return boost::none;
  return Node(*closest);
}

template <typename NodeType, bool CombinedType>
bool TimestampedCombinedNodes<NodeType, CombinedType>::Contains(const localization_common::Time timestamp) const {
  return timestamp_keys_map_.Contains(timestamp);
}

template <typename NodeType, bool CombinedType>
template <class ARCHIVE>
void TimestampedCombinedNodes<NodeType, CombinedType>::serialize(ARCHIVE& ar, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(nodes_);
  ar& BOOST_SERIALIZATION_NVP(timestamp_keys_map_);
}
}  // namespace nodes

#endif  // NODES_TIMESTAMPED_COMBINED_NODES_H_
