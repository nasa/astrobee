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

#include <localization_common/timestamped_set.h>
#include <localization_common/time.h>
#include <nodes/values.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include <boost/optional.hpp>
#include <boost/serialization/serialization.hpp>

#include <map>
#include <utility>
#include <vector>

namespace nodes {
template <typename NodeType>
// Container for timestamped nodes with multiple values per node.
// Enables a NodeAdder to couple multiple values at the same timestamp that are always added and updated together into a
// single combined node. For example, a visual-intertial NodeAdder may always add and update pose, velocity, and
// IMU-biases together and never individually, so these should be grouped into a combined node rather than treated
// separately. The Add and Node functions must be specialized for the desired combined node. For nodes with a single
// value, such as just a pose or position, use TimestampedNodes, which is a specialization of TimestampedCombinedNodes
// with CombinedType = false.
class TimestampedCombinedNodes {
 public:
  // Values should be provided by a graph optimizer and the same values
  // should be passed to all timestamped nodes in the graph.
  explicit TimestampedCombinedNodes(std::shared_ptr<Values> values);

  // For serialization only
  TimestampedCombinedNodes() = default;

  virtual ~TimestampedCombinedNodes() = default;

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
  boost::optional<T> Value(const gtsam::Key& key) const;

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

  // Returns the closest node timestamp to the provided timestamp.
  boost::optional<localization_common::Time> ClosestTimestamp(const localization_common::Time timestamp) const;

  // Returns the lower bounded or equal in time timestamped node to the provided timestamp.
  boost::optional<localization_common::TimestampedValue<NodeType>> LowerBoundOrEqual(
    const localization_common::Time timestamp) const;

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

  // Const accessor for internal gtsam values.
  const gtsam::Values& gtsam_values() const;

 protected:
  std::shared_ptr<Values> values_;

 private:
  // Removes a node with the provided keys if it exists.
  bool Remove(const gtsam::KeyVector& keys);

  // Adds a node and returns the keys associated with it.
  virtual gtsam::KeyVector AddNode(const NodeType& node) = 0;

  // Helper function that returns a node with the provied keys and timestamp if it exists.
  virtual boost::optional<NodeType> GetNode(const gtsam::KeyVector& keys,
                                            const localization_common::Time timestamp) const = 0;

  // Helper function that returns a node with the provided timestamped keys if it exists.
  boost::optional<NodeType> Node(const localization_common::TimestampedValue<gtsam::KeyVector>& timestamped_keys) const;

  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/);

  localization_common::TimestampedSet<gtsam::KeyVector> timestamp_keys_map_;
};

// Implementation
template <typename NodeType>
TimestampedCombinedNodes<NodeType>::TimestampedCombinedNodes(std::shared_ptr<Values> values)
    : values_(std::move(values)) {}

template <typename NodeType>
gtsam::KeyVector TimestampedCombinedNodes<NodeType>::Add(const localization_common::Time timestamp,
                                                         const NodeType& node) {
  if (Contains(timestamp)) return gtsam::KeyVector();
  gtsam::KeyVector keys = AddNode(node);
  timestamp_keys_map_.Add(timestamp, keys);
  return keys;
}

template <typename NodeType>
boost::optional<NodeType> TimestampedCombinedNodes<NodeType>::Node(const localization_common::Time timestamp) const {
  const auto keys = Keys(timestamp);
  if (keys.empty()) return boost::none;
  return GetNode(keys, timestamp);
}

template <typename NodeType>
boost::optional<NodeType> TimestampedCombinedNodes<NodeType>::Node(
  const localization_common::TimestampedValue<gtsam::KeyVector>& timestamped_keys) const {
  return GetNode(timestamped_keys.value, timestamped_keys.timestamp);
}

template <typename NodeType>
template <typename T>
boost::optional<T> TimestampedCombinedNodes<NodeType>::Value(const gtsam::Key& key) const {
  return values_->Value<T>(key);
}

template <typename NodeType>
gtsam::KeyVector TimestampedCombinedNodes<NodeType>::Keys(const localization_common::Time timestamp) const {
  if (!Contains(timestamp)) return {};
  return (timestamp_keys_map_.Get(timestamp))->value;
}

template <typename NodeType>
bool TimestampedCombinedNodes<NodeType>::Remove(const localization_common::Time& timestamp) {
  const auto value = timestamp_keys_map_.Get(timestamp);
  if (!value) return false;
  bool successful_removal = true;
  successful_removal = successful_removal && timestamp_keys_map_.Remove(timestamp);
  successful_removal = successful_removal && Remove(value->value);
  return successful_removal;
}

template <typename NodeType>
bool TimestampedCombinedNodes<NodeType>::Remove(const gtsam::KeyVector& keys) {
  return values_->Remove(keys);
}

template <typename NodeType>
size_t TimestampedCombinedNodes<NodeType>::size() const {
  return timestamp_keys_map_.size();
}

template <typename NodeType>
bool TimestampedCombinedNodes<NodeType>::empty() const {
  return timestamp_keys_map_.empty();
}

template <typename NodeType>
boost::optional<localization_common::Time> TimestampedCombinedNodes<NodeType>::OldestTimestamp() const {
  const auto oldest = timestamp_keys_map_.Oldest();
  if (!oldest) return boost::none;
  return oldest->timestamp;
}

template <typename NodeType>
boost::optional<NodeType> TimestampedCombinedNodes<NodeType>::OldestNode() const {
  const auto oldest = timestamp_keys_map_.Oldest();
  if (!oldest) return boost::none;
  return Node(*oldest);
}

template <typename NodeType>
boost::optional<localization_common::Time> TimestampedCombinedNodes<NodeType>::LatestTimestamp() const {
  const auto latest = timestamp_keys_map_.Latest();
  if (!latest) return boost::none;
  return latest->timestamp;
}

template <typename NodeType>
boost::optional<NodeType> TimestampedCombinedNodes<NodeType>::LatestNode() const {
  const auto latest = timestamp_keys_map_.Latest();
  if (!latest) return boost::none;
  return Node(*latest);
}

template <typename NodeType>
std::pair<boost::optional<localization_common::Time>, boost::optional<localization_common::Time>>
TimestampedCombinedNodes<NodeType>::LowerAndUpperBoundTimestamps(const localization_common::Time timestamp) const {
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

template <typename NodeType>
std::pair<boost::optional<NodeType>, boost::optional<NodeType>>
TimestampedCombinedNodes<NodeType>::LowerAndUpperBoundNodes(const localization_common::Time timestamp) const {
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

template <typename NodeType>
boost::optional<localization_common::TimestampedValue<NodeType>> TimestampedCombinedNodes<NodeType>::LowerBoundOrEqual(
  const localization_common::Time timestamp) const {
  const auto lower_bound_or_equal = timestamp_keys_map_.LowerBoundOrEqual(timestamp);
  if (!lower_bound_or_equal) return boost::none;
  const auto node = Node(lower_bound_or_equal->timestamp);
  if (!node) return boost::none;
  return localization_common::TimestampedValue<NodeType>(lower_bound_or_equal->timestamp, *node);
}

template <typename NodeType>
std::vector<localization_common::Time> TimestampedCombinedNodes<NodeType>::Timestamps() const {
  return timestamp_keys_map_.Timestamps();
}

template <typename NodeType>
double TimestampedCombinedNodes<NodeType>::Duration() const {
  return timestamp_keys_map_.Duration();
}

template <typename NodeType>
std::vector<NodeType> TimestampedCombinedNodes<NodeType>::OldNodes(
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

template <typename NodeType>
gtsam::KeyVector TimestampedCombinedNodes<NodeType>::OldKeys(
  const localization_common::Time oldest_allowed_timestamp) const {
  const auto old_timestamp_key_sets = timestamp_keys_map_.OldValues(oldest_allowed_timestamp);
  gtsam::KeyVector all_old_keys;
  for (const auto& old_timestamp_key_set : old_timestamp_key_sets) {
    const auto& old_keys = old_timestamp_key_set.value;
    all_old_keys.insert(all_old_keys.end(), old_keys.begin(), old_keys.end());
  }
  return all_old_keys;
}

template <typename NodeType>
int TimestampedCombinedNodes<NodeType>::RemoveOldNodes(const localization_common::Time oldest_allowed_timestamp) {
  const auto old_values = timestamp_keys_map_.OldValues(oldest_allowed_timestamp);
  timestamp_keys_map_.RemoveOldValues(oldest_allowed_timestamp);
  int num_removed_nodes = 0;
  for (const auto& old_value : old_values)
    if (Remove(old_value.value)) ++num_removed_nodes;
  return num_removed_nodes;
}

template <typename NodeType>
boost::optional<NodeType> TimestampedCombinedNodes<NodeType>::ClosestNode(
  const localization_common::Time timestamp) const {
  const auto closest = timestamp_keys_map_.Closest(timestamp);
  if (!closest) return boost::none;
  return Node(*closest);
}

template <typename NodeType>
boost::optional<localization_common::Time> TimestampedCombinedNodes<NodeType>::ClosestTimestamp(
  const localization_common::Time timestamp) const {
  const auto closest = timestamp_keys_map_.Closest(timestamp);
  if (!closest) return boost::none;
  return closest->timestamp;
}

template <typename NodeType>
const gtsam::Values& TimestampedCombinedNodes<NodeType>::gtsam_values() const {
  return values_->gtsam_values();
}

template <typename NodeType>
bool TimestampedCombinedNodes<NodeType>::Contains(const localization_common::Time timestamp) const {
  return timestamp_keys_map_.Contains(timestamp);
}

template <typename NodeType>
template <class ARCHIVE>
void TimestampedCombinedNodes<NodeType>::serialize(ARCHIVE& ar, const unsigned int /*version*/) {
  ar& BOOST_SERIALIZATION_NVP(values_);
  ar& BOOST_SERIALIZATION_NVP(timestamp_keys_map_);
}
}  // namespace nodes

#endif  // NODES_TIMESTAMPED_COMBINED_NODES_H_
