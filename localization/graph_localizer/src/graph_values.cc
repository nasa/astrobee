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

#include <graph_localizer/graph_values.h>

#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/ImuBias.h>
#include <gtsam/navigation/NavState.h>

#include <glog/logging.h>

#include <iomanip>

namespace graph_localizer {
namespace lc = localization_common;
GraphValues::GraphValues(const GraphValuesParams& params) : params_(params), feature_key_index_(0) {
  VLOG(2) << "GraphValues: Window duration: " << params_.ideal_duration;
  VLOG(2) << "GraphValues: Window min num states: " << params_.min_num_states;
}

boost::optional<lc::CombinedNavState> GraphValues::LatestCombinedNavState() const {
  if (Empty()) {
    LOG(ERROR) << "LatestCombinedNavState: No combined nav states available.";
    return boost::none;
  }

  const lc::Time timestamp = timestamp_key_index_map_.crbegin()->first;
  return GetCombinedNavState(timestamp);
}

boost::optional<lc::CombinedNavState> GraphValues::OldestCombinedNavState() const {
  if (Empty()) {
    LOG(ERROR) << "OldestCombinedNavState: No combined nav states available.";
    return boost::none;
  }
  const lc::Time timestamp = timestamp_key_index_map_.cbegin()->first;
  return GetCombinedNavState(timestamp);
}

boost::optional<lc::Time> GraphValues::OldestTimestamp() const {
  if (Empty()) {
    LOG(ERROR) << "OldestTimestamp: No states available.";
    return boost::none;
  }
  return timestamp_key_index_map_.cbegin()->first;
}

boost::optional<lc::Time> GraphValues::LatestTimestamp() const {
  if (Empty()) {
    LOG(ERROR) << "LatestTimestamp: No states available.";
    return boost::none;
  }
  return timestamp_key_index_map_.crbegin()->first;
}

boost::optional<lc::Time> GraphValues::ClosestPoseTimestamp(const lc::Time timestamp) const {
  if (Empty()) {
    LOG(ERROR) << "ClosestPoseTimestamp: No states available.";
    return boost::none;
  }

  const auto lower_and_upper_bound_timestamp = LowerAndUpperBoundTimestamp(timestamp);
  if (!lower_and_upper_bound_timestamp.first && !lower_and_upper_bound_timestamp.second) {
    LOG(ERROR) << "ClosestPoseTimestamp: Failed to get lower or upper bound timestamp.";
    return boost::none;
  }

  lc::Time closest_timestamp;
  if (!lower_and_upper_bound_timestamp.first) {
    closest_timestamp = *(lower_and_upper_bound_timestamp.second);
  } else if (!lower_and_upper_bound_timestamp.second) {
    closest_timestamp = *(lower_and_upper_bound_timestamp.first);
  } else {
    const lc::Time lower_bound_timestamp = *(lower_and_upper_bound_timestamp.first);
    const lc::Time upper_bound_timestamp = *(lower_and_upper_bound_timestamp.second);
    const double upper_bound_dt = std::abs(timestamp - upper_bound_timestamp);
    const double lower_bound_dt = std::abs(timestamp - lower_bound_timestamp);
    closest_timestamp = (upper_bound_dt < lower_bound_dt) ? upper_bound_timestamp : lower_bound_timestamp;
  }

  VLOG(2) << "ClosestPoseTimestamp: dt is " << std::abs(timestamp - closest_timestamp);
  return closest_timestamp;
}

std::pair<boost::optional<lc::Time>, boost::optional<lc::Time>> GraphValues::LowerAndUpperBoundTimestamp(
  const lc::Time timestamp) const {
  if (Empty()) {
    LOG(ERROR) << "LowerAndUpperBoundTimestamp: No states available.";
    return {boost::none, boost::none};
  }

  // lower bound returns first it >= query, call this upper bound
  const auto upper_bound_it = timestamp_key_index_map_.lower_bound(timestamp);
  if (upper_bound_it == timestamp_key_index_map_.cend()) {
    VLOG(2) << "LowerAndUpperBoundTimestamp: No upper bound timestamp exists.";
    const lc::Time lower_bound_time = (timestamp_key_index_map_.crbegin())->first;
    return {boost::optional<lc::Time>(lower_bound_time), boost::none};
  } else if (upper_bound_it == timestamp_key_index_map_.cbegin()) {
    VLOG(2) << "LowerAndUpperBoundTimestamp: No lower bound timestamp exists.";
    return {boost::none, boost::optional<lc::Time>(upper_bound_it->first)};
  }
  const auto lower_bound_it = std::prev(upper_bound_it);

  return {lower_bound_it->first, upper_bound_it->first};
}

boost::optional<gtsam::Key> GraphValues::GetKey(KeyCreatorFunction key_creator_function,
                                                const localization_common::Time timestamp) const {
  if (timestamp_key_index_map_.count(timestamp) == 0) {
    LOG(ERROR) << "GetKey: No key index found at timestamp.";
    return boost::none;
  }

  const int key_index = timestamp_key_index_map_.at(timestamp);

  const auto key = key_creator_function(key_index);
  if (!values_.exists(key)) {
    LOG(ERROR) << "GetKey: Key not present in values.";
    return boost::none;
  }

  return key;
}

bool GraphValues::HasKey(const lc::Time timestamp) const { return (timestamp_key_index_map_.count(timestamp) != 0); }

bool GraphValues::Empty() const { return timestamp_key_index_map_.empty(); }

boost::optional<gtsam::Key> GraphValues::PoseKey(const lc::Time timestamp) const { return GetKey(&sym::P, timestamp); }

boost::optional<lc::CombinedNavState> GraphValues::GetCombinedNavState(const lc::Time timestamp) const {
  if (!HasKey(timestamp)) {
    LOG(ERROR) << "GetCombinedNavState: No CombinedNavState found at timestamp.";
    return boost::none;
  }

  const auto key_index = KeyIndex(timestamp);
  if (!key_index) {
    LOG(ERROR) << "GetCombinedNavState: Failed to get key index.";
    return boost::none;
  }

  const auto pose = at<gtsam::Pose3>(sym::P(*key_index));
  if (!pose) {
    LOG(ERROR) << "GetCombinedNavState: Failed to get pose for key index.";
    return boost::none;
  }

  const auto velocity = at<gtsam::Velocity3>(sym::V(*key_index));
  if (!velocity) {
    LOG(ERROR) << "GetCombinedNavState: Failed to get velocity for key index.";
    return boost::none;
  }

  const auto bias = at<gtsam::imuBias::ConstantBias>(sym::B(*key_index));
  if (!bias) {
    LOG(ERROR) << "GetCombinedNavState: Failed to get bias for key index.";
    return boost::none;
  }

  return lc::CombinedNavState{*pose, *velocity, *bias, timestamp};
}

double GraphValues::Duration() const {
  if (Empty()) return 0;
  return (*LatestTimestamp() - *OldestTimestamp());
}

int GraphValues::NumStates() const { return timestamp_key_index_map_.size(); }

boost::optional<lc::Time> GraphValues::Timestamp(const int key_index) const {
  for (const auto& timestamp_key_index_pair : timestamp_key_index_map_) {
    if (timestamp_key_index_pair.second == key_index) return timestamp_key_index_pair.first;
  }
  return boost::none;
}

bool GraphValues::HasFeature(const localization_measurements::FeatureId id) const {
  return (feature_ids_.count(id) > 0);
}

bool GraphValues::AddFeature(const localization_measurements::FeatureId id, const gtsam::Point3& feature_point) {
  if (HasFeature(id)) {
    LOG(ERROR) << "AddFeature: Feature already exists.";
    return false;
  }

  const auto key = sym::F(feature_key_index_++);
  if (values_.exists(key)) {
    LOG(ERROR) << "AddFeature: Key already exists in values.";
  }

  feature_ids_.emplace(id, key);
  values_.insert(key, feature_point);
  return true;
}

boost::optional<int> GraphValues::LatestCombinedNavStateKeyIndex() const {
  if (Empty()) {
    LOG(ERROR) << "LatestCombinedNavStateKeyIndex: No combined nav states available.";
    return boost::none;
  }
  return timestamp_key_index_map_.crbegin()->second;
}

boost::optional<int> GraphValues::OldestCombinedNavStateKeyIndex() const {
  if (Empty()) {
    LOG(ERROR) << "OldestCombinedNavStateKeyIndex: No combined nav states available.";
    return boost::none;
  }
  return timestamp_key_index_map_.cbegin()->second;
}

boost::optional<std::pair<gtsam::imuBias::ConstantBias, lc::Time>> GraphValues::LatestBias() const {
  if (Empty()) {
    LOG(ERROR) << "LatestBias: No bias values available.";
    return boost::none;
  }

  const lc::Time timestamp = timestamp_key_index_map_.crbegin()->first;
  const int key_index = timestamp_key_index_map_.crbegin()->second;

  if (!values_.exists(sym::B(key_index))) {
    LOG(ERROR) << "LatestBias: Bias key not present in values.";
    return boost::none;
  }

  const auto bias = at<gtsam::imuBias::ConstantBias>(sym::B(key_index));
  if (!bias) {
    LOG(ERROR) << "LatestBias: Failed to get bias at key index.";
    return boost::none;
  }

  return std::pair<gtsam::imuBias::ConstantBias, lc::Time>{*bias, timestamp};
}

boost::optional<lc::Time> GraphValues::LowerBoundOrEqualTimestamp(const lc::Time timestamp) const {
  const auto lower_and_upper_bound_timestamp = LowerAndUpperBoundTimestamp(timestamp);
  if (!lower_and_upper_bound_timestamp.first && !lower_and_upper_bound_timestamp.second) {
    LOG(ERROR) << "LowerBoundOrEqualTimestamp: Failed to get lower or upper bound timestamps.";
    return boost::none;
  }

  // Only return upper bound timestamp if it is equal to timestamp
  if (lower_and_upper_bound_timestamp.second && *(lower_and_upper_bound_timestamp.second) == timestamp) {
    return lower_and_upper_bound_timestamp.second;
  }

  return lower_and_upper_bound_timestamp.first;
}

boost::optional<lc::CombinedNavState> GraphValues::LowerBoundOrEqualCombinedNavState(const lc::Time timestamp) const {
  const auto lower_bound_or_equal_timestamp = LowerBoundOrEqualTimestamp(timestamp);
  if (!lower_bound_or_equal_timestamp) {
    LOG(ERROR) << "LowerBoundOrEqualCombinedNavState: Failed to get lower bound or equal timestamp.";
    return boost::none;
  }

  return GetCombinedNavState(*lower_bound_or_equal_timestamp);
}

boost::optional<lc::Time> GraphValues::SlideWindowNewOldestTime() const {
  if (Empty()) {
    LOG(WARNING) << "SlideWindowOldestTime: No states in map.";
    return boost::none;
  }

  if (NumStates() <= params_.min_num_states) {
    LOG(WARNING) << "SlideWindowOldestTime: Not enough states to remove.";
    return boost::none;
  }

  const double total_duration = timestamp_key_index_map_.crbegin()->first - timestamp_key_index_map_.cbegin()->first;
  VLOG(2) << "SlideWindowOldestTime: Starting total num states: " << timestamp_key_index_map_.size();
  VLOG(2) << "SlideWindowOldestTime: Starting total duration is " << total_duration;
  const lc::Time ideal_oldest_allowed_state =
    std::max(0.0, timestamp_key_index_map_.crbegin()->first - params_.ideal_duration);

  int num_states_to_be_removed = 0;
  // Ensures that new oldest time is consistent with a number of states <= max_num_states
  // and >= min_num_states.
  // Assumes min_num_states < max_num_states.
  for (const auto& timestamp_key_pair : timestamp_key_index_map_) {
    ++num_states_to_be_removed;
    const int new_num_states = NumStates() - num_states_to_be_removed;
    if (new_num_states > params_.max_num_states) continue;
    const auto& time = timestamp_key_pair.first;
    if (new_num_states <= params_.min_num_states) return time;
    if (time >= ideal_oldest_allowed_state) return time;
  }

  // Shouldn't occur
  return boost::none;
}

// Add timestamp and keys to timestamp_key_index_map, and values to values
bool GraphValues::AddCombinedNavState(const lc::CombinedNavState& combined_nav_state, const int key_index) {
  if (HasKey(combined_nav_state.timestamp())) {
    LOG(ERROR) << "AddCombinedNavState: Timestamp key index map already "
                  "contains timestamp.";
    return false;
  }
  timestamp_key_index_map_.emplace(combined_nav_state.timestamp(), key_index);
  if (values_.exists(sym::P(key_index))) {
    LOG(ERROR) << "AddCombinedNavState: Pose key already in values.";
    return false;
  }
  if (values_.exists(sym::V(key_index))) {
    LOG(ERROR) << "AddCombinedNavState: Velocity key already in values.";
    return false;
  }
  if (values_.exists(sym::B(key_index))) {
    LOG(ERROR) << "AddCombinedNavState: Bias key already in values.";
    return false;
  }

  values_.insert(sym::P(key_index), combined_nav_state.pose());
  values_.insert(sym::V(key_index), combined_nav_state.velocity());
  values_.insert(sym::B(key_index), combined_nav_state.bias());

  VLOG(2) << "AddCombinedNavState: Added key_index " << key_index;
  VLOG(2) << "AddCombinedNavState: Added timestamp " << std::setprecision(15) << combined_nav_state.timestamp();
  return true;
}

boost::optional<int> GraphValues::KeyIndex(const lc::Time timestamp) const {
  if (!HasKey(timestamp)) {
    LOG(ERROR) << "KeyIndex: No key found for timestamp.";
    return boost::none;
  }

  return timestamp_key_index_map_.at(timestamp);
}

void GraphValues::UpdateValues(const gtsam::Values& new_values) { values_ = new_values; }

gtsam::NonlinearFactorGraph GraphValues::RemoveOldFactors(const gtsam::KeyVector& old_keys,
                                                          gtsam::NonlinearFactorGraph& graph) {
  gtsam::NonlinearFactorGraph removed_factors;
  for (auto factor_it = graph.begin(); factor_it != graph.end();) {
    bool found_key = false;
    for (const auto& key : old_keys) {
      if ((*factor_it)->find(key) != (*factor_it)->end()) {
        found_key = true;
        break;
      }
    }
    if (found_key) {
      removed_factors.push_back(*factor_it);
      factor_it = graph.erase(factor_it);
    } else {
      ++factor_it;
    }
  }

  return removed_factors;
}

int GraphValues::RemoveOldCombinedNavStates(const lc::Time oldest_allowed_time) {
  int num_states_removed = 0;
  while (timestamp_key_index_map_.begin()->first < oldest_allowed_time) {
    RemoveCombinedNavState(timestamp_key_index_map_.begin()->first);
    ++num_states_removed;
  }
  VLOG(2) << "RemoveOldCombinedNavStates: New total num states: " << timestamp_key_index_map_.size();
  const double new_total_duration =
    timestamp_key_index_map_.crbegin()->first - timestamp_key_index_map_.cbegin()->first;
  VLOG(2) << "RemoveOldCombinedNavStates: New total duration is " << new_total_duration;
  VLOG(2) << "RemoveOldCombinedNavStates: Num states removed: " << num_states_removed;
  return num_states_removed;
}

gtsam::KeyVector GraphValues::OldKeys(const lc::Time oldest_allowed_time) const {
  gtsam::KeyVector old_keys;
  for (const auto& timestamp_key_index_pair : timestamp_key_index_map_) {
    if (timestamp_key_index_pair.first >= oldest_allowed_time) break;
    const auto& key_index = timestamp_key_index_pair.second;
    old_keys.emplace_back(sym::P(key_index));
    old_keys.emplace_back(sym::V(key_index));
    old_keys.emplace_back(sym::B(key_index));
  }

  return old_keys;
}

// Removes keys from timestamp_key_index_map, values from values
// Assumes for each stamped_key_index there is a Pose, Velocity, and Bias key
bool GraphValues::RemoveCombinedNavState(const lc::Time timestamp) {
  if (!HasKey(timestamp)) {
    LOG(ERROR) << "RemoveCombinedNavState: Timestamp not found in timestamp "
                  "key index map.";
    return false;
  }
  const int key_index = timestamp_key_index_map_.at(timestamp);
  timestamp_key_index_map_.erase(timestamp);
  bool removed_values = true;

  // Remove key/value pairs from values
  if (values_.exists(sym::P(key_index))) {
    values_.erase(sym::P(key_index));
  } else {
    LOG(ERROR) << "RemoveCombinedNavState: Pose key not present in values.";
    removed_values = false;
  }
  if (values_.exists(sym::V(key_index))) {
    values_.erase(sym::V(key_index));
  } else {
    LOG(ERROR) << "RemoveCombinedNavState: Velocity key not present in values.";
    removed_values = false;
  }
  if (values_.exists(sym::B(key_index))) {
    values_.erase(sym::B(key_index));
  } else {
    LOG(ERROR) << "RemoveCombinedNavState: Bias key not present in values.";
    removed_values = false;
  }

  VLOG(2) << "RemoveCombinedNavState: Removed key index " << key_index;
  VLOG(2) << "RemoveCombinedNavState: Removed timestamp" << std::setprecision(15) << timestamp;
  return removed_values;
}
}  // namespace graph_localizer
