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
namespace lm = localization_measurements;
GraphValues::GraphValues(const double window_ideal_duration, const int window_min_num_states)
    : kWindowIdealDuration(window_ideal_duration), kWindowMinNumStates(window_min_num_states) {
  DLOG(INFO) << "GraphValues: Window duration: " << kWindowIdealDuration;
  DLOG(INFO) << "GraphValues: Window min num states: " << kWindowMinNumStates;
}

// Removes keys from timestamp map, values from values.
// Also removes any factors using these keys from graph argument
bool GraphValues::RemoveCombinedNavStateAndFactors(const lc::Time timestamp, gtsam::NonlinearFactorGraph& graph) {
  if (!HasKey(timestamp)) {
    LOG(WARNING) << "RemoveCombinedNavStateAndFactors: No key index for "
                    "timestamp exists.";
    return false;
  }

  const int key_index = timestamp_key_index_map_.at(timestamp);
  bool successful = RemoveCombinedNavState(timestamp);
  int removed_factors = 0;
  for (auto factor_it = graph.begin(); factor_it != graph.end();) {
    if (ContainsCombinedNavStateKey(**factor_it, key_index)) {
      factor_it = graph.erase(factor_it);
      ++removed_factors;
      continue;
    }
    ++factor_it;
  }
  DLOG(INFO) << "RemoveCombinedNavStateAndFactors: Removed " << removed_factors << " factors.";
  return successful;
}

lm::CombinedNavState GraphValues::LatestCombinedNavState() const {
  // TODO(rsoussan): account for this better -> use boost::optional?
  if (Empty()) {
    LOG(FATAL) << "LatestCombinedNavState: No combined nav states available.";
  }
  const lc::Time timestamp = timestamp_key_index_map_.crbegin()->first;
  return GetCombinedNavState(timestamp);
}

lm::CombinedNavState GraphValues::OldestCombinedNavState() const {
  // TODO(rsoussan): account for this better -> use boost::optional?
  if (Empty()) {
    LOG(FATAL) << "OldestCombinedNavState: No combined nav states available.";
  }
  const lc::Time timestamp = timestamp_key_index_map_.cbegin()->first;
  return GetCombinedNavState(timestamp);
}

lc::Time GraphValues::OldestTimestamp() const {
  // TODO(rsoussan): account for this better -> use boost::optional?
  if (Empty()) {
    LOG(FATAL) << "OldestTimestamp: No states available.";
  }
  return timestamp_key_index_map_.cbegin()->first;
}

lc::Time GraphValues::LatestTimestamp() const {
  // TODO(rsoussan): account for this better -> use boost::optional?
  if (Empty()) {
    LOG(FATAL) << "LatestTimestamp: No states available.";
  }
  return timestamp_key_index_map_.crbegin()->first;
}

// TODO(rsoussan): unify this with lowerandupperboundtimestamp?
lc::Time GraphValues::ClosestPoseTimestamp(const lc::Time timestamp) const {
  // TODO(rsoussan): account for this better -> use boost::optional?
  if (Empty()) {
    LOG(FATAL) << "ClosestPoseTimestamp: No states available.";
  }

  // Find closest timestamped pose
  int key_index;
  lc::Time closest_timestamp;
  // lower bound returns first it >= query, call this upper bound
  const auto upper_bound_it = timestamp_key_index_map_.lower_bound(timestamp);
  if (upper_bound_it == timestamp_key_index_map_.cend()) {
    key_index = std::prev(upper_bound_it)->second;
    closest_timestamp = std::prev(upper_bound_it)->first;
  } else if (upper_bound_it == timestamp_key_index_map_.cbegin()) {
    key_index = upper_bound_it->second;
    closest_timestamp = upper_bound_it->first;
  } else {
    const double upper_bound_dt = std::abs(timestamp - upper_bound_it->first);
    const auto lower_bound_it = std::prev(upper_bound_it);
    const double lower_bound_dt = std::abs(timestamp - lower_bound_it->first);
    if (upper_bound_dt < lower_bound_dt) {
      key_index = upper_bound_it->second;
      closest_timestamp = upper_bound_it->first;
    } else {
      key_index = lower_bound_it->second;
      closest_timestamp = lower_bound_it->first;
    }
  }

  DLOG(INFO) << "ClosestPoseTimestamp: dt is " << std::abs(timestamp - closest_timestamp);

  if (!values_.exists(sym::P(key_index))) {
    LOG(FATAL) << "ClosestPoseTimestamp: Pose key not present in values.";
  }

  return closest_timestamp;
}

std::pair<lc::Time, lc::Time> GraphValues::LowerAndUpperBoundTimestamp(const lc::Time timestamp) const {
  // TODO(rsoussan): account for this better -> use boost::optional?
  if (Empty()) {
    LOG(FATAL) << "LowerAndUpperBoundTimestamp: No states available.";
  }

  // lower bound returns first it >= query, call this upper bound
  const auto upper_bound_it = timestamp_key_index_map_.lower_bound(timestamp);
  if (upper_bound_it == timestamp_key_index_map_.cend()) {
    LOG(FATAL) << "LowerAndUpperBoundTimestamp: No upper bound timestamp exists.";
  } else if (upper_bound_it == timestamp_key_index_map_.cbegin()) {
    LOG(FATAL) << "LowerAndUpperBoundTimestamp: No lower bound timestamp exists.";
  }
  const auto lower_bound_it = std::prev(upper_bound_it);

  const auto upper_bound_key_index = upper_bound_it->second;
  if (!values_.exists(sym::P(upper_bound_key_index))) {
    LOG(FATAL) << "LowerAndUpperBoundTimestamp: Upper bound pose key not "
                  "present in values.";
  }

  const auto lower_bound_key_index = lower_bound_it->second;
  if (!values_.exists(sym::P(lower_bound_key_index))) {
    LOG(FATAL) << "LowerAndUpperBoundTimestamp: Lower bound pose key not "
                  "present in values.";
  }

  return {lower_bound_it->first, upper_bound_it->first};
}

bool GraphValues::HasKey(const lc::Time timestamp) const { return (timestamp_key_index_map_.count(timestamp) != 0); }

bool GraphValues::Empty() const { return timestamp_key_index_map_.empty(); }

gtsam::Key GraphValues::PoseKey(const lc::Time timestamp) const {
  // TODO(rsoussan): account for this better -> use boost::optional?
  if (timestamp_key_index_map_.count(timestamp) == 0) {
    LOG(FATAL) << "PoseKey: No CombinedNavState found at timestamp." << std::setprecision(15)
               << "timestamp: " << timestamp << std::endl
               << std::setprecision(15) << "oldest: " << OldestTimestamp() << std::endl
               << std::setprecision(15) << "latest: " << LatestTimestamp();
  }

  const int key_index = timestamp_key_index_map_.at(timestamp);

  if (!values_.exists(sym::P(key_index))) {
    LOG(FATAL) << "PoseKey: Pose key not present in values.";
  }

  return sym::P(key_index);
}

lm::CombinedNavState GraphValues::GetCombinedNavState(const lc::Time timestamp) const {
  // TODO(rsoussan): account for this better -> use boost::optional?
  if (!HasKey(timestamp)) {
    LOG(FATAL) << "GetCombinedNavState: No CombinedNavState found at timestamp.";
  }
  const auto key_index = KeyIndex(timestamp);

  return lm::CombinedNavState(at<gtsam::Pose3>(sym::P(key_index)), at<gtsam::Velocity3>(sym::V(key_index)),
                              at<gtsam::imuBias::ConstantBias>(sym::B(key_index)), timestamp);
}

int GraphValues::LatestCombinedNavStateKeyIndex() const {
  // TODO(rsoussan): account for this better -> use boost::optional?
  if (Empty()) {
    LOG(FATAL) << "LatestCombinedNavStateKeyIndex: No combined nav states available.";
  }
  return timestamp_key_index_map_.crbegin()->second;
}

int GraphValues::OldestCombinedNavStateKeyIndex() const {
  // TODO(rsoussan): account for this better -> use boost::optional?
  if (Empty()) {
    LOG(FATAL) << "OldestCombinedNavStateKeyIndex: No combined nav states available.";
  }
  return timestamp_key_index_map_.cbegin()->second;
}

gtsam::imuBias::ConstantBias GraphValues::LatestBias() const {
  // TODO(rsoussan): account for this better -> use boost::optional?
  if (Empty()) {
    LOG(FATAL) << "LatestBias: No bias values available.";
  }

  const int key_index = timestamp_key_index_map_.crbegin()->second;

  if (!values_.exists(sym::B(key_index))) {
    LOG(FATAL) << "LatestBias: Bias key not present in values.";
  }

  return at<gtsam::imuBias::ConstantBias>(sym::B(key_index));
}

int GraphValues::SlideWindow(gtsam::NonlinearFactorGraph& graph) {
  if (Empty()) {
    LOG(FATAL) << "SlideWindow: No states in map.";
    return 0;
  }
  const double total_duration = timestamp_key_index_map_.crbegin()->first - timestamp_key_index_map_.cbegin()->first;
  DLOG(INFO) << "SlideWindow: Starting total num states: " << timestamp_key_index_map_.size();
  DLOG(INFO) << "SlideWindow: Starting total duration is " << total_duration;
  const lc::Time ideal_oldest_allowed_state =
      std::max(0.0, timestamp_key_index_map_.crbegin()->first - kWindowIdealDuration);
  int num_states_removed = 0;
  while (timestamp_key_index_map_.begin()->first < ideal_oldest_allowed_state &&
         timestamp_key_index_map_.size() > kWindowMinNumStates) {
    RemoveCombinedNavStateAndFactors(timestamp_key_index_map_.begin()->first, graph);
    ++num_states_removed;
  }
  DLOG(INFO) << "SlideWindow: New total num states: " << timestamp_key_index_map_.size();
  const double new_total_duration =
      timestamp_key_index_map_.crbegin()->first - timestamp_key_index_map_.cbegin()->first;
  DLOG(INFO) << "SlideWindow: New total duration is " << new_total_duration;
  DLOG(INFO) << "SlideWindow: Num states removed: " << num_states_removed;
  return num_states_removed;
}

// Add timestamp and keys to timestamp_key_index_map, and values to values
void GraphValues::AddCombinedNavState(const lm::CombinedNavState& combined_nav_state, const int key_index) {
  // TODO(rsoussan): remove or add option to disable these checks
  if (HasKey(combined_nav_state.timestamp())) {
    LOG(ERROR) << "AddCombinedNavState: Timestamp key index map already "
                  "contains timestamp.";
    return;
  }
  timestamp_key_index_map_.emplace(combined_nav_state.timestamp(), key_index);
  if (values_.exists(sym::P(key_index))) {
    LOG(ERROR) << "AddCombinedNavState: Pose key already in values.";
  }
  if (values_.exists(sym::V(key_index))) {
    LOG(ERROR) << "AddCombinedNavState: Velocity key already in values.";
  }
  if (values_.exists(sym::B(key_index))) {
    LOG(ERROR) << "AddCombinedNavState: Bias key already in values.";
  }

  values_.insert(sym::P(key_index), combined_nav_state.pose());
  values_.insert(sym::V(key_index), combined_nav_state.velocity());
  values_.insert(sym::B(key_index), combined_nav_state.bias());

  DLOG(INFO) << "AddCombinedNavState: Added key_index " << key_index;
  DLOG(INFO) << "AddCombinedNavState: Added timestamp " << std::setprecision(15) << combined_nav_state.timestamp();
}

int GraphValues::KeyIndex(const lc::Time timestamp) const {
  if (!HasKey(timestamp)) {
    LOG(FATAL) << "KeyIndex: No key found for timestamp.";
  }

  return timestamp_key_index_map_.at(timestamp);
}

void GraphValues::UpdateValues(const gtsam::Values& new_values) { values_ = new_values; }

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

  DLOG(INFO) << "RemoveCombinedNavState: Removed key index " << key_index;
  DLOG(INFO) << "RemoveCombinedNavState: Removed timestamp" << std::setprecision(15) << timestamp;
  return removed_values;
}
}  // namespace graph_localizer
