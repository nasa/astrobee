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

#ifndef GRAPH_LOCALIZER_GRAPH_VALUES_H_
#define GRAPH_LOCALIZER_GRAPH_VALUES_H_

#include <localization_measurements/combined_nav_state.h>
#include <localization_measurements/time.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <glog/logging.h>

#include <map>
#include <utility>

namespace graph_localizer {
// TODO(rsoussan): this already exists in graph_localizer.h, is that legal?
namespace sym = gtsam::symbol_shorthand;
class GraphValues {
 public:
  explicit GraphValues(const double window_ideal_duration, const int window_min_num_states);

  // Add timestamp and keys to timestamp_key_index_map, and values to values
  void AddCombinedNavState(const localization_measurements::CombinedNavState &combined_nav_state, const int key_index);

  // Removes keys from timestamp map, values from values.
  // Also removes any factors using these keys from graph argument
  bool RemoveCombinedNavStateAndFactors(const localization_measurements::Time timestamp,
                                        gtsam::NonlinearFactorGraph &graph);

  localization_measurements::CombinedNavState LatestCombinedNavState() const;

  localization_measurements::CombinedNavState OldestCombinedNavState() const;

  int OldestCombinedNavStateKeyIndex() const;

  int LatestCombinedNavStateKeyIndex() const;

  gtsam::imuBias::ConstantBias LatestBias() const;

  // Removes keys and their values that are too old.
  // Also removes any factors using these keys from graph argument.
  // Returns number of states removed.
  int SlideWindow(gtsam::NonlinearFactorGraph &graph);

  int KeyIndex(const localization_measurements::Time timestamp) const;

  void UpdateValues(const gtsam::Values &new_values);

  const gtsam::Values &values() const { return values_; }

  gtsam::Key PoseKey(const localization_measurements::Time timestamp) const;

  localization_measurements::Time OldestTimestamp() const;

  localization_measurements::Time LatestTimestamp() const;

  localization_measurements::Time ClosestPoseTimestamp(const localization_measurements::Time timestamp) const;

  // Assumes timestamp is within bounds of graph values timestamps.
  std::pair<localization_measurements::Time, localization_measurements::Time> LowerAndUpperBoundTimestamp(
      const localization_measurements::Time timestamp) const;

  bool HasKey(const localization_measurements::Time timestamp) const;

  // TODO(rsoussan): move this somewhere else
  template <class FACTOR>
  static bool ContainsCombinedNavStateKey(const FACTOR &factor, const int key_index) {
    if (factor.find(sym::P(key_index)) != factor.end()) return true;
    if (factor.find(sym::V(key_index)) != factor.end()) return true;
    if (factor.find(sym::B(key_index)) != factor.end()) return true;
    return false;
  }

  localization_measurements::CombinedNavState GetCombinedNavState(
      const localization_measurements::Time timestamp) const;

  template <typename ValueType>
  ValueType at(const gtsam::Key &key) const {
    if (!values_.exists(key)) {
      LOG(FATAL) << "at: Key not present in values.";
    }

    return values_.at<ValueType>(key);
  }

 private:
  // Removes keys from timestamp_key_index_map, values from values
  bool RemoveCombinedNavState(const localization_measurements::Time timestamp);

  bool Empty() const;

  // Ideal since this is only kept if there are at least kWindowMinNumberStates
  const double kWindowIdealDuration;
  // Don't leave less than kWindowMinNumberStates per state in window if
  // possible
  const double kWindowMinNumStates;
  gtsam::Values values_;
  std::map<localization_measurements::Time, int> timestamp_key_index_map_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_VALUES_H_
