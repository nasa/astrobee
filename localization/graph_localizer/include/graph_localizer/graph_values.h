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

#include <graph_localizer/key_info.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/time.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <glog/logging.h>

#include <boost/optional.hpp>

#include <map>
#include <utility>

namespace graph_localizer {
// TODO(rsoussan): this already exists in graph_localizer.h, is that legal?
namespace sym = gtsam::symbol_shorthand;
class GraphValues {
 public:
  explicit GraphValues(const double window_ideal_duration, const int window_min_num_states);

  // Add timestamp and keys to timestamp_key_index_map, and values to values
  bool AddCombinedNavState(const localization_common::CombinedNavState& combined_nav_state, const int key_index);

  // Removes keys from timestamp map, values from values.
  // Also removes any factors using these keys from graph argument
  bool RemoveCombinedNavStateAndFactors(const localization_common::Time timestamp, gtsam::NonlinearFactorGraph& graph);

  boost::optional<localization_common::CombinedNavState> LatestCombinedNavState() const;

  boost::optional<localization_common::CombinedNavState> OldestCombinedNavState() const;

  boost::optional<int> OldestCombinedNavStateKeyIndex() const;

  boost::optional<int> LatestCombinedNavStateKeyIndex() const;

  boost::optional<std::pair<gtsam::imuBias::ConstantBias, localization_common::Time>> LatestBias() const;

  // Removes keys and their values that are too old.
  // Also removes any factors using these keys from graph argument.
  // Returns number of states removed.
  int SlideWindow(gtsam::NonlinearFactorGraph& graph);

  boost::optional<int> KeyIndex(const localization_common::Time timestamp) const;

  void UpdateValues(const gtsam::Values& new_values);

  const gtsam::Values& values() const { return values_; }

  boost::optional<gtsam::Key> PoseKey(const localization_common::Time timestamp) const;

  boost::optional<gtsam::Key> GetKey(KeyCreatorFunction key_creator_function,
                                     const localization_common::Time timestamp) const;

  boost::optional<localization_common::Time> OldestTimestamp() const;

  boost::optional<localization_common::Time> LatestTimestamp() const;

  boost::optional<localization_common::Time> ClosestPoseTimestamp(const localization_common::Time timestamp) const;

  // Assumes timestamp is within bounds of graph values timestamps.
  std::pair<boost::optional<localization_common::Time>, boost::optional<localization_common::Time>>
  LowerAndUpperBoundTimestamp(const localization_common::Time timestamp) const;

  boost::optional<localization_common::CombinedNavState> LowerBoundOrEqualCombinedNavState(
      const localization_common::Time timestamp) const;

  bool HasKey(const localization_common::Time timestamp) const;

  // TODO(rsoussan): move this somewhere else
  template <class FACTOR>
  static bool ContainsCombinedNavStateKey(const FACTOR& factor, const int key_index) {
    if (factor.find(sym::P(key_index)) != factor.end()) return true;
    if (factor.find(sym::V(key_index)) != factor.end()) return true;
    if (factor.find(sym::B(key_index)) != factor.end()) return true;
    return false;
  }

  boost::optional<localization_common::CombinedNavState> GetCombinedNavState(
      const localization_common::Time timestamp) const;

  template <typename ValueType>
  boost::optional<ValueType> at(const gtsam::Key& key) const {
    if (!values_.exists(key)) {
      LOG(ERROR) << "at: Key not present in values.";
      return boost::none;
    }

    return values_.at<ValueType>(key);
  }

 private:
  // Removes keys from timestamp_key_index_map, values from values
  bool RemoveCombinedNavState(const localization_common::Time timestamp);

  bool Empty() const;

  boost::optional<localization_common::Time> LowerBoundOrEqualTimestamp(
      const localization_common::Time timestamp) const;

  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(values_);
    ar& BOOST_SERIALIZATION_NVP(timestamp_key_index_map_);
  }

  // Ideal since this is only kept if there are at least kWindowMinNumberStates
  const double kWindowIdealDuration;
  // Don't leave less than kWindowMinNumberStates per state in window if
  // possible
  const double kWindowMinNumStates;
  gtsam::Values values_;
  std::map<localization_common::Time, int> timestamp_key_index_map_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_GRAPH_VALUES_H_
