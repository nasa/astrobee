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

#ifndef GRAPH_OPTIMIZER_GRAPH_VALUES_H_
#define GRAPH_OPTIMIZER_GRAPH_VALUES_H_

#include <graph_localizer/graph_values_params.h>
#include <graph_localizer/key_info.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/logger.h>
#include <localization_common/time.h>
#include <localization_measurements/feature_point.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>
#include <boost/serialization/unordered_map.hpp>

#include <map>
#include <unordered_map>
#include <utility>

namespace graph_localizer {
namespace sym = gtsam::symbol_shorthand;
class GraphValues {
 public:
  explicit GraphValues(const GraphValuesParams& params = GraphValuesParams());

  // Add timestamp and keys to timestamp_key_index_map, and values to values
  bool AddCombinedNavState(const localization_common::CombinedNavState& combined_nav_state, const int key_index);

  boost::optional<localization_common::CombinedNavState> LatestCombinedNavState() const;

  boost::optional<localization_common::CombinedNavState> OldestCombinedNavState() const;

  boost::optional<int> OldestCombinedNavStateKeyIndex() const;

  boost::optional<int> LatestCombinedNavStateKeyIndex() const;

  boost::optional<std::pair<gtsam::imuBias::ConstantBias, localization_common::Time>> LatestBias() const;

  // Returns the oldest time that will be in graph values once the window is slid using params
  boost::optional<localization_common::Time> SlideWindowNewOldestTime() const;

  boost::optional<int> KeyIndex(const localization_common::Time timestamp) const;

  void UpdateValues(const gtsam::Values& new_values);

  // TODO(rsoussan): Put this somewhere else?
  static gtsam::NonlinearFactorGraph RemoveOldFactors(const gtsam::KeyVector& old_keys,
                                                      gtsam::NonlinearFactorGraph& graph);

  gtsam::KeyVector OldFeatureKeys(const gtsam::NonlinearFactorGraph& factors) const;

  void RemoveOldFeatures(const gtsam::KeyVector& old_feature_keys);

  int RemoveOldCombinedNavStates(const localization_common::Time oldest_allowed_time);

  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time) const;

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

  template <class FACTOR>
  static bool ContainsCombinedNavStateKey(const FACTOR& factor, const int key_index) {
    if (factor.find(sym::P(key_index)) != factor.end()) return true;
    if (factor.find(sym::V(key_index)) != factor.end()) return true;
    if (factor.find(sym::B(key_index)) != factor.end()) return true;
    return false;
  }

  boost::optional<localization_common::CombinedNavState> GetCombinedNavState(
    const localization_common::Time timestamp) const;

  double Duration() const;

  int NumStates() const;

  template <typename ValueType>
  boost::optional<ValueType> at(const gtsam::Key& key) const {
    if (!values_.exists(key)) {
      LogError("at: Key not present in values.");
      return boost::none;
    }

    return values_.at<ValueType>(key);
  }

  boost::optional<localization_common::Time> Timestamp(const int key_index) const;

  bool HasFeature(const localization_measurements::FeatureId id) const;

  boost::optional<gtsam::Key> FeatureKey(const localization_measurements::FeatureId id) const;

  // TODO(rsoussan): This shouldn't be const, modify when changes are made to projection factor adder
  gtsam::Key CreateFeatureKey() const;

  bool AddFeature(const localization_measurements::FeatureId id, const gtsam::Point3& feature_point,
                  const gtsam::Key& key);

  gtsam::KeyVector FeatureKeys() const;

  int NumFeatures() const;

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
    ar& BOOST_SERIALIZATION_NVP(feature_id_key_map_);
    ar& BOOST_SERIALIZATION_NVP(feature_key_index_);
  }

  GraphValuesParams params_;
  gtsam::Values values_;
  std::map<localization_common::Time, int> timestamp_key_index_map_;
  std::unordered_map<localization_measurements::FeatureId, gtsam::Key> feature_id_key_map_;
  // Modified by projection_factor_adder, remove mutable if this changes
  mutable std::uint64_t feature_key_index_;
};
}  // namespace graph_localizer

#endif  // GRAPH_OPTIMIZER_GRAPH_VALUES_H_
