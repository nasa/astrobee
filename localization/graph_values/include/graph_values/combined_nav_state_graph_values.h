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

#ifndef GRAPH_VALUES_COMBINED_NAV_STATE_GRAPH_VALUES_H_
#define GRAPH_VALUES_COMBINED_NAV_STATE_GRAPH_VALUES_H_

#include <graph_optimizer/graph_values.h>
#include <graph_values/combined_nav_state_graph_values_params.h>
#include <localization_common/combined_nav_state.h>
#include <localization_common/logger.h>
#include <localization_common/time.h>
#include <vision_common/feature_point.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <boost/optional.hpp>
#include <boost/serialization/unordered_map.hpp>

#include <map>
#include <unordered_map>
#include <utility>
#include <vector>

namespace graph_values {
namespace sym = gtsam::symbol_shorthand;
class CombinedNavStateGraphValues : public graph_optimizer::GraphValues {
 public:
  CombinedNavStateGraphValues(
    const CombinedNavStateGraphValuesParams& params = CombinedNavStateGraphValuesParams(),
    std::shared_ptr<gtsam::Values> values = std::shared_ptr<gtsam::Values>(new gtsam::Values()));

  // Add timestamp and keys to timestamp_key_index_map, and values to values
  bool AddCombinedNavState(const localization_common::CombinedNavState& combined_nav_state, const int key_index);

  boost::optional<localization_common::CombinedNavState> LatestCombinedNavState() const;

  boost::optional<localization_common::CombinedNavState> OldestCombinedNavState() const;

  boost::optional<int> OldestCombinedNavStateKeyIndex() const;

  boost::optional<int> LatestCombinedNavStateKeyIndex() const;

  boost::optional<std::pair<gtsam::imuBias::ConstantBias, localization_common::Time>> LatestBias() const;

  // Returns the oldest time that will be in graph values once the window is slid using params
  boost::optional<localization_common::Time> SlideWindowNewOldestTime() const final;

  boost::optional<int> KeyIndex(const localization_common::Time timestamp) const;

  int RemoveOldCombinedNavStates(const localization_common::Time oldest_allowed_time);

  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time,
                           const gtsam::NonlinearFactorGraph& graph) const final;

  boost::optional<gtsam::Key> PoseKey(const localization_common::Time timestamp) const;

  boost::optional<gtsam::Key> GetKey(graph_optimizer::KeyCreatorFunction key_creator_function,
                                     const localization_common::Time timestamp) const final;

  boost::optional<localization_common::Time> OldestTimestamp() const final;

  boost::optional<localization_common::Time> LatestTimestamp() const final;

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

  boost::optional<localization_common::Time> Timestamp(const int key_index) const;

  const CombinedNavStateGraphValuesParams& params() const;

  std::vector<localization_common::Time> Timestamps() const;

  boost::optional<localization_common::Time> Timestamp(graph_optimizer::KeyCreatorFunction key_creator_function,
                                                       const gtsam::Key key) const;

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
    ar& BOOST_SERIALIZATION_BASE_OBJECT_NVP(graph_optimizer::GraphValues);
    ar& BOOST_SERIALIZATION_NVP(timestamp_key_index_map_);
    ar& BOOST_SERIALIZATION_NVP(params_);
  }

  CombinedNavStateGraphValuesParams params_;
  std::map<localization_common::Time, int> timestamp_key_index_map_;
};
}  // namespace graph_values

#endif  // GRAPH_VALUES_COMBINED_NAV_STATE_GRAPH_VALUES_H_
