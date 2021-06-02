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

#ifndef GRAPH_LOCALIZER_FEATURE_POINT_GRAPH_VALUES_H_
#define GRAPH_LOCALIZER_FEATURE_POINT_GRAPH_VALUES_H_

#include <graph_optimizer/graph_values.h>
#include <localization_common/logger.h>
#include <localization_common/time.h>
#include <localization_measurements/feature_point.h>

#include <gtsam/geometry/Point3.h>
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
// TODO(rsoussan): Make seperate base class for static graph values so don't have to return boost::none
// for so many virtual fcns
class FeaturePointGraphValues : public graph_optimizer::GraphValues {
 public:
  FeaturePointGraphValues(std::shared_ptr<gtsam::Values> values = std::shared_ptr<gtsam::Values>(new gtsam::Values()));

  // Returns the oldest time that will be in graph values once the window is slid using params
  boost::optional<localization_common::Time> SlideWindowNewOldestTime() const final;

  gtsam::KeyVector OldKeys(const localization_common::Time oldest_allowed_time,
                           const gtsam::NonlinearFactorGraph& graph) const final;

  boost::optional<gtsam::Key> GetKey(graph_optimizer::KeyCreatorFunction key_creator_function,
                                     const localization_common::Time timestamp) const final;

  boost::optional<localization_common::Time> OldestTimestamp() const final;

  boost::optional<localization_common::Time> LatestTimestamp() const final;

  gtsam::KeyVector OldFeatureKeys(const gtsam::NonlinearFactorGraph& factors) const;

  void RemoveOldFeatures(const gtsam::KeyVector& old_feature_keys);

  gtsam::KeyVector FeatureKeys() const;

  int NumFeatures() const;

  bool AddFeature(const localization_measurements::FeatureId id, const gtsam::Point3& feature_point,
                  const gtsam::Key& key);

  bool HasFeature(const localization_measurements::FeatureId id) const;

  boost::optional<gtsam::Key> FeatureKey(const localization_measurements::FeatureId id) const;

  // TODO(rsoussan): This shouldn't be const, modify when changes are made to projection factor adder
  gtsam::Key CreateFeatureKey() const;

 private:
  // Serialization function
  friend class boost::serialization::access;
  template <class ARCHIVE>
  void serialize(ARCHIVE& ar, const unsigned int /*version*/) {
    ar& BOOST_SERIALIZATION_NVP(feature_id_key_map_);
    ar& BOOST_SERIALIZATION_NVP(feature_key_index_);
  }

  std::unordered_map<localization_measurements::FeatureId, gtsam::Key> feature_id_key_map_;
  // Modified by projection_factor_adder, remove mutable if this changes
  mutable std::uint64_t feature_key_index_;
};
}  // namespace graph_localizer

#endif  // GRAPH_LOCALIZER_FEATURE_POINT_GRAPH_VALUES_H_
